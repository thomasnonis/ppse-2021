/**
 * @file SALMO.c
 * @author SALMO Society
 * @brief  Finding new way to reach MPPT
 * @version 0.1
 * @date 2022-04-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "SALMO.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "../tracking-algorithm/sun_tracker.h"
#include "../MPU6050/MPU6050.h"
#include "../HMC5883L/HMC5883L.h"
#include "../PAM7Q/PAM7Q.h"
#include "../timer/pico_timer.h"
#include "../nmea-parser/minmea.h"
#include "../stepper/pico_stepper.h"

// GPS CONFIG
#define DEBUG_GPS_PARSER
#define GPS_MAX_SENTENCES 12
#define GPS_MAX_SENTENCE_LENGTH 100
char gps_raw_sentences[GPS_MAX_SENTENCES][GPS_MAX_SENTENCE_LENGTH];
int gps_read_lines = 0;

// TIMER CONFIG
bool update_position;
bool read_gps;
bool update_motors_position;
struct repeating_timer update_position_timer;
struct repeating_timer gps_read_timer;
struct repeating_timer update_motors_position_timer;
#define UPDATE_SUN_POSITION_TIMER_PERIOD_MS 4000
#define GPS_READ_TIMER_PERIOD_MS 4000
#define UPDATE_MOTORS_POSITION_TIMER_PERIOD_MS 10000

// SALMO Functions state
bool tracking_enable_pressed = false;
bool go_home_enable_pressed = false;

// GLOBAL VARS
Place gps_parsed_place;
PicoStepper stepper1 = {0};
PicoStepper stepper2 = {0};

float compute_compass_degree();
float compute_acc_degree();
int calculate_steps(float current_angle, float desired_angle);
void go_home(PicoStepper *stepper1, PicoStepper *stepper2);
void move_motors_to_the_sun(Position *pos, PicoStepper *stepper1, PicoStepper *stepper2);
void nmea_parse(const char *msg, Place *parsed_place);
void init_position_timer(int time_ms);
void init_motors_timer(int time_ms);
void init_gps_timer(int time_ms);
void gpio_irq_callback(uint gpio, uint32_t events);

void hello_salmo()
{
    printf(
        "\r\n"
        "███████  █████  ██      ███    ███  ██████  \r\n"
        "██      ██   ██ ██      ████  ████ ██    ██ \r\n"
        "███████ ███████ ██      ██ ████ ██ ██    ██ \r\n"
        "     ██ ██   ██ ██      ██  ██  ██ ██    ██ \r\n"
        "███████ ██   ██ ███████ ██      ██  ██████  \r\n\r\n");
}

/**
 * @brief GPS read timer callback
 *
 * @param t gps timer
 */
bool gps_read_callback(struct repeating_timer *t)
{
    read_gps = true;
    return read_gps;
}

/**
 * @brief Set update position flag every t.delay_us
 * @param t Pointer to the timer structure
 */
bool update_position_callback(struct repeating_timer *t)
{
    // printf("Update motor position tim elapsed -> Time: %lld\n", time_us_64());
    update_position = true;
    return update_position;
}

/**
 * @brief Set update position flag every t.delay_us
 * @param t Pointer to the timer structure
 */
bool update_motors_position_callback(struct repeating_timer *t)
{
    // printf("Update motor position tim elapsed -> Time: %lld\n", time_us_64());
    update_motors_position = true;
    return update_motors_position;
}

int main()
{

    /* Used variables */
    int16_t acc_gyro_data = 0;
    Position sun_position;
    char gps_rx_buffer[100];

#ifdef GPS_SIMULATIOR
    // Sample place and its respective conversion into sun position
    Place manual_place = {2030, 2, 28, 10, 10, 10, 55.751244, 37.618423};
    Position manual_position = compute_complete_position(&manual_place);
#endif

// #define FAMOUS_CITIES_SIMULATION
#ifdef FAMOUS_CITIES_SIMULATION
    Place rome = {2022, 6, 5, 10, 0, 20, 41.9027835, 12.4963655};
    Position rome_position = compute_complete_position(&rome);

    // TODO: check why minutes are not mapped correctly
    Place berlin = {2022, 6, 5, 12, 40, 00, 52.520008, 13.404954};
    Position berlin_position = compute_complete_position(&berlin);

    Place rio = {2022, 6, 5, 18, 00, 00, -22.908333, -43.196388};
    Position rio_position = compute_complete_position(&rio);
#endif

    /* Initialization of peripherals*/

    // stdio_init_all();  // equals to usb and uart init
    stdio_usb_init();  // initialize usb cdc
    stdio_uart_init(); // initialize uart0 with 0 baud rate
    PAM7Q_init(UART1_TX, UART1_RX);

#ifdef INTERRUPTS_ARE_WORKING
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(GPS_UART_ID, false, false);
#endif

    /* SET GPS ENABLE PIN */
    gpio_init(GPS_EN);
    gpio_set_dir(GPS_EN, GPIO_OUT);
    gpio_put(GPS_EN, 0); // with zero it's enabled

    update_position = false;

    // SET PWM PIN
    gpio_set_function(BUZZ_EN, GPIO_FUNC_PWM);
#ifdef BUZZER_MOUNTED
    uint pwm_channel = pwm_gpio_to_slice_num(BUZZ_EN);
    pwm_set_enabled(pwm_channel, true);
    pwm_set_wrap(pwm_channel, 500);
    pwm_set_chan_level(pwm_channel, PWM_CHAN_A, 250);
#endif

    /* Stepper motors initialization  */
    picoStepperInit(&stepper1, M1_W11, M1_W12, M1_W21, M1_W22, STEPS_PER_REV, INITIAL_SPEED); // stepper1 = yaw
    picoStepperInit(&stepper2, M2_W11, M2_W12, M2_W21, M2_W22, STEPS_PER_REV, INITIAL_SPEED); // stepper2 = pitch

    /* SET HOME AND GPS_ENABLE SWITCH  */
    gpio_init(NOT_HOME_SW);
    gpio_init(NOT_EN_SW);
    gpio_set_dir(NOT_HOME_SW, GPIO_IN);
    gpio_set_dir(NOT_EN_SW, GPIO_IN);
    gpio_set_input_enabled(NOT_HOME_SW, true);
    gpio_set_input_enabled(NOT_EN_SW, true);
    gpio_pull_up(NOT_HOME_SW);
    gpio_pull_up(NOT_EN_SW);
    // gpio irq handler - NOTE: Currently the GPIO parameter is ignored, and this callback will be called for any enabled GPIO IRQ on any pin.
    gpio_set_irq_enabled_with_callback(NOT_HOME_SW, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_callback);
    gpio_set_irq_enabled_with_callback(NOT_EN_SW, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_callback);

    /* SET I2C Pins */
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C1_SDA, I2C1_SCL, GPIO_FUNC_I2C));
    i2c_init(I2C_PERIPHERAL, I2C_PORT1_BAUD_RATE);

    /* Verify if are needed, probably they are already in the sensor breakout board
        gpio_pull_up(I2C1_SDA);
        gpio_pull_up(I2C1_SCL);
    */

    // Init i2c port1 with defined baud rate
    // I2C MPU port is defined inside every .h device library

    /* Startup message with relative delay */
    sleep_ms(2000);
    hello_salmo();
    sleep_ms(1000);
    printf("Peripherals initialization...\r\n");

    /* MPU6050 Initialization */
    MPU6050_Initialize();
    if (MPU6050_TestConnection())
    {
        printf("MPU6050 connection success\n");
    }
    else
    {
        printf("MPU6050 connection failed\n");
    }
    MPU6050_GetRawAccelGyro(&acc_gyro_data);

    HMC5883L_initialize();
    if (isHMC())
    {
        printf("HMC5883L connection success\n");
    }
    else
    {
        printf("HMC5883L connection failed\n");
    }
    compute_compass_degree();

    /* Timers initialization */
    init_position_timer(UPDATE_SUN_POSITION_TIMER_PERIOD_MS);
    init_gps_timer(GPS_READ_TIMER_PERIOD_MS);
    init_motors_timer(UPDATE_MOTORS_POSITION_TIMER_PERIOD_MS);

    while (true)
    {
        /* Go home button relative behaviour */
        if (go_home_enable_pressed)
        {
            go_home(&stepper1, &stepper2);
            go_home_enable_pressed = false;
            tracking_enable_pressed = false;
        }
        /* Tracking button relative behaviour */
        if (tracking_enable_pressed)
        {
            /* When relative timer elapses the motors are powered */
            if (update_motors_position)
            {
                move_motors_to_the_sun(&sun_position, &stepper1, &stepper2);
                update_motors_position = false;
            }
        }
        if (read_gps)
        {
            gps_read_lines = 0;
            memset(gps_rx_buffer, 0, sizeof(gps_rx_buffer));
            while (gps_read_lines < GPS_MAX_SENTENCES)
            {
                while (uart_is_readable(GPS_UART_ID))
                {
                    char ch = uart_getc(GPS_UART_ID);
                    strncat(gps_rx_buffer, &ch, 1);
                    if (ch == '\n')
                    {
                        strncat(gps_rx_buffer, '\0', 1);
                        strcpy(gps_raw_sentences[gps_read_lines], gps_rx_buffer);
                        memset(gps_rx_buffer, 0, sizeof(gps_rx_buffer));
                        gps_read_lines++;
                    }
                }
            }
            printf("--------->GPS PARSING\r\n");
            read_gps = false;
            for (int i = 0; i < GPS_MAX_SENTENCES; i++)
            {
#ifdef DEBUG_GPS_PARSER
                printf("PARSED LINE%d | %s\r\n", i, gps_raw_sentences[i]);
#endif
                nmea_parse(gps_raw_sentences[i], &gps_parsed_place);
            }
            print_place(&gps_parsed_place);
            printf("<---------END OF GPS PARSING\r\n");
        }

        if (update_position)
        {
#ifdef FAMOUS_CITIES_SIMULATION
            /* Simulate sun position of some famous cities with fixed position and hour*/
            printf("[ROME] ");
            print_place(&rome);
            printf("[ROME] Position elevation %f azimuth %f \r\n\r\n", rome_position.elevation, rome_position.azimuth);

            printf("[BERLIN] ");
            print_place(&berlin);
            printf("[BERLIN] Position elevation %f azimuth %f \r\n\r\n", berlin_position.elevation, berlin_position.azimuth);

            printf("[RIO] ");
            print_place(&rio);
            printf("[RIO] Position elevation %f azimuth %f \r\n\r\n", rio_position.elevation, rio_position.azimuth);
#endif

#ifdef GPS_SIMULATOR
            /* Simulate sun position based on a manual position hardcoded which canges over time*/
            print_place(&manual_place);
            manual_position = compute_complete_position(&manual_place);
            printf("[MANUAL] Position elevation %f azimuth %f \r\n\r\n", manual_position.elevation, manual_position.azimuth);
            if (manual_place.hour + 1 > 23)
            {
                manual_place.hour = 0;
                manual_place.day++;
            }
            else
            {
                manual_place.hour++;
            };
#endif

            /* Compute sun position */
            sun_position = compute_complete_position(&gps_parsed_place);
            printf("Sun elevation %f azimuth %f \r\n\r\n", sun_position.elevation, sun_position.azimuth);
            update_position = false;
            printf("Go home active?: %s, Tracking active?: %s \r\n", go_home_enable_pressed ? "true" : "false", tracking_enable_pressed ? "true" : "false");
            printf("-------\r\n");
        }
        sleep_ms(200);
    }

    return 0;
}

/**
 * @brief gpio irq callback
 *
 * @param gpio gpio number
 * @param events irq event
 */
void gpio_irq_callback(uint gpio, uint32_t events)
{
    printf("GPIO TRIGGERED\r\n");
    if (gpio == NOT_HOME_SW)
    {
        printf("Home button pressed\n");
        go_home_enable_pressed = !go_home_enable_pressed;
    }
    else if (gpio == NOT_EN_SW)
    {
        printf("Tracking enable pressed \r\n");
        tracking_enable_pressed = !tracking_enable_pressed;
    }
}

/**
 * @brief Initialize the timer for position update
 *
 * @param time_ms Timer delay in milliseconds
 */
void init_position_timer(int time_ms)
{
    add_repeating_timer_ms(time_ms, update_position_callback, NULL, &update_position_timer);
}

/**
 * @brief Initialize the timer for gps update
 *
 * @param time_ms Timer delay in milliseconds
 */
void init_motors_timer(int time_ms)
{
    add_repeating_timer_ms(time_ms, update_motors_position_callback, NULL, &update_motors_position_timer);
}

/**
 * @brief Initialize the timer for gps update
 *
 * @param time_ms Timer delay in milliseconds
 */
void init_gps_timer(int time_ms)
{
    add_repeating_timer_ms(time_ms, gps_read_callback, NULL, &gps_read_timer);
}

/**
 * @brief compute compass headings in degree
 * @return compass heading in degree
 */
float compute_compass_degree()
{
    int16_t HMCx = 0;
    int16_t HMCy = 0;
    int16_t HMCz = 0;
    HMC5883L_getHeading(&HMCx, &HMCy, &HMCz);

    // Assuming Z axis is pointing up, Y axis is pointing straight out of the panel and X axis is pointing to the right
    float compass_radians = atan2(HMCx, HMCy); // note: atan2(y,x) = tan^-1 (arg1/arg2) and auto matches the quadrant
    return compass_radians * 180 / M_PI;
}

/**
 * @brief compute accelerometer headings in degree
 * @return accelerometer heading in degree
 */
float compute_acc_degree()
{
    int16_t AccGyro[6];
    int16_t AccX = 0;
    int16_t AccY = 0;
    int16_t AccZ = 0;

    MPU6050_GetRawAccelGyro(AccGyro);
    AccX = AccGyro[0];
    AccY = AccGyro[1];
    AccZ = AccGyro[2];

    // assuming Z axis is pointing up when the panel is flat
    float tilt_radians = atan2(sqrt(pow(AccX, 2) + pow(AccY, 2)), AccZ);
    return tilt_radians * 180 / M_PI;
}

/**
 * @brief calculate steps to move given the current heading and the desired heading
 * @param current_heading current heading in degree
 * @param desired_heading desired heading in degree
 * @return steps to move
 */
int calculate_steps(float current_angle, float desired_angle)
{
    float angle_per_step = 360 / STEPS_PER_REV;
    float steps_to_move = (desired_angle - current_angle) / angle_per_step;
    return round(steps_to_move);
}

/**
 * @brief Move motors to home position
 *
 */
void go_home(PicoStepper *stepper1, PicoStepper *stepper2)
{
    printf("Compass: %.3f', Gyro: %.3f', rotating towards NORD at 45' tilt\n", compute_compass_degree(), compute_acc_degree());
    // yaw (stepper1) for X steps to get to compass headings=0°
    setSpeed(stepper1, 100);
    printf("MOTOR 1 STEPS: %d, MOTOR 2 STEPS %d \r\n", calculate_steps(compute_compass_degree(), 0), calculate_steps(compute_acc_degree(), 45));
    step(stepper1, calculate_steps(compute_compass_degree(), 0));
    // ypitch (stepper2) for X steps to get to acc headings=45°
    setSpeed(stepper2, 100);
    step(stepper2, calculate_steps(compute_acc_degree(), 45));
}

/**
 * @brief Move motors until they will be alligned to the sun position
 *
 */
void move_motors_to_the_sun(Position *pos, PicoStepper *stepper1, PicoStepper *stepper2)
{
    // Sun elevation must meet accelerometer
    int elevation_step = calculate_steps(compute_acc_degree(), pos->elevation);
    // Sun azimuth must meet compass
    int azimuth_step = calculate_steps(compute_compass_degree(), pos->azimuth);

    printf("Tracking on! \r\nElevation step to do: %d, Azimuth step to do: %d\r\n", elevation_step, azimuth_step);
    setSpeed(stepper1, 100);
    setSpeed(stepper2, 100);
    step(stepper1, azimuth_step);
    step(stepper2, elevation_step);
}

/**
 * @brief Parse nmea sentence and store the information into parsed_place
 * Actually this function is working just with RMC and GGA sentences,
 * which are the minimal sentences required to build a Place structure
 *
 * @param msg NMEA Sentence
 * @param parsed_place Place structure to store the information
 */
void nmea_parse(const char *msg, Place *parsed_place)
{
    uint8_t nmea_id = minmea_sentence_id(msg, false);
    struct minmea_sentence_rmc frame_rmc;
    struct minmea_sentence_gga frame_gga;
    switch (nmea_id)
    {

    case MINMEA_SENTENCE_RMC:
        minmea_parse_rmc(&frame_rmc, msg);
#ifdef NMEA_DEBUG
        printf("[20%d:%d:%d][20YY:MM:DD] \r\n", frame_rmc.date.year, frame_rmc.date.month, frame_rmc.date.day);
#endif
        parsed_place->year = 2000 + frame_rmc.date.year;
        parsed_place->month = frame_rmc.date.month;
        parsed_place->day = frame_rmc.date.day;
        break;

    case MINMEA_SENTENCE_GGA:
        minmea_parse_gga(&frame_gga, msg);
#ifdef NMEA_DEBUG
        printf("[%d:%d:%d][HH:MM:SS]\r\n", frame_gga.time.hours, frame_gga.time.minutes, frame_gga.time.seconds);
        printf("Computed latitude: %.2f, Computed longitude: %.2f \r\n", minmea_tocoord(&frame_gga.latitude), minmea_tocoord(&frame_gga.longitude));
#endif
        parsed_place->hour = frame_gga.time.hours;
        parsed_place->minute = frame_gga.time.minutes;
        parsed_place->second = (double)frame_gga.time.seconds;
        parsed_place->latitude = minmea_tocoord(&frame_gga.latitude);
        parsed_place->longitude = minmea_tocoord(&frame_gga.longitude);
        break;

    default:
        break;
    }
}
