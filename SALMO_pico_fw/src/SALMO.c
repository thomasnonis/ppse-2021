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
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "../tracking-algorithm/sun_tracker.h"
#include "../MPU6050/MPU6050.h"
#include "../timer/pico_timer.h"
#include "../nmea-parser/minmea.h"
#include "../HMC5883L/HMC5883L.h"
#include "../stepper/pico_stepper.h"

// UART CONFIG
#define GPS_UART_ID uart1
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

// GPS CONFIG
#define DEBUG_GPS_PARSER
#define GPS_MAX_SENTENCES 12
#define GPS_MAX_SENTENCE_LENGTH 100
char gps_raw_sentences[GPS_MAX_SENTENCES][GPS_MAX_SENTENCE_LENGTH];
int gps_lines = 0;

// TIMER CONFIG
bool update_position;
bool read_gps;
struct repeating_timer update_position_timer;
struct repeating_timer gps_read_timer;

//  GPS INTERRUPT CONFIG
#ifdef INTERRUPTS_ARE_WORKING
static int chars_rxed = 0;
char nmea_buffer[83];
#endif

// GLOBAL VARS
Place gps_parsed_place;
PicoStepper stepper1={0};
PicoStepper stepper2={0};

/**
 * @brief compute compass headings in degree
 * @return compass heading in degree
 */
float compute_compass_degree(){
    int16_t HMCx;
    int16_t HMCy;
    int16_t HMCz;
    HMC5883L_getHeading(&HMCx, &HMCy, &HMCz);

    //Assuming Z axis is pointing up, Y axis is pointing straight out of the panel and X axis is pointing to the right
    float compass_radians = atan2(HMCx, HMCy);  //note: atan2(y,x) = tan^-1 (arg1/arg2) and auto matches the quadrant
    return compass_radians*180/M_PI;
}

/**
 * @brief compute accelerometer headings in degree
 * @return accelerometer heading in degree
 */
float compute_acc_degree(){
    int16_t AccGyro[6];
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;

    MPU6050_GetRawAccelGyro(AccGyro);
    AccX = AccGyro[0];
    AccY = AccGyro[1];
    AccZ = AccGyro[2];

    // assuming Z axis is pointing up when the panel is flat
    float tilt_radians = atan2(sqrt(pow(AccX, 2) + pow(AccY, 2)), AccZ);
    return tilt_radians*180/M_PI;
}

/**
 * @brief calculate steps to move given the current heading and the desired heading
 * @param current_heading current heading in degree
 * @param desired_heading desired heading in degree
 * @return steps to move
 */
int calculate_steps(float current_angle, float desired_angle){
    float angle_per_step = 360/STEPS_PER_REV;
    float steps_to_move = (desired_angle - current_angle)/angle_per_step;
    return round(steps_to_move);
}

/**
 * @brief gpio irq callback
 *
 * @param gpio gpio number
 * @param events irq event
 */
void gpio_irq_callback(uint gpio, uint32_t events) {
    if(gpio==NOT_HOME_SW){
        printf("Home button pressed\n");
        printf("Compass: %.3f', Gyro: %.3f', rotating towards NORD at 45' tilt", compute_compass_degree(), compute_acc_degree());
        //yaw (stepper1) for X steps to get to compass headings=0°
        setSpeed(&stepper1, 100);
        stepMotor(&stepper1, calculate_steps(compute_compass_degree(), 0));
        //ypitch (stepper2) for X steps to get to acc headings=45°
        setSpeed(&stepper2, 100);
        stepMotor(&stepper2, calculate_steps(compute_acc_degree(), 45));
    }
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
    printf("Time: %lld\n", time_us_64());
    update_position = true;
    return update_position;
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
void init_gps_timer(int time_ms)
{
    add_repeating_timer_ms(time_ms, gps_read_callback, NULL, &gps_read_timer);
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
        parsed_place->second = (double) frame_gga.time.seconds;
        parsed_place->latitude = minmea_tocoord(&frame_gga.latitude);
        parsed_place->longitude = minmea_tocoord(&frame_gga.longitude);
        break;

    default:
        break;
    }
}

#ifdef INTERRUPTS_ARE_WORKING
// RX interrupt handler
void on_uart_rx()
{
    if (chars_rxed > 82)
    {
        // printf(nmea_buffer);
        memset(nmea_buffer, 0, sizeof(nmea_buffer)); // empty the string
    }
    while (uart_is_readable(GPS_UART_ID))
    {
        // printf("HELLO\r\n");
        uint8_t ch = uart_getc(GPS_UART_ID);

        nmea_buffer[chars_rxed] = ch;

        // // Can we send it back?
        // if (uart_is_writable(GPS_UART_ID)) {
        //     // Change it slightly first!
        //     ch++;
        //     uart_putc(GPS_UART_ID, ch);
        // }
        chars_rxed++;
    }
}
#endif

void hello_salmo()
{
    printf(
        "███████  █████  ██      ███    ███  ██████  \r\n"
        "██      ██   ██ ██      ████  ████ ██    ██ \r\n"
        "███████ ███████ ██      ██ ████ ██ ██    ██ \r\n"
        "     ██ ██   ██ ██      ██  ██  ██ ██    ██ \r\n"
        "███████ ██   ██ ███████ ██      ██  ██████  \r\n");
}

int main()
{
    /* Initialization */

    // stdio_init_all(); equals to usb and uart init
    stdio_usb_init();  // initialize usb cdc
    stdio_uart_init(); // initialize uart0 with 0 baud rate

    // GPS UART initialization
    uart_init(GPS_UART_ID, BAUD_RATE);
    gpio_set_function(UART1_TX, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(GPS_UART_ID, false, false);
    uart_set_format(GPS_UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(GPS_UART_ID, false);
#ifdef INTERRUPTS_ARE_WORKING
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(GPS_UART_ID, false, false);
#endif
    gpio_init(GPS_EN);
    gpio_set_dir(GPS_EN, GPIO_OUT);
    gpio_put(GPS_EN, 0); // with zero it's enabled

    update_position = false;


    // Stepper motors initialization
    picoStepperInit(&stepper1, M1_W11, M1_W12, M1_W21, M1_W22, STEPS_PER_REV, INITIAL_SPEED);   //stepper1 = yaw
    picoStepperInit(&stepper2, M2_W11, M2_W12, M2_W21, M2_W22, STEPS_PER_REV, INITIAL_SPEED);   //stepper2 = pitch


    // SET I2C Pins
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C1_SDA, I2C1_SCL, GPIO_FUNC_I2C));
    i2c_init(I2C_PERIPHERAL, I2C_PORT1_BAUD_RATE);

    /* Verify if are needed, probably they are already in the sensor breakout board
        gpio_pull_up(I2C1_SDA);
        gpio_pull_up(I2C1_SCL);
    */


    int16_t acc_gyro_data = 0;
    // Init i2c port1 with defined baud rate
    // I2C MPU port is defined inside every .h device library
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
    init_position_timer(4000);
    init_gps_timer(4000);

#ifdef GPS_SIMULATIOR
    // Sample place and its respective conversion into sun position
    Place manual_place = {2030, 2, 28, 10, 10, 10, 55.751244, 37.618423};
    Position manual_position = compute_complete_position(&manual_place);
#endif

    Position sun_position;

#define FAMOUS_CITIES_SIMULATION
#ifdef FAMOUS_CITIES_SIMULATION
    Place rome = {2022, 6, 5, 10, 0, 20, 41.9027835, 12.4963655};
    Position rome_position = compute_complete_position(&rome);

    // TODO: check why minutes are not mapped correctly
    Place berlin = {2022, 6, 5, 12, 40, 00, 52.520008, 13.404954};
    Position berlin_position = compute_complete_position(&berlin);

    Place rio = {2022, 6, 5, 18, 00, 00, -22.908333, -43.196388};
    Position rio_position = compute_complete_position(&rio);

#endif
    while (true)
    {
        if (read_gps)
        {
            gps_lines = 0;
            char buff[100];
            memset(buff, 0, sizeof(buff));
            while (gps_lines < GPS_MAX_SENTENCES)
            {
                while (uart_is_readable(GPS_UART_ID))
                {
                    char ch = uart_getc(GPS_UART_ID);
                    // printf("%c", ch);
                    strncat(buff, &ch, 1);
                    if (ch == '\n')
                    {
                        strncat(buff, '\0', 1);
                        strcpy(gps_raw_sentences[gps_lines], buff);
                        memset(buff, 0, sizeof(buff));
                        gps_lines++;
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
            hello_salmo();
            printf("-------\r\n");
            #ifdef FAMOUS_CITIES_SIMULATION
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

            // TODO: check accelerometer and move motors
            sun_position = compute_complete_position(&gps_parsed_place);
            printf("Sun elevation %f azimuth %f \r\n\r\n", sun_position.elevation, sun_position.azimuth);
            printf("Moving motors............damn so heavy\r\n");
            update_position = false;
            printf("-------\r\n");
        }
        sleep_ms(200);
    }

    //gpio irq handler - NOTE: Currently the GPIO parameter is ignored, and this callback will be called for any enabled GPIO IRQ on any pin.
    gpio_set_irq_enabled_with_callback(1, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_callback);
    
    return 0;
}