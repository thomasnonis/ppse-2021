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
#include "../tracking-algorithm/sun_tracker.h"
#include "../MPU6050/MPU6050.h"
#include "../MPU6050/MPU6050_I2C.h"
#include "../timer/pico_timer.h"
#include "../nmea-parser/lwgps.h"
#include "../HMC5883L/HMC5883L.h"

bool update_position;
struct repeating_timer update_position_timer;
/* GPS handle */
lwgps_t hgps;

#define TEST_HMC
#define I2C_ENABLE

/* Dummy gps data */
const char
gps_rx_data[] = ""
                "$GPRMC,183729,A,3907.356,N,12102.482,W,000.0,360.0,080301,015.5,E*6F\r\n"
                "$GPRMB,A,,,,,,,,,,,,V*71\r\n"
                "$GPGGA,183730,3907.356,N,12102.482,W,1,05,1.6,646.4,M,-24.1,M,,*75\r\n"
                "$GPGSA,A,3,02,,,07,,09,24,26,,,,,1.6,1.6,1.0*3D\r\n"
                "$GPGSV,2,1,08,02,43,088,38,04,42,145,00,05,11,291,00,07,60,043,35*71\r\n"
                "$GPGSV,2,2,08,08,02,145,00,09,46,303,47,24,16,178,32,26,18,231,43*77\r\n"
                "$PGRME,22.0,M,52.9,M,51.0,M*14\r\n"
                "$GPGLL,3907.360,N,12102.481,W,183730,A*33\r\n"
                "$PGRMZ,2062,f,3*2D\r\n"
                "$PGRMM,WGS84*06\r\n"
                "$GPBOD,,T,,M,,*47\r\n"
                "$GPRTE,1,1,c,0*07\r\n"
                "$GPRMC,183731,A,3907.482,N,12102.436,W,000.0,360.0,080301,015.5,E*67\r\n"
                "$GPRMB,A,,,,,,,,,,,,V*71\r\n";

// gps_rx_data2[] = ""
//                 "$GNRMC,141148.00,A,4604.15257,N,01108.97976,E,0.004,,051121,,,A,V*13\r\n"
//                 "$GNVTG,,T,,M,0.004,N,0.008,K,A*31\r\n"
//                 "$GNGGA,141148.00,4604.15257,N,01108.97976,E,1,12,0.59,372.1,M,46.1,M,,*4C\r\n"
//                 "$GNGSA,A,3,05,07,13,14,15,20,30,,,,,,0.97,0.59,0.77,1*03\r\n"
//                 "$GNGSA,A,3,78,69,68,70,80,79,,,,,,,0.97,0.59,0.77,2*0F\r\n"
//                 "$GNGSA,A,3,05,03,31,09,24,01,15,,,,,,0.97,0.59,0.77,3*0F\r\n"
//                 "$GNGSA,A,3,26,24,10,14,21,,,,,,,,0.97,0.59,0.77,4*03\r\n"
//                 "$GPGSV,3,1,12,02,02,225,,05,57,277,44,07,40,058,36,11,04,215,,1*6C\r\n"
//                 "$GPGSV,3,2,12,13,46,294,45,14,42,150,40,15,16,295,43,18,08,327,39,1*6C\r\n"
//                 "$GPGSV,3,3,12,20,53,216,44,27,03,024,,28,,,43,30,73,057,43,1*59\r\n"
//                 "$GPGSV,3,1,11,02,02,225,,05,57,277,39,07,40,058,35,11,04,215,,6*61\r\n"
//                 "$GPGSV,3,2,11,13,46,294,,14,42,150,45,15,16,295,35,18,08,327,31,6*65\r\n"
//                 "$GPGSV,3,3,11,20,53,216,,27,03,024,,30,73,057,41,6*52\r\n"
//                 "$GLGSV,2,1,08,68,43,125,45,69,77,354,42,70,17,317,33,78,46,040,39,1*71\r\n"
//                 "$GLGSV,2,2,08,79,74,182,41,80,16,209,38,85,06,301,10,86,09,345,23,1*70\r\n"
//                 "$GLGSV,2,1,08,68,43,125,43,69,77,354,41,70,17,317,,78,46,040,35,3*7A\r\n"
//                 "$GLGSV,2,2,08,79,74,182,41,80,16,209,21,85,06,301,11,86,09,345,,3*7A\r\n"
//                 "$GAGSV,3,1,09,01,10,036,28,03,11,297,43,04,09,114,,05,63,305,44,7*70\r\n"
//                 "$GAGSV,3,2,09,09,61,106,44,15,12,312,41,24,49,164,40,25,05,198,36,7*7B\r\n"
//                 "$GAGSV,3,3,09,31,53,075,36,7*49\r\n"
//                 "$GAGSV,3,1,09,01,10,036,32,03,11,297,38,04,09,114,,05,63,305,47,2*71\r\n"
//                 "$GAGSV,3,2,09,09,61,106,46,15,12,312,38,24,49,164,46,25,05,198,31,2*73\r\n"
//                 "$GAGSV,3,3,09,31,53,075,42,2*4F\r\n"
//                 "$GBGSV,2,1,05,10,33,067,35,14,79,260,42,21,16,140,24,24,55,208,46,1*76\r\n" ;

/**
 * @brief Set update position flag every t.delay_us
 * @param t Pointer to the timer structure
 */
bool update_position_callback(struct repeating_timer *t) {
    printf("Time: %lld\n", time_us_64());
    update_position = true;
    return update_position;
}
void init_timer(int time_ms){
    add_repeating_timer_ms(time_ms, update_position_callback, NULL, &update_position_timer);
}


int main() {
    stdio_init_all();
    update_position = false;

    #ifdef I2C_ENABLE
    //SET I2C Pins
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C1_SDA, I2C1_SCL, GPIO_FUNC_I2C));
    i2c_init(I2C_PERIPHERAL, I2C_PORT1_BAUD_RATE);
    #endif

    #ifdef MPU_MOUNTED

    /* Verify if are needed, probably they are already in the sensor breakout board
        gpio_pull_up(I2C1_SDA);
        gpio_pull_up(I2C1_SCL);
    */

    int16_t acc_gyro_data=0;
    //Init i2c port1 with defined baud rate
    // I2C MPU port is defined inside every .h device library
    MPU6050_Initialize();

    if(MPU6050_TestConnection())
    {
        printf("MPU6050 connection success\n");
    }
    else
    {
        printf("MPU6050 connection failed\n");
    }
    MPU6050_GetRawAccelGyro(&acc_gyro_data);

    #endif

    #ifdef TEST_HMC

    HMC5883L_initialize();
    if(isHMC())
    {
        printf("HMC5883L connection success\n");
    }
    else
    {
        printf("HMC5883L connection failed\n");
    }
    int16_t x;
    int16_t y;
    int16_t z;
    while(1){
        HMC5883L_getHeading(&x,&y,&z);
        printf("x=%d y=%d z=%d\n",x,y,z);
    }
    
    #endif
    
    //initialize_pico_timer(2000);

    /* Init GPS */
    lwgps_init(&hgps);
    /* Process all input data */
    lwgps_process(&hgps, gps_rx_data, strlen(gps_rx_data));
    /* Init timer with 2 sec delay*/
    init_timer(2000);
    Place manual_place = {2030,2,28,10,0,0,55.751244, 37.618423};
    Place parsed_place = {hgps.year,hgps.month,hgps.date,hgps.hours,hgps.minutes,hgps.seconds,hgps.latitude, hgps.longitude};
    print_place(&parsed_place);
    Position p = compute_complete_position(&manual_place);
    Position parsed_pos = compute_complete_position(&parsed_place);
    //gpio_put(LED_PIN, 1);

    
    // prof preferisce modalita timer libero, get_ms > delay
    // AA55 -> comb 
    while (true) {
        
        if(update_position){
            print_place(&manual_place);
            p = compute_complete_position(&manual_place);
            printf("[MANUAL] Position elevation %f azimuth %f \r\n\r\n", p.elevation, p.azimuth);
            // print_place(&parsed_place);
            // parsed_pos = compute_complete_position(&parsed_place);
            // printf("[PARSED GPS] Position elevation %f azimuth %f \r\n\r\n", parsed_pos.elevation, parsed_pos.azimuth);
            if(manual_place.hour+1>23){
                manual_place.hour=0;
                manual_place.day++;
            }
            else{
                manual_place.hour++;
            };
            update_position = false;
        }
        sleep_ms(200);
        
        //printf("\r\n");
    }
    return 0;
}