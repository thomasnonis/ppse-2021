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
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "../tracking-algorithm/sun_tracker.h"
#include "../MPU6050/MPU6050.h"
#include "../MPU6050/MPU6050_BSP.h"

//TODO:pin mapping alt functions

#define I2C_PORT1_BAUD_RATE 400*1000

int main() {
    stdio_init_all();

    //SET I2C Pins for MPU6050
    gpio_set_function(18, GPIO_FUNC_I2C);
    gpio_set_function(19, GPIO_FUNC_I2C);
    gpio_pull_up(18);
    gpio_pull_up(19);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(18, 19, GPIO_FUNC_I2C));

    uint16_t acc_gyro_data=0;
    //Init i2c port1 with defined baud rate
    i2c_init(i2c1, I2C_PORT1_BAUD_RATE);
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

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    Place place0 = {2030,2,28,10,0,0,55.751244, 37.618423};
    gpio_put(LED_PIN, 1);

    while (true) {
        printf("SALMO!\r\n");

        Position p = compute_complete_position(&place0);
        printf("[PICO] Position elevation %f azimuth %f \r\n", p.elevation, p.azimuth);
        sleep_ms(1000);
        printf("----\r\n");
    }
    return 0;
}