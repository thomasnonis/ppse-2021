/**
 * @file SALMO.h
 * @author SALMO Society
 * @brief Salmo configuration file
 * @version 0.1
 * @date 2022-04-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#define BUZZ_EN 0
#define GPIO1 1
#define GPIO2 2
#define GPIO3 3
#define GPIO4 4
#define NOT_EN_SW 5
#define NOT_HOME_SW 6
#define LED_G 7
#define LED_R 8
#define LED_B 9
#define M2_W11 10
#define M2_W12 11
#define M2_W21 12
#define M2_W22 13
#define M1_W11 14
#define M1_W12 15
#define M1_W21 16
#define M1_W22 17
#define I2C1_SDA 18
#define I2C1_SCL 19
#define CMP_DRDY 20
#define ACC_INT 21
#define GPS_EN 22 // When GPS is enabled, the GPS_EN pin is pulled low
#define GPS_TP 23
#define UART1_TX 24
#define UART1_RX 25
#define V_PANEL 26
#define I_PANEL 27
#define EXT_ADC0 28
#define EXT_ADC1 29

#define I2C_PERIPHERAL (i2c1)
#define I2C_PORT1_BAUD_RATE 400 * 1000

#define STEPS_PER_REV 200
#define INITIAL_SPEED 50
