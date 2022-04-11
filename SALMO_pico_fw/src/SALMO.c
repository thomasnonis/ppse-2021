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
#include "../tracking-algorithm/sun_tracker.h"


int main() {
    stdio_init_all();

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