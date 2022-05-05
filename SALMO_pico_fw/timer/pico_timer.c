#include "pico_timer.h"

volatile bool timer_fired = false;
struct repeating_timer timer;

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    printf("Timer %d fired!\n", (int) id);
    timer_fired = true;
    // Can return a value here in us to fire in the future
    return 0;
}
/**
 * @brief Callback function for the timer, repeteating every t.delay_us*1000
 * 
 * @param t Timer structure
 * @return true 
 */
bool repeating_timer_callback(struct repeating_timer *t) {
    printf("Repeat at %lld\n", time_us_64());
    return true;
}

/**
 * @brief Initialize the timer
 * @param time_ms Delay in ms
 */
void initialize_pico_timer(int time_ms){
    add_repeating_timer_ms(time_ms, repeating_timer_callback, NULL, &timer);
}

// bool cancelled = cancel_repeating_timer(&timer);
