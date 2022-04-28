#include <stdio.h>
#include "pico/stdlib.h"

int64_t alarm_callback(alarm_id_t id, void *user_data);

bool repeating_timer_callback(struct repeating_timer *t);

void initialize_pico_timer(int time_ms);

// bool cancelled = cancel_repeating_timer(&timer);
