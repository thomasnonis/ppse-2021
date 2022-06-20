/*
 * pico_stepper.cpp - Stepper library for Raspberry Pi Pico - Version 0.1
 * 
 * Copyright (C) Lisa Santarossa, Tommaso Canova, Thomas Nonis, Gabriele Berretta, Simone Tollardo; Salmo Society TM
 * Copyright (C) Beshr Kayali
 *
 * Based on Arduino Stepper Library
 * Copyright (C) Arduino LLC. Copyright (C) Sebastian Gassner. Copyright (c) Noah Shibley.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

// #include <cstdlib>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico_stepper.h"


void picoStepperInit(PicoStepper* stepper, int pin1, int pin2, int pin3, int pin4, int total_steps, long initial_speed) {
  stepper->total_steps = total_steps;
  stepper->initial_speed = initial_speed;
  stepper->dir = 0;
  stepper->delay = 60L * 1000L * 1000L / stepper->total_steps / stepper->initial_speed;
  stepper->current_step = 0;
  stepper->last_step_us_time = 0;
  stepper->pin1 = pin1;
  stepper->pin2 = pin2;
  stepper->pin3 = pin3;
  stepper->pin4 = pin4;
  gpio_init(stepper->pin1);
  gpio_init(stepper->pin2);
  gpio_init(stepper->pin3);
  gpio_init(stepper->pin4);
  gpio_set_dir(stepper->pin1, GPIO_OUT);
  gpio_set_dir(stepper->pin2, GPIO_OUT);
  gpio_set_dir(stepper->pin3, GPIO_OUT);
  gpio_set_dir(stepper->pin4, GPIO_OUT);
}

void setSpeed(PicoStepper* stepper, long speed) {
  stepper->delay = 60L * 1000L * 1000L / stepper->total_steps / speed;
}

void step(PicoStepper* stepper, int steps_to_move) {
  int steps_left = abs(steps_to_move);

  if (steps_to_move > 0) {
    stepper->dir = 1;
  } else {
    stepper->dir = 0;
  }

  while (steps_left > 0) {
    uint64_t now = to_us_since_boot(get_absolute_time());

    if (now - stepper->last_step_us_time >= stepper->delay) {
      stepper->last_step_us_time = now;

      if (stepper->dir == 1) {
        stepper->current_step++;

        if (stepper->current_step == stepper->total_steps) {
          stepper->current_step = 0;
        }
      } else {
        if (stepper->current_step == 0) {
          stepper->current_step = stepper->total_steps;
        }

        stepper->current_step--;
      }

      stepMotor(stepper, stepper->current_step % 4);
      steps_left--;
    }
  }
}

void stepMotor(PicoStepper* stepper, int step) {
  switch (step) {
  case 0:  // 1000
    gpio_put(stepper->pin1, 1);
    gpio_put(stepper->pin2, 0);
    gpio_put(stepper->pin3, 0);
    gpio_put(stepper->pin4, 0);
    break;

  case 1:  // 0100
    gpio_put(stepper->pin1, 0);
    gpio_put(stepper->pin2, 1);
    gpio_put(stepper->pin3, 0);
    gpio_put(stepper->pin4, 0);
    break;

  case 2:  // 0010
    gpio_put(stepper->pin1, 0);
    gpio_put(stepper->pin2, 0);
    gpio_put(stepper->pin3, 1);
    gpio_put(stepper->pin4, 0);
    break;

  case 3:  // 0001
    gpio_put(stepper->pin1, 0);
    gpio_put(stepper->pin2, 0);
    gpio_put(stepper->pin3, 0);
    gpio_put(stepper->pin4, 1);
    break;
  }
}

