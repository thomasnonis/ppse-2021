/*
 * pico_stepper.h - Stepper library for Raspberry Pi Pico - Version 0.1
 *
 * Copyright (C) Beshr Kayali
 * Copyright (C) Lisa Santarossa, Tommaso Canova, Thomas Nonis, Gabriele Berretta, Simone Tollardo
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

#pragma once

typedef struct {
  int pin1;
  int pin2;
  int pin3;
  int pin4;
  int total_steps;
  long initial_speed;
  int dir;
  unsigned long delay;
  int current_step;
  unsigned long last_step_us_time;
}PicoStepper;

void picoStepperInit(PicoStepper* stepper, int pin1, int pin2, int pin3, int pin4, int total_steps, long initial_speed);

void setSpeed(PicoStepper* stepper, long speed);

void step(PicoStepper* stepper, int total_steps);

void stepMotor(PicoStepper* stepper, int step);