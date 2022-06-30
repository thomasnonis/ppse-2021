// SALMO Sun Tracker Algorithm header file
// 11/04/2022 by Tollardo Simone, Tommaso Canova, Lisa Santarossa, Gabriele Berretta, Thomas Nonis

/* ============================================
SALMO Sun Tracker Algorithm code is placed under the MIT license
Copyright (c) 2022 Tollardo Simone, Tommaso Canova, Lisa Santarossa, Gabriele Berretta, Thomas Nonis

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef __SUN_TRACKER_H
#define __SUN_TRACKER_H


#include <math.h>

// Constants:
#define PI             3.14159265358979323846    // Pi
#define TWO_PI         6.28318530717958647693    // 2 pi
#define MPI            3.14159265358979323846e6  // One Megapi...
#define R2D            57.2957795130823208768    // Radians to degrees conversion factor
#define R2H            3.81971863420548805845    // Radians to hours conversion factor
#define DEG_TO_RAD     PI/180.0                  // Degree to radians
#define RAD_TO_DEG     180.0/PI

/**
 * @brief Complete struct where mid calcs of the sun tracker algorithm are stored
 * 
 */
typedef struct Position{
  double jd, julian_days_since_epoch, julian_centuries_since_epoch;
  double mean_longitude, mean_anomaly;
  double eccent_earth_orbit, sun_eq_of_center;
  double sun_true_longitude, sun_true_anomaly;
  double sun_rad_vector, sun_app_long;
  double obliq_corr, mean_obliq_ecliptic;
  double right_ascension, declination;
  double gmst, lmst, eq_of_time, hour_angle, elevation, azimuth, refraction;
} Position;

/**
 * @brief Structur needed to store the Place information obtained by the gps
 * Note that the hour is in UTC time, so if the local time is 14:00:00 and you're are in UTC+2 zone,
 * the UTC time will be considered as 12:00:00.
 * Most of GPS provide datetime in utc
 * 
 */
typedef struct Place{
  int year, month, day, hour, minute;
  double second, latitude, longitude;
} Place;

void print_place(Place *p);
void compute_JD(int year, int month, int day,  int hour, int minute, double second, struct Position* pos);

//eliptic coords
void compute_days_since_epoch(struct Position* pos);
void compute_mean_longitude(struct Position* pos);
void compute_mean_anomaly(struct Position* pos);
void compute_eccent_earth_orbit(struct Position* pos);
void compute_sun_eq_center(struct Position* pos);
void compute_sun_true_longitude(struct Position* pos);
void compute_sun_true_anomaly(struct Position* pos);
void compute_mean_obliquity_ecliptic(struct Position* pos);
void compute_obliq_corr(struct Position* pos);
void compute_sun_app_longitude(struct Position* pos);
void compute_eq_of_time(struct Position* pos);

//celestial coords
void compute_right_ascension(struct Position* pos);
void compute_declination(Position* pos);

//local coords
void compute_gmst(int hour,  struct Position* pos);
void compute_lmst(double east_longitude,  struct Position* pos);
void compute_hour_angle(Place* place, struct Position* pos);
void compute_elevation_and_azimuth(double lat,  struct Position* pos);

Position compute_complete_position(Place* place);
#endif