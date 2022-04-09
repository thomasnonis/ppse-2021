#ifndef __SUN_TRACKER_H
#define __SUN_TRACKER_H

#include <stdio.h>
#include <math.h>

#define PICO_SOURCE

#ifdef PICO_SOURCE
  #include "pico/stdlib.h"
  #pragma "printf lib: pico/stdlib.h"
#else
    #include <stdlib.h> 
    #pragma "printf lib: <stdlib.h>"
#endif

//TODO: add input params to struct pos
// Constants:
#define PI             3.14159265358979323846    // Pi
#define TWO_PI         6.28318530717958647693    // 2 pi
#define MPI            3.14159265358979323846e6  // One Megapi...
#define R2D            57.2957795130823208768    // Radians to degrees conversion factor
#define R2H            3.81971863420548805845    // Radians to hours conversion factor
#define DEG_TO_RAD     PI/180.0                  // Degree to radians
#define RAD_TO_DEG     180.0/PI

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

typedef struct Place{
  int year, month, day, hour, minute;
  double second, latitude, longitude;
} Place;

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

void compute_complete_position(Place* place);
#endif