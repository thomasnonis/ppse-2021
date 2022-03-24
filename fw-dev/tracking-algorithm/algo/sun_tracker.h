#ifndef __SUN_TRACKER_H
#define __SUN_TRACKER_H

#include <stdio.h>
#include <math.h>

// Constants:
#define PI             3.14159265358979323846    // Pi
#define TWO_PI         6.28318530717958647693    // 2 pi
#define MPI            3.14159265358979323846e6  // One Megapi...
#define R2D            57.2957795130823208768    // Radians to degrees conversion factor
#define R2H            3.81971863420548805845    // Radians to hours conversion factor
#define DEG_TO_RAD         PI/180.0                  // Degree to radians
#define RAD_TO_DEG     180.0/PI
typedef struct Position{
  double jd, T, ecliptic_coordinates;
  double mean_longitude, mean_anomaly;
  double ecliptic_longitude, ecliptic_obliquity;
  double right_ascension, declination;
  double gmst, lmst, hour_angle, elevation, azimuth, refraction;
} Position;

// /// @brief Date and time to compute the Sun's position for, in UT
// struct Time {
//   int year,month,day,  hour,minute;
//   double second;
// };

// /// @brief Location to compute the Sun's position for
// struct Location {
//   double longitude, latitude;
//   double sinLat, cosLat;
//   double pressure, temperature;
// };


// /// @brief Position of the Sun
// struct Position {
//   double julianDay, tJD, tJC,tJC2;
//   double longitude, distance;
//   double obliquity,cosObliquity, nutationLon;
//   double rightAscension,declination, hourAngle,agst;
//   double altitude, altitudeRefract,azimuthRefract;
//   double hourAngleRefract, declinationRefract;
// };

/// @brief Rise and set data for the Sun
struct RiseSet {
  double riseTime, transitTime, setTime;
  double riseAzimuth, transitAltitude, setAzimuth;
};


void compute_JD(int year, int month, int day,  int hour, int minute, double second, struct Position* pos);

//eliptic coords
void compute_ecliptic_time(double jd, struct Position* pos);
void compute_mean_longitude(double n,  struct Position* pos);
void compute_mean_anomaly(double n,  struct Position* pos);
void compute_ecliptic_longitude(double g, double L,  struct Position* pos);
void compute_ecliptic_obliquity(double n,  struct Position* pos);

//celestial coords
void compute_right_ascension(double ecl_obq, double ec_long,  struct Position* pos);
void compute_declination(double ecl_obq, double ec_long,  struct Position* pos);

//local coords
void compute_gmst(double n, int hour,  struct Position* pos);
void compute_lmst(double gmst, double east_longitude,  struct Position* pos);
void compute_hour_angle(double lmst, double right_ascension,  struct Position* pos);
void compute_elevation_and_azimuth(double lat, double declination, double hour_angle,  struct Position* pos);
void compute_refraction(double elevation,  struct Position* pos);
void compute_mappazzone(double jd, struct Position* pos, double latitude, double longitude);
void compute_noaa(int year, int month, int day,  int hour, int minute, double second, double latitude, double longitude);
#endif