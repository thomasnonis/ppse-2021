// SALMO Sun Tracker Algorithm source file
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

#include "sun_tracker.h"

#define PICO_SOURCE

#ifdef PICO_SOURCE
#include "pico/stdlib.h"
#include <stdio.h>
#else
#include <stdlib.h>
#include <stdio.h>
#endif

void print_place(Place *p)
{
    printf("\tPlace date [YYYY-MM-DD HH-MM-SS]: [%d-%d-%d %d-%d-%.f]\r\n\tLatitude: %f Longitude %f \r\n",
           p->year, p->month, p->day, p->hour, p->minute, p->second, p->latitude, p->longitude);
}

void compute_JD(int year, int month, int day, int hour, int minute, double second, struct Position *pos)
{

    // JD = int(365.25*(Y+4716))+int(30.6001*(M+1))+d+b-1524.5
    // JD Month costraint
    if (month <= 2)
    {
        year -= 1;
        month += 12;
    }
    // B parameter
    int b = (int)floor(year / 100.0);
    b = 2 - b + (int)floor(b / 4.0);
    double jdDay = day + hour / 24.0 + minute / 1440.0 + second / 86400.0;
    double jd = (int)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + jdDay + b - 1524.5;
    pos->jd = jd;
}

void compute_days_since_epoch(Position *pos)
{
    // it returns the n parameter used in the other calcs, like a fancy way to handle time
    pos->julian_days_since_epoch = pos->jd - 2451545.0;
}

void compute_centuries_since_epoch(Position *pos)
{
    pos->julian_centuries_since_epoch = (pos->julian_days_since_epoch) / 36525.0;
}

void compute_mean_longitude(Position *pos)
{
    // L parameter
    double mnlong = fmod(280.46646 + pos->julian_centuries_since_epoch * (36000.76983 + (pos->julian_centuries_since_epoch * 0.0003032)), 360.0);
    pos->mean_longitude = mnlong;
}

void compute_mean_anomaly(Position *pos)
{
    // g parameter
    double g = 357.52911 + pos->julian_centuries_since_epoch * (35999.05029 - (0.0001537 * pos->julian_centuries_since_epoch));
    pos->mean_anomaly = g;
}
// new fun
void compute_eccent_earth_orbit(Position *pos)
{
    pos->eccent_earth_orbit = 0.016708634 - (pos->julian_centuries_since_epoch * (0.000042037 + 0.0000001267 * pos->julian_centuries_since_epoch));
}

// new fun
void compute_sun_eq_center(Position *pos)
{
    pos->sun_eq_of_center = sin(DEG_TO_RAD * (pos->mean_anomaly)) * (1.914602 - pos->julian_centuries_since_epoch * (0.004817 + 0.000014 * pos->julian_centuries_since_epoch)) +
                            sin(DEG_TO_RAD * (2 * pos->mean_anomaly)) * (0.019993 - 0.000101 * pos->julian_centuries_since_epoch) + sin(DEG_TO_RAD * (3 * pos->mean_anomaly)) * 0.000289;
}

// new fun
void compute_sun_true_longitude(Position *pos)
{
    pos->sun_true_longitude = pos->mean_longitude + pos->sun_eq_of_center;
}

// new fun
void compute_sun_true_anomaly(Position *pos)
{
    pos->sun_true_anomaly = pos->mean_anomaly + pos->sun_eq_of_center;
}
// new fun
void compute_mean_obliquity_ecliptic(Position *pos)
{
    pos->mean_obliq_ecliptic = 23 + (26 + ((21.448 - pos->julian_centuries_since_epoch *
                                                         (46.815 + pos->julian_centuries_since_epoch * (0.00059 - pos->julian_centuries_since_epoch * 0.001813)))) /
                                              60) /
                                        60;
}

// new fun
void compute_obliq_corr(Position *pos)
{
    pos->obliq_corr = pos->mean_obliq_ecliptic + 0.00256 * cos(DEG_TO_RAD * (125.04 - 1934.136 * pos->julian_centuries_since_epoch));
}

// new fun
void compute_sun_app_longitude(Position *pos)
{
    pos->sun_app_long = pos->sun_true_longitude - 0.00569 - 0.00478 * sin(DEG_TO_RAD * (125.04 - 1934.136 * pos->julian_centuries_since_epoch));
}

// new fun (eq of time in mins)
void compute_eq_of_time(Position *pos)
{
    double y = tan(DEG_TO_RAD * (pos->obliq_corr / 2)) * tan(DEG_TO_RAD * (pos->obliq_corr / 2));
    // printf("! Y: %f \r\n",y);
    double eq = 4 * RAD_TO_DEG * (y * sin(2 * DEG_TO_RAD * (pos->mean_longitude)) - 2 * pos->eccent_earth_orbit * sin(DEG_TO_RAD * (pos->mean_anomaly)) + 4 * pos->eccent_earth_orbit * y * sin(DEG_TO_RAD * (pos->mean_anomaly)) * cos(2 * DEG_TO_RAD * (pos->mean_longitude)) - 0.5 * y * y * sin(4 * DEG_TO_RAD * (pos->mean_longitude)) - 1.25 * pos->eccent_earth_orbit * pos->eccent_earth_orbit * sin(2 * DEG_TO_RAD * (pos->mean_anomaly)));
    // printf("! eq of time %f \r\n", eq);
    pos->eq_of_time = eq;
}

void compute_right_ascension(Position *pos)
{
    // Note that atan2 args are swapped compared to the excel version
    double right_ascension = RAD_TO_DEG * (atan2((cos(DEG_TO_RAD * pos->obliq_corr)) * sin(DEG_TO_RAD * pos->sun_app_long), (cos(DEG_TO_RAD * (pos->sun_app_long)))));
    pos->right_ascension = right_ascension;
}

void compute_declination(Position *pos)
{
    pos->declination = asin(sin(pos->obliq_corr * DEG_TO_RAD) * sin(pos->sun_app_long * DEG_TO_RAD)) * RAD_TO_DEG;
}

/**
 * @brief Compute Greenwich mean sidereal time (gmst) [in hours]
 *
 * @param n
 * @param hour
 * @return double
 */
void compute_gmst(int hour, struct Position *pos)
{
    double gmst = 6.697375 + (0.0657098242 * (floor(pos->julian_days_since_epoch) + 0.5)) +
                  (1.00273790935 * hour) +
                  (0.000026 * floor(pow(pos->julian_centuries_since_epoch, 2)));
    // double GMST_hours = fmod(floor(gmst),24);
    // double GMST_minutes = (gmst - floor(gmst))*60;
    // double GMST_seconds = (GMST_minutes - floor(GMST_minutes))*60;
    // printf("GMST Time: %d:%d:%d\n",(int)GMST_hours,(int)GMST_minutes,(int)GMST_seconds);

    gmst = fmod(gmst, 24.0);
    pos->gmst = gmst; // gmst in hours
}

/**
 * @brief Compute longitude mean sidereal time (lmst) [in hours]
 *
 * @param gmst
 * @param east_longitude
 * @return double
 */
void compute_lmst(double east_longitude, struct Position *pos)
{
    // east longitude is usually expressed in degrees,
    // and should be divided by 15 to convert to hours
    // East longitude is considered positive, west negative.
    double lmst = pos->gmst + (east_longitude / 15.0);
    lmst = fmod(lmst, 24.0);

    // double LMST_hours = fmod(floor(lmst),24);
    // double LMST_minutes = (lmst - floor(lmst))*60;
    // double LMST_seconds = (LMST_minutes - floor(LMST_minutes))*60;
    // printf("LMST Time: %d:%d:%d\n",(int)LMST_hours,(int)LMST_minutes,(int)LMST_seconds);

    pos->lmst = lmst; // lmst in hours
}

/**
 * @brief Compute hour angle in radians between -pi and pi
 *
 * @param lmst
 * @param right_ascension
 * @return double
 */
void compute_hour_angle(Place *place, Position *pos)
{

    double utc_time = (place->hour + (place->minute / 60.0) + (place->second / 3600.0));
    double longv = 4 * (place->longitude);     //-(15*(place->time_zone)));
    double offset = (pos->eq_of_time) + longv; // correction in minutes
    double lsot = utc_time + (offset / 60.0);
    lsot = fmod(lsot, 24);
    // TODO:could not be useless, maybe we should try with different combinations of longitudes
    // if (lsot < 0)
    // {
    //     lsot += 24;
    // }

    double hour_angle = 15 * (lsot - 12);
    hour_angle = fmod(hour_angle, 360);

    pos->hour_angle = hour_angle;
}

void compute_elevation_and_azimuth(double lat, Position *pos)
{

    double elevation = 90 - (RAD_TO_DEG * (acos(sin(DEG_TO_RAD * (lat)) *
                                                    sin(DEG_TO_RAD * (pos->declination)) +
                                                cos(DEG_TO_RAD * (lat)) *
                                                    cos(DEG_TO_RAD * (pos->declination)) * cos(DEG_TO_RAD * (pos->hour_angle)))));
    double zenit = 90 - elevation;
    double az;
    if (pos->hour_angle > 0)
    {
        az = fmod(RAD_TO_DEG * (acos(((sin(DEG_TO_RAD * (lat)) * cos(DEG_TO_RAD * (zenit))) - sin(DEG_TO_RAD * (pos->declination))) / (cos(DEG_TO_RAD * (lat)) * sin(DEG_TO_RAD * (zenit))))) + 180, 360.0);
    }
    else
    {
        az = fmod(540 - RAD_TO_DEG * (acos(((sin(DEG_TO_RAD * (lat)) * cos(DEG_TO_RAD * (zenit))) - sin(DEG_TO_RAD * (pos->declination))) / (cos(DEG_TO_RAD * (lat)) * sin(DEG_TO_RAD * (zenit))))), 360.0);
    }
    pos->elevation = elevation;
    pos->azimuth = az;
}

Position compute_complete_position(Place *place)
{
    Position pos = {0};

    compute_JD(place->year, place->month, place->day, place->hour, place->minute, place->second, &pos);
    compute_days_since_epoch(&pos);
    compute_centuries_since_epoch(&pos);
    compute_mean_longitude(&pos);
    compute_mean_anomaly(&pos);
    compute_eccent_earth_orbit(&pos);
    compute_sun_eq_center(&pos);
    compute_sun_true_longitude(&pos);
    compute_sun_true_anomaly(&pos);
    compute_sun_app_longitude(&pos);
    compute_mean_obliquity_ecliptic(&pos);
    compute_obliq_corr(&pos);
    compute_right_ascension(&pos);
    compute_declination(&pos);
    compute_gmst(place->hour, &pos);
    compute_lmst(place->longitude, &pos);
    compute_eq_of_time(&pos);
    compute_hour_angle(place, &pos);
    compute_elevation_and_azimuth(place->latitude, &pos);
    // TODO: CHECK AZIMUT

#ifdef DBG_ALGO
    printf("\r\nDate %i/%i/%i %i-%i-%f UTC \r\n", place->year,
           place->month, place->day, place->hour, place->minute, place->second);
    printf("Latitude %f \r\n", place->latitude);
    printf("Longitude %f \r\n\r\n", place->longitude);
    printf("JD: %f \r\n", pos.jd);
    printf("Julian days since epoch (1.1.2000) %f \r\n", pos.julian_days_since_epoch);
    printf("Julian centuries since epoch (1.1.2000) %.8f \r\n", pos.julian_centuries_since_epoch);
    printf("Position Mean Longitude %f \r\n", pos.mean_longitude);
    printf("Position Mean Anomaly %f \r\n", pos.mean_anomaly);
    printf("Eccent earth orbit %f \r\n", pos.eccent_earth_orbit);
    printf("Sun eq center %f \r\n", pos.sun_eq_of_center);
    printf("Sun true longitude %f \r\n", pos.sun_true_longitude);
    printf("Sun true anomaly %f \r\n", pos.sun_true_anomaly);
    printf("Sun app longitude %f \r\n", pos.sun_app_long);
    printf("ok Mean Obliquity ecliptic %f \r\n", pos.mean_obliq_ecliptic);
    printf("Position Obliq correction %f \r\n", pos.obliq_corr);
    printf("ok Position Right Ascension %f \r\n", pos.right_ascension);
    printf("ok Position Declination %f \r\n", pos.declination);
    printf("ok GMST [Hours] %f \r\n", pos.gmst);
    printf("ok LMST [Hours] %f \r\n", pos.lmst);
    printf("Position hour angle %f \r\n", pos.hour_angle);
    printf("Position elevation %f azimuth %f \r\n", pos.elevation, pos.azimuth);
#endif
    return pos;
}

#ifndef PICO_SOURCE
int main(int argc, char *argv[])
{

    // year-month-day-time_zone-hour(utc)-minute-second-latitude-longitude
    if (argc < 2)
    {
        for (int i = 5; i < 25; i++)
        {
            printf("HOUR %i \r\n", i);
            Place place0 = {2030, 2, 28, i, 0, 0, 55.751244, 37.618423};
            compute_complete_position(&place0);
            printf("----------\r\n");
        }
    }
    else if (argc == 9)
    {
        printf("Computing...\r\n");
        Place place0 = {atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atof(argv[6]), atof(argv[7]), atof(argv[8])};
        compute_complete_position(&place0);
    }
    else
    {
        printf("ERROR: Not enough parameters \r\n");
    }

    return 0;
}
#endif