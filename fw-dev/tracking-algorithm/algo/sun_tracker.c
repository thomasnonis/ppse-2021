#include "sun_tracker.h"
#include <stdio.h>
void compute_JD(uint16_t year, uint8_t month, uint8_t day,  uint8_t hour, uint8_t minute, double second, struct Position* pos){
    
    // JD = int(365.25*(Y+4716))+int(30.6001*(M+1))+d+b-1524.5

    // JD Month costraint
    if(month <= 2){
        year -= 1;
        month += 12;
    }

    // B parameter
    uint8_t b = (uint8_t)floor(year/100.0);
    b = 2 - b + (uint8_t)floor(b/4.0);
  
    double jdDay = day + hour/24.0 + minute/1440.0 + second/86400.0;
    double jd = (int)(365.25*(year+4716)) + (int)(30.6001*(month)) + jdDay + b - 1524.5;

    pos->jd = jd;
}

void compute_ecliptic_coordinates(double jd, struct Position* pos){
    // it returns the n parameter used in the other calcs, like a fancy way to handle time
    pos->ecliptic_coordinates = jd - 2451545.0;
}

void compute_mean_longitude(double n,  struct Position* pos){
    // L parameter
    double mnlong = 280.460 + (0.9856474 * n);
    if(fmod(mnlong,360.0) <= 0){
        mnlong += 360;
    }
    pos->mean_longitude = mnlong;
}

void compute_mean_anomaly(double n,  struct Position* pos){
    // g parameter
    double g = 357.528 + (0.9856003 * n);
    if(fmod(g,360.0) <= 0){
        g += 360; 
    }
    pos->mean_anomaly = g * TO_RAD;
}

void compute_ecliptic_longitude(double g, double L,  struct Position* pos){
    double ec_long = L + (1.915 * sin(g)) + (0.020 * sin(2*g));
    if(fmod(ec_long,360.0) <= 0){
        ec_long += 360;
    }
    pos->ecliptic_longitude = ec_long * TO_RAD;
}

void compute_ecliptic_obliquity(double n,  struct Position* pos){
    pos->ecliptic_obliquity = 23.439 - (0.0000004 * n) * TO_RAD;
}

void compute_right_ascension(double ecl_obq, double ec_long,  struct Position* pos){
    double num = cos(ecl_obq) * sin(ec_long);
    double den = cos(ec_long);
    double right_ascension = atan(num/den);
    pos->right_ascension = right_ascension;
}

void compute_declination(double ecl_obq, double ec_long,  struct Position* pos){
    pos->declination = asin(sin(ecl_obq)*sin(ec_long));
}

/**
 * @brief Compute Greenwich mean sidereal time (gmst)
 * 
 * @param n 
 * @param hour 
 * @return double 
 */
void compute_gmst(double n, uint8_t hour,  struct Position* pos){
    // TODO: make a check on hour, is it hour(UT)?
    double gmst = 6.697375 + (0.0657098242 * n) + hour;
    if(fmod(gmst,2.0) <= 0){
        gmst += 24;
    }
    pos->gmst = gmst;
}
/**
 * @brief Compute longitude mean sidereal time
 * 
 * @param gmst 
 * @param east_longitude 
 * @return double 
 */
void compute_lmst(double gmst, double east_longitude,  struct Position* pos){
    // east longitude is usually expressed in degrees, 
    // and should be divided by 15 to convert to hours
    double lmst = gmst + (east_longitude/15.0);
    if(fmod(lmst, 0) <= 0){
        lmst += 24;
    }
    pos->lmst = lmst*15*TO_RAD;
}

/**
 * @brief Compute hour angle in radians between -pi and pi
 * 
 * @param lmst 
 * @param right_ascension 
 * @return double 
 */
void compute_hour_angle(double lmst, double right_ascension,  struct Position* pos){
    double ha = lmst - right_ascension;
    if(ha < - PI){
        ha += TWO_PI;
    }
    if(ha > PI){
        ha -= TWO_PI;
    }
    pos->hour_angle = ha;
}


void compute_elevation_and_azimuth(double lat, double declination, double hour_angle,  struct Position* pos){
    double elevation = asin( ( sin(declination)*sin(lat*TO_RAD) ) + ( cos(declination)*cos(lat*TO_RAD)*cos(hour_angle) ) );
    double az = asin( -cos(declination)*sin(hour_angle)/cos(elevation) );
    //If az == 90 it needs to be put between 0 and 2pi rads
    if(az == 90){
        double critical_elevation = asin(sin(declination)/sin(lat*TO_RAD));
        if(elevation >= critical_elevation){
            az = PI - az;
        }
        if(elevation < critical_elevation && hour_angle > 0){
            az = TWO_PI + az;
        }
    }

    pos->elevation = elevation;
    pos->azimuth   = az;
    
}

void compute_refraction(double elevation, struct Position* pos){
    elevation /= TO_RAD; 
    double refraction;
    if(elevation > -0.56){
        refraction = 3.51561*(0.1594+0.0196*elevation/0.00002*elevation*elevation)/
        (1+ (0.505*elevation+0.0845*elevation*elevation));
    }else{
        refraction = 0.56;
    }
    pos->refraction = refraction;
}

int main(){

    Position pos = {0};
    uint16_t year = 2000;
    uint8_t month = 1;
    uint8_t day = 1;
    uint8_t hour = 10;
    uint8_t minute = 0;
    double second = 0;

    double latitude = 0;
    double longitude = 0;

    compute_JD(year,month,day,hour,minute,second,&pos); //ok
    printf("Position JD: %f \r\n", pos.jd);
    compute_ecliptic_coordinates(pos.jd, &pos); 
    printf("Position Ecliptic coords %f \r\n", pos.ecliptic_coordinates);
    compute_mean_longitude(pos.ecliptic_coordinates, &pos);
    printf("Position Mean Longitude %f \r\n", pos.mean_longitude);
    compute_mean_anomaly(pos.ecliptic_coordinates, &pos);
    printf("Position Mean Anomaly %f \r\n", pos.mean_anomaly);
    compute_ecliptic_longitude(pos.mean_anomaly, pos.mean_longitude, &pos);
    printf("Position Mean Longitude %f \r\n", pos.mean_longitude);
    compute_ecliptic_obliquity(pos.ecliptic_coordinates,&pos);
    printf("Position Ecliptic Obliquiti %f \r\n", pos.ecliptic_obliquity);
    compute_right_ascension(pos.ecliptic_obliquity, pos.ecliptic_longitude, &pos);
    printf("Position Right Ascension %f \r\n", pos.right_ascension);
    compute_declination(pos.ecliptic_obliquity, pos.ecliptic_longitude, &pos);
    printf("Position Declination %f \r\n", pos.declination);
    compute_gmst(pos.ecliptic_coordinates, hour, &pos);
    printf("Position gmst %f \r\n", pos.gmst);
    compute_lmst(pos.gmst, pos.mean_longitude, &pos); //??
    printf("Position lmst %f \r\n", pos.lmst);
    compute_hour_angle(pos.lmst, pos.right_ascension, &pos);
    printf("Position right ascension %f \r\n", pos.right_ascension);
    compute_elevation_and_azimuth(latitude, pos.declination, pos.hour_angle, &pos);
    printf("Position elevation %f azimuth %f \r\n", pos.elevation, pos.azimuth);
    compute_refraction(pos.elevation, &pos);
    printf("Position refraction %f \r\n", pos.refraction);

    return 0;
}