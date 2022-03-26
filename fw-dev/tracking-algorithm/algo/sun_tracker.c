#include "sun_tracker.h"
#include <stdio.h>

void compute_JD(int year, int month, int day,  int hour, int minute, double second, struct Position* pos){
    
    // JD = int(365.25*(Y+4716))+int(30.6001*(M+1))+d+b-1524.5
    // JD Month costraint
    if(month <= 2){
        year -= 1;
        month += 12;
    }
    // B parameter
    int b = (int)floor(year/100.0);
    b = 2 - b + (int)floor(b/4.0);
    double jdDay = day + hour/24.0 + minute/1440.0 + second/86400.0;
    double jd = (int)(365.25*(year+4716)) + (int)(30.6001*(month + 1)) + jdDay + b - 1524.5;
    pos->jd = jd;
}

void compute_days_since_epoch(Position* pos){
    // it returns the n parameter used in the other calcs, like a fancy way to handle time
    pos->julian_days_since_epoch = pos->jd - 2451545.0;
}

void compute_centuries_since_epoch(Position* pos){
    pos->julian_centuries_since_epoch = (pos->julian_days_since_epoch)/36525.0;
}

void compute_mean_longitude(Position* pos){
    // L parameter
    double mnlong = fmod(280.46646+pos->julian_centuries_since_epoch*(36000.76983+(pos->julian_centuries_since_epoch*0.0003032)),360.0);
    pos->mean_longitude = mnlong;
}

void compute_mean_anomaly(Position* pos){
    // g parameter
    double g = 357.52911+pos->julian_centuries_since_epoch*(35999.05029- (0.0001537*pos->julian_centuries_since_epoch));
    pos->mean_anomaly = g;
}
// new fun
void compute_eccent_earth_orbit(Position* pos){
    pos->eccent_earth_orbit = 0.016708634-(pos->julian_centuries_since_epoch*(0.000042037+0.0000001267*pos->julian_centuries_since_epoch));
}

// new fun
void compute_sun_eq_center(Position* pos){
    pos->sun_eq_of_center = sin(DEG_TO_RAD*(pos->mean_anomaly))*(1.914602-pos->julian_centuries_since_epoch*(0.004817+0.000014*pos->julian_centuries_since_epoch))+
    sin(DEG_TO_RAD*(2*pos->mean_anomaly))*(0.019993-0.000101*pos->julian_centuries_since_epoch)+sin(DEG_TO_RAD*(3*pos->mean_anomaly))*0.000289;
}

// new fun
void compute_sun_true_longitude(Position* pos){
    pos->sun_true_longitude = pos->mean_longitude+pos->sun_eq_of_center;
}

// new fun
void compute_sun_true_anomaly(Position* pos){
    pos->sun_true_anomaly = pos->mean_anomaly+pos->sun_eq_of_center;
}
// new fun
void compute_mean_obliquity_ecliptic(Position* pos){
    pos->mean_obliq_ecliptic = 23+(26+((21.448-pos->julian_centuries_since_epoch*
    (46.815+pos->julian_centuries_since_epoch*(0.00059-pos->julian_centuries_since_epoch*0.001813))))/60)/60;
}

//new fun
void compute_obliq_corr(Position* pos){
    pos->obliq_corr = pos->mean_obliq_ecliptic+0.00256*cos(DEG_TO_RAD*(125.04-1934.136*pos->julian_centuries_since_epoch));
}

// new fun
void compute_sun_app_longitude(Position* pos){
    pos->sun_app_long = pos->sun_true_longitude-0.00569-0.00478*sin(DEG_TO_RAD*(125.04-1934.136*pos->julian_centuries_since_epoch));
}

//new fun (eq of time in mins)
void compute_eq_of_time(Position* pos){
    double y = tan(DEG_TO_RAD*(pos->obliq_corr/2))*tan(DEG_TO_RAD*(pos->obliq_corr/2));
    //printf("! Y: %f \r\n",y);
    double eq = 4*RAD_TO_DEG*(y*sin(2*DEG_TO_RAD*(pos->mean_longitude))
    -2*pos->eccent_earth_orbit*sin(DEG_TO_RAD*(pos->mean_anomaly))+4*pos->eccent_earth_orbit
    *y*sin(DEG_TO_RAD*(pos->mean_anomaly))
    *cos(2*DEG_TO_RAD*(pos->mean_longitude))-0.5*y*y*
    sin(4*DEG_TO_RAD*(pos->mean_longitude))-1.25*pos->eccent_earth_orbit*pos->eccent_earth_orbit*sin
    (2*DEG_TO_RAD*(pos->mean_anomaly)));
    //printf("! eq of time %f \r\n", eq);
    pos->eq_of_time = eq;
}

void compute_right_ascension(Position* pos){
    //Note that atan2 args are swapped compared to the excel version
    double right_ascension = RAD_TO_DEG*(atan2((cos(DEG_TO_RAD*pos->obliq_corr))*sin(DEG_TO_RAD*pos->sun_app_long)
    ,(cos(DEG_TO_RAD*(pos->sun_app_long)))));
    pos->right_ascension = right_ascension;
}

void compute_declination(Position* pos){
    pos->declination = asin(sin(pos->obliq_corr*DEG_TO_RAD)*sin(pos->sun_app_long*DEG_TO_RAD))*RAD_TO_DEG;
}

/**
 * @brief Compute Greenwich mean sidereal time (gmst) [in hours]
 * 
 * @param n 
 * @param hour 
 * @return double 
 */
void compute_gmst(int hour,  struct Position* pos){
    double gmst = 6.697375 + (0.0657098242 * (floor(pos->julian_days_since_epoch)+0.5)) + 
                            (1.00273790935*hour) + 
                            (0.000026 * floor(pow(pos->julian_centuries_since_epoch,2)));
    double GMST_hours = fmod(floor(gmst),24);
    double GMST_minutes = (gmst - floor(gmst))*60;
    double GMST_seconds = (GMST_minutes - floor(GMST_minutes))*60;
    printf("GMST Time: %d:%d:%d\n",(int)GMST_hours,(int)GMST_minutes,(int)GMST_seconds);

    gmst = fmod(gmst,24.0);
    pos->gmst = gmst; //gmst in hours
}

/**
 * @brief Compute longitude mean sidereal time (lmst) [in hours]
 * 
 * @param gmst 
 * @param east_longitude 
 * @return double 
 */
void compute_lmst(double east_longitude,  struct Position* pos){
    // east longitude is usually expressed in degrees, 
    // and should be divided by 15 to convert to hours
    // East longitude is considered positive, west negative.
    double lmst = pos->gmst + (east_longitude/15.0);
    lmst = fmod(lmst, 24.0);

    double LMST_hours = fmod(floor(lmst),24);
    double LMST_minutes = (lmst - floor(lmst))*60;
    double LMST_seconds = (LMST_minutes - floor(LMST_minutes))*60;
    printf("LMST Time: %d:%d:%d\n",(int)LMST_hours,(int)LMST_minutes,(int)LMST_seconds);

    pos->lmst = lmst; //lmst in hours
}

/**
 * @brief Compute hour angle in radians between -pi and pi
 * 
 * @param lmst 
 * @param right_ascension 
 * @return double 
 */
void compute_hour_angle(Place* place,Position* pos){

    double local_time = (place->hour+place->time_zone)+(place->minute/60)+(place->second/3600);
    double longv=4*(place->longitude-(15*(place->time_zone)));
    double offset=(pos->eq_of_time)+longv;  //correction in minutes
    double lsot=local_time+(offset/60);
    lsot=fmod(lsot,24);
    //TODO:could not be useless, maybe we should try with different combinations of longitudes
    //if(lsot<0){
    //    lsot+=24;
    //}

    double hour_angle=15*(lsot-12);
    hour_angle=fmod(hour_angle,360);

    pos->hour_angle = hour_angle;

}


void compute_elevation_and_azimuth(double lat, Position* pos){

    double elevation = 90 - (RAD_TO_DEG*(acos(sin(DEG_TO_RAD*(lat))*
    sin(DEG_TO_RAD*(pos->declination))+cos(DEG_TO_RAD*(lat))*
    cos(DEG_TO_RAD*(pos->declination))*cos(DEG_TO_RAD*(pos->hour_angle))))); 
    double zenit = 90 - elevation;
    double az;
    if(pos->hour_angle>0){
        az = fmod(RAD_TO_DEG*(acos(((sin(DEG_TO_RAD*(lat))*cos(DEG_TO_RAD*(zenit)))-sin(DEG_TO_RAD*(pos->declination)))
        /(cos(DEG_TO_RAD*(lat))*sin(DEG_TO_RAD*(zenit)))))+180,360.0);
    }else{
        az = fmod(540-RAD_TO_DEG*(acos(((sin(DEG_TO_RAD*(lat))*cos(DEG_TO_RAD*(zenit)))-sin(DEG_TO_RAD*(pos->declination)))
        /(cos(DEG_TO_RAD*(lat))*sin(DEG_TO_RAD*(zenit))))),360.0);
    }
    pos->elevation = elevation;
    pos->azimuth   = az;
    
}

void compute_complete_position(Place* place){
    Position pos = {0};

    printf("\r\nDate %i/%i/%i %i-%i-%f UTC \r\n", place->year,
    place->month, place->day, place->hour, place->minute, place->second);
    printf("Latitude %f \r\n", place->latitude);
    printf("Longitude %f \r\n\r\n", place->longitude);
    compute_JD(place->year,place->month,place->day,place->hour,place->minute,place->second,&pos);
    printf("JD: %f \r\n", pos.jd);
    compute_days_since_epoch(&pos); 
    printf("Julian days since epoch (1.1.2000) %f \r\n", pos.julian_days_since_epoch);
    compute_centuries_since_epoch(&pos);
    printf("Julian centuries since epoch (1.1.2000) %.8f \r\n", pos.julian_centuries_since_epoch);
    compute_mean_longitude(&pos);
    printf("Position Mean Longitude %f \r\n", pos.mean_longitude);
    compute_mean_anomaly(&pos);
    printf("Position Mean Anomaly %f \r\n", pos.mean_anomaly);
    compute_eccent_earth_orbit(&pos);
    printf("Eccent earth orbit %f \r\n", pos.eccent_earth_orbit);
    compute_sun_eq_center(&pos);
    printf("Sun eq center %f \r\n", pos.sun_eq_of_center);
    compute_sun_true_longitude(&pos);
    printf("Sun true longitude %f \r\n", pos.sun_true_longitude);
    compute_sun_true_anomaly(&pos);
    printf("Sun true anomaly %f \r\n", pos.sun_true_anomaly);
    compute_sun_app_longitude(&pos);
    printf("Sun app longitude %f \r\n", pos.sun_app_long);
    compute_mean_obliquity_ecliptic(&pos);
    printf("ok Mean Obliquity ecliptic %f \r\n", pos.mean_obliq_ecliptic);
    compute_obliq_corr(&pos);
    printf("Position Obliq correction %f \r\n", pos.obliq_corr);
    compute_right_ascension(&pos);
    printf("ok Position Right Ascension %f \r\n", pos.right_ascension);
    compute_declination(&pos);
    printf("ok Position Declination %f \r\n", pos.declination);
    compute_gmst(place->hour, &pos);
    printf("ok GMST [Hours] %f \r\n", pos.gmst);
    compute_lmst(place->longitude, &pos);
    printf("ok LMST [Hours] %f \r\n", pos.lmst);
    compute_eq_of_time(&pos);
    compute_hour_angle(place,&pos);
    printf("Position hour angle %f \r\n", pos.hour_angle);
    compute_elevation_and_azimuth(place->latitude, &pos);
    //TODO: CHECK AZIMUT
    printf("Position elevation %f azimuth %f \r\n", pos.elevation, pos.azimuth);
}



int main(){

    //year-month-day-time_zone-hour(utc)-minute-second-latitude-longitude
    for(int i = 1; i < 25; i++){
        printf("\r\nHOUR %d \r\n", i);
        Place place0 = {2030,2,28,3,i,0,0,55.751244, 37.618423};
        compute_complete_position(&place0);
        printf("------------------------\r\n");
    }
    
   
    // int year = 2030;
    // int month = 2;
    // int day = 28;
    // int hour = 9;   //UTC
    // int minute = 0;
    // double second = 0;

    // double latitude = 55.751244;
    // double longitude = 37.618423;

    

    return 0;
}