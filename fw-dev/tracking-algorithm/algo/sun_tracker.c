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
    long double T = (pos->jd-2451545.0)/36525.0;
    pos->T = T;
}

void compute_ecliptic_time(double jd, struct Position* pos){
    // it returns the n parameter used in the other calcs, like a fancy way to handle time
    pos->ecliptic_coordinates = jd - 2451545.0;
}

void compute_mean_longitude(double n,  struct Position* pos){
    // L parameter
    double mnlong = 280.460 + (0.9856474 * n);
    mnlong = fmod(mnlong,360.0);
    if( mnlong < 0){
        mnlong += 360;
    }
    pos->mean_longitude = mnlong;

    // alternative method
    long double T = (pos->jd-2451545.0)/36525.0; //It's in century

    printf("T: %.8Lf \r\n", T);

    double k = 2*PI/360.0;
    double M = 357.52910 + 35999.05030*T - 0.0001559*T*T - 0.00000048*T*T*T;
    double Lo = 280.46645 + 36000.76983*T + 0.0003032*T*T ;
    //printf("mean longitude altra maniera %f \r\n", Lo * DEG_TO_RAD);
    // end of alternative method
}

void compute_mean_anomaly(double n,  struct Position* pos){
    // g parameter
    double g = 357.52911+pos->T*(35999.05029- (0.0001537*pos->T));
    pos->mean_anomaly = g;
}

void compute_ecliptic_longitude(double g, double L,  struct Position* pos){
    double ec_long = L + (1.915 * sin(g)) + (0.020 * sin(2*g));
    // ec_long = fmod(ec_long,360.0);
    // if( ec_long < 0){
    //     ec_long += 360;
    // }
    pos->ecliptic_longitude = ec_long ;//* DEG_TO_RAD;
}

void compute_ecliptic_obliquity(double n,  struct Position* pos){
    pos->ecliptic_obliquity = 23.439 - (0.0000004 * n) * DEG_TO_RAD;
}

void compute_right_ascension(double ecl_obq, double ec_long,  struct Position* pos){
    double den = cos(ecl_obq*DEG_TO_RAD) * sin(ec_long*DEG_TO_RAD);
    double num = cos(ec_long*DEG_TO_RAD);
    double right_ascension = atan2(den,num)*RAD_TO_DEG;
    pos->right_ascension = right_ascension;
}

void compute_declination(double ecl_obq, double ec_long,  struct Position* pos){
    pos->declination = asin(sin(ecl_obq*DEG_TO_RAD)*sin(ec_long*DEG_TO_RAD))*RAD_TO_DEG;
}

/**
 * @brief Compute Greenwich mean sidereal time (gmst) [in hour]
 * 
 * @param n 
 * @param hour 
 * @return double 
 */
void compute_gmst(double n, int hour,  struct Position* pos){
    // TODO: make a check on hour, is it hour(UT)?
    double gmst = 6.697375 + (0.0657098242 * n) + hour;
    gmst = fmod(gmst,24.0);
    if(gmst < 0){
        gmst += 24;
    }
    pos->gmst = gmst;
}
/**
 * @brief Compute longitude mean sidereal time [in hour]
 * 
 * @param gmst 
 * @param east_longitude 
 * @return double 
 */
void compute_lmst(double gmst, double east_longitude,  struct Position* pos){
    // east longitude is usually expressed in degrees, 
    // and should be divided by 15 to convert to hours
    // East longitude is considered positive.
    double lmst = gmst + (east_longitude/15.0);
    lmst = fmod(lmst, 24.0);
    if(lmst < 0){
        lmst += 24;
    }
    pos->lmst = lmst;//*15*DEG_TO_RAD;
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
    double elevation = asin( ( sin(declination)*sin(lat*DEG_TO_RAD) ) + ( cos(declination)*cos(lat*DEG_TO_RAD)*cos(hour_angle) ) );
    double az = asin( -cos(declination)*sin(hour_angle)/cos(elevation) );
    //If az == 90 it needs to be put between 0 and 2pi rads
    if(az == 90){
        double critical_elevation = asin(sin(declination)/sin(lat*DEG_TO_RAD));
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
    elevation /= DEG_TO_RAD; 
    double refraction;
    if(elevation > -0.56){
        refraction = 3.51561*(0.1594+0.0196*elevation/0.00002*elevation*elevation)/
        (1+ (0.505*elevation+0.0845*elevation*elevation));
    }else{
        refraction = 0.56;
    }
    pos->refraction = refraction;
}

void compute_mappazzone(double jd, struct Position* pos, double latitude, double longitude){
    double T = (jd-2451545.0)/36525.0; //it's in century
    printf("T: %.8f \r\n", T);
    double k = 2*PI/360.0;
    double M = 357.52910 + 35999.05029*T - 0.0001537*T*T; //- 0.00000048*T*T*T;
    M = fmod(M, 360);
    if(M < 360) M = M+360;
    printf("M: %.8f \r\n", M);

    double Lo = 280.46645 + 36000.76983*T + 0.0003032*T*T ;
    Lo = fmod(Lo, 360);
    if(Lo < 360) Lo = Lo+360;
    printf("Lo: %.8f \r\n", Lo);
    double DL = (1.914600 - 0.004817*T - 0.000014*T*T)*sin(k*M)
+ (0.019993 - 0.000101*T)*sin(k*2*M) + 0.000290*sin(k*3*M);
    double L = Lo+DL;
    double eps = 23.0 + 26.0/60.0 + 21.448/3600.0 - (46.8150*T + 0.00059*T*T - 0.001813*T*T*T)/3600.0;
    double X = cos(L);
    double Y = cos(eps)*sin(L);
    double Z = sin(eps)*sin(L);
    double R = sqrt(1.0-Z*Z);

    double delta = atan(Z/R); // in degrees
    double RA = (24/180)*atan(Y/(X+R)) ;// in hours
    double theta0 = 280.46061837 + 360.98564736629*(jd-2451545.0) + 0.000387933*T*T - T*T*T/38710000.0;

    double tau = theta0 + 10 - RA;
    double h = asin(sin(latitude)*sin(delta) + cos(latitude)*cos(delta)*cos(tau));
    double az = atan2(- sin(tau) , (cos(latitude)*tan(delta) - sin(latitude)*cos(tau)));
    //hour angle tau, delta declination, azimuth az, altitude h
    printf("GMST %f \r\n",theta0);
    printf("AZ %f H %f \r\n",az, h);

}   
int yisleap(int year)
{
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

int get_yday(int mon, int day, int year)
{
    static const int days[2][13] = {
        {0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
        {0, 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}
    };
    int leap = yisleap(year);

    return days[leap][mon] + day;
}

void compute_noaa(int year, int month, int day,  int hour, int minute, double second, double latitude, double longitude){
    double den = 365.0;
    if(year % 4 == 0 && year % 100 == 0 && year % 400 == 0) den = 366.0;
    int day_of_the_year = get_yday(month, day, year);
    double y = (2*PI/den)*(day_of_the_year - 1 + ((hour-12)/24));
    double eqtime = 229.18*(0.000075 + 0.001868*cos(y) - 0.032077*sin(y) - 0.014615*cos(2*y) -0.040849*sin(2*y) );
    double decl = 0.006918 - 0.399912*cos(y) + 0.070257*sin(y) - 0.006758*cos(2*y) + 0.000907*sin(2*y) - 0.002697*cos(3*y) + 0.00148*sin (3*y);
    double time_offset = eqtime + 4*longitude ; //-60*timezone
    double tst = hour*60+minute+second/60; //+timeoffset
    double ha = (tst/4)-180;
    double solar_zenit = sin(latitude)*sin(decl) + cos(latitude)*cos(decl)*cos(ha);
    double az = 180 - acos( - (((sin(latitude)*solar_zenit)-sin(decl))/(cos(latitude)*sin(acos(solar_zenit)))) );
    double haa = acos( (cos(90.833)/(cos(latitude)*cos(decl)))-(tan(latitude)*tan(decl)) );
    printf("NOAA az %f h %f", az, haa);


}
long JulianDate(int year, int month, int day) {
	long JD_whole;
	int A, B;
	if (month <= 2) {
		year--;
		month += 12;
	}
	A = year / 100;
	B = 2 - A + A / 4;
	JD_whole = (long) (365.25 * (year + 4716)) + (int) (30.6001 * (month + 1))
			+ day + B - 1524;
	return JD_whole;
}
void calculateSolarPosition(int year, int month, int day,  int hour, int minute, double second, float Latitude,
		float Longitude, struct Position* pos) {

	const float DAYS_PER_JULIAN_CENTURY = 36525.0;
	const long Y2K_JULIAN_DAY = 2451545;

	static float latPrevious;
	static float lonPrevious;

	long JD_whole;
	long JDx;
	float JD_frac;
	float rightAscension;
	float Declination;
	float hourAngle;
	float GreenwichHourAngle;
	float elapsedT;
	float solarLongitude;
	float solarMeanAnomaly;
	float earthOrbitEccentricity;
	float sunCenter;
	float solarTrueLongitude;
	float solarTrueAnomaly;
	float equatorObliquity;

	// if (tParam != timePrevious or Latitude != latPrevious
	// 		or Longitude != lonPrevious) // only calculate if time or location has changed
	// 				{
	// 	breakTime(tParam, timeCandidate);
		JD_whole = JulianDate(year, month, day);
		JD_frac = (hour + minute / 60.0
				+ second / 3600.0) / 24.0 - 0.5;
		printf("JD_whole: %ld \r\n",(JD_whole));
		printf("JD_frac:  %f \r\n",(JD_frac));
		elapsedT = JD_whole - Y2K_JULIAN_DAY;
		elapsedT = (elapsedT + JD_frac) / DAYS_PER_JULIAN_CENTURY;
        printf("ELAPSED T:  %f \r\n",(elapsedT));
		solarLongitude = DEG_TO_RAD
				* fmod(280.46645 + 36000.76983 * elapsedT, 360);
		solarMeanAnomaly = DEG_TO_RAD
				* fmod(357.5291 + 35999.0503 * elapsedT, 360);
		earthOrbitEccentricity = 0.016708617 - 0.000042037 * elapsedT;

		sunCenter = DEG_TO_RAD
				* ((1.9146 - 0.004847 * elapsedT) * sin(solarMeanAnomaly)
						+ (0.019993 - 0.000101 * elapsedT)
								* sin(2 * solarMeanAnomaly)
						+ 0.00029 * sin(3 * solarMeanAnomaly));

		solarTrueAnomaly = solarMeanAnomaly + sunCenter;
		equatorObliquity = DEG_TO_RAD
				* (23 + 26 / 60. + 21.448 / 3600. - 46.815 / 3600 * elapsedT);

		JDx = JD_whole - Y2K_JULIAN_DAY;
        JDx = get_yday(month,day,year);
		printf("JDX_frac: %ld \r\n",(JDx));
		GreenwichHourAngle = 280.46061837 + (360 * JDx) % 360
				+ .98564736629 * JDx + 360.98564736629 * JD_frac;
		GreenwichHourAngle = fmod(GreenwichHourAngle, 360.0);
        printf("GW hour angle: %f \r\n",(GreenwichHourAngle));
		solarTrueLongitude = fmod(sunCenter + solarLongitude, TWO_PI);

		rightAscension = atan2(sin(solarTrueLongitude) * cos(equatorObliquity),
				cos(solarTrueLongitude));

		Declination = asin(sin(equatorObliquity) * sin(solarTrueLongitude));
		hourAngle = DEG_TO_RAD * GreenwichHourAngle + Longitude
				- rightAscension;

		// results:
		// result.distance = 1.000001018
		// 		* (1 - earthOrbitEccentricity * earthOrbitEccentricity)
		// 		/ (1 + earthOrbitEccentricity * cos(solarTrueAnomaly));

		// elevation from the horizon
		pos->elevation = asin(
				sin(Latitude) * sin(Declination)
						+ cos(Latitude) * (cos(Declination) * cos(hourAngle)));

		// Azimuth measured eastward from north.
		pos->azimuth = PI
				+ atan2(sin(hourAngle),
						cos(hourAngle) * sin(Latitude)
								- tan(Declination) * cos(Latitude));
		
		// copy the time
		//result.time = tParam;

		// remember the parameters
		//timePrevious = tParam;
		// latPrevious = Latitude;
		// lonPrevious = Longitude;
	//}
}



int main(){

    Position pos = {0};
    int year = 2000;
    int month = 8;
    int day = 19;
    int hour = 9;
    int minute = 0;
    double second = 0;

    double latitude = 46.07005;
    double longitude = 11.11929;

    printf("\r\nDate %i/%i/%i %i-%i-%f \r\n", year, month, day, hour, minute, second);
    printf("Latitude %f \r\n", latitude);
    printf("Longitude %f \r\n\r\n", longitude);
    compute_JD(year,month,day,hour,minute,second,&pos); //ok
    printf("! Position JD: %f \r\n", pos.jd);
    compute_ecliptic_time(pos.jd, &pos); 
    printf("Position Ecliptic coords %f \r\n", pos.ecliptic_coordinates);
    compute_mean_longitude(pos.ecliptic_coordinates, &pos);
    printf("!Position Mean Longitude %f \r\n", pos.mean_longitude);
    compute_mean_anomaly(pos.ecliptic_coordinates, &pos);
    printf("!Position Mean Anomaly %f \r\n", pos.mean_anomaly);
    compute_ecliptic_longitude(pos.mean_anomaly, pos.mean_longitude, &pos);
    printf("[1 grado off]Position Ecliptic Longitude %f \r\n", pos.ecliptic_longitude);
    compute_ecliptic_obliquity(pos.ecliptic_coordinates,&pos);
    printf("! Position Ecliptic Obliquity %f \r\n", pos.ecliptic_obliquity);
    compute_right_ascension(pos.ecliptic_obliquity, pos.ecliptic_longitude, &pos);
    printf("[2 gradi off dovuto da ecliptic] Position Right Ascension %f \r\n", pos.right_ascension);
    compute_declination(pos.ecliptic_obliquity, pos.ecliptic_longitude, &pos);
    printf("Position Declination %f \r\n", pos.declination);
    compute_gmst(pos.ecliptic_coordinates, hour, &pos);
    printf("! Position gmst %f \r\n", pos.gmst);
    compute_lmst(pos.gmst, longitude, &pos); //??
    printf("!Position lmst %f \r\n", pos.lmst);
    compute_hour_angle(pos.lmst, pos.right_ascension, &pos);
    printf("[1 grado off]Position right ascension %f \r\n", pos.right_ascension);
    compute_elevation_and_azimuth(latitude, pos.declination, pos.hour_angle, &pos);
    printf("Position elevation %f azimuth %f \r\n", pos.elevation*RAD_TO_DEG, pos.azimuth)*RAD_TO_DEG;
    compute_refraction(pos.elevation, &pos);
    printf("Position refraction %f \r\n\r\n", pos.refraction);


        double elapsedT = pos.jd - 2451545;
        double JD_frac = (hour + minute / 60.0
				+ second / 3600.0) / 24.0 - 0.5;
		elapsedT = (elapsedT + JD_frac) / 36525.0;

		double solarLongitude = DEG_TO_RAD
				* fmod(280.46645 + 36000.76983 * elapsedT, 360);
		double solarMeanAnomaly = DEG_TO_RAD
				* fmod(357.5291 + 35999.0503 * elapsedT, 360);
		double earthOrbitEccentricity = 0.016708617 - 0.000042037 * elapsedT;

		double sunCenter = DEG_TO_RAD
				* ((1.9146 - 0.004847 * elapsedT) * sin(solarMeanAnomaly)
						+ (0.019993 - 0.000101 * elapsedT)
								* sin(2 * solarMeanAnomaly)
						+ 0.00029 * sin(3 * solarMeanAnomaly));

		double solarTrueAnomaly = solarMeanAnomaly + sunCenter;
		double equatorObliquity = DEG_TO_RAD
				* (23 + 26 / 60. + 21.448 / 3600. - 46.815 / 3600 * elapsedT);

		long JDx = (int)pos.jd - 36525.0;

		double GreenwichHourAngle = 280.46061837 + (360 * JDx) % 360
				+ .98564736629 * JDx + 360.98564736629 * JD_frac;
		GreenwichHourAngle = fmod(GreenwichHourAngle, 360.0);

		double solarTrueLongitude = fmod(sunCenter + solarLongitude, TWO_PI);

		double rightAscension = atan2(sin(solarTrueLongitude) * cos(equatorObliquity),
				cos(solarTrueLongitude));

		double Declination = asin(sin(equatorObliquity) * sin(solarTrueLongitude));
		double hourAngle = DEG_TO_RAD * GreenwichHourAngle + longitude
				- rightAscension;

		// results:
		double distance = 1.000001018
				* (1 - earthOrbitEccentricity * earthOrbitEccentricity)
				/ (1 + earthOrbitEccentricity * cos(solarTrueAnomaly));

		// elevation from the horizon
		double elevation = asin(
				sin(latitude) * sin(Declination)
						+ cos(latitude) * (cos(Declination) * cos(hourAngle)));

		// Azimuth measured eastward from north.
		double azimuth = PI
				+ atan2(sin(hourAngle),
						cos(hourAngle) * sin(latitude)
								- tan(Declination) * cos(latitude));

    // printf("\r\n!!! Position elevation %f azimuth %f \r\n", elevation*RAD_TO_DEG, azimuth*RAD_TO_DEG);
    
    // printf("MAPPAZZONE \r\n");
    // compute_mappazzone(pos.jd, &pos, latitude, longitude);

    //compute_noaa(year, month, day, hour, minute, second, latitude, longitude);

    // calculateSolarPosition(year, month, day, hour, minute, second, latitude, longitude, &pos);

    // printf("\r\n### Position elevation %f azimuth %f \r\n", pos.elevation*RAD_TO_DEG, pos.azimuth*RAD_TO_DEG);



    // printf("MAPPAZZONE LIBRO \r\n");
    // compute_mappazzone(2448908.5, &pos, latitude, longitude);

    return 0;
}