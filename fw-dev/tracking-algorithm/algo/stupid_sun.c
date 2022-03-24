/* cc sun.c -lm -o sun */
/* altitude and azimuth of Sun *now* on a Unix-like OS 
   method based on page C24 in a recent Astronomical Almanac 
   or the Explanatory Suppliment */

#include <stdio.h>
#include <time.h>
#include <math.h>

/* Your latitude and (east) longitude */
#define LONG 11.11929
#define LAT 46.07005

/* Some constants */
#define J2000 946728000 /* J2000.0 in seconds since Unix epoch */    
#define RADS 0.0174532925199433
#define DEGS 57.2957795130823
#define PI 3.1415927

double range(double y);

int main(void) {

time_t now;
double d, t, L, g, lambda;
double epsilon, y, x, alpha, delta;
double lst, ha, alt, az;

/*  Get Unix time in seconds UT then find
    days and Julian centuries since J2000.0 */

    now = time(NULL);
    if (now == -1) {
        puts("The time() function failed");
    }
    d = 2451769.916667;//(now - J2000)/(86400.0); /* days since J2000.0 */
    /* d = -877.04167; test from Meeus */
    L = range(280.461 + 0.9856474 * d);
    g = range(357.528 + 0.9856003 * d);
    lambda = range(L + 1.915 * sin(g*RADS) + 0.020 * sin(2*g*RADS));
    epsilon = 23.439 - 0.0000004 * d;
    y = cos(epsilon*RADS) * sin(lambda*RADS);
    x = cos(lambda*RADS);
    alpha = atan2(y , x);
    delta = asin(sin(epsilon*RADS)*sin(lambda*RADS));
    lst = range(280.46061837 + 360.98564736629 * d + LONG);
    ha = range(lst - alpha*DEGS);
    alt = asin(sin(delta) * sin(LAT*RADS) + cos(delta) * cos(LAT*RADS) * cos(ha*RADS));
    printf(" alt: %3.0f\n", alt*DEGS);
    y = -cos(delta) * cos(LAT*RADS) * sin(ha*RADS);
    x = sin(delta) - sin(LAT*RADS) * sin(alt);
    az = range(atan2(y, x)*DEGS);
    printf("  az: %3.0f\n", az);
}

/* returns an angle in degrees in the range 0 to 360 */
double
range(double x) {
        double a, b;
        b = x / 360;
    a = 360 * (b - floor(b));
    if (a < 0)
                a = 360 + a;
    return(a);
        }