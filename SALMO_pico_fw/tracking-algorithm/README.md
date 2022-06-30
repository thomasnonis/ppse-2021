# :globe_with_meridians: : Compilation routine for the algo

Sun tracker algorithm to convert Date, Time, Longitude and Latitude into a Sun Position (typically used with elevation and azimuth parameters).

## Compile and execution

:godmode: For real optimization wizards

    gcc sun_tracker.c -O2 -lm -o out 
    ./out

:hatched_chick: For normal people

    gcc sun_tracker.c -lm  
    ./a.out

:zap: Usage with args

    ./out {year} {month} {day} {hour} {minute} {second} {latitude} {longitude}