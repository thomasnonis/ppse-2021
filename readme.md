# :boom: Compilation routine for the algo

    cd fw-dev
    cd tracking-algotithm
    cd algo

:godmode: For real optimization wizards

    gcc sun_tracker.c -O2 -lm -o out 
    ./out

:hatched_chick: For normal people

    gcc sun_tracker.c -lm  
    ./a.out

:zap: Usage with args

    ./out {year} {month} {day} {hour} {minute} {second} {latitude} {longitude}



    