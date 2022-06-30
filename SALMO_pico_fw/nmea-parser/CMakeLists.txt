#add all files required to build the library
add_library(lwgps minmea.c minmea.h)
#link necessary lib
target_link_libraries(lwgps pico_stdlib)
#Fix compilation warning, now minmea_gettime will return UTC time
target_compile_options(lwgps PRIVATE -Dtimegm=mktime)