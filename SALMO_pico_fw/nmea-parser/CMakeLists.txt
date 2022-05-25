#add all files required to build the library
add_library(lwgps lwgps.c lwgps.h lwgps_opts.h lwgps_opts_template.h)
#link necessary lib
target_link_libraries(lwgps pico_stdlib)