#add all files required to build the library
add_library(sun_tracker sun_tracker.c sun_tracker.h)
#link necessary lib to use printf
target_link_libraries(sun_tracker pico_stdlib) 

#TODO: check if it's needed
#target_include_directories(sun_tracker PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")