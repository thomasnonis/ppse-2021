#add all files required to build the library
add_library(stepper pico_stepper.hpp)
#link necessary lib
target_link_libraries(stepper pico_stdlib pico_time)