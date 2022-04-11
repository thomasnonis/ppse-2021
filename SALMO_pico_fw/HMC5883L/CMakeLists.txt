#add all files required to build the library
add_library(HMC5883L HMC5883L.c HMC5883L.h)
#link necessary lib
target_link_libraries(HMC5883L pico_stdlib hardware_i2c) 