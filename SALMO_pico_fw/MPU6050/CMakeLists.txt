#add all files required to build the library
add_library(MPU6050 MPU6050.c MPU6050_I2C.c MPU6050.h MPU6050_I2C.h MPU6050_defines.h)
#link necessary lib
target_link_libraries(MPU6050 pico_stdlib hardware_i2c) 