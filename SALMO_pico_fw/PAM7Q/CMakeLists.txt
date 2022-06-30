#add all files required to build the library
add_library(PAM7Q PAM7Q.c PAM7Q.h)
#link necessary lib
target_link_libraries(PAM7Q pico_stdlib hardware_uart)