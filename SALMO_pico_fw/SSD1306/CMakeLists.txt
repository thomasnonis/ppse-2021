#add all files required to build the library
add_library(SSD1306 ssd1306_fonts.c raspberry26x32.h ssd1306_fonts.h)
#link necessary lib
target_link_libraries(SSD1306 pico_stdlib hardware_i2c)