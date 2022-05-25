if (TARGET tinyusb_device)
    add_executable(SALMO SALMO.c)

    target_link_libraries(SALMO PRIVATE pico_stdlib sun_tracker stepper MPU6050 hardware_i2c pico_timer HMC5883L SSD1306 lwgps)

    include_directories("../tracking-algorithm")
    include_directories("../timer")
    include_directories("../MPU6050")
    include_directories("../HMC5883L")
    include_directories("../SSD1306")
    include_directories("../stepper")
    include_directories("../nmea-parser")
    
    # enable usb output, disable uart output
    pico_enable_stdio_usb(SALMO 1)
    pico_enable_stdio_uart(SALMO 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(SALMO)
   
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()