if (TARGET tinyusb_device)
    add_executable(SALMO SALMO.c)

    target_link_libraries(SALMO PRIVATE pico_stdlib sun_tracker MPU6050)

    include_directories("../tracking-algorithm")
    # enable usb output, disable uart output
    pico_enable_stdio_usb(SALMO 1)
    pico_enable_stdio_uart(SALMO 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(SALMO)
   
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()