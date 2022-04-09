if (TARGET tinyusb_device)
    add_executable(SALMO SALMO.c)

    #add_executable(sun_tracker ../libs/tracking-algorithm/algo/sun_tracker.c)

    # pull in common dependencies
    #include_directories(../libs/tracking-algorithm/algo)
    #link_directories(../libs/tracking-algorithm/algo)
    
    #add_library(sun_tracker ../libs/tracking-algorithm/algo/sun_tracker.c)
#   target_link_libraries((suntracker))
    
    #target_link_libraries(SALMO pico_stdlib)
    target_link_libraries(SALMO PRIVATE pico_stdlib sun_tracker)
#     target_include_directories(${PROJECT_NAME} PUBLIC algo)
#     target_link_directories(${PROJECT_NAME} PRIVATE algo)
    #target_link_libraries(${PROJECT_NAME} sun_tracker)

    #add_subdirectory(../libs/tracking-algorithm/algo)


    include_directories("../tracking-algorithm")
    # enable usb output, disable uart output
    pico_enable_stdio_usb(SALMO 1)
    pico_enable_stdio_uart(SALMO 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(SALMO)

   
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()