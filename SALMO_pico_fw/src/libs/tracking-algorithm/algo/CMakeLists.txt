set(algo_files
        sun_tracker.c
        sun_tracker.h
        )
add_library(algo ${algo_files})
target_link_libraries(sun_tracker pico_stdlib)
target_link_libraries( ${PROJECT_NAME} algo )