add_library(pico_timer pico_timer.c pico_timer.h)

# pull in common dependencies
target_link_libraries(pico_timer pico_stdlib)
