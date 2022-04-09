add_executable(SALMO
        SALMO.c
        )

# pull in common dependencies
target_link_libraries(SALMO pico_stdlib)

#create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(SALMO)