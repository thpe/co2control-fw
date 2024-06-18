add_executable(main
        main.cpp
        )

# pull in common dependencies
target_link_libraries(main pico_stdlib hardware_adc pico_multicore)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)
pico_enable_stdio_usb(main 0)
pico_enable_stdio_uart(main 1)
