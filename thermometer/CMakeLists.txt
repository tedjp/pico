add_executable(thermometer
	thermometer.cpp
	)

# Exclude stuff that writes to STDIO/UART
pico_enable_stdio_uart(thermometer 0)

target_compile_definitions(thermometer PRIVATE
    PICO_PANIC_FUNCTION=
)

target_link_libraries(thermometer hardware_adc hardware_gpio)
# Easy-mode: Grab all of pico_stdlib
target_link_libraries(thermometer pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(thermometer)
