add_executable(thermometer
    thermometer.cpp
)

pico_generate_pio_header(
    thermometer
    ${CMAKE_CURRENT_LIST_DIR}/segment_display.pio
)

# Exclude stuff that writes to STDIO/UART
pico_enable_stdio_uart(thermometer 0)

target_compile_definitions(thermometer PRIVATE
    PICO_PANIC_FUNCTION=
)

target_link_libraries(thermometer hardware_adc hardware_gpio hardware_pio)
# Easy-mode: Grab all of pico_stdlib
target_link_libraries(thermometer pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(thermometer)
