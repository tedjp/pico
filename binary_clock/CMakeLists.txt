add_executable(binary_clock
	binary_clock.c
	)

# Pull in pico_stdlib which pulls in commonly used features
target_link_libraries(binary_clock pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(binary_clock)
