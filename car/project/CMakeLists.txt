add_executable(project
main.c
motor.c
ultrasonic.c
encoder.c
line_sensor.c
pid.c
barcode.c
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(project 
pico_stdlib 
hardware_pwm 
hardware_gpio 
hardware_timer 
hardware_adc)

# create map/bin/hex file etc.
pico_add_extra_outputs(project)

# add url via pico_set_program_url
example_auto_set_url(project)

pico_enable_stdio_usb(project 1)
pico_enable_stdio_uart(project 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(project)