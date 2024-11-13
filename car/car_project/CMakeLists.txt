add_executable(car_project
main.c
motor.c
ultrasonic.c
encoder.c
line_sensor.c
pid.c
barcode.c
)

target_include_directories(car_project PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(car_project 
pico_stdlib 
hardware_pwm 
hardware_gpio 
hardware_timer 
hardware_adc
FreeRTOS-Kernel-Heap4
)

# create map/bin/hex file etc.
pico_add_extra_outputs(car_project)

# add url via pico_set_program_url
example_auto_set_url(car_project)

pico_enable_stdio_usb(car_project 1)
pico_enable_stdio_uart(car_project 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(car_project)