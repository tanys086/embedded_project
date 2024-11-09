if (EXISTS ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c)
    add_executable(remote_control_nosys
            main.c
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
            )
    target_compile_definitions(remote_control_nosys PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            )
    target_include_directories(remote_control_nosys PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping
            )
    target_link_libraries(remote_control_nosys
            pico_cyw43_arch_lwip_threadsafe_background
            pico_stdlib
            hardware_i2c
            pico_lwip_iperf
            FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
            )
    pico_add_extra_outputs(remote_control_nosys)
    pico_enable_stdio_usb(remote_control_nosys 1)

    add_executable(remote_control_sys
        main.c
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
            )
    target_compile_definitions(remote_control_sys PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
            LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
            PING_USE_SOCKETS=1
            )
    target_include_directories(remote_control_sys PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping
            )
    target_link_libraries(remote_control_sys
            hardware_i2c
            pico_cyw43_arch_lwip_sys_freertos
            pico_stdlib
            pico_lwip_iperf
            FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
            )
    pico_add_extra_outputs(remote_control_sys)
    pico_enable_stdio_usb(remote_control_sys 1)
endif()