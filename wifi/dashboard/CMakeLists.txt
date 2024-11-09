if (EXISTS ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c)
    add_executable(dashboard_nosys
            dashboard.c
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
            )
    target_compile_definitions(dashboard_nosys PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            )
    target_include_directories(dashboard_nosys PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping
            )
    target_link_libraries(dashboard_nosys
            pico_cyw43_arch_lwip_threadsafe_background
            pico_stdlib
            hardware_adc
            pico_lwip_iperf
            FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
            )
    pico_add_extra_outputs(dashboard_nosys)
    pico_enable_stdio_usb(dashboard_nosys 1)

    add_executable(dashboard_sys
        dashboard.c
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
            )
    target_compile_definitions(dashboard_sys PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
            LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
            PING_USE_SOCKETS=1
            )
    target_include_directories(dashboard_sys PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping
            )
    target_link_libraries(dashboard_sys
            hardware_adc
            pico_cyw43_arch_lwip_sys_freertos
            pico_stdlib
            hardware_adc
            pico_lwip_iperf
            FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
            )
    pico_add_extra_outputs(dashboard_sys)
    pico_enable_stdio_usb(dashboard_sys 1)
endif()