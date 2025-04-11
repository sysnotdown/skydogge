add_executable(skydogge
        skydogge.cc
        gps_ublox.cc
        tools.cc
        motor.cc
        micro_task_exec.cc
        macro_task_exec.cc
        CSpiBase.cc
        CI2cBase.cc
        icm426xx.cc
        bmp390_spi.cc
        CQMC5883.cc
        CHMC5883.cc
        pio_i2c.cc
        CDual_Lora.cc
        dshot_encoder.cpp
        pico_pio_loader.cpp
        CMotorFilter.cc
        #CMTF01.cc
        CTofSenseF.cc
        )

pico_generate_pio_header(skydogge ${CMAKE_CURRENT_LIST_DIR}/uart_rx.pio)

pico_generate_pio_header(skydogge ${CMAKE_CURRENT_LIST_DIR}/i2c.pio)

pico_generate_pio_header(skydogge ${CMAKE_CURRENT_LIST_DIR}/dshot_encoder.pio)

#copy to ram to run, maybe fail.
#pico_set_binary_type(skydogge copy_to_ram)

#target_sources(skydogge PRIVATE skydogge.cc)

# pull in common dependencies and additional i2c hardware support
target_link_libraries(skydogge PRIVATE pico_stdlib hardware_i2c hardware_spi pico_multicore hardware_pwm hardware_adc hardware_flash hardware_sync hardware_pio)

pico_set_float_implementation(skydogge default)
pico_set_double_implementation(skydogge default)

#show message in usb
pico_enable_stdio_usb(skydogge 1)

#disable printf output via UART.  两个uart都被占用了，不能输出到uart.
pico_enable_stdio_uart(skydogge 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(skydogge)

# add url via pico_set_program_url
#example_auto_set_url(bmp280_filter_i2c)

#add_definitions(-D DEBUGINFO)
#add_definitions(-D USE_PIO_I2C)

#picotool.exe run too slow 没用
# target_compile_definitions(skydogge PRIVATE
#        PICO_NO_COPRO_DIS=1
#        #PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
# )

