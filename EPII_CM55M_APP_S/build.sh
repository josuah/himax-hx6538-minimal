set -eux

rm -rf obj_epii_evb_icv30_bdv10

DFLAGS='-DARMCM55 -DALLON_SENSOR_TFLM -DCM55_BIG -DCOREV_0P9V
-DDBG_MORE -DDEBUG -DEPII_EVB -DEVT_DATAPATH -DIC_PACKAGE_WLCSP65
-DIC_VERSION=30 -DIP_2x2 -DIP_5x5 -DIP_INST_ADCC -DIP_INST_ADCC_HV
-DIP_INST_AON_GPIO -DIP_INST_DMA0 -DIP_INST_DMA1 -DIP_INST_DMA2
-DIP_INST_DMA3 -DIP_INST_GPIO_G0 -DIP_INST_GPIO_G1 -DIP_INST_GPIO_G2
-DIP_INST_GPIO_G3 -DIP_INST_I2S_HOST -DIP_INST_I2S_SLAVE
-DIP_INST_IIC_HOST -DIP_INST_IIC_HOST_MIPI -DIP_INST_IIC_HOST_SENSOR
-DIP_INST_IIIC_SLAVE0 -DIP_INST_IIIC_SLAVE1 -DIP_INST_OSPI_HOST
-DIP_INST_PWM0 -DIP_INST_PWM1 -DIP_INST_PWM2 -DIP_INST_QSPI_HOST
-DIP_INST_RTC0 -DIP_INST_RTC1 -DIP_INST_RTC2 -DIP_INST_SB_GPIO
-DIP_INST_SSPI_HOST -DIP_INST_SSPI_SLAVE -DIP_INST_TIMER0
-DIP_INST_TIMER1 -DIP_INST_TIMER2 -DIP_INST_TIMER3 -DIP_INST_TIMER4
-DIP_INST_TIMER5 -DIP_INST_TIMER6 -DIP_INST_TIMER7 -DIP_INST_TIMER8
-DIP_INST_UART0 -DIP_INST_UART1 -DIP_INST_UART2 -DIP_INST_WDT0
-DIP_INST_WDT1 -DIP_adcc -DIP_adcc_hv -DIP_cdm -DIP_csirx -DIP_csitx
-DIP_dma -DIP_dp -DIP_edm -DIP_gpio -DIP_hxautoi2c_mst -DIP_i2s
-DIP_i3c_mst -DIP_i3c_slv -DIP_iic -DIP_inp -DIP_inp1bitparser
-DIP_inpovparser -DIP_isp -DIP_jpeg -DIP_mb -DIP_mpc -DIP_pdm -DIP_pmu
-DIP_ppc -DIP_pwm -DIP_rtc -DIP_scu -DIP_sensorctrl -DIP_spi
-DIP_swreg_aon -DIP_swreg_lsc -DIP_timer -DIP_tpg -DIP_u55 -DIP_uart
-DIP_vad -DIP_watchdog -DIP_xdma -DLIB_COMMON -DLIB_EVENT
-DLIB_PWRMGMT -DTRUSTZONE -DTRUSTZONE_CFG -DTRUSTZONE_SEC
-DTRUSTZONE_SEC_ONLY -D__GNU__ -D__NEWLIB__'

CFLAGS='-mthumb -mcpu=cortex-m55 -DARMCM55 -mfloat-abi=hard -c
-ffunction-sections -fdata-sections -Wall -fstack-usage
-flax-vector-conversions -specs=nano.specs -mcmse -O2 -g'

mkdir -p obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/ 2> /dev/null
mkdir -p obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/ 2> /dev/null
mkdir -p obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/clib/ 2> /dev/null
mkdir -p obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src// 2> /dev/null

arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/WE2_core.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/WE2_core.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/WE2_core.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/WE2_core.o" "src/WE2_core.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/system_WE2_ARMCM55.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/system_WE2_ARMCM55.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/system_WE2_ARMCM55.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/system_WE2_ARMCM55.o" "src/system_WE2_ARMCM55.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/console_io.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/console_io.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/console_io.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/console_io.o" "src/console_io.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/retarget.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/retarget.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/retarget.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/retarget.o" "src/retarget.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/retarget_io.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/retarget_io.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/retarget_io.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/retarget_io.o" "src/retarget_io.c"
arm-none-eabi-g++ $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/startup_WE2_ARMCM55.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/startup_WE2_ARMCM55.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/startup_WE2_ARMCM55.d" -std=c++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics -nostdlib -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/startup_WE2_ARMCM55.o" "src/startup_WE2_ARMCM55.cc"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/board.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/board.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/board.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/board.o" "src/board.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/pinmux_init.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/pinmux_init.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/pinmux_init.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/pinmux_init.o" "src/pinmux_init.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/platform_driver_init.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/platform_driver_init.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/platform_driver_init.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/platform_driver_init.o" "src/platform_driver_init.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/allon_sensor_tflm.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/allon_sensor_tflm.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/allon_sensor_tflm.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/allon_sensor_tflm.o" "src/allon_sensor_tflm.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/driver_interface.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/driver_interface.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/driver_interface.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/driver_interface.o" "src/driver_interface.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/hardfault_handler.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/hardfault_handler.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/hardfault_handler.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/hardfault_handler.o" "src/hardfault_handler.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/main.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/main.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/main.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/main.o" "src/main.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/timer_interface.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/timer_interface.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/timer_interface.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/timer_interface.o" "src/timer_interface.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/trustzone_cfg.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/trustzone_cfg.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/trustzone_cfg.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/trustzone_cfg.o" "src/trustzone_cfg.c"
arm-none-eabi-gcc $CFLAGS $DFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/xprintf.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/xprintf.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/xprintf.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/xprintf.o" "src/xprintf.c"
cp prebuilt_libs/gnu/libtrustzone_cfg.a obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libtrustzone_cfg.a
cp prebuilt_libs/gnu/libdriver.a obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libdriver.a
cp prebuilt_libs/gnu/libcommon.a obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libcommon.a
cp prebuilt_libs/gnu/libhxevent.a obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libhxevent.a
cp prebuilt_libs/gnu/libpwrmgmt.a obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libpwrmgmt.a
arm-none-eabi-gcc -I./include -mthumb -mcpu=cortex-m55 -DARMCM55 -mfloat-abi=hard -E -P -xc linker.ld -MMD -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65.ld" -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65.ld"
arm-none-eabi-gcc -I./include -Wl,--no-warn-rwx-segments -L"./linker_script/" -L"./linker_script/gcc" -L"./libs" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mthumb -mcpu=cortex-m55 -DARMCM55 -mfloat-abi=hard -L"./linker_script/gcc/" -L"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65" -Xlinker --cmse-implib -Wl,-M,-Map=obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.map -specs=nano.specs -T obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65.ld @objs.in -Wl,--start-group obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libtrustzone_cfg.a obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libdriver.a obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libcommon.a obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libhxevent.a obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/libpwrmgmt.a -lm -lc_nano -lgcc -lstdc++_nano -Wl,--end-group -o obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf
arm-none-eabi-size obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf

cp obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf ../we2_image_gen_local/input_case1_secboot/
cd ../we2_image_gen_local/
./we2_local_image_gen project_case1_blp_wlcsp.json