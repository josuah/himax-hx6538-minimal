set -eux

rm -rf obj_epii_evb_icv30_bdv10

CFLAGS='-mthumb -mcpu=cortex-m55 -DARMCM55 -mfloat-abi=hard -c
-ffunction-sections -fdata-sections -Wall -fstack-usage
-flax-vector-conversions -specs=nano.specs -mcmse -O2 -g'

mkdir -p obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/clib/

arm-zephyr-eabi-gcc $CFLAGS -I./include -fmacro-prefix-map="../obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/"=. -MMD -MP -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/main.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/main.o" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/main.d" -std=gnu11 -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/main.o" "src/main.c"
arm-zephyr-eabi-gcc -I./include -mthumb -mcpu=cortex-m55 -DARMCM55 -mfloat-abi=hard -E -P -xc linker.ld -MMD -MF"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65.d" -MT"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65.ld" -o "obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65.ld"
arm-zephyr-eabi-gcc -I./include -L"./linker_script/" -L"./linker_script/gcc" -L"./libs" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -Xlinker --cref -mthumb -mcpu=cortex-m55 -DARMCM55 -mfloat-abi=hard -L"./linker_script/gcc/" -L"obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65" -Xlinker --cmse-implib -Wl,-M,-Map=obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.map -specs=nano.specs -T obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65.ld obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/src/main.o -Wl,--start-group -lm -lc_nano -lgcc -lstdc++_nano -Wl,--end-group -o obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf
arm-zephyr-eabi-size obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf

cp obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf ../we2_image_gen_local/input_case1_secboot/
cd ../we2_image_gen_local/
./we2_local_image_gen project_case1_blp_wlcsp.json
