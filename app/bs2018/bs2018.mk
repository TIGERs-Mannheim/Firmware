local_name := bs2018
local_dir := app/$(local_name)
local_run_build_dir := app/run/$(local_name)
local_boot_build_dir := app/boot/$(local_name)
local_run_program := $(build_dir)/app/run/$(local_name).elf
local_run_program_d := $(build_dir_d)/app/run/$(local_name).elf
local_boot_program := $(build_dir)/app/boot/$(local_name).elf
local_boot_program_d := $(build_dir_d)/app/boot/$(local_name).elf

local_src_dirs := src src/hal src/usb src/gui src/protobuf-c src/network src/util
local_include_dirs := -I $(local_dir)/src
local_include_dirs += -I lib/common/src/f7xx
local_include_dirs += -I lib/common/src -I lib/common/src/fatfs
local_include_dirs += -I lib/common/src/chibios -I lib/common/src/chibios/kernel -I lib/common/src/chibios/port
local_include_dirs += -I lib/dsp/src
local_include_dirs += -I lib/common/src/ugfx -I lib/common/src/ugfx/drivers/gdisp/SSD1963
local_defines := -DSTM32F7XX -DSTM32F746xx
local_arch := -mcpu=cortex-m7 -mfpu=fpv5-d16 -DARM_MATH_CM7 -D__FPU_PRESENT=1 -DCORTEX_USE_FPU=1 -mfloat-abi=hard
local_flags := $(local_include_dirs) $(local_defines)
local_flags_d := $(local_include_dirs) $(local_defines) -DDEBUG
local_run_flags := -DENV_RUN
local_boot_flags := -DENV_BOOT
local_libs := lib/libf7xx_dp.a lib/libdsp.a
local_libs_d := lib/libf7xx_dpd.a lib/libdsp_d.a
local_ld_flags := -u _printf_float -u _scanf_float

include make/prog.mk