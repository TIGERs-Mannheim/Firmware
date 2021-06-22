local_name := motor2019
local_dir := app/$(local_name)
local_run_build_dir := app/run/$(local_name)
local_boot_build_dir := app/boot/$(local_name)
local_run_program := $(build_dir)/app/run/$(local_name).elf
local_run_program_d := $(build_dir_d)/app/run/$(local_name).elf
local_boot_program := $(build_dir)/app/boot/$(local_name).elf
local_boot_program_d := $(build_dir_d)/app/boot/$(local_name).elf

local_src_dirs := src
local_include_dirs := -I $(local_dir)/src
local_include_dirs += -I lib/common/src/f0xx
local_include_dirs += -I lib/common/src
local_include_dirs += -I lib/dsp/src
local_defines := -DSTM32F0XX
local_arch := -mcpu=cortex-m0 -DARM_MATH_CM0 -D__FPU_PRESENT=0 -DCORTEX_USE_FPU=0 -mfloat-abi=soft
local_flags := $(local_include_dirs) $(local_defines)
local_flags_d := $(local_include_dirs) $(local_defines) -DDEBUG
local_run_flags := -DENV_RUN
local_boot_flags := -DENV_BOOT
local_libs := lib/libf0xx.a lib/libdsp_m0.a
local_libs_d := lib/libf0xx_d.a lib/libdsp_m0_d.a
local_ld_flags := 

include make/prog.mk