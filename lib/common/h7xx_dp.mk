local_dir := lib/common
local_build_dir := lib/h7xx_dp
local_lib := lib/libh7xx_dp.a
local_lib_d := lib/libh7xx_dpd.a
local_src_dirs := src/chibios/kernel src/chibios/port
local_src_dirs += src/h7xx/util src/util src/main src/usb src
local_src_dirs += src/ugfx/src src/ugfx/drivers/gdisp/ILI9341 src/gui
local_src_dirs += src/fatfs src/fatfs/option  

local_include_dirs := -I $(local_dir)/src/chibios -I $(local_dir)/src/chibios/kernel -I $(local_dir)/src/chibios/port
local_include_dirs += -I $(local_dir)/src/h7xx -I $(local_dir)/src
local_include_dirs += -I $(local_dir)/src/ugfx -I $(local_dir)/src/ugfx/drivers/gdisp/ILI9341
local_include_dirs += -I $(local_dir)/src/fatfs
local_include_dirs += -I lib/dsp/src
local_defines := -DSTM32H7XX -DSTM32H743xx
local_arch := -mcpu=cortex-m7 -mfpu=fpv5-d16 -DARM_MATH_CM7 -D__FPU_PRESENT=1 -DCORTEX_USE_FPU=1 -mfloat-abi=hard
local_flags := $(local_include_dirs) $(local_defines)
local_flags_d := $(local_include_dirs) $(local_defines) -DDEBUG

include make/lib.mk