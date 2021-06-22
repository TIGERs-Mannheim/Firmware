local_dir := lib/common
local_build_dir := lib/f0xx
local_lib := lib/libf0xx.a
local_lib_d := lib/libf0xx_d.a
local_src_dirs := src/f0xx src/f0xx/util

local_include_dirs += -I $(local_dir)/src/f0xx -I $(local_dir)/src
local_include_dirs += -I lib/dsp/src
local_defines := -DSTM32F0XX
local_arch := -mcpu=cortex-m0 -DARM_MATH_CM0 -D__FPU_PRESENT=0 -DCORTEX_USE_FPU=0 -mfloat-abi=soft
local_flags := $(local_include_dirs) $(local_defines)
local_flags_d := $(local_include_dirs) $(local_defines) -DDEBUG

include make/lib.mk