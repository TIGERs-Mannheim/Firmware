local_dir := lib/dsp
local_build_dir := lib/dsp
local_lib := lib/libdsp.a
local_lib_d := lib/libdsp_d.a
local_src_dirs := src src/BasicMathFunctions src/CommonTables src/ComplexMathFunctions
local_src_dirs += src/ControllerFunctions src/FilteringFunctions src/MatrixFunctions
local_src_dirs += src/FastMathFunctions src/StatisticsFunctions src/SupportFunctions
local_src_dirs += src/TransformFunctions

local_include_dirs := -I $(local_dir)/src
local_defines := -DHSE_VALUE=12000000
local_arch := -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -DARM_MATH_CM7 -D__FPU_PRESENT=1 -DCORTEX_USE_FPU=1 -mfloat-abi=hard
local_flags := $(local_include_dirs) $(local_defines)
local_flags_d := $(local_include_dirs) $(local_defines) -DDEBUG

include make/lib.mk