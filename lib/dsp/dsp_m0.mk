local_dir := lib/dsp
local_build_dir := lib/dsp_m0
local_lib := lib/libdsp_m0.a
local_lib_d := lib/libdsp_m0_d.a
local_src_dirs := src src/BasicMathFunctions src/CommonTables src/ComplexMathFunctions
local_src_dirs += src/ControllerFunctions src/FilteringFunctions src/MatrixFunctions
local_src_dirs += src/FastMathFunctions src/StatisticsFunctions src/SupportFunctions
local_src_dirs += src/TransformFunctions

local_include_dirs := -I $(local_dir)/src
local_defines := -DHSE_VALUE=12000000
local_arch := -mcpu=cortex-m0 -DARM_MATH_CM0 -D__FPU_PRESENT=0 -DCORTEX_USE_FPU=0 -mfloat-abi=soft
local_flags := $(local_include_dirs) $(local_defines)
local_flags_d := $(local_include_dirs) $(local_defines) -DDEBUG

include make/lib.mk