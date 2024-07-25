set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os")
set(CMAKE_CXX_FLAGS_RELEASE "-Os")
set(CMAKE_ASM_FLAGS "-x assembler-with-cpp -c")

add_compile_options(
    -mthumb
    
    -fno-common
    -fno-strict-aliasing
    -finline
    -finline-functions-called-once
    -fmessage-length=0
    -ffunction-sections
    -fdata-sections
    -fno-tree-loop-distribute-patterns
    -fsingle-precision-constant
    
    -Wall
    -Wextra
    -Wno-sign-compare
    -Wwrite-strings
    -Werror-implicit-function-declaration
    -Wno-format
    -Wno-duplicate-decl-specifier
    -Wno-address-of-packed-member
    -Werror=discarded-qualifiers
    -Werror=array-bounds
    -Werror=incompatible-pointer-types
    -Werror=int-conversion
)

add_link_options(
    -mthumb
    -u_vectors
    -ufaultHandler
    -nostartfiles
    -Xlinker
    --gc-sections
    --specs=nano.specs
)
