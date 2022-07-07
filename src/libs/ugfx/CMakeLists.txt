list(APPEND ugfx_ILI9341_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/gfx_mk.c
    ${CMAKE_CURRENT_LIST_DIR}/src/gfx.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/gdisp/ILI9341/gdisp_lld_ILI9341.c
)

list(APPEND ugfx_ILI9341_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/drivers/gdisp/ILI9341
)

list(APPEND ugfx_SSD1963_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/gfx_mk.c
    ${CMAKE_CURRENT_LIST_DIR}/src/gfx.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/gdisp/SSD1963/gdisp_lld_SSD1963.c
)

list(APPEND ugfx_SSD1963_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/drivers/gdisp/SSD1963
)

set_source_files_properties(
    ${CMAKE_CURRENT_LIST_DIR}/src/gfx_mk.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/gdisp/ILI9341/gdisp_lld_ILI9341.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/gdisp/SSD1963/gdisp_lld_SSD1963.c
    PROPERTIES
    COMPILE_OPTIONS "-Wno-duplicate-decl-specifier"
)    
