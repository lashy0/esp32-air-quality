set(srcs "st7789.c")
set(includes "include")
set(publis_requires
    "driver"
    "esp_lcd"
)

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "${includes}"
    REQUIRES "${publis_requires}"
)