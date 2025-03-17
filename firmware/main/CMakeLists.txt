set(srcs "main.c")
set(publics_requires
    "esp_lcd"
    "st7789"
)

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "."
    REQUIRES "${publics_requires}"
)
