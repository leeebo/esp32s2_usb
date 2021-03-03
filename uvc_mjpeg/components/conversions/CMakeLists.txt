
if(IDF_TARGET STREQUAL "esp32s2")
    set(srcs "esp_jpg_decode.c" "to_rgb.c" "yuv.c" "tjpgd/tjpgd.c")
    set(iclds "." "include" "tjpgd")
else()
    set(srcs "esp_jpg_decode.c" "to_rgb.c" "yuv.c")
    set(iclds "." "include")
endif()

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${iclds}"
                    PRIV_REQUIRES screen)
