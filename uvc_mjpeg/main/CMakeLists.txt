if(CONFIG_SIZE_320_240)
    set(PIC_DIR "320240")
elseif(CONFIG_SIZE_160_120)
    set(PIC_DIR "160120")
endif()

idf_component_register(SRCS ${srcs} main.c
                    INCLUDE_DIRS "."
                    EMBED_FILES ${PIC_DIR}/0001.jpg ${PIC_DIR}/0002.jpg ${PIC_DIR}/0003.jpg ${PIC_DIR}/0004.jpg ${PIC_DIR}/0005.jpg ${PIC_DIR}/0006.jpg ${PIC_DIR}/0007.jpg ${PIC_DIR}/0008.jpg ${PIC_DIR}/0009.jpg ${PIC_DIR}/0010.jpg ${PIC_DIR}/0011.jpg ${PIC_DIR}/0012.jpg ${PIC_DIR}/0013.jpg ${PIC_DIR}/0014.jpg ${PIC_DIR}/0015.jpg
                    )