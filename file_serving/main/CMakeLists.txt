idf_component_register(SRCS "main.c" "file_server.c" "msc_disk.c"
                    INCLUDE_DIRS "." ${COMPONENT_DIR}
                    EMBED_FILES "favicon.ico" "upload_script.html")