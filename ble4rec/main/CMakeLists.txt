file(GLOB_RECURSE srcs "main.c" "src/*.c")

idf_component_register(SRCS "scanner.c" "${srcs}"
                       PRIV_REQUIRES driver bt nvs_flash
                       INCLUDE_DIRS "./include")

                       