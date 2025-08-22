idf_component_register(SRCS lamidi_main.c ledstrip.c
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_timer esp_driver_uart driver
                    )
