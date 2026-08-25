idf_component_register(SRCS "multi_adv_demo.c"
                    PRIV_REQUIRES 
                        bt 
                        nvs_flash
                        esp_wifi
                        esp_event
                        esp_netif
                        esp_http_server
                        json
                    INCLUDE_DIRS ".")
