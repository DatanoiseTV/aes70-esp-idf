idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES aes70 protocol_examples_common nvs_flash esp_netif esp_event esp_driver_tsens
)
