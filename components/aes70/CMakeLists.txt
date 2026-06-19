idf_component_register(
    SRCS "src/aes70_ocp1.c"
         "src/aes70_object.c"
         "src/aes70_classes.c"
         "src/aes70_managers.c"
         "src/aes70_subscription.c"
         "src/aes70_transport.c"
         "src/aes70_device.c"
         "src/aes70_mdns.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES esp_common
    PRIV_REQUIRES lwip esp_timer esp_netif mdns
)
