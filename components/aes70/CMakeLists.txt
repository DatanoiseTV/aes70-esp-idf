idf_component_register(
    SRCS "src/aes70_ocp1.c"
         "src/aes70_object.c"
         "src/aes70_classes.c"
         "src/aes70_dsp.c"
         "src/aes70_managers.c"
         "src/aes70_subscription.c"
         "src/aes70_transport.c"
         "src/aes70_device.c"
         "src/aes70_mdns.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES esp_common
    # esp-tls is listed unconditionally: IDF resolves component requirements
    # before Kconfig is available, so an if(CONFIG_AES70_ENABLE_TLS) guard here
    # would drop it from the dependency graph. The TLS code is #if-compiled out
    # when CONFIG_AES70_ENABLE_TLS is unset, and the linker garbage-collects the
    # unused esp-tls objects, so a non-TLS build pays ~no flash for it.
    PRIV_REQUIRES lwip esp_timer esp_netif mdns esp-tls
)
