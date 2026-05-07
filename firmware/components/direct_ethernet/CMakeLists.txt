idf_component_register(
    SRCS "direct_ethernet.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_event esp_log esp_eth esp_netif
    PRIV_REQUIRES lwip
)