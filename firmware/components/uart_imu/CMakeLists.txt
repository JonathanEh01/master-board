idf_component_register(
    SRCS "uart_imu.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_log esp_driver_uart esp_hal_uart soc
)
