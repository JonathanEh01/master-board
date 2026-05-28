set(srcs "imu_common.c")

if(CONFIG_IMU_MODEL_BNO08X_SHTP)
    list(APPEND srcs "imu_bno08x_shtp.c")
else()
    list(APPEND srcs "imu_microstrain_mip.c")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES freertos log esp_driver_uart esp_hal_uart esp_rom soc
)
