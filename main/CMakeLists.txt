idf_component_register(
    SRCS "IMU_track_plotting_main.c"
         "mpu6050_filter.c"
    INCLUDE_DIRS "."
    REQUIRES driver bt nvs_flash esp_wifi esp_event esp_timer freertos
)