idf_component_register(
    SRCS
        "main.c"
        "shared.c"
        "task_motor.c"
        "task_camera_uart.c"
        "task_navigation.c"
        "task_state_machine.c"
        "task_wifi.c"
        "task_lidar.c"
        "task_flame_detect.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_http_server nvs_flash esp_netif esp_adc esp_driver_ledc)
