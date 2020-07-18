#include <stdint.h>
#include "syncd_tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

void app_main(){
	// Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    xTaskCreate(*mpu6050_task, "mpu6050_task", 2048, NULL, 11, NULL);
    
    vTaskDelay(100000 / portTICK_RATE_MS);
	
}
