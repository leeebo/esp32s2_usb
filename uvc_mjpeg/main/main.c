#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_jpg_decode.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp32/clk.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/clk.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/clk.h"
#endif

#define PIC_NUM 3

extern uint8_t _binary_1_jpg_start;
extern uint8_t _binary_1_jpg_end;
extern uint8_t _binary_2_jpg_start;
extern uint8_t _binary_2_jpg_end;
extern uint8_t _binary_3_jpg_start;
extern uint8_t _binary_3_jpg_end;

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    uint8_t *p_jpeg = NULL;

    int core_num = 2;
#if CONFIG_FREERTOS_UNICORE
    core_num = 1;
#endif
    printf("core: %d frq: %d core_num: %d \n", xPortGetCoreID(),esp_clk_cpu_freq(), core_num);

    size_t pic_size[PIC_NUM] = {&_binary_1_jpg_end - &_binary_1_jpg_start, &_binary_2_jpg_end - &_binary_2_jpg_start, &_binary_3_jpg_end - &_binary_3_jpg_start};
    uint8_t *pic_address[PIC_NUM] = {&_binary_1_jpg_start, &_binary_2_jpg_start, &_binary_3_jpg_start};

    int64_t time_start_us = esp_timer_get_time();
    for (int i = 0; i < PIC_NUM; i++) {
        int64_t time_temp = esp_timer_get_time();
        p_jpeg = (uint8_t *)realloc(p_jpeg, pic_size[i]);
        assert(p_jpeg != NULL);
        memcpy(p_jpeg, pic_address[i], pic_size[i]);
        printf("%d.jpeg memcpy time = %lld\n", i+1, esp_timer_get_time() - time_temp);
        jpg2rgb565(p_jpeg, pic_size[i], NULL, JPG_SCALE_NONE);
        printf("%d.jpeg all process time = %lld\n", i+1, esp_timer_get_time() - time_temp);
    }
    printf("total %d jpeg process time = %lld\n", PIC_NUM, esp_timer_get_time()-time_start_us);
    free(p_jpeg);
    fflush(stdout);
    vTaskDelay(10000000000000 / portTICK_PERIOD_MS);
}
