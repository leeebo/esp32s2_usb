#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_jpg_decode.h"
#include "spi_bus.h"
#include "screen_driver.h"
#include "esp_log.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp32/clk.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/clk.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/clk.h"
#endif

#define PIC_NUM 3
/**< Screen inrerface pins */
#define BOARD_LCD_SPI_HOST 1
#define BOARD_LCD_SPI_CLOCK_FREQ 20000000
#define BOARD_LCD_SPI_MISO_PIN 40
#define BOARD_LCD_SPI_MOSI_PIN 38
#define BOARD_LCD_SPI_CLK_PIN 39
#define BOARD_LCD_SPI_CS_PIN 37
#define BOARD_LCD_SPI_DC_PIN 41
#define BOARD_LCD_SPI_RESET_PIN 0
#define BOARD_LCD_SPI_BL_PIN 21


extern uint8_t _binary_1_jpg_start;
extern uint8_t _binary_1_jpg_end;
extern uint8_t _binary_2_jpg_start;
extern uint8_t _binary_2_jpg_end;
extern uint8_t _binary_3_jpg_start;
extern uint8_t _binary_3_jpg_end;

static const char *TAG = "jpeg decode";
scr_driver_t g_lcd;
static scr_info_t g_lcd_info;

static void screen_clear(scr_driver_t *lcd, int color)
{
    scr_info_t lcd_info;
    lcd->get_info(&lcd_info);
    uint16_t *buffer = malloc(lcd_info.width * sizeof(uint16_t));
    if (NULL == buffer) {
        for (size_t y = 0; y < lcd_info.height; y++) {
            for (size_t x = 0; x < lcd_info.width; x++) {
                lcd->draw_pixel(x, y, color);
            }
        }
    } else {
        for (size_t i = 0; i < lcd_info.width; i++) {
            buffer[i] = color;
        }

        for (int y = 0; y < lcd_info.height; y++) {
            lcd->draw_bitmap(0, y, lcd_info.width, 1, buffer);
        }

        free(buffer);
    }
}

void lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    spi_config_t bus_conf = {
        .miso_io_num = BOARD_LCD_SPI_MISO_PIN,
        .mosi_io_num = BOARD_LCD_SPI_MOSI_PIN,
        .sclk_io_num = BOARD_LCD_SPI_CLK_PIN,
        .max_transfer_sz = 2*320*240 + 10,
    };
    spi_bus_handle_t spi_bus = spi_bus_create(SPI2_HOST, &bus_conf);
    if(spi_bus != NULL) {
        ESP_LOGE(TAG, "spi_bus2 creat failed");
    }

    scr_interface_spi_config_t spi_lcd_cfg = {
        .spi_bus = spi_bus,
        .pin_num_cs = BOARD_LCD_SPI_CS_PIN,
        .pin_num_dc = BOARD_LCD_SPI_DC_PIN,
        .clk_freq = BOARD_LCD_SPI_CLOCK_FREQ,
        .swap_data = true,
    };

    scr_interface_driver_t *iface_drv;
    scr_interface_create(SCREEN_IFACE_SPI, &spi_lcd_cfg, &iface_drv);
    ret = scr_find_driver(SCREEN_CONTROLLER_ILI9341, &g_lcd);
    if (ESP_OK != ret) {
        return;
        ESP_LOGE(TAG, "screen find failed");
    }

    scr_controller_config_t lcd_cfg = {
        .interface_drv = iface_drv,
        .pin_num_rst = BOARD_LCD_SPI_RESET_PIN,
        .pin_num_bckl = BOARD_LCD_SPI_BL_PIN,
        .rst_active_level = 0,
        .bckl_active_level = 1,
        .offset_hor = 0,
        .offset_ver = 0,
        .width = 240,
        .height = 320,
        .rotate = SCR_DIR_BTRL,
    };
    ret = g_lcd.init(&lcd_cfg);
    if (ESP_OK != ret) {
        return;
        ESP_LOGE(TAG, "screen initialize failed");
    }

    g_lcd.get_info(&g_lcd_info);
    ESP_LOGI(TAG, "Screen name:%s | width:%d | height:%d", g_lcd_info.name, g_lcd_info.width, g_lcd_info.height);

    screen_clear(&g_lcd, COLOR_ESP_BKGD);
    vTaskDelay(pdMS_TO_TICKS(500));

}

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
    lcd_init();
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
