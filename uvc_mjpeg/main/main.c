#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_jpg_decode.h"
#include "esp_log.h"
#include "soc/efuse_reg.h"
#include "esp_heap_caps.h"
#include "yuv.h"
#include "taskmonitor.h"
#include "spi_bus.h"
#include "screen_driver.h"

#ifdef USE_PSRAM
#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "esp32/spiram.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/spiram.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/spiram.h"
#else
#error Target CONFIG_IDF_TARGET is not supported
#endif
#endif

#if CONFIG_IDF_TARGET_ESP32
#include "esp32/clk.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/clk.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/clk.h"
#endif

#define PIC_NUM 15
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

extern uint8_t _binary_0001_jpg_start;
extern uint8_t _binary_0001_jpg_end;
extern uint8_t _binary_0002_jpg_start;
extern uint8_t _binary_0002_jpg_end;
extern uint8_t _binary_0003_jpg_start;
extern uint8_t _binary_0003_jpg_end;
extern uint8_t _binary_0004_jpg_start;
extern uint8_t _binary_0004_jpg_end;
extern uint8_t _binary_0005_jpg_start;
extern uint8_t _binary_0005_jpg_end;
extern uint8_t _binary_0006_jpg_start;
extern uint8_t _binary_0006_jpg_end;
extern uint8_t _binary_0007_jpg_start;
extern uint8_t _binary_0007_jpg_end;
extern uint8_t _binary_0008_jpg_start;
extern uint8_t _binary_0008_jpg_end;
extern uint8_t _binary_0009_jpg_start;
extern uint8_t _binary_0009_jpg_end;
extern uint8_t _binary_0010_jpg_start;
extern uint8_t _binary_0010_jpg_end;
extern uint8_t _binary_0011_jpg_start;
extern uint8_t _binary_0011_jpg_end;
extern uint8_t _binary_0012_jpg_start;
extern uint8_t _binary_0012_jpg_end;
extern uint8_t _binary_0013_jpg_start;
extern uint8_t _binary_0013_jpg_end;
extern uint8_t _binary_0014_jpg_start;
extern uint8_t _binary_0014_jpg_end;
extern uint8_t _binary_0015_jpg_start;
extern uint8_t _binary_0015_jpg_end;

static const char *TAG = "jpeg decode";
scr_driver_t g_lcd;
static scr_info_t g_lcd_info;

typedef struct {
        uint16_t width;
        uint16_t height;
        uint16_t data_offset;
        const uint8_t *input;
        uint8_t *output;
} rgb_jpg_decoder;

static void *_malloc(size_t size)
{
#ifdef USE_PSRAM
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    return heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#endif
}

static void lcd_screen_clear(scr_driver_t *lcd, int color)
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

static void lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    spi_config_t bus_conf = {
        .miso_io_num = BOARD_LCD_SPI_MISO_PIN,
        .mosi_io_num = BOARD_LCD_SPI_MOSI_PIN,
        .sclk_io_num = BOARD_LCD_SPI_CLK_PIN,
        .max_transfer_sz = 2*320*240 + 10,
    };
    spi_bus_handle_t spi_bus = spi_bus_create(SPI2_HOST, &bus_conf);
    if(spi_bus == NULL) {
        ESP_LOGE(TAG, "spi_bus2 create failed");
    }

    scr_interface_spi_config_t spi_lcd_cfg = {
        .spi_bus = spi_bus,
        .pin_num_cs = BOARD_LCD_SPI_CS_PIN,
        .pin_num_dc = BOARD_LCD_SPI_DC_PIN,
        .clk_freq = 40000000,
        .swap_data = false,
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

    lcd_screen_clear(&g_lcd, COLOR_ESP_BKGD);
    vTaskDelay(pdMS_TO_TICKS(500));

}

//output buffer and image width
#define BUF_WIDTH 320
#define BUF_HIGHT 48

static bool _lcd_write(void * arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
{
    rgb_jpg_decoder * jpeg = (rgb_jpg_decoder *)arg;

    //ESP_LOGE(TAG,"x=%d y=%d yBUF_HIGHT=%d w=%d, h=%d",x,y,y % BUF_HIGHT, w,h);

    if(!data){
        if(x == 0 && y == 0) {
            //write start
            jpeg->width = w;
            jpeg->height = h;
            //if output is null, this is BMP
            if(!jpeg->output){
                //ESP_LOGI(TAG, "malloc_size = %d ", (BUF_WIDTH*BUF_HIGHT*2)+jpeg->data_offset);
                jpeg->output = (uint8_t *)_malloc((BUF_WIDTH*BUF_HIGHT*2)+jpeg->data_offset);
                if(!jpeg->output){
                    ESP_LOGE(TAG, "malloc failed %s %d malloc_size = %d but reserved = %d", __func__, __LINE__,(BUF_WIDTH*BUF_HIGHT*2)+jpeg->data_offset, esp_get_minimum_free_heap_size());
                    return false;
                }
            }
        } else {
            if(jpeg->output) {
            //ESP_LOGI(TAG, "free_size = %d ", (BUF_WIDTH*BUF_HIGHT*2)+jpeg->data_offset);
            free(jpeg->output);
            }
        }
        return true;
    }

    size_t width_565 = jpeg->width*2;//bytes each row dest
    size_t rowstart_565 = (y % BUF_HIGHT) * width_565;//start byte of row y dest
    size_t rowend_565 = rowstart_565 + (h * width_565);//end byte of block end row

    uint8_t *out_buffer_start = jpeg->output+jpeg->data_offset;
    size_t iy, ix, ix2;


    int w_byte = w*3;

    for(iy=rowstart_565; iy<rowend_565; iy+=width_565) {//start row to end row
        uint8_t *out_buffer = out_buffer_start+x*2+iy;
        for(ix2=ix=0; ix<w_byte; ix+= 3, ix2 +=2) {//start column to end column
            uint16_t r = data[ix];
            uint16_t g = data[ix+1];
            uint16_t b = data[ix+2];
            uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            out_buffer[ix2] = c>>8;
            out_buffer[ix2+1] = c&0xff;
        }
        data+=w_byte;
    }

    if (x >= (320 - w) && y<= 240) //buffer full
    {
        if ((y + h)%48 == 0)
        {
            g_lcd.draw_bitmap(0, (y + h - 48), BUF_WIDTH, BUF_HIGHT, (uint16_t *)(jpeg->output));
        }
        
    }

    return true;
}

//input buffer
static uint32_t _jpg_read(void * arg, size_t index, uint8_t *buf, size_t len)
{
    rgb_jpg_decoder * jpeg = (rgb_jpg_decoder *)arg;
    if(buf) {
        memcpy(buf, jpeg->input + index, len);
    }
    return len;
}

bool jpg2lcd(const uint8_t *src, size_t src_len, uint8_t * out_buffer_start, jpg_scale_t scale)
{
    rgb_jpg_decoder jpeg;
    jpeg.width = 0;
    jpeg.height = 0;
    jpeg.input = src;
    jpeg.output = out_buffer_start;
    jpeg.data_offset = 0;

    if(esp_jpg_decode(src_len, scale, _jpg_read, _lcd_write, (void*)&jpeg) != ESP_OK){
        return false;
    }
    return true;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    uint8_t *p_jpeg = NULL;

    int core_num = 2;
#if CONFIG_FREERTOS_UNICORE
    core_num = 1;
#endif
    ESP_LOGI(TAG, "core: %d frq: %d core_num: %d \n", xPortGetCoreID(),esp_clk_cpu_freq(), core_num);
    taskMonitorInit();
    taskMonitorStart();

    lcd_init();

    size_t pic_size[PIC_NUM] = {
    &_binary_0001_jpg_end - &_binary_0001_jpg_start, &_binary_0002_jpg_end - &_binary_0002_jpg_start, &_binary_0003_jpg_end - &_binary_0003_jpg_start,
    &_binary_0004_jpg_end - &_binary_0004_jpg_start, &_binary_0005_jpg_end - &_binary_0005_jpg_start, &_binary_0006_jpg_end - &_binary_0006_jpg_start,
    &_binary_0007_jpg_end - &_binary_0007_jpg_start, &_binary_0008_jpg_end - &_binary_0008_jpg_start, &_binary_0009_jpg_end - &_binary_0009_jpg_start,
    &_binary_0010_jpg_end - &_binary_0010_jpg_start, &_binary_0011_jpg_end - &_binary_0011_jpg_start, &_binary_0012_jpg_end - &_binary_0012_jpg_start,
    &_binary_0013_jpg_end - &_binary_0013_jpg_start, &_binary_0014_jpg_end - &_binary_0014_jpg_start, &_binary_0015_jpg_end - &_binary_0015_jpg_start
    };
    uint8_t *pic_address[PIC_NUM] = {
    &_binary_0001_jpg_start, &_binary_0002_jpg_start, &_binary_0003_jpg_start, &_binary_0004_jpg_start, &_binary_0005_jpg_start,
    &_binary_0006_jpg_start, &_binary_0007_jpg_start, &_binary_0008_jpg_start, &_binary_0009_jpg_start, &_binary_0010_jpg_start,
    &_binary_0011_jpg_start, &_binary_0012_jpg_start, &_binary_0013_jpg_start, &_binary_0014_jpg_start, &_binary_0015_jpg_start
    };

    int64_t time_start_us = 0;
    int64_t time_temp = 0;
    for (int i = 0; i < 10000000; i++) {
        int index = i % 8;
        if (index == 0) time_start_us = esp_timer_get_time();
        //time_temp = esp_timer_get_time();
        p_jpeg = (uint8_t *)realloc(p_jpeg, pic_size[index]);
        if (p_jpeg == NULL)
        {
            ESP_LOGE(TAG,"realloc fail index=%d size=%d", index, pic_size[index]);
        }
        memcpy(p_jpeg, pic_address[index], pic_size[index]);
        //ESP_LOGI(TAG, "%d.jpeg memcpy time = %lld\n", index+1, esp_timer_get_time() - time_temp);
        jpg2lcd(p_jpeg, pic_size[index], NULL, JPG_SCALE_NONE);
        //ESP_LOGI(TAG, "%d.jpeg all process time = %lld\n", index+1, esp_timer_get_time() - time_temp);
        vTaskDelay(2);
        if (index == 7) ESP_LOGI(TAG, "total %d jpeg process time = %lld\n", PIC_NUM, esp_timer_get_time()-time_start_us);
    }
    free(p_jpeg);
    fflush(stdout);
    vTaskDelay(10000000000000 / portTICK_PERIOD_MS);
}
