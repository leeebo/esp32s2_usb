/* HTTP File Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sys/param.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "protocol_examples_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "tinyusb.h"
#include "tusb.h"
#include "diskio_wl.h"

#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"

//#define WIFI_SSID      "Udp Server"
static char WIFI_SSID[32] = "ESP-DRONE";
static char *WIFI_PWD = "";
// static char WIFI_PWD[64] = "12345678" ;
#define MAX_STA_CONN (2)
static sdmmc_card_t* mount_card = NULL;

/* This example demonstrates how to create file server
 * using esp_http_server. This file has only startup code.
 * Look in file_server.c for the implementation */

static const char *TAG = "example";

// Handle of the wear levelling library instance
wl_handle_t s_wl_handle_1 = WL_INVALID_HANDLE;
BYTE pdrv_msc = 0xFF;

#define USBD_STACK_SIZE     4096
StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;

#define SPI_DMA_CHAN    host.slot

// Mount path for the partition
const char *base_path = "/spiflash";
/* Function to initialize SPIFFS */
static esp_err_t init_fat(void)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    esp_err_t err = ESP_FAIL;
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
#ifdef USE_INTERNAL_FLASH
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 9,
        .format_if_mount_failed = true,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    err = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &s_wl_handle_1);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return ESP_FAIL;
    }

    pdrv_msc = ff_diskio_get_pdrv_wl(s_wl_handle_1);
    ESP_LOGI(TAG, "pdrv_msc = %d !!", pdrv_msc);
#else

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    sdmmc_card_t *card;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 13,
        .miso_io_num = 15,
        .sclk_io_num = 14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    err = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = 12;
    slot_config.host_id = host.slot;

    err = esp_vfs_fat_sdspi_mount(base_path, &host, &slot_config, &mount_config, &card);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return ESP_FAIL;
    }

    sdmmc_card_print_info(stdout, card);
    mount_card = card;
    //ESP_ERROR_CHECK(unmount_card(mount_base_path, mount_card));
#endif

    return ESP_OK;
}

/* Declare the function which starts the file server.
 * Implementation of this function is to be found in
 * file_server.c */
esp_err_t start_file_server(const char *base_path);

// USB Device Driver task
// This top level thread processes all usb events and invokes callbacks
static void usb_device_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "USB task started");

    while (1) {
        if (tusb_inited()) {
            tud_task(); // RTOS forever loop
        }

        vTaskDelay(1);
    }
}

void cdc_task(void *params)
{
    (void) params;

    // RTOS forever loop
    while (1) {
        if (tud_cdc_connected()) {
            // connected and there are data available
            if (tud_cdc_available()) {
                uint8_t buf[64];
                // read and echo back
                uint32_t count = tud_cdc_read(buf, sizeof(buf));

                for (uint32_t i = 0; i < count; i++) {
                    tud_cdc_write_char(buf[i]);

                    if (buf[i] == '\r') {
                        tud_cdc_write_str("\n esp32s2> ");
                    }
                }

                tud_cdc_write_flush();
            }
        }

        // For ESP32-S2 this delay is essential to allow idle how to run and reset wdt
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//--------------------------------------------------------------------+
// tinyusb callbacks
//--------------------------------------------------------------------+

extern void usb_msc_mount();
extern void usb_msc_umount();

// Invoked when device is mounted
void tud_mount_cb(void)
{
    usb_msc_mount();
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    usb_msc_umount();
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allows us to perform remote wakeup
// USB Specs: Within 7ms, device must draw an average current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);

    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());
    uint8_t mac[6];

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    NULL,
                    NULL));

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    sprintf(WIFI_SSID, "ESP32S2_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config;
    memcpy(wifi_config.ap.ssid, WIFI_SSID, strlen(WIFI_SSID) + 1) ;
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    memcpy(wifi_config.ap.password, WIFI_PWD, strlen(WIFI_PWD) + 1) ;
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.channel  = 13;

    if (strlen(WIFI_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap complete.SSID:%s password:%s", WIFI_SSID, WIFI_PWD);

    /* Initialize file storage */
    ESP_ERROR_CHECK(init_fat());

    /* Start the file server */
    ESP_ERROR_CHECK(start_file_server("/spiflash"));

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "USB initialization");

    tinyusb_config_t tusb_cfg = {
        .descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false // In the most cases you need to use a `false` value
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
    // Create a task for tinyusb device stack:
    (void) xTaskCreateStatic(usb_device_task,
                             "usbd",
                             USBD_STACK_SIZE,
                             NULL,
                             5,
                             usb_device_stack,
                             &usb_device_taskdef);
    xTaskCreate(cdc_task, "cdc", 4096, NULL, 4, NULL);
}
