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
#include "protocol_examples_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "tinyusb.h"
#include "tusb.h"
#include "diskio_wl.h"

#include "esp_vfs.h"
#include "esp_vfs_fat.h"

/* This example demonstrates how to create file server
 * using esp_http_server. This file has only startup code.
 * Look in file_server.c for the implementation */

static const char *TAG="example";

// Handle of the wear levelling library instance
wl_handle_t s_wl_handle_1 = WL_INVALID_HANDLE;
BYTE pdrv_msc = 0xFF;

#define USBD_STACK_SIZE     4096
StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;

// Mount path for the partition
const char *base_path = "/spiflash";
/* Function to initialize SPIFFS */
static esp_err_t init_fat(void)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &s_wl_handle_1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return ESP_FAIL;
    }
    pdrv_msc = ff_diskio_get_pdrv_wl(s_wl_handle_1);
    ESP_LOGI(TAG, "pdrv_msc = %d !!", pdrv_msc);
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
        if(tusb_inited()) {
            tud_task(); // RTOS forever loop
        }
    }
    vTaskDelay(1);
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

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /* Initialize file storage */
    ESP_ERROR_CHECK(init_fat());

    /* Start the file server */
    ESP_ERROR_CHECK(start_file_server("/spiflash"));

    vTaskDelay(1000/portTICK_PERIOD_MS);
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
}
