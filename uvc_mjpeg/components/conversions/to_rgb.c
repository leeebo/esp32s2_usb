// Copyright 2020-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stddef.h>
#include <string.h>
#include "soc/efuse_reg.h"
#include "esp_heap_caps.h"
#include "yuv.h"
#include "sdkconfig.h"
#include "esp_jpg_decode.h"
#include "esp_system.h"
#include "esp_log.h"
static const char* TAG = "to_rgb";

#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "esp32/spiram.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/spiram.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/spiram.h"
#else
#error Target CONFIG_IDF_TARGET is not supported
#endif

typedef struct {
        uint16_t width;
        uint16_t height;
        uint16_t data_offset;
        const uint8_t *input;
        uint8_t *output;
} rgb_jpg_decoder;

static void *_malloc(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

//output buffer and image width

static bool _rgb565_write(void * arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
{
    rgb_jpg_decoder * jpeg = (rgb_jpg_decoder *)arg;
    if(!data){
        if(x == 0 && y == 0){
            //write start
            jpeg->width = w;
            jpeg->height = h;
            //if output is null, this is BMP
            if(!jpeg->output){
                jpeg->output = (uint8_t *)_malloc((w*h*3)+jpeg->data_offset);
                if(!jpeg->output){
                    ESP_LOGE(TAG, "malloc failed %s %d malloc_size = %d but reserved = %d", __func__, __LINE__,(w*h*3)+jpeg->data_offset, esp_get_minimum_free_heap_size());
                    return false;
                }
            }
        } else {
            //write end
        }
        return true;
    }

    if (!jpeg->output)
    {
        return true;
    }
    
    size_t jw = jpeg->width*3;//bytes each row source
    size_t jw2 = jpeg->width*2;//bytes each row dest
    size_t t = y * jw; //start byte of row y source
    size_t t2 = y * jw2;//start byte of row y dest
    size_t b = t + (h * jw);//end byte of block end row
    size_t l = x * 2;//start byte of dest column
    uint8_t *out = jpeg->output+jpeg->data_offset;
    uint8_t *o = out;//current point
    size_t iy, iy2, ix, ix2;

    w = w * 3;

    for(iy=t, iy2=t2; iy<b; iy+=jw, iy2+=jw2) {//start row to end row
        o = out+iy2+l;
        for(ix2=ix=0; ix<w; ix+= 3, ix2 +=2) {//start column to end column
            uint16_t r = data[ix];
            uint16_t g = data[ix+1];
            uint16_t b = data[ix+2];
            uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            o[ix2] = c>>8;
            o[ix2+1] = c&0xff;
        }
        data+=w;
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

bool jpg2rgb565(const uint8_t *src, size_t src_len, uint8_t * out, jpg_scale_t scale)
{
    rgb_jpg_decoder jpeg;
    jpeg.width = 0;
    jpeg.height = 0;
    jpeg.input = src;
    jpeg.output = out;
    jpeg.data_offset = 0;

    if(esp_jpg_decode(src_len, scale, _jpg_read, _rgb565_write, (void*)&jpeg) != ESP_OK){
        return false;
    }
    return true;
}
