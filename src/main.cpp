/*--------------------------------------------------------------------
Copyright 2020 fukuen

face detection demo is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This software is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with This software.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <Arduino.h>
#include <Maixduino_OV7740.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "kpu.h"
#ifdef __cplusplus
}
#endif
#include "region_layer.h"
#include "image_process.h"
#include "w25qxx.h"
#include "uarths.h"

#define INCBIN_STYLE INCBIN_STYLE_SNAKE
#define INCBIN_PREFIX
#include "incbin.h"

#define PLL0_OUTPUT_FREQ 800000000UL
#define PLL1_OUTPUT_FREQ 400000000UL

#ifdef M5STICKV
#define AXP192_ADDR 0x34
#define PIN_SDA 29
#define PIN_SCL 28
#endif

#define WIDTH 280
#define HEIGHT 240

//OV7740 supports YUV only
Maixduino_OV7740 camera(FRAMESIZE_QVGA, PIXFORMAT_YUV422);
TFT_eSPI lcd;

volatile uint32_t g_ai_done_flag;

kpu_model_context_t face_detect_task;
static region_layer_t face_detect_rl;
static obj_info_t face_detect_info;
#define ANCHOR_NUM 5
static float anchor[ANCHOR_NUM * 2] = {1.889,2.5245,  2.9465,3.94056, 3.99987,5.3658, 5.155437,6.92275, 6.718375,9.01025};

#define  LOAD_KMODEL_FROM_FLASH  0

#if LOAD_KMODEL_FROM_FLASH
#define KMODEL_SIZE (380 * 1024)
uint8_t *model_data;
#else
INCBIN(model, "detect.kmodel");
#endif

uint64_t time_last = 0U;
uint64_t time_now = 0U;
int time_count = 0;

static void ai_done(void *ctx)
{
    g_ai_done_flag = 1;
}

static void draw_edge(uint32_t *gram, obj_info_t *obj_info, uint32_t index, uint16_t color)
{
    uint32_t data = ((uint32_t)color << 16) | (uint32_t)color;
    uint32_t *addr1, *addr2, *addr3, *addr4, x1, y1, x2, y2;

    x1 = obj_info->obj[index].x1;
    y1 = obj_info->obj[index].y1;
    x2 = obj_info->obj[index].x2;
    y2 = obj_info->obj[index].y2;

    if (x1 <= 0)
        x1 = 1;
    if (x2 >= 319)
        x2 = 318;
    if (y1 <= 0)
        y1 = 1;
    if (y2 >= 239)
        y2 = 238;

    addr1 = gram + (320 * y1 + x1) / 2;
    addr2 = gram + (320 * y1 + x2 - 8) / 2;
    addr3 = gram + (320 * (y2 - 1) + x1) / 2;
    addr4 = gram + (320 * (y2 - 1) + x2 - 8) / 2;
    for (uint32_t i = 0; i < 4; i++)
    {
        *addr1 = data;
        *(addr1 + 160) = data;
        *addr2 = data;
        *(addr2 + 160) = data;
        *addr3 = data;
        *(addr3 + 160) = data;
        *addr4 = data;
        *(addr4 + 160) = data;
        addr1++;
        addr2++;
        addr3++;
        addr4++;
    }
    addr1 = gram + (320 * y1 + x1) / 2;
    addr2 = gram + (320 * y1 + x2 - 2) / 2;
    addr3 = gram + (320 * (y2 - 8) + x1) / 2;
    addr4 = gram + (320 * (y2 - 8) + x2 - 2) / 2;
    for (uint32_t i = 0; i < 8; i++)
    {
        *addr1 = data;
        *addr2 = data;
        *addr3 = data;
        *addr4 = data;
        addr1 += 160;
        addr2 += 160;
        addr3 += 160;
        addr4 += 160;
    }
}

bool axp192_init()
{
    Serial.printf("AXP192 init.\n");
    sysctl_set_power_mode(SYSCTL_POWER_BANK3,SYSCTL_POWER_V33);

    Wire.begin((uint8_t) PIN_SDA, (uint8_t) PIN_SCL, 400000);
    Wire.beginTransmission(AXP192_ADDR);
    int err = Wire.endTransmission();
    if (err)
    {
        Serial.printf("Power management ic not found.\n");
        return false;
    }
    Serial.printf("AXP192 found.\n");

    // Clear the interrupts
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x46);
    Wire.write(0xFF);
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x23);
    Wire.write(0x08); //K210_VCore(DCDC2) set to 0.9V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x33);
    Wire.write(0xC1); //190mA Charging Current
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x36);
    Wire.write(0x6C); //4s shutdown
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x91);
    Wire.write(0xF0); //LCD Backlight: GPIO0 3.3V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x90);
    Wire.write(0x02); //GPIO LDO mode
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x28);
    Wire.write(0xF0); //VDD2.8V net: LDO2 3.3V,  VDD 1.5V net: LDO3 1.8V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x27);
    Wire.write(0x2C); //VDD1.8V net:  DC-DC3 1.8V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x12);
    Wire.write(0xFF); //open all power and EXTEN
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x23);
    Wire.write(0x08); //VDD 0.9v net: DC-DC2 0.9V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x31);
    Wire.write(0x03); //Cutoff voltage 3.2V
    Wire.endTransmission();
    Wire.beginTransmission(AXP192_ADDR);
    Wire.write(0x39);
    Wire.write(0xFC); //Turnoff Temp Protect (Sensor not exist!)
    Wire.endTransmission();

    fpioa_set_function(23, (fpioa_function_t)(FUNC_GPIOHS0 + 26));
    gpiohs_set_drive_mode(26, GPIO_DM_OUTPUT);
    gpiohs_set_pin(26, GPIO_PV_HIGH); //Disable VBUS As Input, BAT->5V Boost->VBUS->Charing Cycle

    msleep(20);
    return true;
}

void setup()
{
    /* Set CPU and dvp clk */
    pll_init();
    sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_OUTPUT_FREQ);
    sysctl_pll_set_freq(SYSCTL_PLL1, PLL1_OUTPUT_FREQ);
    plic_init();
    uarths_init();
    Serial.begin(115200);
    axp192_init();

    /* flash init */
    Serial.printf("flash init\n");
    w25qxx_init(3, 0);
    w25qxx_enable_quad_mode();
#if LOAD_KMODEL_FROM_FLASH
    model_data = (uint8_t *)malloc(KMODEL_SIZE + 255);
    uint8_t *model_data_align = (uint8_t *)(((uintptr_t)model_data+255)&(~255));
    w25qxx_read_data(0xA00000, model_data_align, KMODEL_SIZE, W25QXX_QUAD_FAST);
#else
    uint8_t *model_data_align = model_data;
#endif

    /* LCD init */
    lcd.begin();
    lcd.setRotation(1);

    /* DVP init */
    Serial.printf("DVP init\n");
    if (!camera.begin())
    {
        Serial.printf("camera init fail\n");
        while (true) {}
    }
    else
    {
        Serial.printf("camera init success\n");
    }
    camera.run(true);

    /* init face detect model */
    if (kpu_load_kmodel(&face_detect_task, model_data_align) != 0)
    {
        Serial.printf("\nmodel init error\n");
        while (1);
    }
    face_detect_rl.anchor_number = ANCHOR_NUM;
    face_detect_rl.anchor = anchor;
    face_detect_rl.threshold = 0.7;
    face_detect_rl.nms_value = 0.3;
    region_layer_init(&face_detect_rl, 20, 15, 30, camera.width(), camera.height());

    /* enable global interrupt */
    sysctl_enable_irq();

    /* system start */
    Serial.printf("System start\n");
    time_last = sysctl_get_time_us();
    time_now = sysctl_get_time_us();
    time_count = 0;
}

void loop() {
    uint8_t *img = camera.snapshot();
    if (img == nullptr || img==0)
    {
        Serial.printf("snap fail\n");
        return;
    }

    /* run face detect */
    g_ai_done_flag = 0;
    kpu_run_kmodel(&face_detect_task, camera.getRGB888(), DMAC_CHANNEL5, ai_done, NULL);
    while(!g_ai_done_flag);
    float *output;
    size_t output_size;
    kpu_get_output(&face_detect_task, 0, (uint8_t **)&output, &output_size);
    face_detect_rl.input = output;
    region_layer_run(&face_detect_rl, &face_detect_info);

    /* run key point detect */
    for (uint32_t face_cnt = 0; face_cnt < face_detect_info.obj_number; face_cnt++)
    {
        draw_edge((uint32_t *)img, &face_detect_info, face_cnt, TFT_BLUE);
    }

    lcd.pushImage(0, 0, camera.width(), camera.height(), (uint16_t*)img);

    /* display result */
    time_count ++;
    if(time_count % 100 == 0)
    {
        time_now = sysctl_get_time_us();
        Serial.printf("SPF:%fms\n", (time_now - time_last)/1000.0/100);
        time_last = time_now;
    }
}