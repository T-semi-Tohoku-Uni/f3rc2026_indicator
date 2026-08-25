#include "main.h"
#include "tapeLED.h"

#include <stdio.h>

#define NEOPIXEL_ONE 67
#define NEOPIXEL_ZERO 33
#define NUM_PIXELS 92
#define DMA_BUFF_SIZE (NUM_PIXELS * 24 + 1)

extern TIM_HandleTypeDef htim1; // 旧テープLEDコネクタ (タイマー1 チャンネル4) 用
extern TIM_HandleTypeDef htim2; // テープLED (タイマー2 チャンネル1) 用

uint16_t tapeLEDcount = 0;
uint16_t colorChangeInterval = 900; // 色変更の間隔（1000Hzタイマーのカウント数）
PixelRGB_t pixel[NUM_PIXELS] = {0};
uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};
uint32_t *pBuff;
int i, j;

PixelRGB_t testPixel1 = {0};
PixelRGB_t testPixel2 = {0};
PixelRGB_t testPixel11 = {0};
PixelRGB_t testPixel12 = {0};
PixelRGB_t testPixel21 = {0};
PixelRGB_t testPixel22 = {0};
PixelRGB_t testPixel31 = {0};
PixelRGB_t testPixel32 = {0};

uint16_t tapeLEDtimer1000Hz = 0;           // 1000Hzタイマー
const uint16_t tapeLEDinterval1000Hz = 50; // 1000Hzタイマーでの更新間隔


// 青 水 aky
// 緑 紫 wtn
// // mk rin 平山
//     SetLEDColor(&testPixel11, 0, 100, 25);
//     SetLEDColor(&testPixel12, 80, 65, 10);
// 紫 黄 ymd
// 青 白 otk
// 青 桃 knk
// 橙 白 tkn 245 172 139
// 赤 紫 ide 
void SetLEDColor(PixelRGB_t *pixel, uint8_t r, uint8_t g, uint8_t b)
{
    pixel->color.r = r;
    pixel->color.g = g;
    pixel->color.b = b;
}

void SetUpTapeLED()
{
    SetLEDColor(&testPixel11, 255, 100, 10);
    SetLEDColor(&testPixel12, 255, 0, 80);

    SetLEDColor(&testPixel21, 200, 39, 30);
    SetLEDColor(&testPixel22, 10, 255, 100);

    SetLEDColor(&testPixel31, 90, 40, 135);
    SetLEDColor(&testPixel32, 30, 30, 255);
}

void TapeLEDCallback1000Hz()
{
    tapeLEDtimer1000Hz++;
    if (tapeLEDtimer1000Hz >= tapeLEDinterval1000Hz)
    {
        tapeLEDtimer1000Hz = 0;
        // TapeLEDevent();
        TapeLED2colorChangeEvent();
        TapeLEDeventFlow();
    }
}

void TapeLED2colorChangeEvent()
{
    tapeLEDcount++;

    if (tapeLEDcount % colorChangeInterval < (colorChangeInterval / 3))
    {
        testPixel1 = testPixel11;
        testPixel2 = testPixel12;
    }
    else if (tapeLEDcount % colorChangeInterval < (2 * colorChangeInterval / 3))
    {
        testPixel1 = testPixel21;
        testPixel2 = testPixel22;
    }
    else
    {
        testPixel1 = testPixel31;
        testPixel2 = testPixel32;
    }
}


void TapeLEDevent()
{
    
    for (i = 0; i < NUM_PIXELS; i++)
    {
        pixel[i].data = 0;
    }
    for (i = 0; i < NUM_PIXELS; i++)
    {
        if ((tapeLEDcount % 2 == 0 && i % 2 == 0) || (tapeLEDcount % 2 == 1 && i % 2 == 1))
        {
            pixel[i].data = testPixel1.data;
        }
        else
        {
            pixel[i].data = testPixel2.data;
        }
    }
    // pixel[0].color.g = (66 + tapeLEDcount*3) % 100 + 200;
    // pixel[0].color.r = (33 +tapeLEDcount*3) % 100 + 200;
    // pixel[0].color.b = (0 + tapeLEDcount*3) % 100 + 200;

    PushTapeLEDData();
}

void TapeLEDeventExpand() // 真ん中から左右それぞれの方向に2色が流れる
{
    // tapeLED

    uint8_t middlePixelIndexL = NUM_PIXELS / 2 - 1;
    uint8_t middlePixelIndexR = NUM_PIXELS / 2;

    // 前フレームのピクセルを中央から左に移動
    for (i = 0; i < middlePixelIndexL; i++)
    {
        pixel[i].data = pixel[i + 1].data;
    }
    // 前フレームのピクセルを中央から右に移動
    for (i = NUM_PIXELS - 1; i > middlePixelIndexR; i--)
    {
        pixel[i].data = pixel[i - 1].data;
    }

    float brightnessMultiplier1 = (19 - (tapeLEDcount % 20)) / 20.0f ; // 値をループ
    float brightnessMultiplier2 = (19 - ((tapeLEDcount+10) % 20)) / 20.0f ; // 値をループ
    if (brightnessMultiplier1 < 0.2f) brightnessMultiplier1 = 0.2f; // 最低でも20%の明るさを保つ
    if (brightnessMultiplier2 < 0.2f) brightnessMultiplier2 = 0.2f; // 最低でも20%の明るさを保つ
    uint8_t r1 = testPixel1.color.r;
    uint8_t g1 = testPixel1.color.g;
    uint8_t b1 = testPixel1.color.b;
    uint8_t r2 = testPixel2.color.r;
    uint8_t g2 = testPixel2.color.g;
    uint8_t b2 = testPixel2.color.b;
    pixel[middlePixelIndexL].color.r = (uint8_t)(r1 * brightnessMultiplier1);
    pixel[middlePixelIndexL].color.g = (uint8_t)(g1 * brightnessMultiplier1);
    pixel[middlePixelIndexL].color.b = (uint8_t)(b1 * brightnessMultiplier1);
    pixel[middlePixelIndexR].color.r = (uint8_t)(r2 * brightnessMultiplier2);
    pixel[middlePixelIndexR].color.g = (uint8_t)(g2 * brightnessMultiplier2);
    pixel[middlePixelIndexR].color.b = (uint8_t)(b2 * brightnessMultiplier2);

    PushTapeLEDData();
}

PixelRGB_t FlowPixel = {0};
void TapeLEDeventFlow() // 端から2色交互にまっすぐ流れる
{
    for (i = NUM_PIXELS - 1; i > 0; i--)
    {
        pixel[i].data = pixel[i - 1].data;
    }

    uint8_t loopLedCount = 25;

    if (tapeLEDcount % (loopLedCount*2) == 0){
        if (tapeLEDcount % (loopLedCount * 4) == 0)
        {
            FlowPixel = testPixel1;
        }
        else
        {
            FlowPixel = testPixel2;
        }
    }
    
    float brightnessMultiplier1 = (loopLedCount - 1  - (tapeLEDcount % loopLedCount)) / (float)(loopLedCount - 1); // 値をループ
    // if (brightnessMultiplier1 < 0.2f) brightnessMultiplier1 = 0.2f; // 最低でも20%の明るさを保つ
    uint8_t r1 = FlowPixel.color.r;
    uint8_t g1 = FlowPixel.color.g;
    uint8_t b1 = FlowPixel.color.b;
    pixel[0].color.r = (uint8_t)(r1 * brightnessMultiplier1);
    pixel[0].color.g = (uint8_t)(g1 * brightnessMultiplier1);
    pixel[0].color.b = (uint8_t)(b1 * brightnessMultiplier1);

    PushTapeLEDData();
}

void TapeLEDeventFlowFlash() // 端から2色交互にまっすぐ流れる まぶしい
{
    for (i = NUM_PIXELS - 1; i > 0; i--)
    {
        pixel[i].data = pixel[i - 1].data;
    }

    uint8_t loopLedCount = 25;

    PixelRGB_t newPixel;
    if (tapeLEDcount % 2 == 0)
    {
        newPixel = testPixel1;
    }
    else
    {
        newPixel = testPixel2;
    }

    float brightnessMultiplier1 = (loopLedCount - 1  - (tapeLEDcount % loopLedCount)) / (float)(loopLedCount - 1); // 値をループ
    if (brightnessMultiplier1 < 0.2f) brightnessMultiplier1 = 0.2f; // 最低でも20%の明るさを保つ
    uint8_t r1 = newPixel.color.r;
    uint8_t g1 = newPixel.color.g;
    uint8_t b1 = newPixel.color.b;
    pixel[0].color.r = (uint8_t)(r1 * brightnessMultiplier1);
    pixel[0].color.g = (uint8_t)(g1 * brightnessMultiplier1);
    pixel[0].color.b = (uint8_t)(b1 * brightnessMultiplier1);

    PushTapeLEDData();
}

PixelRGB_t GlowPixel = {0};
void TapeLEDeventGlow() // 全体がゆっくり明滅する
{
    uint8_t loopLedCount = 25;

    if (tapeLEDcount % (loopLedCount*2) == 0){
        if (tapeLEDcount % (loopLedCount * 4) == 0)
        {
            GlowPixel = testPixel1;
        }
        else
        {
            GlowPixel = testPixel2;
        }
    }

    float brightnessMultiplier1 = (loopLedCount - 1  - (tapeLEDcount % loopLedCount)) / (float)(loopLedCount - 1); // 値をループ
    if (brightnessMultiplier1 < 0.2f) brightnessMultiplier1 = 0.2f; // 最低でも20%の明るさを保つ
    for (i = 0; i < NUM_PIXELS; i++)
    {
        uint8_t r1 = GlowPixel.color.r;
        uint8_t g1 = GlowPixel.color.g;
        uint8_t b1 = GlowPixel.color.b;
        pixel[i].color.r = (uint8_t)(r1 * brightnessMultiplier1);
        pixel[i].color.g = (uint8_t)(g1 * brightnessMultiplier1);
        pixel[i].color.b = (uint8_t)(b1 * brightnessMultiplier1);
    }

    PushTapeLEDData();
}


void PushTapeLEDData()
{
    pBuff = dmaBuffer;
    for (i = 0; i < NUM_PIXELS; i++)
    {
        for (j = 23; j >= 0; j--)
        {
            if ((pixel[i].data >> j) & 0x01)
            {
                *pBuff = NEOPIXEL_ONE;
            }
            else
            {
                *pBuff = NEOPIXEL_ZERO;
            }
            pBuff++;
        }
    }
    dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0!

    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, dmaBuffer, DMA_BUFF_SIZE);
}
