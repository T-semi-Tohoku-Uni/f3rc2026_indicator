#ifndef TAPELED_H
#define TAPELED_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32g4xx_hal.h"

typedef union
{
    struct
    {
        uint8_t b;
        uint8_t r;
        uint8_t g;
    } color;
    uint32_t data;
} PixelRGB_t;

void SetLEDColor(PixelRGB_t *pixel, uint8_t r, uint8_t g, uint8_t b);
void SetUpTapeLED();
void TapeLEDCallback1000Hz();
void TapeLED2colorChangeEvent();
void TapeLEDevent();
void TapeLEDeventExpand();
void TapeLEDeventFlow();
void TapeLEDeventFlowFlash();
void TapeLEDeventGlow();
void PushTapeLEDData();


#ifdef __cplusplus
}
#endif
#endif /* TAPELED_H */