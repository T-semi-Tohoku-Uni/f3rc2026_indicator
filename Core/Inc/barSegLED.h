#ifndef BARSEGLED_H
#define BARSEGLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

// BarLED
void BarLED_Display(uint8_t num);
int BarLED_ToggleAnimate(uint8_t one_zero);

// 7セグメント表示
void SevenSeg_Display_AllOff();
void SevenSeg_Display_Hyphen();
void SevenSeg_Display_Number(int8_t num, uint8_t segNo);
void SevenSeg_Display_Slider(uint8_t num);
void SevenSeg_Display_Alphabet_char(char alphabet, uint8_t segNo);
void SevenSeg_Display_Alphabet_idx(uint8_t index, uint8_t segNo);
int SevenSeg_ToggleAnimate_Number1(uint8_t one_zero);
int SevenSeg_ToggleAnimate_Number2(uint8_t one_zero);
int SevenSeg_ToggleAnimate_Slider(uint8_t one_zero);
#ifdef __cplusplus
}
#endif
#endif /* BARSEGLED_H */