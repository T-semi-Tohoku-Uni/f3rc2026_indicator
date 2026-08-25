#include "main.h"

#include <stdio.h>

#define My_HIGH GPIO_PIN_RESET
#define My_LOW GPIO_PIN_SET

uint16_t BarLED_Pins[10] = {BarLED_PA0_Pin, BarLED_PA1_Pin, BarLED_PC0_Pin, BarLED_PC1_Pin, BarLED_PA4_Pin,
                            BarLED_PA5_Pin, BarLED_PA6_Pin, BarLED_PA7_Pin, BarLED_PA8_Pin, BarLED_PA9_Pin};
GPIO_TypeDef *BarLED_Port[10] = {GPIOA, GPIOA, GPIOC, GPIOC, GPIOA,
                                GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};
// seg1が左 seg2が右
uint16_t SevenSeg_Pins[2][8] = {
    {Seg1_PB7_Pin, Seg1_PB8_Pin, Seg1_PB2_Pin, Seg1_PB1_Pin, Seg1_PB0_Pin, Seg1_PB6_Pin, Seg1_PB5_Pin, Seg1_PB4_Pin},
    {Seg2_PB15_Pin, Seg2_PC13_Pin, Seg2_PB11_Pin, Seg2_PB10_Pin, Seg2_PB9_Pin, Seg2_PB14_Pin, Seg2_PB13_Pin, Seg2_PB12_Pin}};
GPIO_TypeDef *SevenSeg_Port[2][8] = {
    {GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB},
    {GPIOB, GPIOC, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB}};
GPIO_PinState GPIO_STATEs_Number[16][7] = {
    {My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_LOW},  // 0
    {My_LOW, My_HIGH, My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW},      // 1
    {My_HIGH, My_HIGH, My_LOW, My_HIGH, My_HIGH, My_LOW, My_HIGH},   // 2
    {My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_LOW, My_HIGH},   // 3
    {My_LOW, My_HIGH, My_HIGH, My_LOW, My_LOW, My_HIGH, My_HIGH},    // 4
    {My_HIGH, My_LOW, My_HIGH, My_HIGH, My_LOW, My_HIGH, My_HIGH},   // 5
    {My_HIGH, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH},  // 6
    {My_HIGH, My_HIGH, My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW},     // 7
    {My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH}, // 8
    {My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_HIGH, My_HIGH},  // 9
    {My_HIGH, My_HIGH, My_HIGH, My_LOW, My_HIGH, My_HIGH, My_HIGH},  // A
    {My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH},   // b
    {My_HIGH, My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_LOW},    // C
    {My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_HIGH},   // d
    {My_HIGH, My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH},   // E
    {My_HIGH, My_LOW, My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH}     // F
};
GPIO_PinState GPIO_STATEs_Alphabet[26][7] = {
    {My_HIGH, My_HIGH, My_HIGH, My_LOW, My_HIGH, My_HIGH, My_HIGH}, // A
    {My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH},  // B
    {My_HIGH, My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_LOW},   // C
    {My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_HIGH},  // D d
    {My_HIGH, My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH},  // E
    {My_HIGH, My_LOW, My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH},   // F
    {My_HIGH, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_LOW},  // G
    {My_LOW, My_LOW, My_HIGH, My_LOW, My_HIGH, My_HIGH, My_HIGH},   // H h
    {My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW},      // I i
    {My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_LOW},   // J
    {My_HIGH, My_LOW, My_HIGH, My_LOW, My_HIGH, My_HIGH, My_HIGH},  // K
    {My_LOW, My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_LOW},    // L
    {My_HIGH, My_LOW, My_HIGH, My_LOW, My_HIGH, My_LOW, My_HIGH},   // M
    {My_LOW, My_LOW, My_HIGH, My_LOW, My_HIGH, My_LOW, My_HIGH},    // N n
    {My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_HIGH},   // O o
    {My_HIGH, My_HIGH, My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH},  // P
    {My_HIGH, My_HIGH, My_HIGH, My_LOW, My_LOW, My_HIGH, My_HIGH},  // Q
    {My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW, My_HIGH},     // R r
    {My_HIGH, My_HIGH, My_LOW, My_HIGH, My_HIGH, My_LOW, My_LOW},   // S
    {My_LOW, My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH},   // T t
    {My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_LOW},    // U u
    {My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_LOW},  // V
    {My_LOW, My_HIGH, My_LOW, My_HIGH, My_LOW, My_HIGH, My_HIGH},   // W
    {My_HIGH, My_LOW, My_HIGH, My_LOW, My_HIGH, My_HIGH, My_HIGH},  // X
    {My_LOW, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_HIGH, My_HIGH},  // Y y
    {My_HIGH, My_HIGH, My_LOW, My_HIGH, My_HIGH, My_LOW, My_LOW}    // Z
};
GPIO_PinState GPIO_STATEs_HorizontalLine[3][7] = {
    {My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW}, // upper line
    {My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH}, // middle line
    {My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW}  // lower line
};
GPIO_PinState GPIO_STATEs_VerticalLine[4][7] = {
    {My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW}, // left upper line
    {My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW}, // right upper line
    {My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW}, // left lower line
    {My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW}  // right lower line
};
GPIO_PinState GPIO_STATEs_LoadingImage[6][7] = {
    {My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW}, // upper line
    {My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW}, // right upper line
    {My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW}, // right lower line
    {My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW}, // lower line
    {My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW}, // left lower line
    {My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW}  // left upper line
};
GPIO_PinState GPIO_STATEs_LoadingEight[8][7] = {
    {My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW}, // upper line
    {My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW}, // right upper line
    {My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH}, // middle line
    {My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW}, // left lower line
    {My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW}, // lower line
    {My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW, My_LOW}, // right lower line
    {My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH}, // middle line
    {My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW}  // left upper line
};
GPIO_PinState GPIO_STATEs_Slider[9][8] = {
    {My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW},         // 0
    {My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_HIGH},        // .
    {My_LOW, My_LOW, My_LOW, My_HIGH, My_LOW, My_LOW, My_LOW, My_HIGH},       // 1
    {My_LOW, My_LOW, My_LOW, My_HIGH, My_HIGH, My_LOW, My_LOW, My_HIGH},      // 2
    {My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_LOW, My_HIGH},     // 3
    {My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_LOW, My_HIGH, My_HIGH},    // 4
    {My_LOW, My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH},   // 5
    {My_LOW, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH},  // 6
    {My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH}, // 7
};
GPIO_PinState GPIO_STATEs_All_ON[1][7] = {
    {My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH, My_HIGH} // all on
};
GPIO_PinState GPIO_STATEs_All_OFF[1][7] = {
    {My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW, My_LOW} // all off
};

void BarLED_Display(uint8_t num)
{

    if (num > 10)
        num = 10; // 0-10以外は10に補正
    for (uint8_t i = 0; i < 10; i++)
    {
        if (i < num)
            HAL_GPIO_WritePin(BarLED_Port[i], BarLED_Pins[i], My_HIGH);
        else
            HAL_GPIO_WritePin(BarLED_Port[i], BarLED_Pins[i], My_LOW);
    }
}
uint8_t barNum = 0;
int BarLED_ToggleAnimate(uint8_t one_zero)
{
    BarLED_Display(barNum);
    if (one_zero == 1)
        barNum = (barNum + 9) % 11;
    else
        barNum = (barNum + 1) % 11;
    return barNum;
}

// 7セグメント表示
void SevenSeg_Display_AllOff()
{
    for (uint8_t j = 0; j < 2; j++)
    {
        for (uint8_t i = 0; i < 7; i++)
        {
            HAL_GPIO_WritePin(SevenSeg_Port[j][i], SevenSeg_Pins[j][i], GPIO_STATEs_All_OFF[0][i]);
        }
    }
}
void SevenSeg_Display_Hyphen()
{
    for (uint8_t j = 0; j < 2; j++)
    {
        for (uint8_t i = 0; i < 7; i++)
        {
            HAL_GPIO_WritePin(SevenSeg_Port[j][i], SevenSeg_Pins[j][i], GPIO_STATEs_HorizontalLine[1][i]);
        }
    }
}
void SevenSeg_Display_Number(int8_t num, uint8_t segNo)
{
    if (num < 0 || num > 15)
    { // 0-15以外は無視 10~15はA~F
        SevenSeg_Display_Hyphen();
        return;
    }
    if (segNo > 1)
        segNo = 1; // 0か1に補正
    for (uint8_t i = 0; i < 7; i++)
    {
        HAL_GPIO_WritePin(SevenSeg_Port[segNo][i], SevenSeg_Pins[segNo][i], GPIO_STATEs_Number[num][i]);
    }
}
void SevenSeg_Display_Slider(uint8_t num)
{
    if (num < 0 || num > 8)
    { // 0-8以外は無視
        SevenSeg_Display_Hyphen();
        return;
    };
    for (uint8_t j = 0; j < 2; j++)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            HAL_GPIO_WritePin(SevenSeg_Port[j][i], SevenSeg_Pins[j][i], GPIO_STATEs_Slider[num][i]);
        }
    }
}
void SevenSeg_Display_Alphabet_char(char alphabet, uint8_t segNo)
{
    if (alphabet >= 'a' && alphabet <= 'z')
    {
        alphabet = alphabet - 'a' + 'A'; // 小文字を大文字に変換
    }
    if (alphabet < 'A' || alphabet > 'Z')
        return; // A-Z以外は無視
    if (segNo > 1)
        segNo = 1; // 0か1に補正
    for (uint8_t i = 0; i < 7; i++)
    {
        HAL_GPIO_WritePin(SevenSeg_Port[segNo][i], SevenSeg_Pins[segNo][i], GPIO_STATEs_Alphabet[alphabet - 'A'][i]);
    }
}
void SevenSeg_Display_Alphabet_idx(uint8_t index, uint8_t segNo)
{
    if (index < 0 || index > 25)
        return; // 0-25以外は無視
    if (segNo > 1)
        segNo = 1; // 0か1に補正
    for (uint8_t i = 0; i < 7; i++)
    {
        HAL_GPIO_WritePin(SevenSeg_Port[segNo][i], SevenSeg_Pins[segNo][i], GPIO_STATEs_Alphabet[index][i]);
    }
}

uint8_t sevenseg_num1 = 0;
int SevenSeg_ToggleAnimate_Number1(uint8_t one_zero)
{
    SevenSeg_Display_Number(sevenseg_num1, 0);
    if (one_zero == 1)
        sevenseg_num1 = (sevenseg_num1 + 9) % 10;
    else
        sevenseg_num1 = (sevenseg_num1 + 1) % 10;
    return sevenseg_num1;
}
uint8_t sevenseg_num2 = 0;
int SevenSeg_ToggleAnimate_Number2(uint8_t one_zero)
{
    SevenSeg_Display_Number(sevenseg_num2, 1);
    if (one_zero == 1)
        sevenseg_num2 = (sevenseg_num2 + 9) % 10;
    else
        sevenseg_num2 = (sevenseg_num2 + 1) % 10;
    return sevenseg_num2;
}
uint8_t slider_num = 0;
int SevenSeg_ToggleAnimate_Slider(uint8_t one_zero)
{
    SevenSeg_Display_Slider(slider_num);
    if (one_zero == 1)
        slider_num = (slider_num + 8) % 9;
    else
        slider_num = (slider_num + 1) % 9;
    return slider_num;
}