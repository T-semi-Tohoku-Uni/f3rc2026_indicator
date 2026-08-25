#ifndef BUZZER_H
#define BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h> // bool
#include "barSegLED.h" // BarLED_ToggleAnimate()

#include "stm32g4xx_hal.h"


typedef struct // Note再生要求
{
    bool isPercussion;
    uint8_t  pitchIndex;
    int durationMs;
} SoundRequest;



typedef struct
{
    volatile uint8_t sounderTimNo;
    volatile int stopTimer1000Hz; // 残り再生時間（ms単位）
    volatile int playingTimer1000Hz; // 再生中の時間（ms単位）
    volatile uint8_t playingPitchIndex;
} SounderData;

// 3. 空・満杯判定
bool SoundQueue_IsEmpty();
bool SoundQueue_IsFull();
// 4. Push（シーケンサ側）
bool SoundQueue_Push(SoundRequest req);
// 5. Pop（音源ドライバ側）
bool SoundQueue_Pop(SoundRequest *out);

void SetSound_Flag(bool flag); // デバッグ
void InitBuzzer_C();
void InitSounderData();
// 時間管理（タイマレイヤ）
void BuzzerCallback1000Hz();
// シーケンサ（譜面解釈レイヤ）
void Sequencer_Update(void);
// 音源ドライバ（実際の発音レイヤ）
void SoundDriver_Update(void);
int GetAvailableSounderIdx(uint8_t requestPitchIndex);
void PeriodSet(float frequency, uint8_t timerNo);
void Sounder_OFF(uint8_t timerNo);
void SetPulseBuzzer();
void Buzzer_ON();
void Buzzer_OFF();
void All_BuzzerSounder_OFF();
void Toggle_sound_flag();
int8_t GetSoundNameFromPitchIndex(uint8_t pitchIndex, uint8_t *alphaPitch, int8_t *numPitch);


#ifdef __cplusplus
}
#endif
#endif /* BUZZER_H */