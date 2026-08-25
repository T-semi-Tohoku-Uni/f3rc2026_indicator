#ifndef SCOREMANAGER_H
#define SCOREMANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h> // bool

#include "stm32g4xx_hal.h"

typedef struct
{
    volatile float bpm;
    volatile uint16_t ticksPerBeat;
    volatile uint8_t octaveShift;
    volatile bool isOrchestra;
} ScoreSettings;
typedef struct
{
    volatile uint16_t startTick;
    volatile bool isPercussion;
    volatile uint8_t pitchIndex;
    volatile uint16_t durationTick;
} Note;
typedef struct
{
    Note *noteData;
    volatile ScoreSettings *scoreSettings;
    volatile uint16_t currentNoteIndex;
    volatile float ticksPerMs;
    volatile float msPerTick;
} ScorePlayData;

void SetPlayData(ScorePlayData *playData_, uint8_t scoreNo);
void GetScorePlayData(ScorePlayData *playData_, Note *noteData_, ScoreSettings *scoreSettings_);
int ConvertTickToMs(uint16_t ticks, ScorePlayData playData_);
uint16_t ConvertMsToTick(int ms, ScorePlayData playData_);

#ifdef __cplusplus
}
#endif
#endif /* SCOREMANAGER_H */
