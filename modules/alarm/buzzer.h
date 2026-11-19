#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

#define BUZZER_PLAY_FOREVER (-1.0f)
#define BUZZER_PLAY_ONCE    (-2.0f)

typedef enum
{
    BUZZER_MUSIC_HARUHIKAGE = 0,
    BUZZER_MUSIC_BAD_APPLE,
    BUZZER_MUSIC_IMPERIAL_MARCH,
    BUZZER_MUSIC_SEE_YOU_AGAIN,
    BUZZER_MUSIC_SEE_YOU_AGAIN_2_SPEED,
    BUZZER_MUSIC_STARTUP,
    BUZZER_MUSIC_COUNT
} BuzzerMusic_e;

void BuzzerInit(void);

/**
 * @brief 播放或续期一首报警音乐
 * @param music 音乐编号
 * @param duration_s 正数表示最后一次调用后继续播放的秒数；
 *                   BUZZER_PLAY_FOREVER表示无限循环；
 *                   BUZZER_PLAY_ONCE表示完整播放一次；0表示停止
 * @note 同一首音乐在有效期内重复调用只刷新截止时间，不会从头播放
 */
void BuzzerPlay(BuzzerMusic_e music, float duration_s);

void BuzzerStop(BuzzerMusic_e music);
void BuzzerStopAll(void);
uint8_t BuzzerIsPlaying(BuzzerMusic_e music);

/** @brief 非阻塞音乐播放任务，建议每10ms调用一次 */
void BuzzerTask(void);

#endif
