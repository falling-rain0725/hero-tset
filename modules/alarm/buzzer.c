#include "buzzer.h"

#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "buzzer_music.h"

#include <stddef.h>

#define BUZZER_MUSIC_END       0xFFu
#define BUZZER_NOTE_GAP_MS     5.0f
#define BUZZER_DEFAULT_VOLUME  0.01f

typedef enum
{
    BUZZER_PHASE_START_NOTE = 0,
    BUZZER_PHASE_PLAY_NOTE,
    BUZZER_PHASE_NOTE_GAP,
} BuzzerPhase_e;

typedef struct
{
    const uint8_t *score;
    float note_unit_ms;
    uint8_t priority;
    float volume;
} BuzzerMusicConfig_s;

typedef struct
{
    uint8_t active;
    uint8_t play_forever;
    uint8_t play_once;
    uint16_t score_index;
    BuzzerPhase_e phase;
    float expire_time_ms;
    float phase_deadline_ms;
} BuzzerMusicState_s;

static PWMInstance *buzzer_pwm;
static int8_t current_music = -1;
static BuzzerMusicState_s music_states[BUZZER_MUSIC_COUNT];

/* 后续新增音乐时，在此集中设置乐谱、音量和抢占优先级，数值越大优先级越高。 */
static const BuzzerMusicConfig_s music_configs[BUZZER_MUSIC_COUNT] = {
    [BUZZER_MUSIC_HARUHIKAGE] = {
        .score = buzzer_music_haruhikage,
        .note_unit_ms = 62.5f,
        .priority = 10,
        .volume = BUZZER_DEFAULT_VOLUME,
    },
    [BUZZER_MUSIC_BAD_APPLE] = {
        .score = buzzer_music_bad_apple,
        .note_unit_ms = 54.3478f,
        .priority = 10,
        .volume = BUZZER_DEFAULT_VOLUME,
    },
    [BUZZER_MUSIC_IMPERIAL_MARCH] = {
        .score = buzzer_music_imperial_march,
        .note_unit_ms = 100.0f,
        .priority = 10,
        .volume = BUZZER_DEFAULT_VOLUME,
    },
    [BUZZER_MUSIC_SEE_YOU_AGAIN] = {
        .score = buzzer_music_see_you_again,
        .note_unit_ms = 96.153846f,
        .priority = 10,
        .volume = BUZZER_DEFAULT_VOLUME,
    },
    [BUZZER_MUSIC_SEE_YOU_AGAIN_2_SPEED] = {
        .score = buzzer_music_see_you_again,
        .note_unit_ms = 48,
        .priority = 10,
        .volume = BUZZER_DEFAULT_VOLUME,
    },
    [BUZZER_MUSIC_STARTUP] = {
        .score = buzzer_music_startup,
        .note_unit_ms = 100.0f,
        .priority = 1,
        .volume = BUZZER_DEFAULT_VOLUME,
    },
};

static uint8_t BuzzerMusicIsValid(BuzzerMusic_e music)
{
    return (uint32_t)music < (uint32_t)BUZZER_MUSIC_COUNT;
}

static void BuzzerOutput(uint16_t frequency_hz, float volume)
{
    if (buzzer_pwm == NULL)
        return;

    if (frequency_hz == 0u || volume <= 0.0f)
    {
        PWMSetDutyRatio(buzzer_pwm, 0.0f);
        return;
    }

    if (volume > 0.5f)
        volume = 0.5f;

    PWMSetPeriod(buzzer_pwm, 1.0f / (float)frequency_hz);
    PWMSetDutyRatio(buzzer_pwm, volume);
}

static void BuzzerResetPlayback(BuzzerMusic_e music)
{
    music_states[music].score_index = 0u;
    music_states[music].phase = BUZZER_PHASE_START_NOTE;
    music_states[music].phase_deadline_ms = 0.0f;
}

static void BuzzerExpireTimedMusic(float now_ms)
{
    for (size_t i = 0; i < BUZZER_MUSIC_COUNT; ++i)
    {
        BuzzerMusicState_s *state = &music_states[i];

        if (state->active && !state->play_forever && !state->play_once &&
            now_ms >= state->expire_time_ms)
        {
            state->active = 0u;
            state->play_once = 0u;
            BuzzerResetPlayback((BuzzerMusic_e)i);
        }
    }
}

static int8_t BuzzerSelectMusic(void)
{
    int8_t selected = -1;
    uint8_t selected_priority = 0u;

    for (size_t i = 0; i < BUZZER_MUSIC_COUNT; ++i)
    {
        if (!music_states[i].active)
            continue;

        if (selected < 0 || music_configs[i].priority > selected_priority)
        {
            selected = (int8_t)i;
            selected_priority = music_configs[i].priority;
        }
    }

    return selected;
}

static void BuzzerAdvanceMusic(BuzzerMusic_e music, float now_ms)
{
    BuzzerMusicState_s *state = &music_states[music];
    const BuzzerMusicConfig_s *config = &music_configs[music];

    switch (state->phase)
    {
    case BUZZER_PHASE_START_NOTE:
    {
        uint8_t note = config->score[state->score_index];

        if (note == BUZZER_MUSIC_END)
        {
            if (state->play_once)
            {
                BuzzerStop(music);
                return;
            }

            state->score_index = 0u;
            note = config->score[0];
        }

        const uint8_t duration = config->score[state->score_index + 1u];
        const uint16_t frequency =
            note < NOTE_COUNT ? buzzer_note_frequencies[note] : 0u;

        BuzzerOutput(frequency, config->volume);
        state->phase_deadline_ms =
            now_ms + (float)duration * config->note_unit_ms;
        state->phase = BUZZER_PHASE_PLAY_NOTE;
        break;
    }

    case BUZZER_PHASE_PLAY_NOTE:
        if (now_ms >= state->phase_deadline_ms)
        {
            BuzzerOutput(0u, 0.0f);
            state->phase_deadline_ms = now_ms + BUZZER_NOTE_GAP_MS;
            state->phase = BUZZER_PHASE_NOTE_GAP;
        }
        break;

    case BUZZER_PHASE_NOTE_GAP:
        if (now_ms >= state->phase_deadline_ms)
        {
            state->score_index += 2u;
            state->phase = BUZZER_PHASE_START_NOTE;
        }
        break;

    default:
        BuzzerResetPlayback(music);
        break;
    }
}

void BuzzerInit(void)
{
    if (buzzer_pwm != NULL)
    {
        BuzzerStopAll();
        return;
    }

    PWM_Init_Config_s buzzer_config = {
        .htim = &htim4,
        .channel = TIM_CHANNEL_3,
        .dutyratio = 0.0f,
        .period = 0.001f,
    };

    buzzer_pwm = PWMRegister(&buzzer_config);
    BuzzerStopAll();
}

void BuzzerPlay(BuzzerMusic_e music, float duration_s)
{
    if (!BuzzerMusicIsValid(music))
        return;

    if (duration_s == 0.0f)
    {
        BuzzerStop(music);
        return;
    }

    if (duration_s < 0.0f &&
        duration_s != BUZZER_PLAY_FOREVER &&
        duration_s != BUZZER_PLAY_ONCE)
        return;

    BuzzerMusicState_s *state = &music_states[music];

    if (!state->active)
    {
        state->active = 1u;
        BuzzerResetPlayback(music);
    }

    if (duration_s == BUZZER_PLAY_FOREVER)
    {
        state->play_forever = 1u;
        state->play_once = 0u;
    }
    else if (duration_s == BUZZER_PLAY_ONCE)
    {
        if (!state->play_forever)
            state->play_once = 1u;
    }
    else if (!state->play_forever && !state->play_once)
    {
        /* 续期而非累加：停止调用duration_s秒后，音乐才会失效。 */
        state->expire_time_ms =
            DWT_GetTimeline_ms() + duration_s * 1000.0f;
    }
}

void BuzzerStop(BuzzerMusic_e music)
{
    if (!BuzzerMusicIsValid(music))
        return;

    music_states[music].active = 0u;
    music_states[music].play_forever = 0u;
    music_states[music].play_once = 0u;
    BuzzerResetPlayback(music);

    if (current_music == (int8_t)music)
    {
        current_music = -1;
        BuzzerOutput(0u, 0.0f);
    }
}

void BuzzerStopAll(void)
{
    for (size_t i = 0; i < BUZZER_MUSIC_COUNT; ++i)
    {
        music_states[i].active = 0u;
        music_states[i].play_forever = 0u;
        music_states[i].play_once = 0u;
        music_states[i].expire_time_ms = 0.0f;
        BuzzerResetPlayback((BuzzerMusic_e)i);
    }

    current_music = -1;
    BuzzerOutput(0u, 0.0f);
}

uint8_t BuzzerIsPlaying(BuzzerMusic_e music)
{
    if (!BuzzerMusicIsValid(music))
        return 0u;

    return music_states[music].active;
}

void BuzzerTask(void)
{
    if (buzzer_pwm == NULL)
        return;

    const float now_ms = DWT_GetTimeline_ms();
    BuzzerExpireTimedMusic(now_ms);

    const int8_t selected_music = BuzzerSelectMusic();

    if (selected_music < 0)
    {
        current_music = -1;
        BuzzerOutput(0u, 0.0f);
        return;
    }

    if (selected_music != current_music)
    {
        current_music = selected_music;
        BuzzerResetPlayback((BuzzerMusic_e)current_music);
    }

    BuzzerAdvanceMusic((BuzzerMusic_e)current_music, now_ms);
}
