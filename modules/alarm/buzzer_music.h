#ifndef BUZZER_MUSIC_H
#define BUZZER_MUSIC_H

#include <stdint.h>

typedef enum
{
    NOTE_P = 0,
    NOTE_A0, NOTE_A0S, NOTE_B0,
    NOTE_C1, NOTE_C1S, NOTE_D1, NOTE_D1S, NOTE_E1, NOTE_F1, NOTE_F1S, NOTE_G1, NOTE_G1S, NOTE_A1, NOTE_A1S, NOTE_B1,
    NOTE_C2, NOTE_C2S, NOTE_D2, NOTE_D2S, NOTE_E2, NOTE_F2, NOTE_F2S, NOTE_G2, NOTE_G2S, NOTE_A2, NOTE_A2S, NOTE_B2,
    NOTE_C3, NOTE_C3S, NOTE_D3, NOTE_D3S, NOTE_E3, NOTE_F3, NOTE_F3S, NOTE_G3, NOTE_G3S, NOTE_A3, NOTE_A3S, NOTE_B3,
    NOTE_C4, NOTE_C4S, NOTE_D4, NOTE_D4S, NOTE_E4, NOTE_F4, NOTE_F4S, NOTE_G4, NOTE_G4S, NOTE_A4, NOTE_A4S, NOTE_B4,
    NOTE_C5, NOTE_C5S, NOTE_D5, NOTE_D5S, NOTE_E5, NOTE_F5, NOTE_F5S, NOTE_G5, NOTE_G5S, NOTE_A5, NOTE_A5S, NOTE_B5,
    NOTE_C6, NOTE_C6S, NOTE_D6, NOTE_D6S, NOTE_E6, NOTE_F6, NOTE_F6S, NOTE_G6, NOTE_G6S, NOTE_A6, NOTE_A6S, NOTE_B6,
    NOTE_C7, NOTE_C7S, NOTE_D7, NOTE_D7S, NOTE_E7, NOTE_F7, NOTE_F7S, NOTE_G7, NOTE_G7S, NOTE_A7, NOTE_A7S, NOTE_B7,
    NOTE_C8,
    NOTE_COUNT
} BuzzerNote_e;

extern const uint16_t buzzer_note_frequencies[NOTE_COUNT];
extern const uint8_t buzzer_music_haruhikage[];
extern const uint8_t buzzer_music_bad_apple[];
extern const uint8_t buzzer_music_imperial_march[];
extern const uint8_t buzzer_music_see_you_again[];
extern const uint8_t buzzer_music_startup[];

#endif
