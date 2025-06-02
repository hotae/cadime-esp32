#ifndef __SOUND_H
#define __SOUND_H

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY       4000
#define LEDC_FREQUENCY  2000

#define NOTE_C4  261
#define NOTE_D4  293
#define NOTE_E4  329
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  493
#define NOTE_C5  523

// 음 길이 정의 (기본 단위는 밀리초)
#define WHOLE   1000
#define HALF    500
#define QUARTER 250

void playTone(int frequency, int duration);
void playSong();

#endif