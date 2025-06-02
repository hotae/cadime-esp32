#include "sound.h"

void playTone(int frequency, int duration) {
    // 주파수 설정
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, frequency);
    
    // 듀티 사이클을 설정하여 소리 출력
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // 음을 지정된 시간만큼 재생
    vTaskDelay(duration / portTICK_PERIOD_MS);

    // 소리 끄기
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    
    // 다음 음을 위한 약간의 쉬는 시간
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void playSong() {
    playTone(NOTE_E4, QUARTER);  // 미
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_C4, QUARTER);  // 도
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_E4, QUARTER);  // 미
    playTone(NOTE_E4, QUARTER);  // 미
    playTone(NOTE_E4, QUARTER);  // 미
    
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_E4, QUARTER);  // 미
    playTone(NOTE_E4, QUARTER);  // 미
    playTone(NOTE_E4, QUARTER);  // 미

    playTone(NOTE_E4, QUARTER);  // 미
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_C4, QUARTER);  // 도
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_E4, QUARTER);  // 미
    playTone(NOTE_E4, QUARTER);  // 미
    playTone(NOTE_E4, QUARTER);  // 미
    
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_E4, QUARTER);  // 미
    playTone(NOTE_D4, QUARTER);  // 레
    playTone(NOTE_C4, QUARTER);  // 도
}   