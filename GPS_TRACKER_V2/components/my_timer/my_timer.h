#ifndef MY_TIMER_H
#define MY_TIMER_H

#include <stdbool.h>
#include <stdio.h>
#include "driver/timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"

#ifdef __cplusplus
extern "C" {
#endif

// Nội dung của my_timer.h
extern volatile bool g_timer_done;
void my_timer_init(void);
void my_timer_start(void);
void my_timer_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MY_TIMER_H
