#ifndef MAIN_H
#define MAIN_H

// Include necessary libraries
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <esp_wifi.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "my_timer.h"

// Define GPIO pins
#define INT_PIN            GPIO_NUM_2
#define GPIO_GPS_TRIGGER   GPIO_NUM_13
#define GPIO_SIM_TRIGGER   GPIO_NUM_12
#define GPIO_PEN           GPIO_NUM_32
#define GPIO_GPS_PPS     GPIO_NUM_4
// Define sleep and wake durations (in seconds)
#define TIME_TO_WAKE       10
// #define TIME_TO_SLEEP    15 // Uncomment and use if you plan to use TIME_TO_SLEEP

// Declare external variables
extern Adafruit_MPU6050 mpu;
extern int wake_count;

// Function prototypes

// Hàm thiết lập mức GPIO
void set_gpio_level(gpio_num_t gpio, int level);

// Hàm in lý do thức dậy
void print_wakeup_reason();

// Hàm dọn dẹp GPIO trước khi vào chế độ ngủ
void set_sleep_gpio();
#endif // MAIN_H
