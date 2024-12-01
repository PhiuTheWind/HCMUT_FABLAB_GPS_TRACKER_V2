#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_mac.h"

#define INT_PIN GPIO_NUM_2         // Chân INT của MPU6050 nối với GPIO2 của ESP32
#define GPIO_GPS_TRIGGER GPIO_NUM_13
#define GPIO_SIM_TRIGGER GPIO_NUM_12
#define GPIO_PEN GPIO_NUM_32
//#define TIME_TO_SLEEP 15           // Thời gian Deep Sleep (giây)
#define TIME_TO_WAKE 15            // Thời gian Awake (giây)

Adafruit_MPU6050 mpu; // Đối tượng MPU6050

// Hàm in lý do đánh thức
void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            printf("Mạch được đánh thức bởi ngắt từ MPU6050!\n");
            break;
        default:
            printf("ESP32 đánh thức vì lý do không xác định.\n");
            break;
    }
}

extern "C" void app_main() {
    // Khởi tạo Serial
    Serial.begin(115200);
    printf("Initializing...\n");

    // In lý do đánh thức
    print_wakeup_reason();

    // Khởi tạo MPU6050
    if (!mpu.begin()) {
        printf("Failed to find MPU6050 chip\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    printf("MPU6050 Found!\n");

    // Cấu hình phát hiện chuyển động
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(1);
    mpu.setInterruptPinPolarity(true);
    mpu.setInterruptPinLatch(false);
    mpu.setMotionInterrupt(true);

    printf("Motion detection setup complete.\n");

    // Cấu hình RTC GPIOs
    rtc_gpio_init(GPIO_GPS_TRIGGER);
    rtc_gpio_set_direction(GPIO_GPS_TRIGGER, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_init(GPIO_SIM_TRIGGER);
    rtc_gpio_set_direction(GPIO_SIM_TRIGGER, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_init(GPIO_PEN);
    rtc_gpio_set_direction(GPIO_PEN, RTC_GPIO_MODE_OUTPUT_ONLY);

    // Kiểm tra lý do đánh thức
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        // Khi được đánh thức: Tắt GPS trigger, bật SIM trigger và GPIO (PEN)
        rtc_gpio_hold_dis(GPIO_GPS_TRIGGER);
        rtc_gpio_set_level(GPIO_GPS_TRIGGER, 0);

        rtc_gpio_hold_dis(GPIO_SIM_TRIGGER);
        rtc_gpio_set_level(GPIO_SIM_TRIGGER, 1);

        rtc_gpio_hold_dis(GPIO_PEN);
        rtc_gpio_set_level(GPIO_PEN, 1);

        printf("ESP32 woke up: Turned off GPIO_GPS_TRIGGER, turned on GPIO_SIM_TRIGGER and GPIO_PEN.\n");

        // Giả lập nhiệm vụ khi thức dậy trong 15 giây
        vTaskDelay(pdMS_TO_TICKS(TIME_TO_WAKE * 1000));
    } else {
        // Lần đầu tiên khởi động
        printf("ESP32 booted for the first time.\n");
    }

    // Trước khi đi vào chế độ sleep: Bật GPS trigger, tắt SIM trigger và GPIO (PEN)
    rtc_gpio_set_level(GPIO_GPS_TRIGGER, 1);
    rtc_gpio_hold_en(GPIO_GPS_TRIGGER);

    rtc_gpio_set_level(GPIO_SIM_TRIGGER, 0);
    rtc_gpio_hold_en(GPIO_SIM_TRIGGER);

    rtc_gpio_set_level(GPIO_PEN, 0);
    rtc_gpio_hold_en(GPIO_PEN);

    printf("ESP32 preparing for sleep: Turned on GPIO_GPS_TRIGGER, turned off GPIO_SIM_TRIGGER and GPIO_PEN.\n");

    // Cấu hình Deep Sleep timer
    //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000); // Sleep for 15 seconds
    esp_sleep_enable_ext0_wakeup(INT_PIN, 0);              // Đánh thức khi INT_PIN = LOW

    printf("ESP32 entering Deep Sleep...\n");

    // Đảm bảo các dữ liệu đã được ghi ra Serial trước khi ngủ
    Serial.flush();

    // Đi vào chế độ Deep Sleep
    esp_deep_sleep_start();
}
