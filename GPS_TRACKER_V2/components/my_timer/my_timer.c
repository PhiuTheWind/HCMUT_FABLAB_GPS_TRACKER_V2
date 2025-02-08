#include "my_timer.h"

/* 
 * Đặt tên TAG để log cho module này,
 * nếu dùng LOG trong ISR nhớ dùng ESP_EARLY_LOGx 
 */
static const char *TAG = "MY_TIMER_MODULE";
volatile bool g_timer_done = false;

/* 
 * Callback được gọi mỗi khi timer chạm ngưỡng alarm.
 * Lưu ý: ISR nên đặt trong IRAM nếu dùng các hàm log hay xử lý nhanh.
 */
static bool IRAM_ATTR my_timer_isr_callback(void *args)
{
    // Timer ngắt khi đạt 5 phút
    // => Đặt cờ
    g_timer_done = true;

    // Trả về false để timer không tự động reload
    return pdFALSE;
}


/**
 * @brief  Khởi tạo timer (prescaler, alarm, auto_reload).
 */
void my_timer_init(void)
{
    timer_config_t config = {
        .divider     = 80,                // 1 tick = 1µs (nếu APB clk = 80 MHz)
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .auto_reload = false,             // <--- KHÔNG auto-reload
        .alarm_en    = TIMER_ALARM_EN
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    // 5 phút = 300 giây = 300 * 1.000.000 tick = 300.000.000
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 300000000ULL);

    // Cho phép interrupt
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    // Đăng ký callback
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, my_timer_isr_callback, NULL, 0);
}


/**
 * @brief  Bắt đầu chạy timer.
 */
void my_timer_start(void)
{
    timer_start(TIMER_GROUP_0, TIMER_0);
    ESP_LOGI(TAG, "Timer started.");
}

/**
 * @brief  Tạm dừng timer.
 */
void my_timer_stop(void)
{
    timer_pause(TIMER_GROUP_0, TIMER_0);
    ESP_LOGI(TAG, "Timer stopped.");
}
