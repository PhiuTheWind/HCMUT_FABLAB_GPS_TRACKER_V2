#ifndef ADC_BATTERY_H
#define ADC_BATTERY_H

#include <stdint.h>

// Định nghĩa cấu hình ADC
#define DEFAULT_VREF    1100        // Giá trị tham chiếu Vref mặc định (mV)
#define ADC_CHANNEL     ADC1_CHANNEL_7  // GPIO35 tương ứng với ADC1_CHANNEL_7
#define ADC_ATTEN       ADC_ATTEN_DB_11
#define ADC_WIDTH       ADC_WIDTH_BIT_12

typedef struct {
    float voltage; // Điện áp (V)
    int soc;       // Dung lượng pin (%)
} soc_lookup_t;

static const soc_lookup_t soc_table[] = {
    {4.2, 100},
    {4.0, 90},
    {3.8, 75},
    {3.7, 50},
    {3.6, 25},
    {3.4, 10},
    {3.0, 0}
};

static const int soc_table_size = sizeof(soc_table) / sizeof(soc_lookup_t);

// Khai báo hàm khởi tạo ADC
void adc_init(void);

// Khai báo hàm đọc điện áp pin
float read_battery_voltage(void);

// Khai báo hàm tính dung lượng pin
int calculate_battery_capacity(float voltage);
#endif // ADC_BATTERY_H
