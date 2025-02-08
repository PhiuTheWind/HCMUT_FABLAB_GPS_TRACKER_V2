#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "adc.h"

static esp_adc_cal_characteristics_t *adc_chars = NULL;
static bool adc_initialized = false;

// Hàm khởi tạo ADC
void adc_init()
{
    if (adc_initialized) {
        return;
    }

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    if (adc_chars == NULL) {
        printf("Không thể cấp phát bộ nhớ cho adc_chars\n");
        return;
    }

    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
        ADC_UNIT_1, 
        ADC_ATTEN, 
        ADC_WIDTH, 
        DEFAULT_VREF, 
        adc_chars
    );

    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Sử dụng Vref từ eFuse: %" PRIu32 " mV\n", adc_chars->vref);
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Sử dụng Vref từ eFuse Two Point: %" PRIu32 " mV\n", adc_chars->vref);
    } else {
        printf("Sử dụng Vref mặc định: %" PRIu32 " mV\n", adc_chars->vref);
    }

    adc_initialized = true;
}

// Hàm đọc điện áp pin
float read_battery_voltage()
{
    if (!adc_initialized) {
        adc_init();
    }

    if (adc_chars == NULL) {
        printf("ADC chưa được khởi tạo đúng cách.\n");
        return 0.0;
    }

    int adc_reading = adc1_get_raw(ADC_CHANNEL);
    if (adc_reading < 0) {
        printf("Lỗi khi đọc ADC\n");
        return 0.0;
    }

    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    float battery_voltage = (float)voltage_mv / 0.768 / 1000;
    //printf("ADC Raw: %d\tVoltage_mv: %" PRIu32 " mV\tBattery Voltage: %.2f V\n", adc_reading, voltage_mv, battery_voltage);

    return battery_voltage;
}

// Hàm tính toán dung lượng pin dựa trên điện áp
int calculate_battery_capacity(float voltage)
{
    // Kiểm tra điện áp ngoài bảng tra cứu
    if (voltage >= soc_table[0].voltage) {
        return soc_table[0].soc;
    }
    if (voltage <= soc_table[soc_table_size - 1].voltage) {
        return soc_table[soc_table_size - 1].soc;
    }

    // Tìm khoảng điện áp phù hợp trong bảng tra cứu
    for (int i = 0; i < soc_table_size - 1; i++) {
        if (voltage >= soc_table[i + 1].voltage && voltage <= soc_table[i].voltage) {
            // Sử dụng phép nội suy tuyến tính
            float voltage_range = soc_table[i].voltage - soc_table[i + 1].voltage;
            float soc_range = soc_table[i].soc - soc_table[i + 1].soc;
            float voltage_diff = soc_table[i].voltage - voltage;
            int soc = soc_table[i].soc - (int)((voltage_diff / voltage_range) * soc_range);
            return soc;
        }
    }

    // Mặc định nếu không tìm thấy
    return 0;
}
