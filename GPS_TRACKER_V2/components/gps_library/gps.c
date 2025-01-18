// GPS.c
#include "gps.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"

static const char *TAG = "GPS_PROCESS";

// Hàm chuyển đổi độ sang radian
static double deg2rad(double deg) {
    return deg * (M_PI / 180.0);
}

bool diff_location = false;
bool diff_location_flag = false; 

//Biến cờ khác địa chỉ toàn cục
int flag_dif_location = 0;
// Định nghĩa biến toàn cục
gps_data_t global_gps_data;

// Hàm Haversine để tính khoảng cách giữa hai điểm GPS (km)
double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0; // Bán kính Trái Đất (km)

    double phi1 = deg2rad(lat1);
    double phi2 = deg2rad(lat2);
    double delta_phi = deg2rad(lat2 - lat1);
    double delta_lambda = deg2rad(lon2 - lon1);

    double a = sin(delta_phi / 2.0) * sin(delta_phi / 2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    double distance = R * c;

    return distance; // Khoảng cách tính bằng km
}

// Hàm chuyển đổi từ NMEA dạng ddmm.mmmmm hoặc dddmm.mmmmm sang decimal degrees
double nmea_to_decimal_degree(const char *nmea, char direction, int degree_len) {
    double decimal_degree = 0.0;
    double degrees = 0.0;
    double minutes = 0.0;

    // Tách degrees và minutes
    char deg_str[4] = {0};   // Tối đa 3 chữ số cho longitude
    char min_str[20] = {0};  // Đủ lớn để chứa phần thập phân

    strncpy(deg_str, nmea, degree_len);
    deg_str[degree_len] = '\0';
    strcpy(min_str, nmea + degree_len);

    degrees = atof(deg_str);
    minutes = atof(min_str);

    decimal_degree = degrees + (minutes / 60.0);

    // Nếu hướng là S hoặc W, giá trị sẽ âm
    if (direction == 'S' || direction == 'W') {
        decimal_degree *= -1.0;
    }

    return decimal_degree;
}

// Hàm chuyển đổi thời gian UTC sang giờ Việt Nam (UTC+7)
void utc_to_vn_time(const char *utc_time_str, char *vn_time_str) {
    int hour, minute, second;
    double fractional_second;

    // Parse giờ, phút, giây từ chuỗi UTC
    sscanf(utc_time_str, "%2d%2d%lf", &hour, &minute, &fractional_second);
    second = (int)fractional_second;

    // Thêm 7 giờ cho múi giờ Việt Nam
    hour += 7;
    if (hour >= 24) {
        hour -= 24;
        // Bạn có thể xử lý ngày mới tại đây nếu cần
    }

    // Định dạng lại chuỗi thời gian
    snprintf(vn_time_str, 9, "%02d:%02d:%02d", hour, minute, second);
}

void processGNRMC(char *gpsData) {
    char *fields[20];
    int field_count = 0;

    char *token = strtok(gpsData, ",");
    while (token != NULL && field_count < 20) {
        fields[field_count++] = token;
        token = strtok(NULL, ",");
    }

    if (field_count < 12) {
        ESP_LOGW(TAG, "Incomplete GNRMC data");
        return;
    }

    // Time (UTC) và chuyển đổi sang giờ Việt Nam
    if (strlen(fields[1]) >= 6) {
        utc_to_vn_time(fields[1], global_gps_data.time);
        ESP_LOGI(TAG, "Time (Vietnam): %s", global_gps_data.time);
    } else {
        strncpy(global_gps_data.time, "Invalid", sizeof(global_gps_data.time));
        ESP_LOGI(TAG, "Time (Vietnam): Invalid");
    }

    // Status
    strncpy(global_gps_data.status, (strcmp(fields[2], "A") == 0) ? "Successfully located" : "Unsuccessfully located", sizeof(global_gps_data.status));
    ESP_LOGI(TAG, "Status: %s", global_gps_data.status);

    // Latitude
    global_gps_data.latitude = nmea_to_decimal_degree(fields[3], fields[4][0], 2);
    ESP_LOGI(TAG, "Latitude: %.6f°", global_gps_data.latitude);

    // Longitude
    global_gps_data.longitude = nmea_to_decimal_degree(fields[5], fields[6][0], 3);
    ESP_LOGI(TAG, "Longitude: %.6f°", global_gps_data.longitude);

    // Distance
    global_gps_data.distance = haversine(GPS_FIXED_LATITUDE, GPS_FIXED_LONGITUDE, global_gps_data.latitude, global_gps_data.longitude);
    ESP_LOGI(TAG, "Distance to Fixed Point: %.3f km", global_gps_data.distance);

    //TEST
    // if (strcmp(global_gps_data.status, "Successfully located") == 0 && global_gps_data.distance > 0.020) {
    //     flag_dif_location++;
    //     if (flag_dif_location >= 5) {
    //         diff_location = true;
    //         global_gps_data.Stolen = true;
    //         flag_dif_location = 0;
    //         ESP_LOGI(TAG, "Distance exceeded threshold. diff_location set to true.");
    //     }
    // }

    //Condition Check
    if (strcmp(global_gps_data.status, "Successfully located") == 0 && global_gps_data.distance > 0.020) {
        diff_location = true;
        global_gps_data.Stolen = true;
        ESP_LOGI(TAG, "Distance exceeded threshold. diff_location set to true.");
    }

    // Date
    if (strlen(fields[9]) >= 6) {
        snprintf(global_gps_data.date, sizeof(global_gps_data.date), "%.*s/%.*s/20%.*s",
                 2, fields[9], 2, fields[9] + 2, 2, fields[9] + 4);
        ESP_LOGI(TAG, "Date: %s", global_gps_data.date);
    } else {
        strncpy(global_gps_data.date, "Invalid", sizeof(global_gps_data.date));
        ESP_LOGI(TAG, "Date: Invalid");
    }

    // Battery Capacity
    float battery_voltage = read_battery_voltage();
    global_gps_data.battery_capacity = calculate_battery_capacity(battery_voltage);
    ESP_LOGI(TAG, "Battery Capacity: %d%%", global_gps_data.battery_capacity);

}
    

void check_dif_location() {
    // Giả lập logic kiểm tra dif_location
    // Ví dụ: Kiểm tra tín hiệu GPS hoặc dữ liệu UART
    if (diff_location == true) {
        diff_location_flag = true;
        diff_location = false;
    } else {
        diff_location_flag = false;
    }
}

// Hàm khởi tạo UART2 cho GPS
void init_uart2()
{
    // Cấu hình UART2
    const uart_config_t uart_config = {
        .baud_rate = GPS_UART2_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Cấu hình tham số UART
    ESP_ERROR_CHECK(uart_param_config(GPS_UART2_PORT_NUM, &uart_config));

    // Đặt chân TX và RX
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART2_PORT_NUM, GPS_UART2_TX_PIN, GPS_UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Cài đặt driver UART
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART2_PORT_NUM, GPS_UART2_BUFFER_SIZE * 2, 0, GPS_UART2_QUEUE_SIZE, NULL, 0));

    ESP_LOGI(TAG, "UART2 initialized with baud rate %d", GPS_UART2_BAUD_RATE);
}


