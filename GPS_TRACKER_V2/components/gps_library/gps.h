// gps.h

#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>

#include <stdint.h>
extern bool diff_location; // CHỈ khai báo extern trong header
extern bool diff_location_flag; //Cờ địa chỉ
// Định nghĩa tọa độ cố định (ví dụ)
#define GPS_FIXED_LATITUDE 10.773128   // Vĩ độ cố định (decimal degrees) Hội trường B4
#define GPS_FIXED_LONGITUDE 106.658896  // Kinh độ cố định (decimal degrees)

// Định nghĩa UART2 cho GPS
#define GPS_UART2_PORT_NUM      UART_NUM_2
#define GPS_UART2_BAUD_RATE     115200
#define GPS_UART2_TX_PIN        17
#define GPS_UART2_RX_PIN        16
#define GPS_UART2_BUFFER_SIZE   1024
#define GPS_UART2_QUEUE_SIZE    10

// Hàm chuyển đổi từ NMEA dạng ddmm.mmmmm hoặc dddmm.mmmmm sang decimal degrees
double nmea_to_decimal_degree(const char *nmea, char direction, int degree_len);

// Hàm chuyển đổi thời gian UTC sang giờ Việt Nam (UTC+7)
void utc_to_vn_time(const char *utc_time_str, char *vn_time_str);

// Hàm Haversine để tính khoảng cách giữa hai điểm GPS (km)
double haversine(double lat1, double lon1, double lat2, double lon2);

// Hàm xử lý câu GNRMC
void processGNRMC(char *gpsData);

// Hàm khởi tạo UART2 cho GPS
void init_uart2();

// Nhiệm vụ UART2 để đọc dữ liệu và xử lý
void uart2_task(void *pvParameters);

void check_dif_location();
#ifdef __cplusplus
}
#endif

#endif // GPS_H
