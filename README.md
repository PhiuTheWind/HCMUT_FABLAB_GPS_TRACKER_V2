# HCMUT_FABLAB_GPS_TRACKER_V2

## Mô tả dự án

**HCMUT_FABLAB_GPS_TRACKER_V2** là một giải pháp theo dõi vị trí GPS thời gian thực sử dụng **ESP32**, tích hợp cảm biến **MPU6050** và mô-đun GSM/GPRS (**SIMA7680C**). Dự án được phát triển trên nền tảng **ESP-IDF v5.1.5**, tích hợp Arduino Core để hỗ trợ lập trình linh hoạt.

---

## Lưu ý quan trọng

- **ESP-IDF Version**: Dự án yêu cầu **ESP-IDF v5.1.5**. Bạn cần đảm bảo sử dụng đúng phiên bản để tránh các vấn đề không tương thích. Hướng dẫn tải và cài đặt ESP-IDF có thể tham khảo tại [ESP-IDF GitHub](https://github.com/espressif/esp-idf).
  - Kiểm tra phiên bản ESP-IDF đã cài đặt bằng lệnh:
    ```bash
    idf.py --version
    ```
  - Nếu phiên bản không phải v5.1.5, bạn cần chuyển đổi phiên bản bằng lệnh:
    ```bash
    git checkout release/v5.1.5
    git submodule update --init --recursive
    ```

---

## Tính năng chính

- **Theo dõi thời gian thực:** Hiển thị vị trí chính xác của thiết bị trên bản đồ.
- **Phát hiện chuyển động:** Sử dụng cảm biến MPU6050 để phát hiện rung lắc hoặc va chạm.
- **Kết nối GSM và Wi-Fi:** Gửi dữ liệu qua mô-đun SIMA7680C hoặc mạng Wi-Fi.
- **Lưu trữ dữ liệu:** Ghi nhận lịch sử vị trí và thông số chuyển động.
- **Cảnh báo thông minh:** Gửi cảnh báo qua SMS hoặc MQTT khi phát hiện hành vi bất thường.

---

## Phần cứng

1. **ESP32:** Vi điều khiển chính hỗ trợ Wi-Fi và Bluetooth.
2. **Mô-đun GPS:** Neo-6M để thu thập tọa độ vị trí.
3. **Mô-đun SIMA7680C:** Giao tiếp GSM/GPRS để gửi dữ liệu và tin nhắn SMS.
4. **MPU6050:** Cảm biến đo gia tốc và góc nghiêng.
5. **Nguồn:** Pin lithium-ion đảm bảo thời gian hoạt động lâu dài.

---

## Phần mềm

- **Nền tảng:** ESP-IDF v5.1.5 với tích hợp Arduino Core.
- **Thư viện GPS và cảm biến:**
  - TinyGPS++ cho GPS.
  - Adafruit MPU6050 cho cảm biến.
- **Thư viện GSM:**
  - TinyGSM để giao tiếp với SIMA7680C.
- **Giao tiếp mạng:** Hỗ trợ MQTT hoặc HTTP với mã hóa TLS/SSL.

---

## Cách triển khai

1. **Cài đặt ESP-IDF v5.1.5:**
   - Theo hướng dẫn chính thức tại [ESP-IDF GitHub](https://github.com/espressif/esp-idf).

2. **Clone repository:**
   ```bash
   git clone https://github.com/PhiuTheWind/HCMUT_FABLAB_GPS_TRACKER_V2.git
   cd HCMUT_FABLAB_GPS_TRACKER_V2
