# HCMUT_FABLAB_GPS_TRACKER_V2

## Mô tả dự án

**HCMUT_FABLAB_GPS_TRACKER_V2** là một giải pháp theo dõi vị trí GPS thời gian thực sử dụng **ESP32**, tích hợp cảm biến **MPU6050** và mô-đun GSM/GPRS (SIM800). Phiên bản này tận dụng **ESP-IDF** với hỗ trợ tích hợp từ **Arduino IDE** để lập trình linh hoạt, mang lại giải pháp tối ưu cho việc giám sát và quản lý vị trí trong thời gian thực.

Mục tiêu dự án:
- **Theo dõi vị trí chính xác.**
- **Phát hiện chuyển động bất thường.**
- **Truyền dữ liệu vị trí và trạng thái qua mạng GSM hoặc Wi-Fi.**

---

## Tính năng chính

- **Theo dõi thời gian thực:** Hiển thị vị trí chính xác trên bản đồ.
- **Phát hiện chuyển động:** Sử dụng cảm biến MPU6050 để phát hiện va chạm hoặc rung lắc.
- **Kết nối GSM và Wi-Fi:** Gửi dữ liệu qua SIM800 hoặc mạng Wi-Fi tùy theo môi trường.
- **Lưu trữ dữ liệu:** Ghi nhận lịch sử hành trình và các thông số vận hành.
- **Cảnh báo thông minh:** Phát hiện hành vi bất thường và gửi cảnh báo qua tin nhắn SMS hoặc server MQTT.

---

## Phần cứng

1. **ESP32:**
   - Vi điều khiển chính, hỗ trợ Wi-Fi và Bluetooth, tích hợp với **ESP-IDF**.

2. **Mô-đun GPS:**
   - Sử dụng Neo-6M để thu thập tọa độ vị trí với độ chính xác cao.

3. **Mô-đun SIM800:**
   - Truyền dữ liệu qua mạng GSM/GPRS, hỗ trợ gửi tin nhắn SMS khi cần.

4. **Cảm biến MPU6050:**
   - Đo gia tốc và góc nghiêng, hỗ trợ phát hiện các chuyển động bất thường.

5. **Nguồn năng lượng:**
   - Pin lithium-ion với mạch bảo vệ, đảm bảo thời gian hoạt động lâu dài.

---

## Phần mềm

- **Nền tảng:** ESP-IDF với tích hợp **Arduino Core**.
- **Thư viện cảm biến và GPS:**
  - **TinyGPS++:** Xử lý dữ liệu từ mô-đun GPS.
  - **Adafruit MPU6050:** Giao tiếp với cảm biến gia tốc.
- **Thư viện GSM:**
  - Sử dụng thư viện **TinyGSM** để gửi/nhận dữ liệu qua mô-đun SIM800.
- **Giao tiếp mạng:**
  - Gửi dữ liệu qua MQTT hoặc HTTP khi kết nối Wi-Fi/GPRS.
- **Mã hóa:** Sử dụng TLS/SSL khi truyền dữ liệu để đảm bảo bảo mật.

---

## Cách triển khai

### Phần cứng

1. Kết nối:
   - **Neo-6M (GPS):** Kết nối qua UART với ESP32.
   - **MPU6050:** Kết nối qua I2C (SDA, SCL) với ESP32.
   - **SIM800:** Kết nối qua UART với ESP32.
2. Cấp nguồn:
   - Sử dụng pin lithium-ion hoặc nguồn DC 5V.

### Phần mềm

1. Cài đặt ESP-IDF:
   - Theo hướng dẫn chính thức tại [ESP-IDF GitHub](https://github.com/espressif/esp-idf).

2. Tích hợp Arduino Core:
   - Kích hoạt Arduino Core trong **menuconfig**:
     ```bash
     idf.py menuconfig
     ```
     Chọn `Component config > Arduino > Enable Arduino as a component`.

3. Clone repository:
   ```bash
   git clone https://github.com/username/HCMUT_FABLAB_GPS_TRACKER_V2.git

   
### Sơ đồ hệ thống
+-----------+      +-------+      +-------------+      +-------------+
|   GPS     | ---> | ESP32 | ---> |     Wi-Fi    | ---> | Server/API |
+-----------+      +-------+      +-------------+      +-------------+
      |                |               |
+-----------+      +-------------+      +--------------+
| MPU6050   | ---> |   SIM800    | ---> | SMS/Server   |
+-----------+      +-------------+      +--------------+

