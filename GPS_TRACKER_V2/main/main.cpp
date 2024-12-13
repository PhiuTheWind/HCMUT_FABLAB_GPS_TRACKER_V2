#include "main.h"

// Sleep control variables
RTC_DATA_ATTR int wake_count = 0;
RTC_DATA_ATTR float initial_roll = 0.0f, initial_pitch = 0.0f;
int flag_for_wake = 0;
bool steel_mode = false;
RTC_DATA_ATTR float gyro_bias_x = 0.0, gyro_bias_y = 0.0;
RTC_DATA_ATTR bool calibration_done = false;
bool read_task_delete = false;

// Logging tag
static const char *TAG = "MPU6050_APP";

// MPU6050 object
Adafruit_MPU6050 mpu;

// Complementary filter constant
const float alpha = 0.98f;

// Roll and Pitch angles
float roll = 0.0f, pitch = 0.0f;

// Calibration parameters
const int calibration_duration_ms = 2000;     // 2 seconds
const unsigned long read_interval_ms = 10;    // 10 ms
const float tilt_threshold = 20.0f;           // degrees

TaskHandle_t readTaskHandle; // Global variable to track the task
TaskHandle_t appMainTaskHandle = NULL; // Global variable to store app_main handle

// Function to print wake-up reason
void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            printf("Woke up by MPU6050 interrupt!\n");
            break;
        default:
            printf("Unknown wakeup reason.\n");
            break;
    }
}

// Function to print reset reason
void print_reset_reason() {
    esp_reset_reason_t reset_reason = esp_reset_reason();
    printf("Reset reason: %d\n", reset_reason);
}

// Function to setup MPU6050
void setup_mpu6050() {
    if (!mpu.begin()) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        int retry_count = 0;
        while (!mpu.begin() && retry_count < 5) {
            ESP_LOGE(TAG, "Failed to initialize MPU6050, retrying in 10 seconds...");
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
        if (retry_count == 5) {
            ESP_LOGE(TAG, "MPU6050 failed after 5 retries. Halting.");
            while (1);
        }
    }
    ESP_LOGI(TAG, "MPU6050 initialized successfully");

    // Configure MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Configure motion detection interrupt
    mpu.setMotionDetectionThreshold(10);
    mpu.setMotionDetectionDuration(1);
    mpu.setInterruptPinLatch(false);
    mpu.setInterruptPinPolarity(true);
    
    mpu.setMotionInterrupt(true);

    ESP_LOGI(TAG, "MPU6050 configuration done.");
}

// Function to set GPIO level
void set_gpio_level(gpio_num_t gpio, int level) {
    rtc_gpio_hold_dis(gpio);
    rtc_gpio_set_level(gpio, level);
    rtc_gpio_hold_en(gpio);
}

// Function to calculate orientation using Complementary filter
void calculate_orientation(float ax, float ay, float az, float gx, float gy, float dt,
                           float gyro_bias_x, float gyro_bias_y, float alpha,
                           float &roll, float &pitch) {


    // Convert gyroscope data from rad/s to deg/s and correct bias
    float gx_deg = (gx - gyro_bias_x) * 180.0f / M_PI; // deg/s
    float gy_deg = (gy - gyro_bias_y) * 180.0f / M_PI; // deg/s

    // Calculate angles from accelerometer data
    float roll_acc = atan2f(ay, az) * 180.0f / M_PI;
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    // Apply Complementary filter to combine accelerometer and gyroscope data
    roll = alpha * (roll + gx_deg * dt) + (1.0f - alpha) * roll_acc;
    pitch = alpha * (pitch + gy_deg * dt) + (1.0f - alpha) * pitch_acc;
}

// MPU6050 read and processing task
void read_mpu6050_task(void *pvParameter) {
    // Nhận handle của app_main
    TaskHandle_t appMainHandle = (TaskHandle_t) pvParameter;
    ESP_LOGI(TAG, "read_mpu6050_task started.");

    // Khởi tạo thời gian để đếm 20 giây
    TickType_t startTick = xTaskGetTickCount();
    const TickType_t runDuration = pdMS_TO_TICKS(20000); // 20 giây
    const TickType_t idleDuration = pdMS_TO_TICKS(5000); // 5 giây đầu không làm gì

    // Perform gyroscope calibration if not done
    if (!calibration_done) {
        ESP_LOGI(TAG, "Starting gyroscope calibration...");
        const int calibration_samples_count = calibration_duration_ms / read_interval_ms;
        for (int i = 0; i < calibration_samples_count; i++) {
            sensors_event_t a, g, temp;
            if (mpu.getEvent(&a, &g, &temp)) {
                gyro_bias_x += g.gyro.x;
                gyro_bias_y += g.gyro.y;
            } else {
                ESP_LOGE(TAG, "Failed to read MPU6050 during calibration.");
            }
            vTaskDelay(pdMS_TO_TICKS(read_interval_ms));
        }

        // Calculate average bias
        gyro_bias_x /= calibration_samples_count;
        gyro_bias_y /= calibration_samples_count;

        ESP_LOGI(TAG, "Calibration done!");
        ESP_LOGI(TAG, "Gyro Bias X: %.6f rad/s", gyro_bias_x);
        ESP_LOGI(TAG, "Gyro Bias Y: %.6f rad/s", gyro_bias_y);

        // Set calibration done flag
        calibration_done = true;
    } else {
        ESP_LOGI(TAG, "Using existing gyroscope calibration.");
    }

    // Khởi tạo thời gian cho tính dt
    unsigned long previous_micros = esp_timer_get_time();

    while (1) {


        // Kiểm tra xem đã chạy đủ 20 giây chưa
        if ((xTaskGetTickCount() - startTick) >= runDuration) {
            ESP_LOGI(TAG, "20 seconds elapsed. Terminating task.");

            // Gửi thông báo tới app_main
            if (appMainHandle != NULL) {
                xTaskNotifyGive(appMainHandle);
                ESP_LOGI(TAG, "Notification sent to app_main.");
            } else {
                ESP_LOGE(TAG, "appMainHandle is NULL. Cannot notify.");
            }

            // Đặt cờ trạng thái
            read_task_delete = true;
            ESP_LOGI(TAG, "read_task_delete set to true.");

            // Xóa task hiện tại
            vTaskDelete(NULL); // NULL nghĩa là xóa chính task hiện tại
        }

        // Đọc sensor data
        sensors_event_t a, g, temp;
        if (!mpu.getEvent(&a, &g, &temp)) {
            ESP_LOGE(TAG, "Failed to read MPU6050 sensor.");
            continue; // Bỏ qua lần đọc này và tiếp tục
        }

        // Accelerometer data
        float ax = a.acceleration.x;
        float ay = a.acceleration.y;
        float az = a.acceleration.z;

        // Gyroscope data (rad/s), đã hiệu chỉnh
        float gx = g.gyro.x;
        float gy = g.gyro.y;

        // Tính toán góc sử dụng bộ lọc Complementary
        calculate_orientation(ax, ay, az, gx, gy, 0.01f, gyro_bias_x, gyro_bias_y, alpha, roll, pitch);

        // Điều chỉnh góc nằm trong khoảng [-180, 180] độ
        roll = fmodf(roll, 360.0f);
        if (roll > 180.0f) roll -= 360.0f;
        if (roll < -180.0f) roll += 360.0f;

        pitch = fmodf(pitch, 360.0f);
        if (pitch > 180.0f) pitch -= 360.0f;
        if (pitch < -180.0f) pitch += 360.0f;

        // In dữ liệu cho mục đích gỡ lỗi
        ESP_LOGI(TAG, "Roll: %.2f | Pitch: %.2f", roll, pitch);

        // Đặt tọa độ ban đầu khi bắt đầu chạy
        if (wake_count <= 1) {
            initial_roll = roll;
            initial_pitch = pitch;
            continue; // Bỏ qua kiểm tra nghiêng cho lần đầu tiên
        }

        if (wake_count > 1) {
            // Kiểm tra nếu trong 5 giây đầu thì bỏ qua xử lý
            if ((xTaskGetTickCount() - startTick) <= idleDuration) {
                continue; // Bỏ qua xử lý
            }

            // Kiểm tra nghiêng quá ngưỡng (ví dụ: >20 độ từ vị trí ban đầu)
            if ((fabsf(roll - initial_roll) >= tilt_threshold || fabsf(pitch - initial_pitch) >= tilt_threshold)) {
                ESP_LOGI(TAG, "Device is tilted beyond threshold!");
                ESP_LOGI(TAG, "Roll: %.2f | Pitch: %.2f", roll, pitch);
                // Thực hiện hành động khi nghiêng quá ngưỡng nếu cần
                flag_for_wake++;  // Tăng flag để đánh dấu
            }
        }

        // Delay trước khi đọc lần tiếp theo
        vTaskDelay(pdMS_TO_TICKS(read_interval_ms));
    }
}



// Main application function
extern "C" void app_main() {
    //In lý do thức dậy để debug
    //print_wakeup_reason();

    // In lý do reset để debug
    //print_reset_reason();

    ESP_LOGI(TAG, "Starting MPU6050 Application");

    // Initialize Serial
    Wire.begin(21, 22, 100000); // SDA, SCL, tốc độ 100kHz
    setup_mpu6050();

    // Initialize GPIOs
    rtc_gpio_init(GPIO_GPS_TRIGGER);
    rtc_gpio_set_direction(GPIO_GPS_TRIGGER, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_init(GPIO_SIM_TRIGGER);
    rtc_gpio_set_direction(GPIO_SIM_TRIGGER, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_init(GPIO_PEN);
    rtc_gpio_set_direction(GPIO_PEN, RTC_GPIO_MODE_OUTPUT_ONLY);

    //Cài đặt GPIOs cho chế độ mặc định
    set_gpio_level(GPIO_GPS_TRIGGER, 1);
    set_gpio_level(GPIO_SIM_TRIGGER, 0);
    set_gpio_level(GPIO_PEN, 0);

    wake_count++;
    ESP_LOGI(TAG, "Wake count: %d", wake_count);
    ESP_LOGI(TAG, "Initial pitch: %f", initial_pitch);
    ESP_LOGI(TAG, "Initial roll: %f", initial_roll);

     // Lưu handle của app_main
    appMainTaskHandle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "app_main started. Handle: %p", (void*)appMainTaskHandle);

    // Kiểm tra lý do thức dậy từ deep sleep
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        ESP_LOGI(TAG, "Woke up from EXT0 interrupt.");

        if (wake_count > 1) {
            ESP_LOGI(TAG, "wake_count > 1. Handling accordingly.");

            // Tạo task để xử lý đọc dữ liệu MPU6050
            if (xTaskCreate(&read_mpu6050_task, "read_mpu6050_task", 8192, (void*)appMainTaskHandle, 5, &readTaskHandle) == pdPASS) {
                ESP_LOGI(TAG, "read_mpu6050_task created successfully. Waiting for 20 seconds...");

                // Chờ 20 giây để task chạy và cập nhật flag_for_wake
                vTaskDelay(pdMS_TO_TICKS(20000)); // 20 giây

                ESP_LOGI(TAG, "20 seconds elapsed. Checking flag_for_wake: %d", flag_for_wake);

                if (flag_for_wake <= 50) {
                    ESP_LOGI(TAG, "flag_for_wake <= 50. Entering deep sleep.");
                    // Cấu hình điều kiện đánh thức trước khi ngủ
                    esp_sleep_enable_ext0_wakeup(INT_PIN, 0);
                    Serial.flush();
                    esp_deep_sleep_start();
                } else {
                    ESP_LOGI(TAG, "flag_for_wake > 50. Continuing app_main.");
                    // Có thể thêm các hành động khác nếu cần
                }

                // Không xóa task từ app_main, vì task đã tự xóa mình
            } else {
                if (wake_count == 1) {
                ESP_LOGE(TAG, "Failed to create read_mpu6050_task.");
                }
            }
        }
    }

    // Xử lý khi wake_count == 1
    if (wake_count == 1) {
        ESP_LOGI(TAG, "wake_count == 1. Creating read_mpu6050_task without sleep timer.");

        // Tạo task đọc dữ liệu từ MPU6050
        if (xTaskCreate(&read_mpu6050_task, "read_mpu6050_task", 8192, (void*)appMainTaskHandle, 5, NULL) == pdPASS) {
            ESP_LOGI(TAG, "read_mpu6050_task created successfully.");
        } else {
            ESP_LOGE(TAG, "Failed to create read_mpu6050_task.");
        }

        // Đợi 20 giây trước khi ngủ sâu lần đầu tiên
        ESP_LOGI(TAG, "Waiting for 20 seconds before deep sleep...");
        vTaskDelay(pdMS_TO_TICKS(20000)); // 20 giây
        ESP_LOGI(TAG, "After 20 seconds delay.");
        ESP_LOGI(TAG, "Initial orientation set.");
        ESP_LOGI(TAG, "Initial Roll: %.2f | Initial Pitch: %.2f", initial_roll, initial_pitch);
        // Cấu hình điều kiện đánh thức trước khi ngủ
        esp_sleep_enable_ext0_wakeup(INT_PIN, 0);
        ESP_LOGI(TAG, "Wake-up condition configured.");

        ESP_LOGI(TAG, "Entering deep sleep for the first time.");
        Serial.flush();
        esp_deep_sleep_start();
    }

    // Kiểm tra wake_count >1 đã được xử lý ở trên
    // Kiểm tra giá trị flag_for_wake để quyết định hành động
    if (wake_count > 1) {
        if (flag_for_wake > 50) {
            ESP_LOGI(TAG, "flag_for_wake > 50. Setting GPIOs for Steel mode.");
            set_gpio_level(GPIO_GPS_TRIGGER, 0);
            set_gpio_level(GPIO_SIM_TRIGGER, 1);
            set_gpio_level(GPIO_PEN, 1);
            ESP_LOGI(TAG, "Steel mode is on...");
            printf("Steel mode is on...\n");
        }
    }

    ESP_LOGI(TAG, "app_main completed.");

    // Đảm bảo rằng app_main không kết thúc bằng cách sử dụng vòng lặp vô hạn
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

