// main.cpp

// ------------------------- Include Libraries -------------------------
#include "main.h"
#include "yaw.h"
#include "gps.h" // Bao gồm tệp header của GPS
#include "driver/uart.h"


// ------------------------- Sleep Control Variables ------------------------
RTC_DATA_ATTR int wake_count = 0;
RTC_DATA_ATTR float initial_roll = 0.0f, initial_pitch = 0.0f, initial_yaw = 0.0f;
RTC_DATA_ATTR float gyro_bias_x = 0.0, gyro_bias_y = 0.0, gyro_bias_z = 0.0;
RTC_DATA_ATTR bool calibration_done = false;

int flag_for_wake = 0;
bool steel_mode = false;
bool read_task_delete = false;

// ------------------------- Logging Tag -------------------------
static const char *TAG = "MPU6050_APP";

// ------------------------- MPU6050 Variables -------------------------
Adafruit_MPU6050 mpu;
KalmanFilter yawKalman; // Tính toán trục Z

// Complementary filter constant
const float alpha = 0.98f;

// Roll, Pitch, và Yaw angles
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

// Calibration parameters
const int calibration_duration_ms = 2000;     // 2 seconds
const unsigned long read_interval_ms = 10;    // 10 ms
const float tilt_threshold = 20.0f;           // degrees
const float tilt_threshold_yaw = 10.0f;        // degrees


// ------------------------- Task Handles ------------------------
TaskHandle_t readTaskHandle;                   // Global variable to track the task
TaskHandle_t appMainTaskHandle = NULL;        // Global variable to store app_main handle

// ------------------------- GPS Variables -------------------------
int gps_flag = 0;
volatile bool gps_task_done = false; // Cờ báo hiệu task đã hoàn thành

// Define Constants
#define UART2_TASK_STACK_SIZE 4096
#define UART2_TASK_PRIORITY 5
#define UART2_TASK_DELAY_MS 120000 //2 minutes 
#define MAX_UART2_TASK_RETRIES 3
#define WAKE_THRESHOLD 50





// ====================================================================
//                      Utility Functions
// ====================================================================

// Function to print wake-up reason
/**
 * @brief In ra lý do thức dậy từ chế độ ngủ.
 */
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
/**
 * @brief In ra lý do reset của thiết bị.
 */
void print_reset_reason() {
    esp_reset_reason_t reset_reason = esp_reset_reason();
    printf("Reset reason: %d\n", reset_reason);
}

// Function to setup MPU6050
// ====================================================================
//                      MPU6050 Functions
// ====================================================================

/**
 * @brief Cấu hình và khởi tạo MPU6050.
 */
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
    mpu.setMotionDetectionThreshold(5);
    mpu.setMotionDetectionDuration(1);
    mpu.setInterruptPinLatch(false);
    mpu.setInterruptPinPolarity(true);
    
    mpu.setMotionInterrupt(true);

    ESP_LOGI(TAG, "MPU6050 configuration done.");
}



// Function to calculate orientation using Complementary filter
void calculate_orientation(float ax, float ay, float az, float gx, float gy, float gz, float dt,
                           float gyro_bias_x, float gyro_bias_y, float gyro_bias_z, float alpha,
                           float &roll, float &pitch, float &yaw) {

    // Convert gyroscope data from rad/s to deg/s and correct bias
    float gx_deg = (gx - gyro_bias_x) * 180.0f / M_PI; // deg/s
    float gy_deg = (gy - gyro_bias_y) * 180.0f / M_PI; // deg/s
    float gz_deg = (gz - gyro_bias_z) * 180.0f / M_PI; // deg/s

    // Calculate angles from accelerometer data
    float roll_acc = atan2f(ay, az) * 180.0f / M_PI;
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    // Apply Complementary filter to combine accelerometer and gyroscope data
    roll = alpha * (roll + gx_deg * dt) + (1.0f - alpha) * roll_acc;
    pitch = alpha * (pitch + gy_deg * dt) + (1.0f - alpha) * pitch_acc;
    //yaw += gz_deg * dt; // Integrate gyroscope z-axis data to compute yaw

    // Calculate yaw using Kalman Filter (gyro only)
    yaw = yawKalman.getAngle(yaw, gz_deg, dt);
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
                gyro_bias_z += g.gyro.z; // Thêm trục Z
            } else {
                ESP_LOGE(TAG, "Failed to read MPU6050 during calibration.");
            }
            vTaskDelay(pdMS_TO_TICKS(read_interval_ms));
        }

        // Calculate average bias
        gyro_bias_x /= calibration_samples_count;
        gyro_bias_y /= calibration_samples_count;
        gyro_bias_z /= calibration_samples_count; // Thêm trục Z

        ESP_LOGI(TAG, "Calibration done!");
        ESP_LOGI(TAG, "Gyro Bias X: %.6f rad/s", gyro_bias_x);
        ESP_LOGI(TAG, "Gyro Bias Y: %.6f rad/s", gyro_bias_y);
        ESP_LOGI(TAG, "Gyro Bias Z: %.6f rad/s", gyro_bias_z); // Thêm trục Z

        // Set calibration done flag
        calibration_done = true;
    } else {
        ESP_LOGI(TAG, "Using existing gyroscope calibration.");
    }


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
        float gz = g.gyro.z;

        // Tính toán góc sử dụng bộ lọc Complementary (roll, pitch) và Kalman (yaw)
        calculate_orientation(ax, ay, az, gx, gy, gz, 0.01f, gyro_bias_x, gyro_bias_y, gyro_bias_z, alpha, roll, pitch, yaw);

        // Điều chỉnh góc nằm trong khoảng [-180, 180] độ
        roll = fmodf(roll, 360.0f);
        if (roll > 180.0f) roll -= 360.0f;
        if (roll < -180.0f) roll += 360.0f;

        pitch = fmodf(pitch, 360.0f);
        if (pitch > 180.0f) pitch -= 360.0f;
        if (pitch < -180.0f) pitch += 360.0f;

        yaw = fmodf(yaw, 360.0f);
        if (yaw > 180.0f) yaw -= 360.0f;
        if (yaw < -180.0f) yaw += 360.0f;



        // Đặt tọa độ ban đầu
        if (wake_count <= 1) {
            initial_roll = roll;
            initial_pitch = pitch;
            initial_yaw = yaw;
            // In dữ liệu cho mục đích gỡ lỗi
            ESP_LOGI(TAG, "Roll: %.2f | Pitch: %.2f | Yaw: %.2f", roll, pitch, yaw);
            continue; // Bỏ qua kiểm tra nghiêng cho lần đầu tiên
        }


        if (wake_count > 1) {
            // Kiểm tra nếu trong 5 giây đầu thì bỏ qua xử lý
            if ((xTaskGetTickCount() - startTick) <= idleDuration) {
                continue; // Bỏ qua xử lý
            }

            // Kiểm tra nghiêng quá ngưỡng (ví dụ: >20 độ từ vị trí ban đầu)
            if ((fabsf(roll - initial_roll) >= tilt_threshold || 
                fabsf(pitch - initial_pitch) >= tilt_threshold || 
                fabsf(yaw - initial_yaw) >= tilt_threshold_yaw)) {
                ESP_LOGI(TAG, "Device is tilted beyond threshold!");
                ESP_LOGI(TAG, "Roll: %.2f | Pitch: %.2f | Yaw: %.2f", roll, pitch, yaw);
                // Thực hiện hành động khi nghiêng quá ngưỡng nếu cần

                // Đặt cờ để đánh dấu   
                if (flag_for_wake >= 50) {
                    flag_for_wake = 50;
                }
                else flag_for_wake++;  // Tăng flag để đánh dấu

            }
        }

        // Delay trước khi đọc lần tiếp theo
        vTaskDelay(pdMS_TO_TICKS(read_interval_ms));
    }
}

// ====================================================================
//                      GPS Functions
// ====================================================================

/**
 * @brief Theo dõi xung PPS từ GPS trong 2 phút.
 */
void gps_pps_monitor() {
    TickType_t startTick = xTaskGetTickCount(); // Lấy thời gian bắt đầu
        const TickType_t runDuration = pdMS_TO_TICKS(2 * 60 * 1000); // 2 phút

    ESP_LOGI(TAG, "Starting GPS PPS monitoring for 2 minutes...");

    // Cấu hình GPIO
    gpio_reset_pin(GPIO_GPS_PPS);
    gpio_set_direction(GPIO_GPS_PPS, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_GPS_PPS, GPIO_PULLDOWN_ONLY); // Kéo xuống mặc định

    // Vòng lặp chạy trong 2 phút
    while ((xTaskGetTickCount() - startTick) < runDuration) {
        if (gpio_get_level(GPIO_GPS_PPS) == 1) {
            gps_flag++;
            ESP_LOGI(TAG, "GPS PPS detected! gps_flag = %d", gps_flag);
            vTaskDelay(pdMS_TO_TICKS(100)); // Chống rung (debounce)
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Kiểm tra mỗi 10ms
        
        if (gps_flag >= 10)
        {
            break;
        }
    }

    ESP_LOGI(TAG, "2 minutes elapsed. Final gps_flag count: %d", gps_flag);

    // Đặt cờ báo hiệu task đã hoàn thành
    gps_task_done = true;
}

/**
 * @brief Đọc và xử lý dữ liệu từ UART2.
 * 
 * @param buffer Bộ đệm lưu dữ liệu UART2
 * @param buffer_size Kích thước tối đa của buffer
 * @return true nếu đọc và xử lý thành công, false nếu thất bại hoặc gặp lỗi.
 */
bool read_uart2_data(char *buffer, int buffer_size) {
    static int buffer_pos = 0;

    // Đọc dữ liệu từ UART2
    int len = uart_read_bytes(GPS_UART2_PORT_NUM,
                              (uint8_t *)(buffer + buffer_pos),
                              buffer_size - buffer_pos - 1,
                              20 / portTICK_PERIOD_MS);
    if (len > 0) {
        buffer_pos += len;
        buffer[buffer_pos] = '\0'; // Null-terminate chuỗi

        // Tìm ký tự xuống dòng
        char *newline_ptr;
        while ((newline_ptr = strchr(buffer, '\n')) != NULL) {
            *newline_ptr = '\0'; // Kết thúc chuỗi tại ký tự xuống dòng

            // Loại bỏ ký tự carriage return (CRLF) nếu có
            if (newline_ptr > buffer && *(newline_ptr - 1) == '\r') {
                *(newline_ptr - 1) = '\0';
            }

            // Xử lý dữ liệu bắt đầu bằng "$GNRMC"
            if (strncmp(buffer, "$GNRMC", 6) == 0) {
                ESP_LOGI(TAG, "Received GNRMC Data: %s", buffer);
                processGNRMC(buffer);
                buffer_pos = 0; // Reset buffer sau khi xử lý
                return true;
            }

            // Di chuyển phần còn lại của buffer về đầu
            int remaining = buffer_pos - (newline_ptr - buffer) - 1;
            memmove(buffer, newline_ptr + 1, remaining);
            buffer_pos = remaining;
            buffer[buffer_pos] = '\0';
        }

        // Xử lý trường hợp buffer đầy mà không có ký tự newline
        if (buffer_pos >= buffer_size - 1) {
            ESP_LOGW(TAG, "Buffer full without receiving newline. Clearing buffer.");
            buffer_pos = 0;
            buffer[buffer_pos] = '\0';
        }
    }

    return false; // Không có dữ liệu hợp lệ được xử lý
}

// ------------------ Task UART2 ------------------
void uart2_task(void *pvParameters) {
    
    // Cấp phát buffer
    char *buffer = (char *)malloc(GPS_UART2_BUFFER_SIZE);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for UART2 buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Start uart2_task, monitoring UART2 data...");

    TickType_t startTick = xTaskGetTickCount();
    const TickType_t runDuration = pdMS_TO_TICKS(5 * 60 * 1000); // 5 phút

    while ((xTaskGetTickCount() - startTick) < runDuration) {
        bool success = read_uart2_data(buffer, GPS_UART2_BUFFER_SIZE);
        if (success) {
            ESP_LOGI(TAG, "Data successfully processed.");
            // Thực hiện các hành động khác nếu cần
        }

        // Nghỉ 1 giây để tránh chiếm CPU
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    free(buffer); // Giải phóng bộ nhớ trước khi kết thúc
    ESP_LOGI(TAG, "uart2_task completed after 5 minutes.");
    vTaskDelete(NULL); // Xóa task sau khi hoàn thành
}

// ------------------ Quản Lý Task UART2 ------------------
TaskHandle_t uart2TaskHandle = NULL;

void start_uart2_task() {
    if (uart2TaskHandle == NULL) {
        ESP_LOGI(TAG, "Creating uart2_task...");
        xTaskCreate(uart2_task, "uart2_task", 4096, NULL, 5, &uart2TaskHandle);
    } else {
        ESP_LOGW(TAG, "uart2_task is already running.");
    }
}

void stop_uart2_task() {
    if (uart2TaskHandle != NULL) {
        ESP_LOGI(TAG, "Stopping uart2_task...");
        vTaskDelete(uart2TaskHandle); // Xóa trực tiếp task
        uart2TaskHandle = NULL;
    } else {
        ESP_LOGW(TAG, "uart2_task is not running.");
    }
}

// ------------------ Chế Độ Ngủ (Deep Sleep) ------------------
void enter_deep_sleep() {
    ESP_LOGI(TAG, "Entering deep sleep mode...");
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 0); // Cấu hình chân GPIO để đánh thức
    esp_deep_sleep_start();
}



// ------------------ Vòng Lặp Chính ------------------
void monitor_uart_with_condition() {
    while (1) {
        ESP_LOGI(TAG, "Checking dif_location condition...");

        check_dif_location(); // Cập nhật giá trị `dif_location`

        if (diff_location_flag == false) {
            ESP_LOGI(TAG, "dif_location == false. Starting UART2 Task for 5 minutes.");

            // Khởi tạo Task UART2
            start_uart2_task();

            // Chờ Task UART2 chạy 5 phút
            vTaskDelay(pdMS_TO_TICKS(5 * 60 * 1000)); // 5 phút

            // Dừng Task UART2
            stop_uart2_task();

        } 
        else {
            ESP_LOGI(TAG, "dif_location == true. Continuing UART2 Task.");

            // Nếu Task chưa được khởi tạo, tạo lại
            if (uart2TaskHandle == NULL) {
                start_uart2_task();
            } 
            else {
                ESP_LOGI(TAG, "UART2 Task is already running, continuing...");
            }

            // Kiểm tra lại sau 1 phút
            vTaskDelay(pdMS_TO_TICKS(60000)); // 1 phút
        }
    }
}

// Function to set-up GPIO level
/**
 * @brief Cài đặt mức GPIO
 */
void set_gpio_level(gpio_num_t gpio, int level) {
    rtc_gpio_hold_dis(gpio);
    rtc_gpio_set_level(gpio, level);
    rtc_gpio_hold_en(gpio);
}

//Set GPIO về chế độ ngủ
void set_sleep_gpio()
{
    // Cài đặt GPIOs cho chế độ mặc định
    set_gpio_level(GPIO_GPS_TRIGGER, 1);
    set_gpio_level(GPIO_SIM_TRIGGER, 0);
    set_gpio_level(GPIO_PEN, 0);
}



// ====================================================================
//                      Main Application
// ====================================================================

// Main application function
extern "C" void app_main() {
    // In lý do thức dậy để debug
    // print_wakeup_reason();

    // In lý do reset để debug
    // print_reset_reason();

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

    // Cài đặt GPIOs cho chế độ mặc định
    set_gpio_level(GPIO_GPS_TRIGGER, 1);
    set_gpio_level(GPIO_SIM_TRIGGER, 0);
    set_gpio_level(GPIO_PEN, 0);

    wake_count++;
    ESP_LOGI(TAG, "Wake count: %d", wake_count);
    ESP_LOGI(TAG, "Initial pitch: %f", initial_pitch);
    ESP_LOGI(TAG, "Initial roll: %f", initial_roll);
    ESP_LOGI(TAG, "Initial yaw: %f", initial_yaw);

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

                if (flag_for_wake < 50) {
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
        ESP_LOGI(TAG, "Initial Roll: %.2f | Initial Pitch: %.2f | Initial Yaw: %.2f", initial_roll, initial_pitch, initial_yaw);
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
        if (flag_for_wake >= 50) {
            ESP_LOGI(TAG, "flag_for_wake = 50. Setting GPIOs for Steel mode.");
            set_gpio_level(GPIO_GPS_TRIGGER, 0);
            //set_gpio_level(GPPIO_GPS_PPS, 1);
            ESP_LOGI(TAG, "Steel mode is on...");
            printf("Steel mode is on...\n");
        }
    }


    //GPS SECTION
    ESP_LOGI(TAG, "Starting GPS Tracking...");
    // Khởi tạo UART2
    init_uart2();


    gps_pps_monitor();
    if (gps_flag >= 10)
    {   
        //SIM WAKE-UP
        set_gpio_level(GPIO_SIM_TRIGGER, 1);
        set_gpio_level(GPIO_PEN, 1);

        ESP_LOGI(TAG, "Starting UART Monitoring with Condition...");

        // Bắt đầu vòng lặp kiểm tra điều kiện và UART
        monitor_uart_with_condition();

        //Setting for sleep mode
        esp_sleep_enable_ext0_wakeup(INT_PIN, 0);
        Serial.flush();

        set_sleep_gpio();
        esp_deep_sleep_start();
    }
    
    else
    {
        printf("NO GPS SIGNAL. Starting to sleep...\n");
        // Cấu hình điều kiện đánh thức trước khi ngủ
        esp_sleep_enable_ext0_wakeup(INT_PIN, 0);
        Serial.flush();

        set_sleep_gpio();
        esp_deep_sleep_start();
    }

    // Đảm bảo rằng app_main không kết thúc bằng cách sử dụng vòng lặp vô hạn
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
