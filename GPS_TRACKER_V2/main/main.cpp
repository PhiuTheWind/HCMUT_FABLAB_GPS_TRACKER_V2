//Thư viện
#include "main.h"


/// Khai biến để xử lý MPU6050
static const char *TAG = "MPU6050_APP";
Adafruit_MPU6050 mpu;

// Khai báo biến để xử lý đọc dữ liệu từ MPU6050
float alpha = 0.98;  // Hệ số của bộ lọc Complementary
float roll = 0, pitch = 0; // Góc Roll và Pitch


//Khai biến biến thời gian
unsigned long previous_time = 0;
unsigned long last_read_time = 0;
const unsigned long read_interval = 10; // 10 ms


//Khai biến xử lí sleep
RTC_DATA_ATTR int wake_count = 0;
//Tọa độ ban đầu của Roll và Pitch
RTC_DATA_ATTR float initial_roll = 0, initial_pitch = 0;



// Cờ chống ngủ
int flag_for_wake = 0;
bool steel_mode = false;

// Calibration parameters
const int calibration_duration_ms = 2000; // 2 seconds
const unsigned long read_interval_ms = 10; // 10 ms
const float tilt_threshold = 20.0; // degrees
// Gyroscope bias variables (preserved across deep sleep cycles)
RTC_DATA_ATTR float gyro_bias_x = 0.0, gyro_bias_y = 0.0;
// Calibration flag (preserved across deep sleep cycles)
RTC_DATA_ATTR bool calibration_done = false;


// Hàm setup MPU6050
void setup_mpu6050() {
    if (!mpu.begin()) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    ESP_LOGI(TAG, "MPU6050 initialized successfully");

    // Setup motion detection
  // Configure MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Setup for interrupt
    mpu.setMotionDetectionThreshold(5);
    mpu.setMotionDetectionDuration(1);

    // Latch ngắt không duy trì (một xung ngắt sẽ reset sau đó)
    mpu.setInterruptPinLatch(false);


    // Cực tính ban đầu thấp, tín hiệu ngắt sẽ chuyển lên cao khi có ngắt
    mpu.setInterruptPinPolarity(false);

    // Kích hoạt ngắt chuyển động
    mpu.setMotionInterrupt(true);
    previous_time = esp_timer_get_time();
}

// Hàm set GPIO level
void set_gpio_level(gpio_num_t gpio, int level) {
    rtc_gpio_hold_dis(gpio);
    rtc_gpio_set_level(gpio, level);
    rtc_gpio_hold_en(gpio);
}

void calculate_orientation(float ax, float ay, float az, float gx, float gy, float dt,
                           float gyro_bias_x, float gyro_bias_y, float alpha,
                           float &roll, float &pitch) {
    // Convert gyroscope readings from rad/s to deg/s and apply bias correction
    float gx_deg = (gx - gyro_bias_x) * 180.0 / M_PI; // deg/s
    float gy_deg = (gy - gyro_bias_y) * 180.0 / M_PI; // deg/s

    // Calculate angles from accelerometer data
    float roll_acc = atan2(ay, az) * 180.0 / M_PI;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

    // Apply complementary filter to combine accelerometer and gyroscope data
    roll = alpha * (roll + gx_deg * dt) + (1.0 - alpha) * roll_acc;
    pitch = alpha * (pitch + gy_deg * dt) + (1.0 - alpha) * pitch_acc;
}


// Hàm đọc dữ liệu từ MPU6050 và xử lý
void read_mpu6050_task(void *pvParameter) {
    // Initialize timing variables
    unsigned long previous_micros = esp_timer_get_time();
    unsigned long last_read_time_task = millis();

    // Perform gyroscope calibration if not yet calibrated
    if (!calibration_done) {
        ESP_LOGI(TAG, "Starting gyroscope calibration...");
        const int calibration_samples_count = calibration_duration_ms / read_interval_ms;
        for (int i = 0; i < calibration_samples_count; i++) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            gyro_bias_x += g.gyro.x;
            gyro_bias_y += g.gyro.y;
            vTaskDelay(pdMS_TO_TICKS(read_interval_ms));
        }

        // Calculate average bias
        gyro_bias_x /= calibration_samples_count;
        gyro_bias_y /= calibration_samples_count;

        ESP_LOGI(TAG, "Calibration done!");
        ESP_LOGI(TAG, "Gyro Bias X: %.6f rad/s", gyro_bias_x);
        ESP_LOGI(TAG, "Gyro Bias Y: %.6f rad/s", gyro_bias_y);

        // Set calibration_done flag
        calibration_done = true;
    } else {
        ESP_LOGI(TAG, "Using existing gyroscope calibration");
    }

    // Initialize previous_time for dt calculation
    previous_micros = esp_timer_get_time();

    while (1) {
        unsigned long current_time_millis = millis();
        unsigned long current_time_micros = esp_timer_get_time();
        float dt = (current_time_micros - previous_micros) / 1000000.0; // Delta time in seconds
        previous_micros = current_time_micros;

        // Check if it's time to read the sensor
        if ((current_time_millis - last_read_time_task) >= read_interval_ms) {
            last_read_time_task = current_time_millis;

            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            // Accelerometer data
            float ax = a.acceleration.x;
            float ay = a.acceleration.y;
            float az = a.acceleration.z;

            // Gyroscope data (rad/s), bias already applied
            float gx = g.gyro.x;
            float gy = g.gyro.y;

            // Calculate orientation using the complementary filter
            calculate_orientation(ax, ay, az, gx, gy, dt, gyro_bias_x, gyro_bias_y, alpha, roll, pitch);

            // Initialize reference orientation
            if (wake_count == 1) {
                initial_roll = roll;
                initial_pitch = pitch;
                ESP_LOGI(TAG, "Initial orientation set.");
                ESP_LOGI(TAG, "Initial Roll: %.2f | Initial Pitch: %.2f", initial_roll, initial_pitch);
            }

            // Check for significant tilt (e.g., >20 degrees from initial orientation)
            if ((abs(roll - initial_roll) >= tilt_threshold || abs(pitch - initial_pitch) >= tilt_threshold)) {
                ESP_LOGI(TAG, "Device is tilted beyond threshold!");
                ESP_LOGI(TAG, "Roll: %.2f | Pitch: %.2f", roll, pitch);
                flag_for_wake++;  // Set flag to wake up
            }

            // Optional: Print all data for debugging
            /*
            Serial.print("Roll: "); Serial.print(roll);
            Serial.print(" | Pitch: "); Serial.print(pitch);
            Serial.print(" | Roll_acc: "); Serial.print(roll_acc);
            Serial.print(" | Pitch_acc: "); Serial.print(pitch_acc);
            Serial.print(" | gx: "); Serial.print(gx_deg);
            Serial.print(" | gy: "); Serial.println(gy_deg);
            */

            // No need for additional delay here; timing is managed by millis()
        }
        if (steel_mode == true) {
            printf("Kết thúc task read_mpu6050_task.\n");
            printf("Steel mode is enabled...\n");
            vTaskDelete(NULL);
        }
        // Yield to allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//Hàm in lý do thức dậy
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

// Hàm chính
extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting MPU6050 Application");

    Wire.begin();
    setup_mpu6050();

    // Initialize GPIOs
    rtc_gpio_init(GPIO_GPS_TRIGGER);
    rtc_gpio_set_direction(GPIO_GPS_TRIGGER, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_init(GPIO_SIM_TRIGGER);
    rtc_gpio_set_direction(GPIO_SIM_TRIGGER, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_init(GPIO_PEN);
    rtc_gpio_set_direction(GPIO_PEN, RTC_GPIO_MODE_OUTPUT_ONLY);

    wake_count++;
    printf("Wake count: %d\n", wake_count);
    printf("Initial pitch: %f\n", initial_pitch);
    printf("Initial roll: %f\n", initial_roll);

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        set_gpio_level(GPIO_GPS_TRIGGER, 0);
        set_gpio_level(GPIO_SIM_TRIGGER, 1);
        set_gpio_level(GPIO_PEN, 1);

        xTaskCreate(&read_mpu6050_task, "read_mpu6050_task", 4096, NULL, 5, NULL);
        vTaskDelay(pdMS_TO_TICKS(TIME_TO_WAKE * 1000));
    }

    set_gpio_level(GPIO_GPS_TRIGGER, 1);
    set_gpio_level(GPIO_SIM_TRIGGER, 0);
    set_gpio_level(GPIO_PEN, 0);

    if (wake_count == 1) {
        // Tạo task đọc dữ liệu từ MPU6050
        xTaskCreate(&read_mpu6050_task, "read_mpu6050_task", 4096, NULL, 5, NULL);

        // Đợi một khoảng thời gian để task có thể chạy
        ESP_LOGI(TAG, "Waiting for 10 seconds before deep sleep...");
        vTaskDelay(pdMS_TO_TICKS(10000)); // Đợi 10 giây

        // Cấu hình điều kiện đánh thức
        esp_sleep_enable_ext0_wakeup(INT_PIN, 1);
        ESP_LOGI(TAG, "Entering deep sleep for the first time");
        Serial.flush();
        // Thực hiện ngủ sâu
        esp_deep_sleep_start();
    }

    // Cấu hình điều kiện đánh thức
    esp_sleep_enable_ext0_wakeup(INT_PIN, 1);



    if (flag_for_wake <= 10) 
    {
        printf("Sleep starts... \n");
        Serial.flush();
        esp_deep_sleep_start();
    }
    
    else
    {
        steel_mode = true;
    }
}
