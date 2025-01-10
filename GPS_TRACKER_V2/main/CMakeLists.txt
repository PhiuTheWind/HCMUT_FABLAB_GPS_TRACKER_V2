set(EXTRA_COMPONENT_DIRS components)

idf_component_register(
    SRCS "main.cpp"  # File mã nguồn chính của ứng dụng
    INCLUDE_DIRS
        "../components/arduino/libraries/Adafruit_MPU6050"  # Thư mục chứa Adafruit_MPU6050.h
        "../components/arduino/libraries/Adafruit_BusIO"    # Thư mục chứa Adafruit_BusIO.h
        "../components/arduino/libraries/Adafruit_Unified_Sensor"  # Thư mục chứa Adafruit_Unified_Sensor.h
        "../components/main_library"                       # Thư mục chứa các file header main.h
        "../components/mpu_process"                         # Thư mục chứa các file header mpu_process.h
        "../components/gps_library"                         # Thư mục chứa các file header gps.h
    REQUIRES arduino
)

