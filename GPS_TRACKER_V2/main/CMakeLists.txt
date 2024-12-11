set(EXTRA_COMPONENT_DIRS components)

idf_component_register(
    SRCS "main.cpp"  # File mã nguồn chính của ứng dụng
    INCLUDE_DIRS
        "../components/arduino/libraries/Adafruit_MPU6050"  # Thư mục chứa Adafruit_MPU6050.h
        "../components/arduino/libraries/Adafruit_BusIO"    # Thư mục chứa Adafruit_BusIO.h
        "../components/arduino/libraries/Adafruit_Unified_Sensor"  # Thư mục chứa Adafruit_Unified_Sensor.h
        "../components/main_library"                       # Thư mục chứa các file header khác, ví dụ Adafruit_Sensor.h
    REQUIRES arduino
)

