idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "../components/arduino/libraries/Adafruit_MPU6050"  # Thêm thư mục chứa Adafruit_MPU6050.h
    INCLUDE_DIRS "../components/arduino/libraries/Adafruit_BusIO"  # Thêm thư mục chứa Adafruit_BusIO.h
    INCLUDE_DIRS "../components/arduino/libraries/Adafruit_Unified_Sensor"  # Thêm thư mục chứa Adafruit_Unified_Sensor.h
    REQUIRES arduino
)
