idf_component_register(
    SRCS "gps.c"             # File mã nguồn chính của ứng dụng
    INCLUDE_DIRS .           # Thư mục hiện tại chứa file header
    REQUIRES arduino         # Liệt kê các component mà thư viện này phụ thuộc
    PRIV_REQUIRES adc_battery
)
