#ifndef SIM_A7680C_H
#define SIM_A7680C_H



#ifdef __cplusplus
extern "C" {
#endif

//Library
#include "esp_err.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"
#include "gps.h"

//DEFINE
#define UART_SIM        UART_NUM_0
#define TXD_PIN         GPIO_NUM_1 // Default TXD pin for UART0
#define RXD_PIN         GPIO_NUM_3 // Default RXD pin for UART0
#define RTS_PIN         UART_PIN_NO_CHANGE
#define CTS_PIN         UART_PIN_NO_CHANGE
#define UART_BUFFER     1024
#define BAUD_RATE       115200
#define BUFFER_SIZE     300

#define READ_TIMEOUT_MS 5000
#define COMMAND_DELAY_MS 2000
// Function declarations
void uartsim_init(void);
void send_at_command(const char *command);
void read_uart_response(void);
void mqtt_connect(void);
void mqtt_publish(const char *topic, const char *message);
void send_gps_data_to_mqtt(void);

#ifdef __cplusplus
}
#endif

#endif // SIM_A7680C_H
