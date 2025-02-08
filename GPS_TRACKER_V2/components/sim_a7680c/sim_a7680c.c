#include "sim_a7680c.h"
bool send_connect_mqtt_sever = false;
bool connect_mqtt_sever = false;

static const char *TAG = "SIM_A7680C_UART";

void uartsim_init(void) {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    // Use UART0 and configure its pins
    ESP_ERROR_CHECK(uart_driver_install(UART_SIM, UART_BUFFER * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_SIM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_SIM, TXD_PIN, RXD_PIN, RTS_PIN, CTS_PIN));
}

void send_at_command(const char *command) {
    char cmd_buffer[BUFFER_SIZE];
    snprintf(cmd_buffer, sizeof(cmd_buffer), "%s\r\n", command);
    ESP_LOGI(TAG, "Send AT: %s", cmd_buffer);
    uart_write_bytes(UART_SIM, cmd_buffer, strlen(cmd_buffer));
}

void read_uart_response(void)
{
    uint8_t buf[BUFFER_SIZE];
    int total_len = 0;
    char response[UART_BUFFER];
    memset(response, 0, sizeof(response));
    int64_t start_time = esp_timer_get_time();

    while (true) {
        int64_t now = esp_timer_get_time();
        if ((now - start_time) / 1000 > READ_TIMEOUT_MS) {
            break;
        }
        int len = uart_read_bytes(UART_SIM, buf, sizeof(buf), 50 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (total_len + len < UART_BUFFER) {
                memcpy(response + total_len, buf, len);
                total_len += len;
            }
            if (strstr(response, "\r\nOK\r\n") || strstr(response, "\r\nERROR\r\n")) {
                if(strstr(response, "\r\nOK\r\n"))
                {
                    connect_mqtt_sever = true;
                }
                break;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    if (total_len > 0) {
        response[total_len] = '\0';
        ESP_LOGI(TAG, "Response: %s", response);
    } else {
        ESP_LOGI(TAG, "No response or timeout");
    }
}

void mqtt_connect(void)
{   
    int retry_count = 0;
    bool connected = false;
    ESP_LOGI(TAG, "=== MQTT CONNECT START ===");
    while (retry_count < 5 && !connected) {

        ESP_LOGI(TAG, "Attempt %d to connect MQTT", retry_count + 1);
        send_at_command("AT+CMQTTDISC=0,60");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        read_uart_response();

        send_at_command("AT+CMQTTREL=0");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        read_uart_response();

        send_at_command("AT+CMQTTSTOP");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        read_uart_response();

        send_at_command("AT+CMQTTSTART");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        read_uart_response();

        send_at_command("AT+CMQTTACCQ=0,\"ESP32\",0");
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        read_uart_response();

        const char *mqtt_connect_cmd = "AT+CMQTTCONNECT=0,\"tcp://52.140.102.169:1883\",60,1";
        send_at_command(mqtt_connect_cmd);
        send_connect_mqtt_sever = true;
        vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        read_uart_response();
        send_connect_mqtt_sever = false;

        if (connect_mqtt_sever == true) {
            ESP_LOGI(TAG, "CONNECTED SUCESSFULLY");
            connect_mqtt_sever = false;
            break;
        } else {
            ESP_LOGI(TAG, "CONNECTED UNSUCESSFULLY");
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(2000));  // Đợi trước khi thử lại
        }
    }
    ESP_LOGI(TAG, "Attempt %d to connect MQTT", retry_count);
    ESP_LOGI(TAG, "=== MQTT CONNECT END ===");
}


void mqtt_publish(const char *topic, const char *message) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d", (int)strlen(topic));
    send_at_command(cmd);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    read_uart_response();

    send_at_command(topic);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    read_uart_response();

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d", (int)strlen(message));
    send_at_command(cmd);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    read_uart_response();

    send_at_command(message);
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    read_uart_response();

    send_at_command("AT+CMQTTPUB=0,0,60,1");
    vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
    read_uart_response();
}

void send_gps_data_to_mqtt(void) {
    char mqtt_payload[BUFFER_SIZE];
    snprintf(mqtt_payload, sizeof(mqtt_payload),
             "["
             "{\"name\":\"latitude\",\"value\":%.2f,\"timestamp\":\"%s\"},"
             "{\"name\":\"longitude\",\"value\":%.2f,\"timestamp\":\"%s\"},"
             "{\"name\":\"battery\",\"value\":\"%d%%\",\"timestamp\":\"%s\"},"
             "{\"name\":\"stolen\",\"value\":%s,\"timestamp\":\"%s\"}"
             "]",
             global_gps_data.latitude,
             global_gps_data.time,
             global_gps_data.longitude,
             global_gps_data.time,
             global_gps_data.battery_capacity,
             global_gps_data.time,
             global_gps_data.Stolen ? "true" : "false",
             global_gps_data.time);

    mqtt_publish("gps_tracker/data", mqtt_payload);
    ESP_LOGI(TAG, "Sent MQTT message: %s", mqtt_payload);
}
