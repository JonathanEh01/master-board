#include "uart_imu.h"
#include "imu_common.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/uart_ll.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define UART_NUM UART_NUM_1
#define PIN_TXD 32
#define PIN_RXD 35

#define BNO_UART_BAUD 3'000'000
#define BNO_UART_RX_BUF_SIZE 2'048
#define BNO_UART_EVENT_QUEUE_SIZE 16

#define BNO_UART_FLAG 0x7E
#define BNO_UART_ESCAPE 0x7D
#define BNO_UART_ESCAPE_XOR 0x20
#define BNO_UART_PROTOCOL_CONTROL 0x00
#define BNO_UART_PROTOCOL_SHTP 0x01

#define SHTP_HEADER_LEN 4
#define SHTP_MAX_FRAME_LEN 512
#define SHTP_CHANNEL_COMMAND 0
#define SHTP_CHANNEL_EXECUTABLE 1
#define SHTP_CHANNEL_CONTROL 2
#define SHTP_CHANNEL_INPUT_NORMAL 3
#define SHTP_CHANNEL_INPUT_WAKE 4
#define SHTP_CHANNEL_GYRO_RV 5

#define SH2_REPORT_ACCELEROMETER 0x01
#define SH2_REPORT_GYROSCOPE_CALIBRATED 0x02
#define SH2_REPORT_LINEAR_ACCELERATION 0x04
#define SH2_REPORT_ROTATION_VECTOR 0x05
#define SH2_REPORT_GAME_ROTATION_VECTOR 0x08
#define SH2_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SH2_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SH2_REPORT_SET_FEATURE 0xFD
#define SH2_REPORT_GET_FEATURE_RESPONSE 0xFC
#define SH2_REPORT_TIMEBASE_REFERENCE 0xFB
#define SH2_REPORT_TIMESTAMP_REBASE 0xFA
#define SH2_REPORT_FRS_WRITE_REQUEST 0xF7
#define SH2_REPORT_FRS_WRITE_DATA 0xF6
#define SH2_REPORT_FRS_WRITE_RESPONSE 0xF5
#define SH2_REPORT_COMMAND_REQUEST 0xF2
#define SH2_REPORT_COMMAND_RESPONSE 0xF1

#define SH2_REPORT_LEN_VECTOR 10
#define SH2_REPORT_LEN_GAME_RV 12
#define SH2_REPORT_LEN_RV 14
#define SH2_REPORT_LEN_TIMEBASE 5
#define SH2_REPORT_LEN_PRODUCT_ID_RESPONSE 16
#define SH2_REPORT_LEN_FEATURE_RESPONSE 17
#define SH2_REPORT_LEN_FRS_WRITE_RESPONSE 4
#define SH2_REPORT_LEN_COMMAND_RESPONSE 16

#define SH2_Q_ACCEL 8
#define SH2_Q_GYRO 9
#define SH2_Q_QUATERNION 14

#define SH2_FRS_SYSTEM_ORIENTATION 0x2D3E
#define SH2_FRS_WRITE_STATUS_WORDS_RECEIVED 0
#define SH2_FRS_WRITE_STATUS_COMPLETED 3
#define SH2_FRS_WRITE_STATUS_READY 4
#define SH2_FRS_WRITE_STATUS_RECORD_VALID 8

#define SH2_COMMAND_TARE 0x03
#define SH2_COMMAND_INITIALIZE 0x04
#define SH2_COMMAND_INITIALIZE_UNSOLICITED 0x84
#define SH2_TARE_SET_REORIENTATION 0x02
#define SH2_EXECUTABLE_RESET_COMPLETE 0x01

#define STANDARD_GRAVITY_MPS2 9.80665f

#ifndef CONFIG_IMU_BNO08X_PERSIST_ORIENTATION_FRS
#define CONFIG_IMU_BNO08X_PERSIST_ORIENTATION_FRS 0
#endif

static const char *UART_TAG = "imu_bno08x_shtp";

static QueueHandle_t uart_queue;
static uint8_t tx_sequence[6];
static uint8_t command_sequence;
static uint8_t last_report_sequence[256];
static bool report_sequence_seen[256];
static volatile bool bno_executable_reset_seen;
static volatile bool bno_sh2_init_seen;
static volatile bool bno_product_id_seen;
static volatile uint8_t frs_write_response_count;
static volatile uint8_t frs_write_status;
static volatile uint16_t frs_write_offset;

struct uart_frame_parser
{
    uint8_t frame[SHTP_MAX_FRAME_LEN];
    size_t len;
    bool in_frame;
    bool escaped;
};

static struct uart_frame_parser parser;

static int16_t read_i16_le(const uint8_t *buffer, size_t offset)
{
    return (int16_t)((uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8));
}

static void write_u16_le(uint8_t *buffer, size_t offset, uint16_t value)
{
    buffer[offset] = (uint8_t)(value & 0xFF);
    buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

static void write_i16_le(uint8_t *buffer, size_t offset, int16_t value)
{
    write_u16_le(buffer, offset, (uint16_t)value);
}

static void write_u32_le(uint8_t *buffer, size_t offset, uint32_t value)
{
    buffer[offset] = (uint8_t)(value & 0xFF);
    buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
    buffer[offset + 2] = (uint8_t)((value >> 16) & 0xFF);
    buffer[offset + 3] = (uint8_t)((value >> 24) & 0xFF);
}

static void write_i32_le(uint8_t *buffer, size_t offset, int32_t value)
{
    write_u32_le(buffer, offset, (uint32_t)value);
}

static float fixed_to_float(int16_t value, int q_point)
{
    return (float)value / (float)(1 << q_point);
}

static void check_report_sequence(uint8_t report_id, uint8_t sequence)
{
    if (report_sequence_seen[report_id] &&
        (uint8_t)(last_report_sequence[report_id] + 1) != sequence)
    {
        ESP_LOGW(UART_TAG, "report 0x%02x sequence jump %u -> %u",
                 report_id,
                 last_report_sequence[report_id],
                 sequence);
    }

    report_sequence_seen[report_id] = true;
    last_report_sequence[report_id] = sequence;
}

static void handle_vector_report(uint8_t report_id, const uint8_t *report)
{
    check_report_sequence(report_id, report[1]);

    const float x = fixed_to_float(read_i16_le(report, 4), SH2_Q_ACCEL) / STANDARD_GRAVITY_MPS2;
    const float y = fixed_to_float(read_i16_le(report, 6), SH2_Q_ACCEL) / STANDARD_GRAVITY_MPS2;
    const float z = fixed_to_float(read_i16_le(report, 8), SH2_Q_ACCEL) / STANDARD_GRAVITY_MPS2;

    if (report_id == SH2_REPORT_ACCELEROMETER)
    {
        imu_common_set_acc_g(x, y, z);
    }
    else if (report_id == SH2_REPORT_LINEAR_ACCELERATION)
    {
        imu_common_set_linear_acc_g(x, y, z);
    }
}

static void handle_gyro_report(const uint8_t *report)
{
    check_report_sequence(SH2_REPORT_GYROSCOPE_CALIBRATED, report[1]);

    imu_common_set_gyro_rads(
        fixed_to_float(read_i16_le(report, 4), SH2_Q_GYRO),
        fixed_to_float(read_i16_le(report, 6), SH2_Q_GYRO),
        fixed_to_float(read_i16_le(report, 8), SH2_Q_GYRO));
}

static void handle_rotation_vector_report(uint8_t report_id, const uint8_t *report)
{
    check_report_sequence(report_id, report[1]);

    const float x = fixed_to_float(read_i16_le(report, 4), SH2_Q_QUATERNION);
    const float y = fixed_to_float(read_i16_le(report, 6), SH2_Q_QUATERNION);
    const float z = fixed_to_float(read_i16_le(report, 8), SH2_Q_QUATERNION);
    const float w = fixed_to_float(read_i16_le(report, 10), SH2_Q_QUATERNION);

    imu_common_set_quaternion(w, x, y, z);
}

static void handle_frs_write_response(const uint8_t *report)
{
    frs_write_status = report[1];
    frs_write_offset = (uint16_t)report[2] | ((uint16_t)report[3] << 8);
    frs_write_response_count++;

    ESP_LOGD(UART_TAG, "FRS write response status %u offset %u",
             frs_write_status,
             frs_write_offset);
}

static void handle_command_response(const uint8_t *report)
{
    ESP_LOGD(UART_TAG,
             "command response cmd 0x%02x seq %u status %u",
             report[2],
             report[3],
             report[5]);

    if (report[2] == SH2_COMMAND_INITIALIZE ||
        report[2] == SH2_COMMAND_INITIALIZE_UNSOLICITED)
    {
        bno_sh2_init_seen = (report[5] == 0);
    }
}

static void handle_product_id_response(const uint8_t *report)
{
    const uint16_t sw_patch = (uint16_t)report[12] | ((uint16_t)report[13] << 8);

    bno_product_id_seen = true;
    ESP_LOGI(UART_TAG,
             "BNO08x product ID reset cause %u SW %u.%u.%u",
             (unsigned)report[1],
             (unsigned)report[2],
             (unsigned)report[3],
             (unsigned)sw_patch);
}

static size_t report_len(uint8_t report_id)
{
    switch (report_id)
    {
    case SH2_REPORT_ACCELEROMETER:
    case SH2_REPORT_GYROSCOPE_CALIBRATED:
    case SH2_REPORT_LINEAR_ACCELERATION:
        return SH2_REPORT_LEN_VECTOR;

    case SH2_REPORT_GAME_ROTATION_VECTOR:
        return SH2_REPORT_LEN_GAME_RV;

    case SH2_REPORT_ROTATION_VECTOR:
        return SH2_REPORT_LEN_RV;

    case SH2_REPORT_TIMEBASE_REFERENCE:
        return SH2_REPORT_LEN_TIMEBASE;

    case SH2_REPORT_PRODUCT_ID_RESPONSE:
        return SH2_REPORT_LEN_PRODUCT_ID_RESPONSE;

    case SH2_REPORT_GET_FEATURE_RESPONSE:
        return SH2_REPORT_LEN_FEATURE_RESPONSE;

    case SH2_REPORT_FRS_WRITE_RESPONSE:
        return SH2_REPORT_LEN_FRS_WRITE_RESPONSE;

    case SH2_REPORT_COMMAND_RESPONSE:
        return SH2_REPORT_LEN_COMMAND_RESPONSE;

    case SH2_REPORT_TIMESTAMP_REBASE:
        return 5;

    default:
        return 0;
    }
}

static void process_sh2_reports(uint8_t channel, const uint8_t *cargo, size_t cargo_len)
{
    size_t offset = 0;

    while (offset < cargo_len)
    {
        const uint8_t report_id = cargo[offset];
        const size_t len = report_len(report_id);

        if (len == 0 || offset + len > cargo_len)
        {
            ESP_LOGD(UART_TAG, "unhandled report 0x%02x on channel %u len %u",
                     report_id,
                     channel,
                     (unsigned)(cargo_len - offset));
            break;
        }

        const uint8_t *report = &cargo[offset];
        switch (report_id)
        {
        case SH2_REPORT_ACCELEROMETER:
        case SH2_REPORT_LINEAR_ACCELERATION:
            handle_vector_report(report_id, report);
            break;

        case SH2_REPORT_GYROSCOPE_CALIBRATED:
            handle_gyro_report(report);
            break;

        case SH2_REPORT_ROTATION_VECTOR:
        case SH2_REPORT_GAME_ROTATION_VECTOR:
            handle_rotation_vector_report(report_id, report);
            break;

        case SH2_REPORT_FRS_WRITE_RESPONSE:
            handle_frs_write_response(report);
            break;

        case SH2_REPORT_COMMAND_RESPONSE:
            handle_command_response(report);
            break;

        case SH2_REPORT_PRODUCT_ID_RESPONSE:
            handle_product_id_response(report);
            break;

        case SH2_REPORT_TIMEBASE_REFERENCE:
        case SH2_REPORT_TIMESTAMP_REBASE:
        case SH2_REPORT_GET_FEATURE_RESPONSE:
        default:
            break;
        }

        offset += len;
    }
}

static void handle_shtp_frame(const uint8_t *payload, size_t len)
{
    if (len < SHTP_HEADER_LEN)
        return;

    const uint16_t raw_len = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    const bool continuation = (raw_len & 0x8000) != 0;
    const uint16_t packet_len = raw_len & 0x7FFF;
    const uint8_t channel = payload[2];

    if (continuation)
    {
        ESP_LOGW(UART_TAG, "fragmented SHTP packet ignored on channel %u", channel);
        return;
    }

    if (packet_len < SHTP_HEADER_LEN || packet_len > len)
    {
        ESP_LOGW(UART_TAG, "bad SHTP length %u in UART payload %u",
                 packet_len,
                 (unsigned)len);
        return;
    }

    const uint8_t *cargo = &payload[SHTP_HEADER_LEN];
    const size_t cargo_len = packet_len - SHTP_HEADER_LEN;

    if (channel == SHTP_CHANNEL_EXECUTABLE)
    {
        if (cargo_len >= 1 && cargo[0] == SH2_EXECUTABLE_RESET_COMPLETE)
        {
            bno_executable_reset_seen = true;
            ESP_LOGD(UART_TAG, "BNO08x executable reset complete");
        }
        return;
    }

    if (channel == SHTP_CHANNEL_COMMAND)
        return;

    process_sh2_reports(channel, cargo, cargo_len);
}

static void handle_uart_frame(const uint8_t *frame, size_t len)
{
    if (len == 0)
        return;

    const uint8_t protocol_id = frame[0];
    if (protocol_id == BNO_UART_PROTOCOL_SHTP)
    {
        handle_shtp_frame(&frame[1], len - 1);
    }
    else if (protocol_id == BNO_UART_PROTOCOL_CONTROL)
    {
        if (len >= 3)
        {
            const uint16_t host_write_available = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
            ESP_LOGD(UART_TAG, "BNO write credit %u bytes", (unsigned)host_write_available);
        }
    }
}

static void process_uart_byte(uint8_t byte)
{
    if (byte == BNO_UART_FLAG)
    {
        if (parser.in_frame && parser.len > 0)
        {
            handle_uart_frame(parser.frame, parser.len);
        }

        parser.in_frame = true;
        parser.escaped = false;
        parser.len = 0;
        return;
    }

    if (!parser.in_frame)
        return;

    if (parser.escaped)
    {
        byte ^= BNO_UART_ESCAPE_XOR;
        parser.escaped = false;
    }
    else if (byte == BNO_UART_ESCAPE)
    {
        parser.escaped = true;
        return;
    }

    if (parser.len >= sizeof(parser.frame))
    {
        ESP_LOGW(UART_TAG, "UART frame too large, dropping");
        parser.in_frame = false;
        parser.len = 0;
        parser.escaped = false;
        return;
    }

    parser.frame[parser.len++] = byte;
}

static void process_uart_bytes(const uint8_t *rxbuf, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        process_uart_byte(rxbuf[i]);
    }
}

static void uart_event_task(void *arg)
{
    uart_event_t event;
    uint8_t rxbuf[512];

    while (true)
    {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                size_t buffered = event.size;
                while (buffered > 0)
                {
                    const size_t to_read = buffered < sizeof(rxbuf) ? buffered : sizeof(rxbuf);
                    const int len = uart_read_bytes(UART_NUM, rxbuf, to_read, pdMS_TO_TICKS(2));
                    if (len <= 0)
                        break;

                    process_uart_bytes(rxbuf, (size_t)len);

                    if (uart_get_buffered_data_len(UART_NUM, &buffered) != ESP_OK)
                        break;
                }
                break;
            }

            case UART_FIFO_OVF:
                ESP_ERROR_CHECK(uart_flush_input(UART_NUM));
                xQueueReset(uart_queue);
                break;

            case UART_BUFFER_FULL:
                ESP_ERROR_CHECK(uart_flush_input(UART_NUM));
                xQueueReset(uart_queue);
                break;

            case UART_BREAK:
            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
            default:
                break;
            }
        }
    }
}

static void uart_write_byte_spaced(uint8_t byte)
{
    uart_write_bytes(UART_NUM, &byte, 1);
    uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(20));
    esp_rom_delay_us(120);
}

static void uart_write_escaped_byte(uint8_t byte)
{
    if (byte == BNO_UART_FLAG || byte == BNO_UART_ESCAPE)
    {
        uart_write_byte_spaced(BNO_UART_ESCAPE);
        uart_write_byte_spaced(byte ^ BNO_UART_ESCAPE_XOR);
    }
    else
    {
        uart_write_byte_spaced(byte);
    }
}

static void write_uart_frame(uint8_t protocol_id, const uint8_t *payload, size_t payload_len)
{
    uart_write_byte_spaced(BNO_UART_FLAG);
    uart_write_escaped_byte(protocol_id);
    for (size_t i = 0; i < payload_len; ++i)
    {
        uart_write_escaped_byte(payload[i]);
    }
    uart_write_byte_spaced(BNO_UART_FLAG);
}

static void send_shtp_packet(uint8_t channel, const uint8_t *cargo, size_t cargo_len)
{
    uint8_t packet[SHTP_HEADER_LEN + 32];
    const size_t packet_len = SHTP_HEADER_LEN + cargo_len;

    if (packet_len > sizeof(packet))
    {
        ESP_LOGE(UART_TAG, "SHTP TX packet too large: %u", (unsigned)packet_len);
        return;
    }

    packet[0] = (uint8_t)(packet_len & 0xFF);
    packet[1] = (uint8_t)((packet_len >> 8) & 0x7F);
    packet[2] = channel;
    packet[3] = tx_sequence[channel]++;
    memcpy(&packet[SHTP_HEADER_LEN], cargo, cargo_len);

    write_uart_frame(BNO_UART_PROTOCOL_SHTP, packet, packet_len);
}

static bool wait_for_bno_startup(TickType_t timeout_ticks)
{
    const TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < timeout_ticks)
    {
        if (bno_executable_reset_seen && bno_sh2_init_seen)
            return true;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(UART_TAG,
             "BNO08x startup sync timed out: exec=%u sh2=%u",
             (unsigned)bno_executable_reset_seen,
             (unsigned)bno_sh2_init_seen);
    return false;
}

static bool wait_for_product_id_response(TickType_t timeout_ticks)
{
    const TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < timeout_ticks)
    {
        if (bno_product_id_seen)
            return true;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(UART_TAG, "BNO08x product ID response timed out");
    return false;
}

static void reset_bno_startup_markers(void)
{
    bno_executable_reset_seen = false;
    bno_sh2_init_seen = false;
    bno_product_id_seen = false;
}

static bool is_successful_frs_write_status(uint8_t status)
{
    return status == SH2_FRS_WRITE_STATUS_WORDS_RECEIVED ||
           status == SH2_FRS_WRITE_STATUS_COMPLETED ||
           status == SH2_FRS_WRITE_STATUS_READY ||
           status == SH2_FRS_WRITE_STATUS_RECORD_VALID;
}

static bool wait_for_frs_write_response(uint8_t previous_count,
                                        const char *operation,
                                        TickType_t timeout_ticks)
{
    const TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < timeout_ticks)
    {
        if (frs_write_response_count != previous_count)
        {
            if (!is_successful_frs_write_status(frs_write_status))
            {
                ESP_LOGW(UART_TAG,
                         "%s failed: FRS status %u offset %u",
                         operation,
                         frs_write_status,
                         frs_write_offset);
                return false;
            }

            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGW(UART_TAG, "%s timed out waiting for FRS response", operation);
    return false;
}

static bool send_frs_write_request(uint16_t frs_type, uint16_t word_len)
{
    uint8_t payload[6] = {0};

    payload[0] = SH2_REPORT_FRS_WRITE_REQUEST;
    write_u16_le(payload, 2, word_len);
    write_u16_le(payload, 4, frs_type);

    const uint8_t previous_count = frs_write_response_count;
    send_shtp_packet(SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
    return wait_for_frs_write_response(previous_count,
                                       "FRS write request",
                                       pdMS_TO_TICKS(200));
}

static bool send_frs_write_data(uint16_t word_offset, int32_t data0, int32_t data1)
{
    uint8_t payload[12] = {0};

    payload[0] = SH2_REPORT_FRS_WRITE_DATA;
    write_u16_le(payload, 2, word_offset);
    write_i32_le(payload, 4, data0);
    write_i32_le(payload, 8, data1);

    const uint8_t previous_count = frs_write_response_count;
    send_shtp_packet(SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
    return wait_for_frs_write_response(previous_count,
                                       "FRS write data",
                                       pdMS_TO_TICKS(200));
}

static void persist_system_orientation_frs(void)
{
    const int32_t qx = CONFIG_IMU_BNO08X_ORIENTATION_QX_Q30;
    const int32_t qy = CONFIG_IMU_BNO08X_ORIENTATION_QY_Q30;
    const int32_t qz = CONFIG_IMU_BNO08X_ORIENTATION_QZ_Q30;
    const int32_t qw = CONFIG_IMU_BNO08X_ORIENTATION_QW_Q30;

    if (!send_frs_write_request(SH2_FRS_SYSTEM_ORIENTATION, 4))
        return;

    if (!send_frs_write_data(0, qx, qy))
        return;

    if (!send_frs_write_data(2, qz, qw))
        return;

    ESP_LOGI(UART_TAG, "BNO08x system orientation FRS write requested");
}

static void send_runtime_reorientation(void)
{
    uint8_t payload[12] = {0};

    payload[0] = SH2_REPORT_COMMAND_REQUEST;
    payload[1] = command_sequence++;
    payload[2] = SH2_COMMAND_TARE;
    payload[3] = SH2_TARE_SET_REORIENTATION;
    write_i16_le(payload, 4, (int16_t)CONFIG_IMU_BNO08X_ORIENTATION_QX_Q14);
    write_i16_le(payload, 6, (int16_t)CONFIG_IMU_BNO08X_ORIENTATION_QY_Q14);
    write_i16_le(payload, 8, (int16_t)CONFIG_IMU_BNO08X_ORIENTATION_QZ_Q14);
    write_i16_le(payload, 10, (int16_t)CONFIG_IMU_BNO08X_ORIENTATION_QW_Q14);

    send_shtp_packet(SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
    ESP_LOGI(UART_TAG,
             "BNO08x runtime orientation q14 x=%d y=%d z=%d w=%d",
             CONFIG_IMU_BNO08X_ORIENTATION_QX_Q14,
             CONFIG_IMU_BNO08X_ORIENTATION_QY_Q14,
             CONFIG_IMU_BNO08X_ORIENTATION_QZ_Q14,
             CONFIG_IMU_BNO08X_ORIENTATION_QW_Q14);
}

static void configure_bno_orientation(void)
{
    if (CONFIG_IMU_BNO08X_PERSIST_ORIENTATION_FRS)
    {
        persist_system_orientation_frs();
    }

    send_runtime_reorientation();
}

static void send_device_on(void)
{
    const uint8_t payload[] = {0x02};
    send_shtp_packet(SHTP_CHANNEL_EXECUTABLE, payload, sizeof(payload));
}

static void send_device_reset(void)
{
    const uint8_t payload[] = {0x01};
    send_shtp_packet(SHTP_CHANNEL_EXECUTABLE, payload, sizeof(payload));
}

static void send_product_id_request(void)
{
    const uint8_t payload[] = {SH2_REPORT_PRODUCT_ID_REQUEST, 0};

    bno_product_id_seen = false;
    send_shtp_packet(SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
}

static void send_set_feature(uint8_t report_id, uint32_t interval_us)
{
    uint8_t payload[17] = {0};  // total packet length of 21 bytes including 4 bytes SHTP header

    payload[0] = SH2_REPORT_SET_FEATURE;
    payload[1] = report_id;
    payload[2] = 0;
    write_u32_le(payload, 5, interval_us);
    write_u32_le(payload, 9, 0);
    write_u32_le(payload, 13, 0);

    send_shtp_packet(SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
}

static void configure_bno_reports(void)
{
    const uint32_t interval_us = CONFIG_IMU_BNO08X_REPORT_INTERVAL_US;

    send_device_on();
    configure_bno_orientation();
    send_set_feature(SH2_REPORT_ACCELEROMETER, interval_us);
    send_set_feature(SH2_REPORT_GYROSCOPE_CALIBRATED, interval_us);
    send_set_feature(SH2_REPORT_LINEAR_ACCELERATION, interval_us);
    send_set_feature(SH2_REPORT_GAME_ROTATION_VECTOR, interval_us);
}

int parse_IMU_data()
{
    return 0;
}

int imu_init()
{
    imu_common_reset();

    uart_config_t uart_config = {
        .baud_rate = BNO_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM,
                                        BNO_UART_RX_BUF_SIZE,
                                        0,
                                        BNO_UART_EVENT_QUEUE_SIZE,
                                        &uart_queue,
                                        0));

    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
        .rx_timeout_thresh = 3,
        .rxfifo_full_thresh = 64,
        .txfifo_empty_intr_thresh = 0,
    };
    ESP_ERROR_CHECK(uart_intr_config(UART_NUM, &uart_intr));

    xTaskCreate(uart_event_task, "imu_bno_uart", 6144, NULL, 10, NULL);

    reset_bno_startup_markers();
    send_device_reset();
    memset(tx_sequence, 0, sizeof(tx_sequence));
    command_sequence = 0;
    wait_for_bno_startup(pdMS_TO_TICKS(1000));
    send_product_id_request();
    wait_for_product_id_response(pdMS_TO_TICKS(300));
    configure_bno_reports();

    ESP_LOGI(UART_TAG, "BNO08x SHTP UART initialized at %d baud", BNO_UART_BAUD);
    return 0;
}
