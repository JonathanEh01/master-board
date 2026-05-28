#include "uart_imu.h"
#include "imu_common.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/uart_ll.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UART_NUM UART_NUM_1
#define BUF_SIZE 128
#define PIN_TXD 32
#define PIN_RXD 35

#define ACCX_POS 6
#define ACCY_POS 10
#define ACCZ_POS 14
#define GYRX_POS 20
#define GYRY_POS 24
#define GYRZ_POS 28

#define EFR_POS 6
#define EFP_POS 10
#define EFY_POS 14
#define EFLINACCX_POS 22
#define EFLINACCY_POS 26
#define EFLINACCZ_POS 30

static const char *UART_TAG = "imu_microstrain_mip";
static QueueHandle_t uart_queue;
static QueueHandle_t imu_mailbox;
static QueueHandle_t ef_mailbox;

static uint8_t rxbuf_imu[128] = {0};
static uint8_t rxbuf_ef[128] = {0};

static float read_be_float(const uint8_t *buffer, size_t offset)
{
    union
    {
        uint32_t u;
        float f;
    } value;

    value.u = ((uint32_t)buffer[offset] << 24) |
              ((uint32_t)buffer[offset + 1] << 16) |
              ((uint32_t)buffer[offset + 2] << 8) |
              (uint32_t)buffer[offset + 3];
    return value.f;
}

static void process_uart_bytes(const uint8_t *rxbuf, size_t rx_fifo_len)
{
    size_t i = 0;

    while ((i + 4) < rx_fifo_len)
    {
        if (rxbuf[i] != 0x75 || rxbuf[i + 1] != 0x65)
        {
            ESP_LOGW(UART_TAG, "unexpected MIP header");
            break;
        }

        const int size = rxbuf[i + 3] + 2 + 4;
        if ((size_t)size > (rx_fifo_len - i))
        {
            ESP_LOGW(UART_TAG, "MIP data length mismatch");
            break;
        }

        switch (rxbuf[i + 2])
        {
        case 0x80:
            xQueueOverwrite(imu_mailbox, &rxbuf[i]);
            break;

        case 0x82:
            xQueueOverwrite(ef_mailbox, &rxbuf[i]);
            break;

        default:
            break;
        }

        i += size;
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
                const int len = uart_read_bytes(
                    UART_NUM,
                    rxbuf,
                    event.size < sizeof(rxbuf) ? event.size : sizeof(rxbuf),
                    0);
                if (len > 0)
                {
                    process_uart_bytes(rxbuf, (size_t)len);
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

static bool check_mip_checksum(const uint8_t *data, int len)
{
    if (len < 2)
        return false;

    uint8_t checksum_byte1 = 0;
    uint8_t checksum_byte2 = 0;
    for (int i = 0; i < (len - 2); i++)
    {
        checksum_byte1 += data[i];
        checksum_byte2 += checksum_byte1;
    }

    return data[len - 2] == checksum_byte1 && data[len - 1] == checksum_byte2;
}

int parse_IMU_data()
{
    if (xQueuePeek(imu_mailbox, &rxbuf_imu, 0) == pdTRUE &&
        check_mip_checksum(rxbuf_imu, 34))
    {
        imu_common_set_acc_g(
            read_be_float(rxbuf_imu, ACCX_POS),
            read_be_float(rxbuf_imu, ACCY_POS),
            read_be_float(rxbuf_imu, ACCZ_POS));
        imu_common_set_gyro_rads(
            read_be_float(rxbuf_imu, GYRX_POS),
            read_be_float(rxbuf_imu, GYRY_POS),
            read_be_float(rxbuf_imu, GYRZ_POS));
    }

    if (xQueuePeek(ef_mailbox, &rxbuf_ef, 0) == pdTRUE &&
        check_mip_checksum(rxbuf_ef, 38))
    {
        imu_common_set_attitude_rad(
            read_be_float(rxbuf_ef, EFR_POS),
            read_be_float(rxbuf_ef, EFP_POS),
            read_be_float(rxbuf_ef, EFY_POS));
        imu_common_set_linear_acc_g(
            read_be_float(rxbuf_ef, EFLINACCX_POS),
            read_be_float(rxbuf_ef, EFLINACCY_POS),
            read_be_float(rxbuf_ef, EFLINACCZ_POS));
    }

    return 0;
}

int imu_init()
{
    imu_common_reset();

    imu_mailbox = xQueueCreate(1, 128);
    ef_mailbox = xQueueCreate(1, 128);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    const int uart_buffer_size = BUF_SIZE * 2;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, 0, 10, &uart_queue, 0));

    const uint8_t cmd0[8] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x02, 0xE1, 0xC7};
    const uint8_t cmd1[16] = {0x75, 0x65, 0x0C, 0x0A, 0x0A, 0x08, 0x01, 0x02, 0x04, 0x00, 0x01, 0x05, 0x00, 0x01, 0x10, 0x73};
    const uint8_t cmd2[16] = {0x75, 0x65, 0x0C, 0x0A, 0x0A, 0x0A, 0x01, 0x02, 0x05, 0x00, 0x01, 0x0D, 0x00, 0x01, 0x1B, 0xA3};
    const uint8_t cmd3[16] = {0x75, 0x65, 0x0C, 0x0A, 0x05, 0x11, 0x01, 0x01, 0x01, 0x05, 0x11, 0x01, 0x03, 0x01, 0x24, 0xCC};
    const uint8_t cmd4[12] = {0x75, 0x65, 0x0D, 0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0xF6, 0xE4};
    const uint8_t cmd5[8] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x06, 0xE5, 0xCB};
    const uint8_t cmd6[13] = {0x75, 0x65, 0x0C, 0x07, 0x07, 0x40, 0x01, 0x00, 0x0E, 0x10, 0x00, 0x53, 0x9D};

    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write_bytes(UART_NUM, cmd0, sizeof(cmd0));
    vTaskDelay(3);
    uart_write_bytes(UART_NUM, cmd1, sizeof(cmd1));
    vTaskDelay(3);
    uart_write_bytes(UART_NUM, cmd2, sizeof(cmd2));
    vTaskDelay(3);
    uart_write_bytes(UART_NUM, cmd3, sizeof(cmd3));
    vTaskDelay(3);
    uart_write_bytes(UART_NUM, cmd4, sizeof(cmd4));
    vTaskDelay(3);
    uart_write_bytes(UART_NUM, cmd5, sizeof(cmd5));
    vTaskDelay(3);
    uart_write_bytes(UART_NUM, cmd6, sizeof(cmd6));
    vTaskDelay(3);

    uart_set_baudrate(UART_NUM, 921600);

    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
        .rx_timeout_thresh = 3,
        .rxfifo_full_thresh = 16,
        .txfifo_empty_intr_thresh = 0,
    };
    ESP_ERROR_CHECK(uart_intr_config(UART_NUM, &uart_intr));
    xTaskCreate(uart_event_task, "imu_mip_uart", 4096, NULL, 10, NULL);

    return 0;
}
