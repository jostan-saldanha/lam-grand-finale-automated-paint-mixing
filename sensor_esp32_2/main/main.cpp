// LAM Research Challenge 2026 - Stage 3 - Day 1 (Industrial Paint Mixing)
// ESP32 #2 (U7) = SENSOR NODE (Native ESP-IDF v5.x)
//   Counts flow-sensor pulses FS1..FS5 with hardware pulse counter (PCNT) and streams
//   running totals to ESP32 #1 over UART at 50 Hz.
//
// Frame format:  $seq,ms,c1,c2,c3,c4,c5,fsw*HH\n
//   c1..c5 = cumulative pulse totals (32-bit)
//   fsw    = bit0: FSW5 active, bit1: FSW6 active
//   HH     = XOR checksum of characters between '$' and '*', two hex digits

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "config.h"

static const char *TAG = "SENSOR_NODE";

static const int FS_PIN[5] = {PIN_FS1, PIN_FS2, PIN_FS3, PIN_FS4, PIN_FS5};

static pcnt_unit_handle_t unitH[5];
static bool     unitOk[5]   = {false, false, false, false, false};
static int      lastRaw[5]  = {0, 0, 0, 0, 0};
static uint32_t total[5]    = {0, 0, 0, 0, 0};
static uint32_t seqNo       = 0;

static void initCounter(int i) {
    if (FS_PIN[i] < 0) {
        ESP_LOGW(TAG, "FS%d pin not configured - skipped", i + 1);
        return;
    }

    gpio_set_pull_mode((gpio_num_t)FS_PIN[i], GPIO_PULLUP_ONLY);

    pcnt_unit_config_t uc = {};
    uc.low_limit  = -32768;
    uc.high_limit = 32767;
    uc.flags.accum_count = 0;

    esp_err_t err = pcnt_new_unit(&uc, &unitH[i]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FS%d unit creation failed: %s", i + 1, esp_err_to_name(err));
        return;
    }

    pcnt_chan_config_t cc = {};
    cc.edge_gpio_num  = FS_PIN[i];
    cc.level_gpio_num = -1;

    pcnt_channel_handle_t ch = NULL;
    err = pcnt_new_channel(unitH[i], &cc, &ch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FS%d channel creation failed: %s", i + 1, esp_err_to_name(err));
        return;
    }

    pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(ch, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    pcnt_glitch_filter_config_t fc = {};
    fc.max_glitch_ns = PCNT_GLITCH_NS;
    pcnt_unit_set_glitch_filter(unitH[i], &fc);

    pcnt_unit_enable(unitH[i]);
    pcnt_unit_clear_count(unitH[i]);
    pcnt_unit_start(unitH[i]);
    unitOk[i] = true;
    ESP_LOGI(TAG, "FS%d counting on GPIO %d", i + 1, FS_PIN[i]);
}

static void pollCounters() {
    for (int i = 0; i < 5; i++) {
        if (!unitOk[i]) continue;
        int c = 0;
        pcnt_unit_get_count(unitH[i], &c);
        int d = c - lastRaw[i];
        if (d < 0) d += 32768;
        total[i] += (uint32_t)d;
        if (c > 16000) {
            pcnt_unit_clear_count(unitH[i]);
            lastRaw[i] = 0;
        } else {
            lastRaw[i] = c;
        }
    }
}

static uint32_t readFloats() {
    uint32_t v = 0;
    if (PIN_FSW5 >= 0 && gpio_get_level((gpio_num_t)PIN_FSW5) == 0) v |= 1;
    if (PIN_FSW6 >= 0 && gpio_get_level((gpio_num_t)PIN_FSW6) == 0) v |= 2;
    return v;
}

static void sendFrame() {
    char body[100];
    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);
    int n = snprintf(body, sizeof(body), "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
                     (unsigned long)seqNo++, (unsigned long)nowMs,
                     (unsigned long)total[0], (unsigned long)total[1], (unsigned long)total[2],
                     (unsigned long)total[3], (unsigned long)total[4], (unsigned long)readFloats());

    uint8_t x = 0;
    for (int i = 0; i < n; i++) x ^= (uint8_t)body[i];

    char out[128];
    int m = snprintf(out, sizeof(out), "$%s*%02X\n", body, x);
    uart_write_bytes(LINK_UART_NUM, out, m);
}

static void initHardware() {
    // 1. Configure Float switches
    uint64_t pin_mask = 0;
    if (PIN_FSW5 >= 0) pin_mask |= (1ULL << PIN_FSW5);
    if (PIN_FSW6 >= 0) pin_mask |= (1ULL << PIN_FSW6);
    if (pin_mask) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = pin_mask;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;   // External 10k on PCB
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&io_conf);
    }

    // 2. Configure UART2 link to ESP32 #1
    uart_config_t uart_config = {};
    uart_config.baud_rate = LINK_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(LINK_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART_NUM, LINK_TX_PIN, LINK_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART_NUM, 512, 512, 0, NULL, 0));

    // 3. Initialize flow pulse counters FS1..FS5
    for (int i = 0; i < 5; i++) {
        initCounter(i);
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting ESP32 #2 Sensor Node (PCNT + UART2)");
    initHardware();

    int64_t lastTxUs = esp_timer_get_time();
    int64_t lastLogUs = esp_timer_get_time();

    while (1) {
        pollCounters();

        int64_t nowUs = esp_timer_get_time();
        if ((nowUs - lastTxUs) >= (TX_PERIOD_MS * 1000)) {
            lastTxUs = nowUs;
            sendFrame();
        }

        if ((nowUs - lastLogUs) >= 1000000) {
            lastLogUs = nowUs;
            ESP_LOGI(TAG, "Pulses FS1..5: %lu %lu %lu %lu %lu | FSW: %lu",
                     (unsigned long)total[0], (unsigned long)total[1], (unsigned long)total[2],
                     (unsigned long)total[3], (unsigned long)total[4], (unsigned long)readFloats());
        }

        vTaskDelay(1);
    }
}

