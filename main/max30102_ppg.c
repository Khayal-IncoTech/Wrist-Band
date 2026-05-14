#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

#define I2C_PORT          I2C_NUM_0
#define I2C_SDA_GPIO      8
#define I2C_SCL_GPIO      9
#define I2C_FREQ_HZ       400000

#define MAX30102_ADDR     0x57

#define REG_INT_STATUS_1  0x00
#define REG_INT_STATUS_2  0x01
#define REG_FIFO_WR_PTR   0x04
#define REG_OVF_COUNTER   0x05
#define REG_FIFO_RD_PTR   0x06
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONFIG   0x08
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C   // Red
#define REG_LED2_PA       0x0D   // IR
#define REG_PART_ID       0xFF

#define EXPECTED_PART_ID  0x15

static const char *TAG = "ppg";
static i2c_master_dev_handle_t s_dev;

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, 100);
}

static esp_err_t max30102_init(void)
{
    uint8_t pid = 0;
    ESP_ERROR_CHECK(reg_read(REG_PART_ID, &pid, 1));
    ESP_LOGI(TAG, "Part ID: 0x%02X (expected 0x%02X)", pid, EXPECTED_PART_ID);
    if (pid != EXPECTED_PART_ID) return ESP_ERR_NOT_FOUND;

    // Soft reset
    ESP_ERROR_CHECK(reg_write(REG_MODE_CONFIG, 0x40));
    vTaskDelay(pdMS_TO_TICKS(20));

    // FIFO: avg 4 samples, rollover on, almost-full at 17 unread
    // [SMP_AVE=010 | FIFO_ROLLOVER_EN=1 | FIFO_A_FULL=0xF] = 0x5F
    ESP_ERROR_CHECK(reg_write(REG_FIFO_CONFIG, 0x5F));

    // SpO2 mode (Red + IR)
    ESP_ERROR_CHECK(reg_write(REG_MODE_CONFIG, 0x03));

    // SpO2 cfg: ADC range 16384 nA, 100 sps, 411 us pulse (18-bit)
    // [SPO2_ADC_RGE=11 | SPO2_SR=001 | LED_PW=11] = 0x67
    ESP_ERROR_CHECK(reg_write(REG_SPO2_CONFIG, 0x67));

    // LED currents - wrist needs much more than fingertip.
    // Each LSB = 0.2 mA. 0x7F = ~25 mA, 0xFF = ~51 mA (max).
    // Start moderate; raise if AC swing is too small.
    ESP_ERROR_CHECK(reg_write(REG_LED1_PA, 0x60));   // Red ~19 mA
    ESP_ERROR_CHECK(reg_write(REG_LED2_PA, 0x7F));   // IR  ~25 mA

    // Reset FIFO pointers
    ESP_ERROR_CHECK(reg_write(REG_FIFO_WR_PTR, 0));
    ESP_ERROR_CHECK(reg_write(REG_OVF_COUNTER, 0));
    ESP_ERROR_CHECK(reg_write(REG_FIFO_RD_PTR, 0));

    return ESP_OK;
}

#define MAX_BATCH 32

static int max30102_read_fifo(uint32_t *red, uint32_t *ir)
{
    uint8_t wr = 0, rd = 0;
    if (reg_read(REG_FIFO_WR_PTR, &wr, 1) != ESP_OK) return -1;
    if (reg_read(REG_FIFO_RD_PTR, &rd, 1) != ESP_OK) return -1;
    int n = (wr - rd) & 0x1F;
    if (n == 0) return 0;
    if (n > MAX_BATCH) n = MAX_BATCH;

    uint8_t buf[MAX_BATCH * 6];
    if (reg_read(REG_FIFO_DATA, buf, n * 6) != ESP_OK) return -1;

    for (int i = 0; i < n; i++) {
        uint8_t *p = &buf[i * 6];
        red[i] = (((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2]) & 0x3FFFF;
        ir[i]  = (((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5]) & 0x3FFFF;
    }
    return n;
}

void app_main(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MAX30102_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_dev));

    ESP_ERROR_CHECK(max30102_init());
    ESP_LOGI(TAG, "Streaming PPG at ~100 Hz. CSV columns: ir_raw,ir_ac,red_raw,bpm");

    // First-order DC-removal high-pass on IR. With Fs=100 Hz, a=0.96 -> fc ~0.65 Hz.
    // y[n] = a * (y[n-1] + x[n] - x[n-1])
    const float a = 0.96f;
    float prev_x = 0.0f, prev_y = 0.0f;

    // Adaptive peak detector with refractory period.
    // Track running peak amplitude of the AC signal to set a dynamic threshold.
    float peak_envelope = 0.0f;
    const float env_decay = 0.995f;       // slow decay so threshold tracks beat amplitude
    int above = 0;
    int64_t last_beat_us = 0;
    const int64_t min_rr_us = 333000;     // 333 ms -> 180 BPM ceiling
    const int64_t max_rr_us = 2000000;    // 2 s    -> 30 BPM floor
    float bpm = 0.0f;

    uint32_t red[MAX_BATCH], ir[MAX_BATCH];

    while (1) {
        int n = max30102_read_fifo(red, ir);
        if (n < 0) {
            ESP_LOGW(TAG, "FIFO read failed");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        for (int i = 0; i < n; i++) {
            float x = (float)ir[i];
            float y = a * (prev_y + x - prev_x);
            prev_x = x;
            prev_y = y;

            float absy = y < 0 ? -y : y;
            if (absy > peak_envelope) peak_envelope = absy;
            else                      peak_envelope *= env_decay;

            float threshold = 0.5f * peak_envelope;
            int64_t now = esp_timer_get_time();

            if (!above && y > threshold && peak_envelope > 50.0f) {
                above = 1;
                if (last_beat_us != 0) {
                    int64_t rr = now - last_beat_us;
                    if (rr >= min_rr_us && rr <= max_rr_us) {
                        float inst = 60000000.0f / (float)rr;
                        bpm = (bpm == 0.0f) ? inst : 0.8f * bpm + 0.2f * inst;
                    }
                }
                last_beat_us = now;
            } else if (above && y < 0.25f * threshold) {
                above = 0;
            }

            // Drop stale BPM if no beat in the last 3 s — avoids carrying old values
            // across runs and through dropout periods.
            if (last_beat_us != 0 && (now - last_beat_us) > 3000000) {
                bpm = 0.0f;
            }

            printf("%lu,%.1f,%lu,%.1f\n",
                   (unsigned long)ir[i], y,
                   (unsigned long)red[i], bpm);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
