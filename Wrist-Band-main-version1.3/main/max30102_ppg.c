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

    // LED currents - wrist needs much more than fingertip, and off-pulse-point
    // placement needs MORE light to penetrate tissue and find the artery.
    // Each LSB = 0.2 mA. 0x7F = ~25 mA, 0xFF = ~51 mA (max).
    // Start higher (was 0x7F) so first-time users get a signal even with sloppy placement.
    ESP_ERROR_CHECK(reg_write(REG_LED1_PA, 0x80));   // Red ~26 mA
    ESP_ERROR_CHECK(reg_write(REG_LED2_PA, 0xA0));   // IR  ~33 mA

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

    // Bandpass = high-pass (removes DC) followed by low-pass (removes HF noise).
    // HR band is 0.7-3.5 Hz (42-210 BPM); anything outside is noise or DC drift.
    const float hp_a = 0.96f;             // fc ~0.65 Hz
    float hp_prev_x = 0.0f, hp_prev_y = 0.0f;
    int filter_primed = 0;                // init filter state from first sample to avoid startup spike
    const float lp_a = 0.22f;             // EMA alpha for fc ~4 Hz at fs=100
    float lp_y = 0.0f;

    // Adaptive peak detector with refractory period.
    float peak_envelope = 0.0f;
    const float env_decay_slow = 0.995f;  // tracks beat amplitude when signal is healthy
    const float env_decay_fast = 0.97f;   // recovers fast from transients (e.g. finger placement spike)
    const float spike_clip    = 3000.0f;  // cap envelope update; real wrist pulse is ~500-2000
    int above = 0;
    int64_t last_beat_us = 0;
    const int64_t min_rr_us = 333000;     // 333 ms -> 180 BPM ceiling
    const int64_t max_rr_us = 2000000;    // 2 s    -> 30 BPM floor
    float bpm = 0.0f;

    // Warmup: 1.5 s. With priming the filter settles fast.
    const int warmup_samples = 150;       // 1.5 s at 100 Hz
    int sample_count = 0;
    int warmup_announced = 0;

    // Skin-on detection. When ir_raw is below skin_off_thresh the sensor isn't
    // touching skin (or is barely lit). When above skin_on_thresh, it is.
    // Hysteresis prevents flapping at the boundary. We treat "sensor off skin"
    // as the only condition that zeroes BPM — minor jolts while ON skin keep
    // the displayed value steady.
    const uint32_t skin_off_thresh = 8000;
    const uint32_t skin_on_thresh  = 20000;
    int is_on_skin = 0;
    uint32_t prev_ir_raw = 0;

    // Adaptive LED control: keeps ir_raw strong even if sensor is off-spot
    // or skin is dark/hairy/thick. Pushes for a higher target (60k-180k)
    // so off-pulse-point placement still yields a usable AC swing.
    uint8_t ir_led_current = 0xA0;          // matches init value above
    uint64_t ir_sum = 0;
    int ir_sum_count = 0;
    const int led_check_interval = 200;     // 2 s at 100 Hz
    const uint32_t led_target_lo = 60000;
    const uint32_t led_target_hi = 180000;
    const uint8_t led_step = 0x10;
    const uint8_t led_min = 0x30;           // ~10 mA floor
    const uint8_t led_max = 0xFF;           // ~51 mA datasheet max

    // Long-term smoothed BPM for anxiety/emotion tracking.
    // Single missed beats won't break it; emotion trends emerge over seconds.
    float bpm_smoothed = 0.0f;
    int valid_beat_count = 0;
    const int min_beats_stable = 5;         // 5 beats before we trust the value

    // Sliding-window median over the last RR_WINDOW intervals. Median is
    // robust to occasional missed/double beats — even if 2 out of 5 RRs are
    // wrong, the median picks the right value. We wait for 3 RRs before
    // showing anything so the first displayed BPM is already trustworthy,
    // then the value stays stable instead of climbing/drifting via EMA.
    #define RR_WINDOW 5
    float rr_window[RR_WINDOW];
    int rr_window_count = 0;
    int rr_window_idx = 0;

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

            // Skin-on/off detection with hysteresis.
            int was_on_skin = is_on_skin;
            if (ir[i] >= skin_on_thresh)        is_on_skin = 1;
            else if (ir[i] <  skin_off_thresh)  is_on_skin = 0;

            // Sensor just left the skin → zero everything, no detection.
            if (was_on_skin && !is_on_skin) {
                bpm = 0.0f;
                bpm_smoothed = 0.0f;
                rr_window_count = 0;
                rr_window_idx = 0;
                valid_beat_count = 0;
                last_beat_us = 0;
                filter_primed = 0;
                sample_count = 0;
            }
            // Off skin: emit zero and skip detection entirely.
            if (!is_on_skin) {
                printf("%lu,%.1f,%lu,%.1f\n",
                       (unsigned long)ir[i], 0.0f,
                       (unsigned long)red[i], 0.0f);
                prev_ir_raw = ir[i];
                continue;
            }

            // First sample after returning to skin (or boot) — prime filter so
            // the placement step doesn't show up as a huge transient.
            if (!filter_primed) {
                hp_prev_x = x;
                hp_prev_y = 0.0f;
                lp_y = 0.0f;
                peak_envelope = 0.0f;
                sample_count = 0;
                bpm = 0.0f;
                last_beat_us = 0;
                filter_primed = 1;
                prev_ir_raw = ir[i];
                rr_window_count = 0;
                rr_window_idx = 0;
                bpm_smoothed = 0.0f;
                valid_beat_count = 0;
            }
            prev_ir_raw = ir[i];

            // High-pass: y[n] = a * (y[n-1] + x[n] - x[n-1])
            float hp = hp_a * (hp_prev_y + x - hp_prev_x);
            hp_prev_x = x;
            hp_prev_y = hp;

            // Low-pass: y[n] = y[n-1] + alpha * (x[n] - y[n-1])
            lp_y += lp_a * (hp - lp_y);
            float y = lp_y;

            sample_count++;
            int64_t now = esp_timer_get_time();

            if (sample_count < warmup_samples) {
                warmup_announced = 0;
                printf("%lu,%.1f,%lu,%.1f\n",
                       (unsigned long)ir[i], y,
                       (unsigned long)red[i], 0.0f);
                continue;
            }
            if (!warmup_announced) {
                ESP_LOGI(TAG, "Warmup done. Detecting heartbeat now.");
                warmup_announced = 1;
            }

            float absy = y < 0 ? -y : y;
            if (absy > peak_envelope) {
                peak_envelope = absy < spike_clip ? absy : spike_clip;
            } else {
                float decay = (absy < 0.3f * peak_envelope) ? env_decay_fast : env_decay_slow;
                peak_envelope *= decay;
            }

            float threshold = 0.35f * peak_envelope;

            // Threshold crossing: rising edge above threshold triggers a beat.
            // No strict slope requirement — small filter ringing was making us
            // miss valid beats. Refractory period (min_rr_us) handles dicrotic notch.
            if (!above && y > threshold && peak_envelope > 80.0f) {
                above = 1;
                if (last_beat_us != 0) {
                    int64_t rr = now - last_beat_us;
                    if (rr >= min_rr_us && rr <= max_rr_us) {
                        // Push into the sliding-window RR buffer.
                        rr_window[rr_window_idx] = (float)rr;
                        rr_window_idx = (rr_window_idx + 1) % RR_WINDOW;
                        if (rr_window_count < RR_WINDOW) rr_window_count++;

                        // Show BPM as soon as we have at least 3 RRs (still robust).
                        if (rr_window_count >= 3) {
                            float sorted[RR_WINDOW];
                            for (int k = 0; k < rr_window_count; k++) sorted[k] = rr_window[k];
                            // Insertion sort (tiny array).
                            for (int a = 1; a < rr_window_count; a++) {
                                float key = sorted[a];
                                int b = a - 1;
                                while (b >= 0 && sorted[b] > key) {
                                    sorted[b + 1] = sorted[b];
                                    b--;
                                }
                                sorted[b + 1] = key;
                            }
                            float median_rr = sorted[rr_window_count / 2];
                            bpm = 60000000.0f / median_rr;
                            bpm_smoothed = bpm;
                        }
                        if (valid_beat_count < 10000) valid_beat_count++;
                    }
                }
                last_beat_us = now;
            } else if (above && y < 0.2f * threshold) {
                above = 0;
            }

            // Hold BPM for 5 s so a single missed beat doesn't drop it to 0.
            if (last_beat_us != 0 && (now - last_beat_us) > 5000000) {
                bpm = 0.0f;
            }
            // Smoothed BPM has a longer hold (15 s) so the emotion trend
            // survives short dropouts and weak-signal moments.
            if (last_beat_us != 0 && (now - last_beat_us) > 15000000) {
                bpm_smoothed = 0.0f;
                valid_beat_count = 0;
            }

            // Report the raw (lightly smoothed) BPM directly. The peak detector's
            // own 0.7/0.3 smoothing already gives stable output; an extra slow
            // EMA on top was hiding real changes.
            float report_bpm = bpm;

            printf("%lu,%.1f,%lu,%.1f\n",
                   (unsigned long)ir[i], y,
                   (unsigned long)red[i], report_bpm);

            // Adaptive LED tuning: average ir_raw over a window, then nudge
            // LED current if the average is outside the target band.
            ir_sum += ir[i];
            ir_sum_count++;
            if (ir_sum_count >= led_check_interval && filter_primed) {
                uint32_t avg = (uint32_t)(ir_sum / ir_sum_count);
                ir_sum = 0;
                ir_sum_count = 0;

                uint8_t new_current = ir_led_current;
                if (avg < led_target_lo && ir_led_current < led_max) {
                    new_current = (ir_led_current > led_max - led_step)
                                  ? led_max : ir_led_current + led_step;
                } else if (avg > led_target_hi && ir_led_current > led_min) {
                    new_current = (ir_led_current < led_min + led_step)
                                  ? led_min : ir_led_current - led_step;
                }

                if (new_current != ir_led_current) {
                    ESP_LOGI(TAG, "Adaptive LED: ir_avg=%lu -> IR_PA 0x%02X (was 0x%02X)",
                             (unsigned long)avg, new_current, ir_led_current);
                    ir_led_current = new_current;
                    reg_write(REG_LED2_PA, ir_led_current);
                    filter_primed = 0;   // re-prime; DC will shift
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
