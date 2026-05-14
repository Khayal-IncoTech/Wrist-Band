#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_PORT          I2C_NUM_0
#define I2C_SDA_GPIO      8
#define I2C_SCL_GPIO      9
#define I2C_PROBE_TIMEOUT 50    // ms per address
#define SCAN_PERIOD_MS    5000

static const char *TAG = "i2c_scan";

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

    while (1) {
        ESP_LOGI(TAG, "Scanning I2C bus (SDA=GPIO%d, SCL=GPIO%d)...",
                 I2C_SDA_GPIO, I2C_SCL_GPIO);

        printf("\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");

        int found = 0;
        uint8_t addrs[128];

        for (int row = 0; row < 128; row += 16) {
            printf("%02x:", row);
            for (int col = 0; col < 16; col++) {
                int addr = row + col;
                // Reserved ranges per I2C spec: 0x00-0x07 and 0x78-0x7F
                if (addr < 0x08 || addr > 0x77) {
                    printf("   ");
                    continue;
                }
                esp_err_t err = i2c_master_probe(bus_handle, addr, I2C_PROBE_TIMEOUT);
                if (err == ESP_OK) {
                    printf(" %02x", addr);
                    addrs[found++] = addr;
                } else if (err == ESP_ERR_NOT_FOUND) {
                    printf(" --");
                } else {
                    // Bus error (e.g. SDA stuck low, no pull-ups)
                    printf(" ??");
                }
            }
            printf("\n");
        }

        if (found == 0) {
            ESP_LOGW(TAG, "No I2C devices found. Check wiring and pull-ups (4.7k typical).");
        } else {
            ESP_LOGI(TAG, "Found %d device(s):", found);
            for (int i = 0; i < found; i++) {
                ESP_LOGI(TAG, "  - 0x%02X", addrs[i]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SCAN_PERIOD_MS));
    }
}
