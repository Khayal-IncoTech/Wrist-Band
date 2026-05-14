#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define I2C_BUS         "/dev/i2c-1"   // Pi 4 user I2C: SDA=GPIO2 (pin 3), SCL=GPIO3 (pin 5)
#define SCAN_PERIOD_S   5

// Probe one address. Returns 0 if a device ACKs, -1 otherwise.
// Mirrors `i2cdetect` auto mode: SMBus-read for 0x30-0x37 and 0x50-0x5F
// (where some chips hang on a write), quick-write elsewhere.
static int probe_addr(int fd, int addr)
{
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        return -1;
    }

    struct i2c_smbus_ioctl_data args;
    union i2c_smbus_data data;

    if ((addr >= 0x30 && addr <= 0x37) || (addr >= 0x50 && addr <= 0x5F)) {
        args.read_write = I2C_SMBUS_READ;
        args.command    = 0;
        args.size       = I2C_SMBUS_BYTE;
        args.data       = &data;
    } else {
        args.read_write = I2C_SMBUS_WRITE;
        args.command    = 0;
        args.size       = I2C_SMBUS_QUICK;
        args.data       = NULL;
    }

    return ioctl(fd, I2C_SMBUS, &args) < 0 ? -1 : 0;
}

int main(void)
{
    int fd = open(I2C_BUS, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", I2C_BUS, strerror(errno));
        fprintf(stderr, "Is I2C enabled? Run: sudo raspi-config -> Interface Options -> I2C\n");
        return 1;
    }

    for (;;) {
        printf("\nScanning %s ...\n", I2C_BUS);
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");

        int found = 0;
        unsigned char addrs[128];

        for (int row = 0; row < 128; row += 16) {
            printf("%02x:", row);
            for (int col = 0; col < 16; col++) {
                int addr = row + col;
                // Reserved per I2C spec: 0x00-0x07 and 0x78-0x7F
                if (addr < 0x08 || addr > 0x77) {
                    printf("   ");
                    continue;
                }
                if (probe_addr(fd, addr) == 0) {
                    printf(" %02x", addr);
                    addrs[found++] = addr;
                } else {
                    printf(" --");
                }
            }
            printf("\n");
        }

        if (found == 0) {
            printf("No I2C devices found. Check wiring; Pi has internal 1.8k pull-ups on SDA/SCL.\n");
        } else {
            printf("Found %d device(s):\n", found);
            for (int i = 0; i < found; i++) {
                printf("  - 0x%02X\n", addrs[i]);
            }
        }

        sleep(SCAN_PERIOD_S);
    }

    close(fd);
    return 0;
}
