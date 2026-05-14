"""
Epson RX8130CE RTC driver for Raspberry Pi 4 (I2C, 7-bit address 0x32).

Setup on the Pi:
    sudo apt install python3-smbus      # or: pip install smbus2
    sudo raspi-config                   # Interface Options -> I2C -> Enable
    sudo usermod -aG i2c $USER          # log out/in afterwards

Wiring (40-pin header):
    RTC SDA -> GPIO 2  (pin 3)
    RTC SCL -> GPIO 3  (pin 5)
    RTC VDD -> 3V3     (pin 1)
    RTC GND -> GND     (pin 6)

Run:
    python3 rx8130.py             # show RTC time, set from host if VLF flag is up
    python3 rx8130.py --set       # force-set RTC to host time
    python3 rx8130.py --watch     # print RTC time once a second
"""

import argparse
import sys
import time
from datetime import datetime

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus  # python3-smbus package — same API for what we use


RX8130_ADDR = 0x32
I2C_BUS     = 1                  # /dev/i2c-1 on the Pi 4

# Register map (RX8130CE datasheet, page-0)
REG_SEC      = 0x10
REG_MIN      = 0x11
REG_HOUR     = 0x12
REG_WEEK     = 0x13
REG_DAY      = 0x14
REG_MONTH    = 0x15
REG_YEAR     = 0x16
REG_FLAG     = 0x1D
REG_CTRL0    = 0x1E
REG_CTRL1    = 0x1F

# FLAG bits
FLAG_VLF     = 1 << 1            # voltage-low / oscillator stopped — time invalid

# CTRL0 bits
CTRL0_STOP   = 1 << 6            # halt second counter while writing time
CTRL0_1224   = 1 << 5            # 0 = 24h mode (we force this)


def _bcd2int(b):
    return (b >> 4) * 10 + (b & 0x0F)

def _int2bcd(n):
    return ((n // 10) << 4) | (n % 10)


class RX8130:
    def __init__(self, bus=I2C_BUS, addr=RX8130_ADDR):
        self._bus = SMBus(bus)
        self._addr = addr

    def close(self):
        self._bus.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # --- low-level ---
    def _r(self, reg):
        return self._bus.read_byte_data(self._addr, reg)

    def _w(self, reg, val):
        self._bus.write_byte_data(self._addr, reg, val & 0xFF)

    def _rblk(self, reg, n):
        return self._bus.read_i2c_block_data(self._addr, reg, n)

    def _wblk(self, reg, data):
        self._bus.write_i2c_block_data(self._addr, reg, list(data))

    # --- public API ---
    def voltage_low(self):
        """True if VLF flag is set => oscillator stopped, time is invalid."""
        return bool(self._r(REG_FLAG) & FLAG_VLF)

    def clear_voltage_low(self):
        self._w(REG_FLAG, self._r(REG_FLAG) & ~FLAG_VLF)

    def get_datetime(self):
        regs = self._rblk(REG_SEC, 7)
        sec    = _bcd2int(regs[0] & 0x7F)
        minute = _bcd2int(regs[1] & 0x7F)
        hour   = _bcd2int(regs[2] & 0x3F)   # 24h mode: 6 valid bits
        # regs[3] = week (bit-encoded day-of-week) — derived from date instead
        day    = _bcd2int(regs[4] & 0x3F)
        month  = _bcd2int(regs[5] & 0x1F)
        year   = _bcd2int(regs[6]) + 2000
        return datetime(year, month, day, hour, minute, sec)

    def set_datetime(self, dt):
        # Stop counters and force 24h mode while writing.
        ctrl0 = self._r(REG_CTRL0)
        self._w(REG_CTRL0, (ctrl0 | CTRL0_STOP) & ~CTRL0_1224)

        # WEEK reg is one-hot, bit 0 = Sunday … bit 6 = Saturday.
        # Python weekday(): Mon=0 .. Sun=6 → rotate so Sun=0.
        rtc_dow_bit = (dt.weekday() + 1) % 7
        week = 1 << rtc_dow_bit

        self._wblk(REG_SEC, [
            _int2bcd(dt.second),
            _int2bcd(dt.minute),
            _int2bcd(dt.hour),
            week,
            _int2bcd(dt.day),
            _int2bcd(dt.month),
            _int2bcd(dt.year - 2000),
        ])

        # Restart counters and clear the voltage-low flag.
        self._w(REG_CTRL0, ctrl0 & ~CTRL0_STOP & ~CTRL0_1224)
        self.clear_voltage_low()


def main():
    ap = argparse.ArgumentParser(description="RX8130CE RTC tool")
    ap.add_argument("--set", action="store_true",
                    help="force-set RTC to host time")
    ap.add_argument("--watch", action="store_true",
                    help="print RTC time once a second")
    args = ap.parse_args()

    with RX8130() as rtc:
        if args.set or rtc.voltage_low():
            if rtc.voltage_low() and not args.set:
                print("VLF flag is set — RTC has lost time. Setting from host clock.")
            rtc.set_datetime(datetime.now())
            print("RTC set to:", rtc.get_datetime().isoformat(sep=" "))

        if args.watch:
            try:
                while True:
                    rtc_t  = rtc.get_datetime()
                    host_t = datetime.now().replace(microsecond=0)
                    drift  = (host_t - rtc_t).total_seconds()
                    print(f"RTC {rtc_t}  Host {host_t}  drift {drift:+.0f}s")
                    time.sleep(1)
            except KeyboardInterrupt:
                print()
        else:
            print("RTC time :", rtc.get_datetime().isoformat(sep=" "))
            print("Host time:", datetime.now().replace(microsecond=0).isoformat(sep=" "))


if __name__ == "__main__":
    sys.exit(main())
