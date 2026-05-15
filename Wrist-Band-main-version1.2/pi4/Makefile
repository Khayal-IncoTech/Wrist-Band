CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra
TARGET   = i2c_scan

$(TARGET): i2c_scan.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)

.PHONY: clean
