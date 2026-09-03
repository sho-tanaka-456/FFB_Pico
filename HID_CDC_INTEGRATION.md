# HID FFB + CDC composite integration

## Files
- `ffb_hid_cdc.c`: FFB firmware plus TinyUSB CDC logs
- `usb_descriptors_hid_cdc.c`: original HID/PID descriptor plus CDC ACM

## Required tusb_config.h
Keep existing MCU, OS and endpoint-0 settings, and ensure:

```c
#define CFG_TUD_HID 1
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_HID_EP_BUFSIZE 64
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_CDC_EP_BUFSIZE 64
```

## CMake
Link `hardware_spi`, `tinyusb_device`, and `tinyusb_board`. Do not use
`pico_enable_stdio_usb(target 1)` with this custom TinyUSB composite device.

```cmake
target_link_libraries(your_target
    pico_stdlib pico_multicore hardware_pwm hardware_adc hardware_uart
    hardware_spi tinyusb_device tinyusb_board)
```

## Linux log
```bash
ls -l /dev/ttyACM* /dev/serial/by-id/
picocom -b 115200 /dev/ttyACM0
```

The MCP3204 monitor currently samples all four channels every 10 ms on Core 1,
outside the PWM IRQ, and logs at 10 Hz on Core 0. This is for signal checks only.
Use PWM-timed SPI DMA before using these samples for closed-loop current control.
