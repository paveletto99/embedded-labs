# Arel OS on esp32c6

## Hardware

- [ESP32-C6-DEV-KIT-N8-M](https://manuals.plus/asin/B0CKR4R5VM)
- [BME280](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bme280/)
- 

## Flashing

```shell
TODO: add instructions for flashing with espflash
like flash --chip esp32c6 --port /dev/ttyACM0 --baud 115200 --no-stub build/bin/espressif-esp32-c6-devkitc-1/cargo/riscv32imac-unknown-none-elf/release/hello-fosdem
```

## Build, flash and run
```shell
laze build -b espressif-esp32-c6-devkitc-1 run
```

### Erase flash

```shell
espflash erase-flash -p /dev/cu.usbmodem212401                                                                        
[2026-02-19T07:51:52Z INFO ] Serial port: '/dev/cu.usbmodem212401'
[2026-02-19T07:51:52Z INFO ] Connecting...
[2026-02-19T07:51:52Z INFO ] Using flash stub
[2026-02-19T07:51:52Z INFO ] Erasing Flash...
[2026-02-19T07:51:55Z INFO ] Flash has been erased!
```