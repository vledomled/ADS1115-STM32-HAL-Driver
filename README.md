# ADS1115-STM32-HAL-Driver
A lightweight, dependency-free STM32 HAL driver for the Texas Instruments ADS1115 16-bit I²C ADC. Supports single-ended reads, programmable gain, data-rate selection, and comparator thresholds, with a simple C API.

**Features**

✅ Single-ended input reads (AIN0…AIN3)

✅ Programmable gain amplifier (±6.144 V … ±0.256 V)

✅ Selectable data rates (8–860 SPS)

✅ Single-shot / power-down conversions

✅ Comparator/alert configuration with hi/lo thresholds

✅ Small, clear API (init → configure → read)


**Wiring (Typical)**

ADS1115 VDD → 3V3

GND → GND

SDA → MCU SDA (with pull-up)

SCL → MCU SCL (with pull-up)

ADDR → GND/3V3/SDA/SCL to set I²C address (0x48…0x4B)

In the driver, the 7-bit address is shifted left once (address << 1) to match STM32 HAL’s 8-bit addressing.


**Key Handle Fields**

mux — input selection (e.g., ADS1115_MUX_SINGLE_0)

pga — full-scale range (e.g., ADS1115_PGA_ONE = ±4.096 V)

data_rate — conversion rate

single_shot — 0=continuous, 1=single-shot

comp_* — comparator options

lo_thresh, hi_thresh — comparator thresholds

lsb — volts per code (auto-computed from pga)

LSB size is computed as FSR / 32768 where FSR depends on pga.

Example: ±4.096 V → 4.096 / 32768 ≈ 125 µV/code.
