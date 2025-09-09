/*
 * ADS1115.h
 *
 *  Created on: Aug 4, 2025
 *      Author: VladyslavApanasenko
 */

#include "stm32f3xx_hal.h"
#include <stdint.h>

#ifndef INC_ADS1115_H_
#define INC_ADS1115_H_

/* Definitions */
#define ADS1115_OS (0b1 << 7) // Default

#define ADS1115_MODE (0b1) // Default

/* ADS1115 register configurations */
#define ADS1115_REG_POINTER_CONVERT   0x00  // Conversion register
#define ADS1115_REG_POINTER_CONFIG    0x01  // Config register
#define ADS1115_REG_POINTER_LOW_THRESH  0x02 // Lo_thresh
#define ADS1115_REG_POINTER_HIGH_THRESH 0x03 // Hi_thresh

// Старший байт (bits 15…8)
#define ADS1115_OS_SINGLE       (1U << 15)   // запускает однократную конверсию :contentReference[oaicite:21]{index=21}
#define ADS1115_MUX_SHIFT       12
#define ADS1115_MUX_MASK        (0x7U << ADS1115_MUX_SHIFT)  // мультиплексор :contentReference[oaicite:22]{index=22}
#define ADS1115_PGA_SHIFT       9
#define ADS1115_PGA_MASK        (0x7U << ADS1115_PGA_SHIFT)  // усиление      :contentReference[oaicite:23]{index=23}
#define ADS1115_MODE_SINGLESHOT (1U << 8)    // single-shot (default) :contentReference[oaicite:24]{index=24}

// Младший байт (bits 7…0)
#define ADS1115_DR_SHIFT        5
#define ADS1115_DR_MASK         (0x7U << ADS1115_DR_SHIFT)   // скорость выборки :contentReference[oaicite:25]{index=25}
#define ADS1115_COMP_MODE       (1U << 4)    // традиц. или оконный компаратор :contentReference[oaicite:26]{index=26}
#define ADS1115_COMP_POL        (1U << 3)    // полярность ALERT/RDY        :contentReference[oaicite:27]{index=27}
#define ADS1115_COMP_LAT        (1U << 2)    // защёлкивание компаратора    :contentReference[oaicite:28]{index=28}
#define ADS1115_COMP_QUE_MASK   0x03U        // очередь компаратора         :contentReference[oaicite:29]{index=29}


typedef enum {
    ADS1115_MUX_DIFF_0_1 = 0x0000, // AIN0 - AIN1
    ADS1115_MUX_DIFF_0_3 = 0x1000, // AIN0 - AIN3
    ADS1115_MUX_DIFF_1_3 = 0x2000, // AIN1 - AIN3
    ADS1115_MUX_DIFF_2_3 = 0x3000, // AIN2 - AIN3
    ADS1115_MUX_SINGLE_0 = 0x4000, // AIN0 - GND
    ADS1115_MUX_SINGLE_1 = 0x5000, // AIN1 - GND
    ADS1115_MUX_SINGLE_2 = 0x6000, // AIN2 - GND
    ADS1115_MUX_SINGLE_3 = 0x7000  // AIN3 - GND
} mux_t;
typedef enum
{
	ADS1115_PGA_TWOTHIRDS = (0b000 << 1), 		// 2/3x Gain	-- 0.1875 mV by one bit		MAX: +- VDD + 0.3V
	ADS1115_PGA_ONE = (0b001 << 1), 			// 1x Gain		-- 0.125 mV by one bit		MAX: +- VDD + 0.3V
	ADS1115_PGA_TWO = (0b010 << 1), 			// 2x Gain		-- 0.0625 mV by one bit		MAX: +- 2.048 V
	ADS1115_PGA_FOUR = (0b011 << 1), 			// 4x Gain		-- 0.03125 mV by one bit	MAX: +- 1.024 V
	ADS1115_PGA_EIGHT = (0b100 << 1), 			// 8x Gain		-- 0.015625 mV by one bit	MAX: +- 0.512 V
	ADS1115_PGA_SIXTEEN = (0b111 << 1) 			// 16x Gain		-- 0.0078125 mV by one bit	MAX: +- 0.256 V
} pga_t;

typedef enum
{

	ADS1115_DATA_RATE_8	= (0b000 << 5),				// 8SPS
	ADS1115_DATA_RATE_16 = (0b001 << 5),			// 16SPS
	ADS1115_DATA_RATE_32 = (0b010 << 5),			// 32SPS
	ADS1115_DATA_RATE_64 = 	(0b011 << 5),			// 64SPS
	ADS1115_DATA_RATE_128 =	(0b100 << 5),			// 128SPS
	ADS1115_DATA_RATE_250 =	(0b101 << 5),			// 250SPS
	ADS1115_DATA_RATE_475 = (0b110 << 5),			// 475SPS
	ADS1115_DATA_RATE_860 = (0b111 << 5)			// 860SPS

} data_rate_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;    // I²C-хэндлер
    uint8_t             address; // 7-битный адрес (0x48…0x4B)

    mux_t       mux;
    pga_t       pga;
    uint8_t     single_shot; // true — single-shot, false — continuous
    data_rate_t data_rate;

    uint8_t		comp_enabled;
    uint8_t		comp_mode;   // window vs traditional
    uint8_t		comp_polarity;
    uint8_t		comp_latch;
    uint8_t		comp_queue;
    uint16_t    lo_thresh;
    uint16_t    hi_thresh;

    float               lsb;        // В/бит, рассчитывается как FSR/2^15
    int16_t             last_raw;   // последний «сырый» код
} ADS1115_HandleTypeDef;


/* Function prototypes. */
HAL_StatusTypeDef ADS1115_Init   (ADS1115_HandleTypeDef *dev,
                                  I2C_HandleTypeDef     *hi2c,
                                  uint8_t                address);

HAL_StatusTypeDef ADS1115_Configure(ADS1115_HandleTypeDef *dev);

HAL_StatusTypeDef ADS1115_ReadRaw    (ADS1115_HandleTypeDef *dev, int16_t *out);
HAL_StatusTypeDef ADS1115_ReadVoltage(ADS1115_HandleTypeDef *dev, float  *out);

HAL_StatusTypeDef ADS1115_SetThresholds(ADS1115_HandleTypeDef *dev,
                                        uint16_t lo, uint16_t hi);
float ADS1115_ReadSingleEnded(ADS1115_HandleTypeDef *dev, mux_t mux_single);



#endif /* INC_ADS1115_H_ */
