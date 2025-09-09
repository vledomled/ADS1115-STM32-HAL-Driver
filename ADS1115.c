/*
 * ADS1115.c
 *
 *  Created on: Aug 6, 2025
 *      Author: VladyslavApanasenko
 */

#include "ADS1115.h"
#include "stm32f3xx_hal.h"

#ifndef ADS1115_I2C_TIMEOUT
#define ADS1115_I2C_TIMEOUT  100
#endif

static float ADS1115_ComputeLSB(pga_t pga) {
    switch (pga) {
        case ADS1115_PGA_TWOTHIRDS: return 6.144f  / 32768.0f; // ±6.144 В
        case ADS1115_PGA_ONE:       return 4.096f  / 32768.0f; // ±4.096 В
        case ADS1115_PGA_TWO:       return 2.048f  / 32768.0f; // ±2.048 В
        case ADS1115_PGA_FOUR:      return 1.024f  / 32768.0f; // ±1.024 В
        case ADS1115_PGA_EIGHT:     return 0.512f  / 32768.0f; // ±0.512 В
        case ADS1115_PGA_SIXTEEN:   return 0.256f  / 32768.0f; // ±0.256 В
        default:                    return 2.048f  / 32768.0f;
    }
}

// -----------------------------------------------------------------------------
HAL_StatusTypeDef ADS1115_Init(ADS1115_HandleTypeDef *dev,
                               I2C_HandleTypeDef     *hi2c,
                               uint8_t                address)
{
    dev->hi2c    = hi2c;
    dev->address = address << 1;


    dev->mux         = ADS1115_MUX_SINGLE_0;
    dev->pga         = ADS1115_PGA_TWOTHIRDS;
    dev->single_shot = 0;
    dev->data_rate   = ADS1115_DATA_RATE_128;
    dev->comp_enabled = 0;
    dev->comp_mode    = 0;
    dev->comp_polarity= 0;
    dev->comp_latch   = 0;
    dev->comp_queue   = ADS1115_COMP_QUE_MASK;
    dev->lo_thresh    = 0x8000;
    dev->hi_thresh    = 0x7FFF;

    dev->lsb = ADS1115_ComputeLSB(dev->pga);

    return ADS1115_Configure(dev);
}

// -----------------------------------------------------------------------------
HAL_StatusTypeDef ADS1115_Configure(ADS1115_HandleTypeDef *dev)
{
    uint16_t cfg = 0;
    cfg |= ADS1115_OS_SINGLE;
    cfg |= dev->mux;
    cfg |= dev->pga;
    if (dev->single_shot) cfg |= ADS1115_MODE_SINGLESHOT;


    cfg |= dev->data_rate;
    if (dev->comp_mode)    cfg |= ADS1115_COMP_MODE;
    if (dev->comp_polarity)cfg |= ADS1115_COMP_POL;
    if (dev->comp_latch)   cfg |= ADS1115_COMP_LAT;
    cfg |= (dev->comp_queue & ADS1115_COMP_QUE_MASK);

    dev->last_raw = cfg;

    uint8_t buf[3] = {
        ADS1115_REG_POINTER_CONFIG,
        (uint8_t)(cfg >> 8),
        (uint8_t)(cfg & 0xFF)
    };
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->address, buf, 3, ADS1115_I2C_TIMEOUT);
}

// -----------------------------------------------------------------------------
HAL_StatusTypeDef ADS1115_ReadRaw(ADS1115_HandleTypeDef *dev, int16_t *out)
{
    uint8_t ptr = ADS1115_REG_POINTER_CONVERT;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(dev->hi2c, dev->address, &ptr, 1, ADS1115_I2C_TIMEOUT);
    if (ret != HAL_OK) return ret;

    // Читаем 2 байта
    uint8_t raw[2];
    ret = HAL_I2C_Master_Receive(dev->hi2c, dev->address, raw, 2, ADS1115_I2C_TIMEOUT);
    if (ret != HAL_OK) return ret;

    *out = (uint16_t)((raw[0] << 8) | raw[1]);
    dev->last_raw = *out;
    return HAL_OK;
}

// -----------------------------------------------------------------------------
HAL_StatusTypeDef ADS1115_ReadVoltage(ADS1115_HandleTypeDef *dev, float *out)
{
    int16_t raw;
    HAL_StatusTypeDef ret = ADS1115_ReadRaw(dev, &raw);
    if (ret != HAL_OK) return ret;
    *out = raw * dev->lsb;
    return HAL_OK;
}

// -----------------------------------------------------------------------------
HAL_StatusTypeDef ADS1115_SetThresholds(ADS1115_HandleTypeDef *dev, uint16_t lo, uint16_t hi)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[2];

    buf[0] = (uint8_t)(lo >> 8);
    buf[1] = (uint8_t)(lo & 0xFF);
    ret = HAL_I2C_Mem_Write(dev->hi2c, dev->address,
                            ADS1115_REG_POINTER_LOW_THRESH,
                            I2C_MEMADD_SIZE_8BIT, buf, 2, ADS1115_I2C_TIMEOUT);
    if (ret != HAL_OK) return ret;
    dev->lo_thresh = lo;

    buf[0] = (uint8_t)(hi >> 8);
    buf[1] = (uint8_t)(hi & 0xFF);
    ret = HAL_I2C_Mem_Write(dev->hi2c, dev->address,
                            ADS1115_REG_POINTER_HIGH_THRESH,
                            I2C_MEMADD_SIZE_8BIT, buf, 2, ADS1115_I2C_TIMEOUT);
    if (ret != HAL_OK) return ret;
    dev->hi_thresh = hi;

    return HAL_OK;
}

// -----------------------------------------------------------------------------
float ADS1115_ReadSingleEnded(ADS1115_HandleTypeDef *dev, mux_t mux_single)
{
    uint16_t cfg = ADS1115_OS_SINGLE |
                   mux_single |
                   dev->pga |
                   ADS1115_MODE_SINGLESHOT |
                   dev->data_rate |
                   (dev->comp_queue & ADS1115_COMP_QUE_MASK);

    uint8_t wr[3] = { ADS1115_REG_POINTER_CONFIG, cfg >> 8, cfg & 0xFF };
    if (HAL_I2C_Master_Transmit(dev->hi2c, dev->address, wr, 3, ADS1115_I2C_TIMEOUT) != HAL_OK)
        return -1;

    HAL_Delay(10);

    uint8_t ptr = ADS1115_REG_POINTER_CONVERT, raw[2];
    if (HAL_I2C_Master_Transmit(dev->hi2c, dev->address, &ptr, 1, ADS1115_I2C_TIMEOUT) != HAL_OK) return 100;
    if (HAL_I2C_Master_Receive(dev->hi2c, dev->address, raw, 2, ADS1115_I2C_TIMEOUT) != HAL_OK)   return 101;

    int16_t code = (int16_t)((raw[0] << 8) | raw[1]);
    return code * dev->lsb;
}



