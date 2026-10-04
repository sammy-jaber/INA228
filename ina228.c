/*
 * INA228 I2C Driver (STM32 HAL)
 */

#include "ina228.h"

/* LSB sizes from the datasheet */
#define INA228_VBUS_LSB_V          195.3125e-6f
#define INA228_VSHUNT_LSB_RANGE0_V 312.5e-9f
#define INA228_VSHUNT_LSB_RANGE1_V 78.125e-9f
#define INA228_DIETEMP_LSB_C       0.0078125f

#define INA228_DEFAULT_TIMEOUT_MS  100U

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* Sign-extend a 20-bit two's complement value */
static int32_t sign_extend_20(uint32_t v)
{
    v &= 0xFFFFFU;
    if (v & 0x80000U) {
        v |= 0xFFF00000U;
    }
    return (int32_t)v;
}

/* Sign-extend a 40-bit two's complement value */
static int64_t sign_extend_40(uint64_t v)
{
    v &= 0xFFFFFFFFFFULL;
    if (v & (1ULL << 39)) {
        v |= 0xFFFFFF0000000000ULL;
    }
    return (int64_t)v;
}

/* ------------------------------------------------------------------ */
/* Low-level                                                           */
/* ------------------------------------------------------------------ */

static HAL_StatusTypeDef read_bytes(INA228 *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->i2cHandle, dev->addr, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, dev->timeout_ms);
}

HAL_StatusTypeDef INA228_ReadReg16(INA228 *dev, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = read_bytes(dev, reg, buf, 2);
    if (st == HAL_OK) {
        *value = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return st;
}

HAL_StatusTypeDef INA228_WriteReg16(INA228 *dev, uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return HAL_I2C_Mem_Write(dev->i2cHandle, dev->addr, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, 2, dev->timeout_ms);
}

HAL_StatusTypeDef INA228_ReadReg24(INA228 *dev, uint8_t reg, uint32_t *value)
{
    uint8_t buf[3];
    HAL_StatusTypeDef st = read_bytes(dev, reg, buf, 3);
    if (st == HAL_OK) {
        *value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    }
    return st;
}

HAL_StatusTypeDef INA228_ReadReg40(INA228 *dev, uint8_t reg, uint64_t *value)
{
    uint8_t buf[5];
    HAL_StatusTypeDef st = read_bytes(dev, reg, buf, 5);
    if (st == HAL_OK) {
        *value = ((uint64_t)buf[0] << 32) | ((uint64_t)buf[1] << 24) |
                 ((uint64_t)buf[2] << 16) | ((uint64_t)buf[3] << 8) | buf[4];
    }
    return st;
}

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef INA228_Reset(INA228 *dev)
{
    return INA228_WriteReg16(dev, INA228_REG_CONFIG, INA228_CONFIG_RST);
}

HAL_StatusTypeDef INA228_ResetAccumulators(INA228 *dev)
{
    uint16_t cfg;
    HAL_StatusTypeDef st = INA228_ReadReg16(dev, INA228_REG_CONFIG, &cfg);
    if (st != HAL_OK) return st;
    return INA228_WriteReg16(dev, INA228_REG_CONFIG, cfg | INA228_CONFIG_RSTACC);
}

HAL_StatusTypeDef INA228_ConfigureADC(INA228 *dev, INA228_Mode mode,
                                      INA228_ConvTime vbusct, INA228_ConvTime vshct,
                                      INA228_ConvTime vtct, INA228_Avg avg)
{
    uint16_t v = ((uint16_t)(mode   & 0xF) << 12) |
                 ((uint16_t)(vbusct & 0x7) << 9)  |
                 ((uint16_t)(vshct  & 0x7) << 6)  |
                 ((uint16_t)(vtct   & 0x7) << 3)  |
                 ((uint16_t)(avg    & 0x7));
    return INA228_WriteReg16(dev, INA228_REG_ADCCONFIG, v);
}

HAL_StatusTypeDef INA228_Initialise(INA228 *dev, I2C_HandleTypeDef *i2cHandle,
                                    uint8_t addr7, float shunt_ohms,
                                    float max_current_amps, INA228_AdcRange range)
{
    HAL_StatusTypeDef st;
    uint16_t id;

    if (!dev || !i2cHandle || shunt_ohms <= 0.0f || max_current_amps <= 0.0f) {
        return HAL_ERROR;
    }

    dev->i2cHandle  = i2cHandle;
    dev->addr       = (uint16_t)(addr7 << 1);
    dev->timeout_ms = INA228_DEFAULT_TIMEOUT_MS;
    dev->shunt_ohms = shunt_ohms;
    dev->adc_range  = range;

    dev->shuntvoltage_volts = 0.0f;
    dev->busvoltage_volts   = 0.0f;
    dev->current_amps       = 0.0f;
    dev->power_watts        = 0.0f;
    dev->temperature_c      = 0.0f;
    dev->energy_joules      = 0.0;
    dev->charge_coulombs    = 0.0;

    /* Identify the device */
    st = INA228_ReadReg16(dev, INA228_REG_MANUFACTURER_ID, &id);
    if (st != HAL_OK) return st;
    if (id != INA228_MANUFACTURER_ID) return HAL_ERROR;

    st = INA228_ReadReg16(dev, INA228_REG_DEVICE_ID, &id);
    if (st != HAL_OK) return st;
    if ((id >> 4) != INA228_DEVICE_ID_VALUE) return HAL_ERROR;

    /* Reset to known state */
    st = INA228_Reset(dev);
    if (st != HAL_OK) return st;
    HAL_Delay(2);

    /* Make sure the shunt can handle the requested max current */
    float full_scale_v = (range == INA228_RANGE_163MV) ? 0.16384f : 0.04096f;
    if (max_current_amps * shunt_ohms > full_scale_v) {
        return HAL_ERROR;
    }

    /* CURRENT_LSB = Imax / 2^19 */
    dev->current_lsb = max_current_amps / 524288.0f;

    /* SHUNT_CAL = 13107.2e6 * CURRENT_LSB * RSHUNT   (x4 when ADCRANGE = 1) */
    double cal = 13107.2e6 * (double)dev->current_lsb * (double)shunt_ohms;
    if (range == INA228_RANGE_41MV) {
        cal *= 4.0;
    }
    cal += 0.5; /* round */
    if (cal < 1.0 || cal > 32767.0) { /* SHUNT_CAL is 15 bits */
        return HAL_ERROR;
    }

    uint16_t cfg = (range == INA228_RANGE_41MV) ? INA228_CONFIG_ADCRANGE : 0;
    st = INA228_WriteReg16(dev, INA228_REG_CONFIG, cfg);
    if (st != HAL_OK) return st;

    return INA228_WriteReg16(dev, INA228_REG_SHUNT_CAL, (uint16_t)cal);
}

/* ------------------------------------------------------------------ */
/* Measurements                                                        */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef INA228_ReadShuntVoltage(INA228 *dev)
{
    uint32_t raw;
    HAL_StatusTypeDef st = INA228_ReadReg24(dev, INA228_REG_VSHUNT, &raw);
    if (st != HAL_OK) return st;

    float lsb = (dev->adc_range == INA228_RANGE_163MV) ? INA228_VSHUNT_LSB_RANGE0_V
                                                       : INA228_VSHUNT_LSB_RANGE1_V;
    dev->shuntvoltage_volts = (float)sign_extend_20(raw >> 4) * lsb;
    return HAL_OK;
}

HAL_StatusTypeDef INA228_ReadBusVoltage(INA228 *dev)
{
    uint32_t raw;
    HAL_StatusTypeDef st = INA228_ReadReg24(dev, INA228_REG_VBUS, &raw);
    if (st != HAL_OK) return st;

    /* VBUS is always positive; 20-bit value in [23:4] */
    dev->busvoltage_volts = (float)(raw >> 4) * INA228_VBUS_LSB_V;
    return HAL_OK;
}

HAL_StatusTypeDef INA228_ReadCurrent(INA228 *dev)
{
    uint32_t raw;
    HAL_StatusTypeDef st = INA228_ReadReg24(dev, INA228_REG_CURRENT, &raw);
    if (st != HAL_OK) return st;

    dev->current_amps = (float)sign_extend_20(raw >> 4) * dev->current_lsb;
    return HAL_OK;
}

HAL_StatusTypeDef INA228_ReadPower(INA228 *dev)
{
    uint32_t raw;
    HAL_StatusTypeDef st = INA228_ReadReg24(dev, INA228_REG_POWER, &raw);
    if (st != HAL_OK) return st;

    /* POWER = 3.2 * CURRENT_LSB * register (24-bit unsigned) */
    dev->power_watts = 3.2f * dev->current_lsb * (float)raw;
    return HAL_OK;
}

HAL_StatusTypeDef INA228_ReadTemperature(INA228 *dev)
{
    uint16_t raw;
    HAL_StatusTypeDef st = INA228_ReadReg16(dev, INA228_REG_DIETEMP, &raw);
    if (st != HAL_OK) return st;

    dev->temperature_c = (float)(int16_t)raw * INA228_DIETEMP_LSB_C;
    return HAL_OK;
}

HAL_StatusTypeDef INA228_ReadEnergy(INA228 *dev)
{
    uint64_t raw;
    HAL_StatusTypeDef st = INA228_ReadReg40(dev, INA228_REG_ENERGY, &raw);
    if (st != HAL_OK) return st;

    /* ENERGY = 16 * 3.2 * CURRENT_LSB * register (40-bit unsigned), in joules */
    dev->energy_joules = 16.0 * 3.2 * (double)dev->current_lsb * (double)raw;
    return HAL_OK;
}

HAL_StatusTypeDef INA228_ReadCharge(INA228 *dev)
{
    uint64_t raw;
    HAL_StatusTypeDef st = INA228_ReadReg40(dev, INA228_REG_CHARGE, &raw);
    if (st != HAL_OK) return st;

    /* CHARGE = CURRENT_LSB * register (40-bit signed), in coulombs */
    dev->charge_coulombs = (double)dev->current_lsb * (double)sign_extend_40(raw);
    return HAL_OK;
}

HAL_StatusTypeDef INA228_ReadAll(INA228 *dev)
{
    HAL_StatusTypeDef st;

    if ((st = INA228_ReadBusVoltage(dev))   != HAL_OK) return st;
    if ((st = INA228_ReadShuntVoltage(dev)) != HAL_OK) return st;
    if ((st = INA228_ReadCurrent(dev))      != HAL_OK) return st;
    if ((st = INA228_ReadPower(dev))        != HAL_OK) return st;
    return INA228_ReadTemperature(dev);
}

HAL_StatusTypeDef INA228_ReadDiagAlert(INA228 *dev, uint16_t *diag)
{
    return INA228_ReadReg16(dev, INA228_REG_DIAG_ALRT, diag);
}
