/*
 * INA228 I2C Driver (STM32 HAL)
 *
 * 85V, 20-bit power/energy/charge monitor with I2C interface.
 * Register map and scaling per TI INA228 datasheet (SBOSA20).
 */

#ifndef INA228_H
#define INA228_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h" /* Change to your family's HAL header if needed */
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* I2C address (7-bit). A1/A0 strapping, see datasheet Table 7-2.      */
/* ------------------------------------------------------------------ */
#define INA228_ADDR_A1GND_A0GND   0x40
#define INA228_ADDR_A1GND_A0VS    0x41
#define INA228_ADDR_A1GND_A0SDA   0x42
#define INA228_ADDR_A1GND_A0SCL   0x43
#define INA228_ADDR_A1VS_A0GND    0x44
#define INA228_ADDR_A1VS_A0VS     0x45

/* ------------------------------------------------------------------ */
/* Registers                                                           */
/* ------------------------------------------------------------------ */
#define INA228_REG_CONFIG          0x00 /* 16-bit */
#define INA228_REG_ADCCONFIG       0x01 /* 16-bit */
#define INA228_REG_SHUNT_CAL       0x02 /* 16-bit */
#define INA228_REG_SHUNT_TEMPCO    0x03 /* 16-bit */
#define INA228_REG_VSHUNT          0x04 /* 24-bit, signed, 20-bit value in [23:4] */
#define INA228_REG_VBUS            0x05 /* 24-bit, 20-bit value in [23:4] */
#define INA228_REG_DIETEMP         0x06 /* 16-bit, signed */
#define INA228_REG_CURRENT         0x07 /* 24-bit, signed, 20-bit value in [23:4] */
#define INA228_REG_POWER           0x08 /* 24-bit, unsigned */
#define INA228_REG_ENERGY          0x09 /* 40-bit, unsigned */
#define INA228_REG_CHARGE          0x0A /* 40-bit, signed */
#define INA228_REG_DIAG_ALRT       0x0B /* 16-bit */
#define INA228_REG_SOVL            0x0C /* 16-bit */
#define INA228_REG_SUVL            0x0D /* 16-bit */
#define INA228_REG_BOVL            0x0E /* 16-bit */
#define INA228_REG_BUVL            0x0F /* 16-bit */
#define INA228_REG_TEMP_LIMIT      0x10 /* 16-bit */
#define INA228_REG_PWR_LIMIT       0x11 /* 16-bit */
#define INA228_REG_MANUFACTURER_ID 0x3E /* 16-bit, reads 0x5449 ("TI") */
#define INA228_REG_DEVICE_ID       0x3F /* 16-bit, [15:4] = 0x228, [3:0] = rev */

#define INA228_MANUFACTURER_ID     0x5449
#define INA228_DEVICE_ID_VALUE     0x228

/* ------------------------------------------------------------------ */
/* CONFIG register bits                                                */
/* ------------------------------------------------------------------ */
#define INA228_CONFIG_RST          (1U << 15)
#define INA228_CONFIG_RSTACC       (1U << 14)
#define INA228_CONFIG_TEMPCOMP     (1U << 5)
#define INA228_CONFIG_ADCRANGE     (1U << 4)

/* ------------------------------------------------------------------ */
/* DIAG_ALRT bits (subset)                                             */
/* ------------------------------------------------------------------ */
#define INA228_DIAG_MEMSTAT        (1U << 0)
#define INA228_DIAG_CNVRF          (1U << 1)
#define INA228_DIAG_MATHOF         (1U << 9)

/* ------------------------------------------------------------------ */
/* ADC_CONFIG fields                                                   */
/* ------------------------------------------------------------------ */
typedef enum {
    INA228_MODE_SHUTDOWN          = 0x0,
    INA228_MODE_TRIG_VBUS         = 0x1,
    INA228_MODE_TRIG_VSHUNT       = 0x2,
    INA228_MODE_TRIG_VBUS_VSHUNT  = 0x3,
    INA228_MODE_TRIG_TEMP         = 0x4,
    INA228_MODE_TRIG_TEMP_VBUS    = 0x5,
    INA228_MODE_TRIG_TEMP_VSHUNT  = 0x6,
    INA228_MODE_TRIG_ALL          = 0x7,
    INA228_MODE_CONT_VBUS         = 0x9,
    INA228_MODE_CONT_VSHUNT       = 0xA,
    INA228_MODE_CONT_VBUS_VSHUNT  = 0xB,
    INA228_MODE_CONT_TEMP         = 0xC,
    INA228_MODE_CONT_TEMP_VBUS    = 0xD,
    INA228_MODE_CONT_TEMP_VSHUNT  = 0xE,
    INA228_MODE_CONT_ALL          = 0xF  /* default */
} INA228_Mode;

/* Conversion time, applies to VBUSCT / VSHCT / VTCT */
typedef enum {
    INA228_CT_50US   = 0,
    INA228_CT_84US   = 1,
    INA228_CT_150US  = 2,
    INA228_CT_280US  = 3,
    INA228_CT_540US  = 4,
    INA228_CT_1052US = 5, /* default */
    INA228_CT_2074US = 6,
    INA228_CT_4120US = 7
} INA228_ConvTime;

/* Averaging count */
typedef enum {
    INA228_AVG_1    = 0, /* default */
    INA228_AVG_4    = 1,
    INA228_AVG_16   = 2,
    INA228_AVG_64   = 3,
    INA228_AVG_128  = 4,
    INA228_AVG_256  = 5,
    INA228_AVG_512  = 6,
    INA228_AVG_1024 = 7
} INA228_Avg;

/* Shunt full-scale range */
typedef enum {
    INA228_RANGE_163MV = 0, /* +/-163.84 mV, 312.5 nV/LSB  */
    INA228_RANGE_41MV  = 1  /* +/-40.96 mV,  78.125 nV/LSB */
} INA228_AdcRange;

/* ------------------------------------------------------------------ */
/* Device struct                                                       */
/* ------------------------------------------------------------------ */
typedef struct {
    I2C_HandleTypeDef *i2cHandle;
    uint16_t           addr;          /* 8-bit (7-bit address << 1) for HAL */
    uint32_t           timeout_ms;

    /* Calibration */
    float              shunt_ohms;
    float              current_lsb;   /* amps per LSB */
    INA228_AdcRange    adc_range;

    /* Latest measurements */
    float              shuntvoltage_volts;
    float              busvoltage_volts;
    float              current_amps;
    float              power_watts;
    float              temperature_c;
    double             energy_joules;
    double             charge_coulombs;
} INA228;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/*
 * Reset the device, verify IDs, set ADC range, write SHUNT_CAL.
 * addr7:           7-bit I2C address (e.g. INA228_ADDR_A1VS_A0VS)
 * shunt_ohms:      shunt resistor value
 * max_current_amps expected maximum current (sets CURRENT_LSB = max / 2^19)
 * Returns HAL_ERROR if IDs don't match or the calibration is out of range
 * (e.g. max_current * shunt exceeds the selected ADC range).
 */
HAL_StatusTypeDef INA228_Initialise(INA228 *dev, I2C_HandleTypeDef *i2cHandle,
                                    uint8_t addr7, float shunt_ohms,
                                    float max_current_amps, INA228_AdcRange range);

HAL_StatusTypeDef INA228_Reset(INA228 *dev);
HAL_StatusTypeDef INA228_ResetAccumulators(INA228 *dev); /* clears ENERGY and CHARGE */
HAL_StatusTypeDef INA228_ConfigureADC(INA228 *dev, INA228_Mode mode,
                                      INA228_ConvTime vbusct, INA228_ConvTime vshct,
                                      INA228_ConvTime vtct, INA228_Avg avg);

/* Individual measurements (each updates its field in the struct) */
HAL_StatusTypeDef INA228_ReadShuntVoltage(INA228 *dev);
HAL_StatusTypeDef INA228_ReadBusVoltage(INA228 *dev);
HAL_StatusTypeDef INA228_ReadCurrent(INA228 *dev);
HAL_StatusTypeDef INA228_ReadPower(INA228 *dev);
HAL_StatusTypeDef INA228_ReadTemperature(INA228 *dev);
HAL_StatusTypeDef INA228_ReadEnergy(INA228 *dev);
HAL_StatusTypeDef INA228_ReadCharge(INA228 *dev);

/* Reads VBUS, VSHUNT, CURRENT, POWER, DIETEMP in one call */
HAL_StatusTypeDef INA228_ReadAll(INA228 *dev);

/* Status */
HAL_StatusTypeDef INA228_ReadDiagAlert(INA228 *dev, uint16_t *diag);

/* Low-level (big-endian on the wire, handled here) */
HAL_StatusTypeDef INA228_ReadReg16(INA228 *dev, uint8_t reg, uint16_t *value);
HAL_StatusTypeDef INA228_WriteReg16(INA228 *dev, uint8_t reg, uint16_t value);
HAL_StatusTypeDef INA228_ReadReg24(INA228 *dev, uint8_t reg, uint32_t *value);
HAL_StatusTypeDef INA228_ReadReg40(INA228 *dev, uint8_t reg, uint64_t *value);

#ifdef __cplusplus
}
#endif

#endif /* INA228_H */
