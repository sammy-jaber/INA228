/*
 *
 * INA228 I2C Driver
 *
 * Author: Sammy Jaber
 * Created 17/06/2022
 *
 */

#ifndef INA228_I2C_DRIVER_H
#define INA228_I2C_DRIVER_H

#include "stm32f4xx.h" /* Needed for I2C */

/*
 * DEFINES
 */
#define INA226_I2C_ADDR		(0x45 << 1) /* A0 & A1 - VS > Ox45  (p.19) */

#define INA226_REG_MANUFACTURER_ID		0xFE
#define INA226_REG_DEVICE_ID			0xFF

/*
 *  REGISTERS (p.21)
 */

#define INA226_REG_CONFIG			0x00
#define INA226_REG_ADCCONFIG			0x01
#define INA226_REG_SHUNT_CAL			0x02
#define INA226_REG_SHUNT_TEMPCO			0x03
#define INA226_REG_VSHUNT			0x04
#define INA226_REG_VBUS				0x05
#define INA226_REG_DIETEMP			0x06
#define INA226_REG_CURRENT			0x07
#define INA226_REG_POWER			0x08
#define INA226_REG_ENERGY			0x09
#define INA226_REG_CHARGE			0x0A
#define INA226_REG_DIAG_ALRT			0x0B
#define INA226_REG_SOVL				0x0C
#define INA226_REG_SUVL				0x0D
#define INA226_REG_BOVL				0x0E
#define INA226_REG_BUVL				0x0F
#define INA226_REG_TEMP_LIMIT			0x10
#define INA226_REG_PWR_LIMIT			0x11

/*
 * SENSOR STRUCT
 */

typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	/* Shunt voltage data in volts*/
	float shuntvoltage_volts;

	/* Bus voltage data in volts*/
	float busvoltage_volts;

	/* Power data in watts*/
	float power_watts;

	/* Current  data in amps*/
	float current_amps;

} INA228;

/*
 * INITIALISATION
 */
uint8_t INA228_Initialise( INA228 *dev, I2C_HandleTypeDef *i2cHandle);

/*
 * DATA ACQUISITION
 */

HAL_StatusTypeDef INA228_ReadBusVoltage( INA228 *DEV );
HAL_StatusTypeDef INA228_ReadCurrent  ( INA228 *DEV );
HAL_StatusTypeDef INA228_GetPower( INA228 *DEV );

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef INA228_ReadRegister(  INA228 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef INA228_ReadRegisters( INA228 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef INA228_WriteRegister( INA228 *dev, uint8_t reg, uint8_t *data );

#endif
