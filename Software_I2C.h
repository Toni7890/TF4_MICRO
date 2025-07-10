#ifndef SOFTWARE_I2C_H
#define SOFTWARE_I2C_H

#include "config.h"

// Define los pines para SCL y SDA
#define SCL_PIN         LATCbits.LATC3
#define SDA_PIN         LATCbits.LATC4
#define SCL_DIR         TRISCbits.TRISC3
#define SDA_DIR         TRISCbits.TRISC4
#define SDA_PORT_READ   PORTCbits.RC4

typedef enum { SW_I2C_SUCCESS, SW_I2C_NACK, SW_I2C_FAIL } sw_i2c_error_t;

void SW_I2C_Init(void);
void SW_I2C_Start(void);
void SW_I2C_Stop(void);
void SW_I2C_Restart(void);
uint8_t SW_I2C_Read(bool ack);
sw_i2c_error_t SW_I2C_Write(uint8_t data);
bool SW_I2C_WriteRegister(uint8_t device_addr, uint8_t reg, uint8_t data);
uint8_t SW_I2C_ReadRegister(uint8_t device_addr, uint8_t reg);
bool SW_I2C_ReadRegisters(uint8_t device_addr, uint8_t reg, uint8_t count, uint8_t *dest);

#endif