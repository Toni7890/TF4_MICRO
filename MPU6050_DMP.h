#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include "config.h"

// --- Constantes del MPU6050 ---
#define MPU6050_I2C_ADDR   0x68 // Direcci�n I2C del MPU6050 (puede ser 0x69 si AD0 est� en alto)
#define MPU6050_RA_WHO_AM_I 0x75
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_GYRO_XOUT_H 0x43

// --- Prototipos de Funciones ---

/**
 * @brief Inicializa el sensor MPU6050.
 * @return true si la inicializaci�n fue exitosa, false en caso contrario.
 */
bool MPU6050_Init(void);

/**
 * @brief Realiza una calibraci�n b�sica del sensor.
 * @return true si la calibraci�n fue exitosa.
 */
bool MPU6050_Calibrate(void);

/**
 * @brief Lee los valores crudos del aceler�metro y giroscopio.
 * @param ax Puntero para guardar el valor del aceler�metro en X.
 * @param ay Puntero para guardar el valor del aceler�metro en Y.
 * @param az Puntero para guardar el valor del aceler�metro en Z.
 * @param gx Puntero para guardar el valor del giroscopio en X.
 * @param gy Puntero para guardar el valor del giroscopio en Y.
 * @param gz Puntero para guardar el valor del giroscopio en Z.
 */
void MPU6050_Read_Raw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

/**
 * @brief Calcula el �ngulo de inclinaci�n (pitch).
 * @return El �ngulo de inclinaci�n en grados.
 */
float MPU6050_GetPitch(void);

/**
 * @brief Lee m�ltiples bytes desde un registro del MPU6050.
 * @param reg Direcci�n del primer registro a leer.
 * @param length N�mero de bytes a leer.
 * @param data Puntero al buffer donde se guardar�n los datos.
 * @return true si la lectura fue exitosa.
 */
bool MPU6050_ReadBytes(uint8_t reg, uint8_t length, uint8_t *data);

/**
 * @brief Escribe un byte en un registro del MPU6050.
 * @param reg Direcci�n del registro.
 * @param data El byte a escribir.
 * @return true si la escritura fue exitosa.
 */
bool MPU6050_WriteByte(uint8_t reg, uint8_t data);

#endif /* MPU6050_DMP_H */