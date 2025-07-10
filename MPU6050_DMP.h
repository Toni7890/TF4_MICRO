#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include "config.h"

// --- Constantes del MPU6050 ---
#define MPU6050_I2C_ADDR   0x68 // Dirección I2C del MPU6050 (puede ser 0x69 si AD0 está en alto)
#define MPU6050_RA_WHO_AM_I 0x75
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_GYRO_XOUT_H 0x43

// --- Prototipos de Funciones ---

/**
 * @brief Inicializa el sensor MPU6050.
 * @return true si la inicialización fue exitosa, false en caso contrario.
 */
bool MPU6050_Init(void);

/**
 * @brief Realiza una calibración básica del sensor.
 * @return true si la calibración fue exitosa.
 */
bool MPU6050_Calibrate(void);

/**
 * @brief Lee los valores crudos del acelerómetro y giroscopio.
 * @param ax Puntero para guardar el valor del acelerómetro en X.
 * @param ay Puntero para guardar el valor del acelerómetro en Y.
 * @param az Puntero para guardar el valor del acelerómetro en Z.
 * @param gx Puntero para guardar el valor del giroscopio en X.
 * @param gy Puntero para guardar el valor del giroscopio en Y.
 * @param gz Puntero para guardar el valor del giroscopio en Z.
 */
void MPU6050_Read_Raw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

/**
 * @brief Calcula el ángulo de inclinación (pitch).
 * @return El ángulo de inclinación en grados.
 */
float MPU6050_GetPitch(void);

/**
 * @brief Lee múltiples bytes desde un registro del MPU6050.
 * @param reg Dirección del primer registro a leer.
 * @param length Número de bytes a leer.
 * @param data Puntero al buffer donde se guardarán los datos.
 * @return true si la lectura fue exitosa.
 */
bool MPU6050_ReadBytes(uint8_t reg, uint8_t length, uint8_t *data);

/**
 * @brief Escribe un byte en un registro del MPU6050.
 * @param reg Dirección del registro.
 * @param data El byte a escribir.
 * @return true si la escritura fue exitosa.
 */
bool MPU6050_WriteByte(uint8_t reg, uint8_t data);

#endif /* MPU6050_DMP_H */