#include "MPU6050_DMP.h"
#include "Software_I2C.h" // Se incluye la librería I2C por software
#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

bool MPU6050_Init(void) {
    SW_I2C_Init(); // Inicializa los pines para I2C
    
    // Verificar la conexión con el MPU6050
    if (SW_I2C_ReadRegister(MPU6050_I2C_ADDR, MPU6050_RA_WHO_AM_I) != 0x68) {
        return false; // No se encontró el dispositivo
    }

    // Despertar el MPU6050
    MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x00);
    __delay_ms(100);
    
    return true;
}

bool MPU6050_Calibrate(void) {
    // Aquí iría una rutina de calibración más compleja si fuera necesario,
    // como promediar lecturas en reposo para obtener los offsets.
    // Por ahora, solo verificamos que el dispositivo responda.
    return (SW_I2C_ReadRegister(MPU6050_I2C_ADDR, MPU6050_RA_WHO_AM_I) == 0x68);
}

void MPU6050_Read_Raw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[14];
    
    if (SW_I2C_ReadRegisters(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14, buffer)) {
        *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
        *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
        *az = (((int16_t)buffer[4]) << 8) | buffer[5];
        // Los bytes 6 y 7 son de temperatura, los ignoramos
        *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
        *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
        *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
    } else {
        // En caso de error, poner todo a cero
        *ax = *ay = *az = *gx = *gy = *gz = 0;
    }
}

float MPU6050_GetPitch(void){
    int16_t ax, ay, az, gx, gy, gz;
    MPU6050_Read_Raw(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Cálculo del ángulo Pitch (inclinación adelante/atrás) usando el acelerómetro
    // atan2 maneja correctamente los cuadrantes y evita divisiones por cero
    return atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
}


// --- Funciones de bajo nivel para comunicación I2C ---

bool MPU6050_WriteByte(uint8_t reg, uint8_t data) {
    return SW_I2C_WriteRegister(MPU6050_I2C_ADDR, reg, data);
}

// Esta es la función corregida que reemplaza a la antigua SW_I2C_ReadBlock
bool MPU6050_ReadBytes(uint8_t reg, uint8_t length, uint8_t *data) {
    return SW_I2C_ReadRegisters(MPU6050_I2C_ADDR, reg, length, data);
}