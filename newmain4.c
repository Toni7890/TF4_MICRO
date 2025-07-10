/*******************************************************************************
 * MAIN DE CALIBRACIÓN Y DIAGNÓSTICO PARA SILLA DE RUEDAS - VERSIÓN CORREGIDA
 *
 * PROYECTO: Silla de Ruedas Inteligente - Módulo de Ajuste
 * MICROCONTROLADOR: PIC18F57Q43 @ 64MHz
 *
 * CORRECCIÓN:
 * - Eliminada la redefinición del enum 'estado_silla_t' que causaba el error
 * de compilación. Ahora se usa la definición de "L298N_Dual.h".
 *******************************************************************************/

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// === INCLUDES DEL PROYECTO ===
#include "config.h"
#include "Software_I2C.h"
#include "UART_LIB.h"
#include "L298N_Dual.h" // La definición de 'estado_silla_t' viene de aquí
#include "HC_SR04.h"

/*******************************************************************************
 * CONFIGURACIÓN DE SENSIBILIDAD - ¡AQUÍ ESTÁ LA CLAVE!
 *******************************************************************************/
// AJUSTA ESTOS VALORES PARA CALIBRAR LA RESPUESTA
// Si la silla no se mueve, HAZ ESTOS VALORES MÁS PEQUEÑOS.
// Si la silla es muy sensible, HAZ ESTOS VALORES MÁS GRANDES.

#define ZONA_MUERTA_ACCEL   0.15f
#define ZONA_MUERTA_GYRO    15.0f
#define UMBRAL_ADELANTE     0.25f
#define UMBRAL_ATRAS       -0.25f
#define UMBRAL_DERECHA     -25.0f
#define UMBRAL_IZQUIERDA    25.0f

/*******************************************************************************
 * DEFINICIONES Y ESTRUCTURAS
 *******************************************************************************/
#define MPU6050_ADDR            0x68
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_WHO_AM_I        0x75

// ¡ELIMINADO! El 'enum estado_silla_t' ahora se incluye desde "L298N_Dual.h"

typedef struct { float x, y, z; } Vector3D_t;

// Variables globales
static Vector3D_t accel_offset = {0, 0, 0};
static Vector3D_t gyro_offset = {0, 0, 0};
static char uart_buffer[128];

// Prototipos de funciones
static bool InicializarSistema(void);
static bool CalibracionManualMPU6050(void);
static bool LeerValoresMPU6050(Vector3D_t *accel, Vector3D_t *gyro);
static estado_silla_t InterpretarMovimiento(Vector3D_t accel, Vector3D_t gyro);
static const char* EstadoToString(estado_silla_t estado);


/*******************************************************************************
 * FUNCIÓN PRINCIPAL
 *******************************************************************************/
void main(void) {
    if (!InicializarSistema()) {
        while(1) {
            LED_Toggle();
            __delay_ms(100);
        }
    }
    
    UART1_Println("\n=== MODO CALIBRACION Y DIAGNOSTICO ===");
    UART1_Println("Mueva el sensor y observe los valores.");
    UART1_Println("Ajuste los UMBRALES en el codigo si es necesario.");
    UART1_Println("----------------------------------------");
    
    Vector3D_t acelerometro, giroscopio;
    
    while(1) {
        if (LeerValoresMPU6050(&acelerometro, &giroscopio)) {
            estado_silla_t estado_detectado = InterpretarMovimiento(acelerometro, giroscopio);
            
            sprintf(uart_buffer, "AY: % .2f | GZ: % .2f | Estado: %s",
                    acelerometro.y,
                    giroscopio.z,
                    EstadoToString(estado_detectado));
            UART1_Println(uart_buffer);

        } else {
            UART1_Println("[ERROR] No se pudo leer el MPU6050.");
        }
        
        __delay_ms(250);
    }
}

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES
 *******************************************************************************/

static bool InicializarSistema(void) {
    ClockInit();
    PinInit();
    UART1_Init();
    __delay_ms(500);
    UART1_Println("[INIT] Hardware basico inicializado.");
    
    SW_I2C_Init();
    SW_I2C_SetSpeed(SW_I2C_SPEED_NORMAL);
    __delay_ms(100);
    UART1_Println("[INIT] I2C inicializado.");

    uint8_t who_am_i = SW_I2C_ReadRegister(MPU6050_ADDR, MPU6050_WHO_AM_I);
    if (who_am_i != 0x68) {
        UART1_Println("[ERROR] MPU6050 no responde.");
        return false;
    }
    
    SW_I2C_WriteRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    __delay_ms(100);
    SW_I2C_WriteRegister(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x08);
    SW_I2C_WriteRegister(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x08);
    UART1_Println("[INIT] MPU6050 configurado.");

    L298N_Dual_Init();
    L298N_Dual_Stop();
    UART1_Println("[INIT] Motores L298N inicializados.");

    return CalibracionManualMPU6050();
}

static bool CalibracionManualMPU6050(void) {
    UART1_Println("\n=== CALIBRACION DEL SENSOR ===");
    UART1_Println("Mantenga el sensor en posicion NEUTRAL y QUIETO.");
    
    for (uint8_t i = 5; i > 0; i--) {
        sprintf(uart_buffer, "Calibrando en %u...", i);
        UART1_Println(uart_buffer);
        __delay_ms(1000);
    }
    
    UART1_Println("CALIBRANDO... NO MOVER");
    LED_ON();
    
    const uint16_t muestras = 200;
    float suma_ax = 0, suma_ay = 0, suma_az = 0;
    float suma_gx = 0, suma_gy = 0, suma_gz = 0;
    uint16_t muestras_validas = 0;
    Vector3D_t temp_accel, temp_gyro;
    
    for (uint16_t i = 0; i < muestras; i++) {
        if (LeerValoresMPU6050(&temp_accel, &temp_gyro)) {
            suma_ax += temp_accel.x + accel_offset.x;
            suma_ay += temp_accel.y + accel_offset.y;
            suma_az += temp_accel.z + accel_offset.z;
            suma_gx += temp_gyro.x + gyro_offset.x;
            suma_gy += temp_gyro.y + gyro_offset.y;
            suma_gz += temp_gyro.z + gyro_offset.z;
            muestras_validas++;
        }
        __delay_ms(5);
    }
    
    LED_OFF();
    
    if (muestras_validas < (muestras * 0.8f)) {
        UART1_Println("[ERROR] Calibracion fallida.");
        return false;
    }
    
    accel_offset.x = suma_ax / muestras_validas;
    accel_offset.y = suma_ay / muestras_validas;
    accel_offset.z = (suma_az / muestras_validas) - 1.0f;
    gyro_offset.x = suma_gx / muestras_validas;
    gyro_offset.y = suma_gy / muestras_validas;
    gyro_offset.z = suma_gz / muestras_validas;
    
    UART1_Println("CALIBRACION COMPLETADA!");
    return true;
}

static bool LeerValoresMPU6050(Vector3D_t *accel, Vector3D_t *gyro) {
    uint8_t datos[14];
    if (SW_I2C_ReadBlock(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, datos, 14) != SW_I2C_SUCCESS) {
        return false;
    }
    
    int16_t raw_ax = (datos[0] << 8) | datos[1];
    int16_t raw_ay = (datos[2] << 8) | datos[3];
    int16_t raw_az = (datos[4] << 8) | datos[5];
    int16_t raw_gx = (datos[8] << 8) | datos[9];
    int16_t raw_gy = (datos[10] << 8) | datos[11];
    int16_t raw_gz = (datos[12] << 8) | datos[13];
    
    accel->x = (raw_ax / 8192.0f) - accel_offset.x;
    accel->y = (raw_ay / 8192.0f) - accel_offset.y;
    accel->z = (raw_az / 8192.0f) - accel_offset.z;
    gyro->x = (raw_gx / 65.5f) - gyro_offset.x;
    gyro->y = (raw_gy / 65.5f) - gyro_offset.y;
    gyro->z = (raw_gz / 65.5f) - gyro_offset.z;
    
    return true;
}

static estado_silla_t InterpretarMovimiento(Vector3D_t accel, Vector3D_t gyro) {
    if (accel.y > UMBRAL_ADELANTE) return SILLA_ADELANTE;
    if (accel.y < UMBRAL_ATRAS) return SILLA_ATRAS;
    if (gyro.z > UMBRAL_IZQUIERDA) return SILLA_GIRO_IZQUIERDA;
    if (gyro.z < UMBRAL_DERECHA) return SILLA_GIRO_DERECHA;
    return SILLA_PARADA;
}

static const char* EstadoToString(estado_silla_t estado) {
    switch(estado) {
        case SILLA_PARADA: return "PARADO";
        case SILLA_ADELANTE: return "ADELANTE";
        case SILLA_ATRAS: return "ATRAS";
        case SILLA_GIRO_IZQUIERDA: return "IZQUIERDA";
        case SILLA_GIRO_DERECHA: return "DERECHA";
        case SILLA_ERROR: return "ERROR";
        default: return "DESCONOCIDO";
    }
}