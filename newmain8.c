/*******************************************************************************
 * SISTEMA DE CONTROL DE SILLA DE RUEDAS - MPU6050 + L298N + HC-SR04
 * 
 * MICROCONTROLADOR: PIC18F57Q43 @ 64MHz
 * PROYECTO: Silla de Ruedas Inteligente para Personas con Discapacidad
 * 
 * CARACTERÍSTICAS PRINCIPALES:
 * - Control por inclinación de cabeza usando MPU6050
 * - Control de velocidad variable según ángulo de inclinación
 * - Sistema de seguridad con sensor ultrasónico HC-SR04
 * - Control dual de motores con L298N
 * - Comunicación UART para monitoreo
 * - Preparado para LCD I2C y funciones adicionales
 * 
 * CONEXIONES:
 * MPU6050: SCL=RB1, SDA=RB2 (Software I2C)
 * L298N: IN1=RD0, IN2=RD1, ENA=RD2, IN3=RD4, IN4=RD5, ENB=RD3
 * HC-SR04: TRIG=RF5, ECHO=RF4
 * UART: TX=RC2, RX=RC3
 * I2C LCD: SCL=RC5, SDA=RC6 (Hardware I2C) - Futuro
 * LED Estado: RF3
 *******************************************************************************/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// Includes del proyecto
#include "config.h"
#include "Software_I2C.h"
#include "UART_LIB.h"
#include "L298N_Dual.h"
#include "HC_SR04.h"

/*******************************************************************************
 * DEFINICIONES Y CONFIGURACIONES
 *******************************************************************************/

// MPU6050 Registros
#define MPU6050_ADDR        0x68
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_WHO_AM_I    0x75

// Umbrales de control ajustados para personas con discapacidad
#define UMBRAL_INCLINACION_MIN    0.15f  // ~8.6 grados
#define UMBRAL_INCLINACION_MAX    0.70f  // ~40 grados
#define UMBRAL_GIRO_MIN          0.20f  // ~11.5 grados
#define UMBRAL_GIRO_MAX          0.60f  // ~34 grados

// Velocidades PWM (0-255)
#define VELOCIDAD_MIN       80   // Velocidad mínima para arrancar
#define VELOCIDAD_MAX       255  // Velocidad máxima
#define VELOCIDAD_GIRO_BASE 120  // Velocidad base para giros

// Sistema de seguridad
#define DISTANCIA_SEGURIDAD_CM    30  // Distancia mínima en reversa
#define DISTANCIA_ALERTA_CM       50  // Distancia de alerta
#define TIEMPO_ESTABILIDAD_MS     300 // Tiempo para estabilizar comando

// Control de sistema
#define TIEMPO_ACTUALIZACION_MS   50  // Actualización cada 50ms (20Hz)
#define TIEMPO_REPORTE_MS        1000 // Reporte UART cada segundo

/*******************************************************************************
 * ESTRUCTURAS Y TIPOS
 *******************************************************************************/

// Vector 3D para acelerómetro y giroscopio
typedef struct {
    float x;
    float y;
    float z;
} Vector3D_t;

// Estados de la silla
typedef enum {
    SILLA_PARADA = 0,
    SILLA_ADELANTE,
    SILLA_ATRAS,
    SILLA_GIRO_IZQUIERDA,
    SILLA_GIRO_DERECHA,
    SILLA_EMERGENCIA
} estado_silla_t;

// Estructura principal del sistema
typedef struct {
    estado_silla_t estado_actual;
    estado_silla_t estado_anterior;
    uint8_t velocidad_motor_izq;
    uint8_t velocidad_motor_der;
    uint16_t distancia_trasera_cm;
    bool obstaculo_detectado;
    bool sistema_activo;
    uint32_t tiempo_ultimo_comando;
    uint32_t tiempo_ultimo_reporte;
    Vector3D_t accel_offset;
    Vector3D_t gyro_offset;
} sistema_silla_t;

/*******************************************************************************
 * VARIABLES GLOBALES
 *******************************************************************************/

static sistema_silla_t silla = {
    .estado_actual = SILLA_PARADA,
    .estado_anterior = SILLA_PARADA,
    .velocidad_motor_izq = 0,
    .velocidad_motor_der = 0,
    .distancia_trasera_cm = 999,
    .obstaculo_detectado = false,
    .sistema_activo = false,
    .tiempo_ultimo_comando = 0,
    .tiempo_ultimo_reporte = 0
};

static char uart_buffer[128];
static uint32_t tiempo_sistema_ms = 0;

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES
 *******************************************************************************/

// Inicialización
static bool InicializarSistema(void);
static bool InicializarMPU6050(void);
static bool CalibrarMPU6050(void);

// Control principal
static void ProcesarControl(void);
static estado_silla_t DetectarComando(Vector3D_t *accel, Vector3D_t *gyro);
static void EjecutarMovimiento(estado_silla_t comando);
static uint8_t CalcularVelocidad(float inclinacion, float min, float max);

// Sensores
static bool LeerMPU6050(Vector3D_t *accel, Vector3D_t *gyro);
static uint16_t LeerDistanciaTrasera(void);

// Comunicación y diagnóstico
static void EnviarEstadoUART(void);
static void ProcesarComandosUART(void);
static const char* EstadoToString(estado_silla_t estado);

// Utilidades
static void ActualizarTiempo(void);
static void IndicadorLED(void);

/*******************************************************************************
 * FUNCIÓN PRINCIPAL
 *******************************************************************************/

void main(void) {
    
    // Inicializar sistema completo
    if (!InicializarSistema()) {
        // Error crítico - LED parpadeante
        while(1) {
            LED_Toggle();
            __delay_ms(200);
        }
    }
    
    // Sistema iniciado correctamente
    UART1_Println("\n=== SILLA DE RUEDAS INTELIGENTE v3.0 ===");
    UART1_Println("Sistema: ACTIVO Y FUNCIONANDO");
    UART1_Println("Control: Inclinación de cabeza (MPU6050)");
    UART1_Println("Seguridad: Sensor ultrasónico trasero");
    UART1_Println("=========================================\n");
    
    // Bucle principal
    while(1) {
        
        // Actualizar tiempo del sistema
        ActualizarTiempo();
        
        // Procesar control principal cada 50ms
        if ((tiempo_sistema_ms % TIEMPO_ACTUALIZACION_MS) == 0) {
            ProcesarControl();
        }
        
        // Enviar estado por UART cada segundo
        if ((tiempo_sistema_ms - silla.tiempo_ultimo_reporte) >= TIEMPO_REPORTE_MS) {
            EnviarEstadoUART();
            silla.tiempo_ultimo_reporte = tiempo_sistema_ms;
        }
        
        // Procesar comandos UART si hay datos
        if (UART1_DATA_READY()) {
            ProcesarComandosUART();
        }
        
        // Indicador visual de funcionamiento
        if ((tiempo_sistema_ms % 500) == 0) {
            if (silla.estado_actual == SILLA_PARADA) {
                LED_Toggle();
            } else {
                LED_ON();
            }
        }
        
        __delay_ms(1);
    }
}

/*******************************************************************************
 * IMPLEMENTACIÓN: INICIALIZACIÓN
 *******************************************************************************/

static bool InicializarSistema(void) {
    
    // Configurar hardware básico
    ClockInit();      // 64MHz
    PinInit();        // Configurar todos los pines
    
    // Inicializar UART
    UART1_Init();
    __delay_ms(100);
    UART1_Println("[INIT] Hardware básico configurado");
    
    // Inicializar I2C Software
    SW_I2C_Init();
    SW_I2C_SetSpeed(SW_I2C_SPEED_NORMAL);
    __delay_ms(50);
    UART1_Println("[INIT] I2C Software inicializado");
    
    // Inicializar MPU6050
    if (!InicializarMPU6050()) {
        UART1_Println("[ERROR] MPU6050 no responde");
        return false;
    }
    UART1_Println("[INIT] MPU6050 detectado y configurado");
    
    // Calibrar MPU6050
    UART1_Println("[INIT] Calibrando MPU6050 - NO MOVER LA SILLA");
    if (!CalibrarMPU6050()) {
        UART1_Println("[ERROR] Calibración fallida");
        return false;
    }
    UART1_Println("[INIT] Calibración completada");
    
    // Inicializar control de motores
    L298N_Dual_Init();
    L298N_Dual_Stop();
    __delay_ms(50);
    UART1_Println("[INIT] Sistema de motores L298N listo");
    
    // Inicializar sensor ultrasónico
    HC_SR04_Init();
    __delay_ms(50);
    UART1_Println("[INIT] Sensor HC-SR04 inicializado");
    
    // Sistema listo
    silla.sistema_activo = true;
    LED_ON();
    
    return true;
}

static bool InicializarMPU6050(void) {
    
    uint8_t who_am_i;
    
    // Despertar MPU6050
    if (SW_I2C_WriteByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00) != SW_I2C_SUCCESS) {
        return false;
    }
    __delay_ms(100);
    
    // Verificar comunicación
    if (SW_I2C_ReadByte(MPU6050_ADDR, MPU6050_WHO_AM_I, &who_am_i) != SW_I2C_SUCCESS) {
        return false;
    }
    
    if (who_am_i != 0x68 && who_am_i != 0x98) {
        return false;
    }
    
    // Configurar acelerómetro ±4g
    SW_I2C_WriteByte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x08);
    
    // Configurar giroscopio ±500°/s
    SW_I2C_WriteByte(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x08);
    
    __delay_ms(50);
    
    return true;
}

static bool CalibrarMPU6050(void) {
    
    Vector3D_t accel_sum = {0, 0, 0};
    Vector3D_t gyro_sum = {0, 0, 0};
    Vector3D_t accel, gyro;
    const int num_muestras = 100;
    
    // Tomar múltiples muestras
    for (int i = 0; i < num_muestras; i++) {
        if (!LeerMPU6050(&accel, &gyro)) {
            return false;
        }
        
        accel_sum.x += accel.x;
        accel_sum.y += accel.y;
        accel_sum.z += accel.z;
        
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum.z += gyro.z;
        
        __delay_ms(10);
        
        // Mostrar progreso
        if ((i % 20) == 0) {
            sprintf(uart_buffer, "[CAL] Progreso: %d%%", (i * 100) / num_muestras);
            UART1_Println(uart_buffer);
        }
    }
    
    // Calcular offsets
    silla.accel_offset.x = accel_sum.x / num_muestras;
    silla.accel_offset.y = accel_sum.y / num_muestras;
    silla.accel_offset.z = (accel_sum.z / num_muestras) - 1.0f; // Gravedad en Z
    
    silla.gyro_offset.x = gyro_sum.x / num_muestras;
    silla.gyro_offset.y = gyro_sum.y / num_muestras;
    silla.gyro_offset.z = gyro_sum.z / num_muestras;
    
    return true;
}

/*******************************************************************************
 * IMPLEMENTACIÓN: CONTROL PRINCIPAL
 *******************************************************************************/

static void ProcesarControl(void) {
    
    Vector3D_t accel, gyro;
    estado_silla_t comando;
    
    // Leer sensores
    if (!LeerMPU6050(&accel, &gyro)) {
        // Error de lectura - parada de emergencia
        EjecutarMovimiento(SILLA_EMERGENCIA);
        return;
    }
    
    // Leer distancia trasera
    silla.distancia_trasera_cm = LeerDistanciaTrasera();
    
    // Detectar comando basado en inclinación
    comando = DetectarComando(&accel, &gyro);
    
    // Verificar seguridad en reversa
    if (comando == SILLA_ATRAS && silla.distancia_trasera_cm < DISTANCIA_SEGURIDAD_CM) {
        comando = SILLA_EMERGENCIA;
        silla.obstaculo_detectado = true;
    } else {
        silla.obstaculo_detectado = false;
    }
    
    // Ejecutar comando si es diferente al actual o si cambió la velocidad
    if (comando != silla.estado_actual || 
        (tiempo_sistema_ms - silla.tiempo_ultimo_comando) > TIEMPO_ESTABILIDAD_MS) {
        
        EjecutarMovimiento(comando);
        silla.estado_anterior = silla.estado_actual;
        silla.estado_actual = comando;
        silla.tiempo_ultimo_comando = tiempo_sistema_ms;
    }
}

static estado_silla_t DetectarComando(Vector3D_t *accel, Vector3D_t *gyro) {
    
    // Aplicar offsets de calibración
    float ax = accel->x - silla.accel_offset.x;
    float ay = accel->y - silla.accel_offset.y;
    float az = accel->z - silla.accel_offset.z;
    
    // Normalizar para obtener ángulos
    float pitch = atan2f(ax, sqrtf(ay*ay + az*az));  // Inclinación adelante/atrás
    float roll = atan2f(ay, sqrtf(ax*ax + az*az));   // Inclinación izquierda/derecha
    
    // Convertir a valores absolutos para comparación
    float abs_pitch = fabsf(pitch);
    float abs_roll = fabsf(roll);
    
    // Prioridad: Movimiento lineal sobre giros
    
    // ADELANTE: Inclinar hacia adelante
    if (pitch > UMBRAL_INCLINACION_MIN && abs_roll < UMBRAL_GIRO_MIN) {
        return SILLA_ADELANTE;
    }
    
    // ATRÁS: Inclinar hacia atrás
    if (pitch < -UMBRAL_INCLINACION_MIN && abs_roll < UMBRAL_GIRO_MIN) {
        return SILLA_ATRAS;
    }
    
    // GIRO IZQUIERDA: Inclinar a la izquierda
    if (roll > UMBRAL_GIRO_MIN && abs_pitch < UMBRAL_INCLINACION_MIN) {
        return SILLA_GIRO_IZQUIERDA;
    }
    
    // GIRO DERECHA: Inclinar a la derecha
    if (roll < -UMBRAL_GIRO_MIN && abs_pitch < UMBRAL_INCLINACION_MIN) {
        return SILLA_GIRO_DERECHA;
    }
    
    // PARADA: Posición neutral
    return SILLA_PARADA;
}

static void EjecutarMovimiento(estado_silla_t comando) {
    
    Vector3D_t accel, gyro;
    float velocidad_factor = 1.0f;
    
    // Obtener inclinación actual para control proporcional
    if (LeerMPU6050(&accel, &gyro)) {
        float ax = accel.x - silla.accel_offset.x;
        float ay = accel.y - silla.accel_offset.y;
        float az = accel.z - silla.accel_offset.z;
        
        float pitch = atan2f(ax, sqrtf(ay*ay + az*az));
        float roll = atan2f(ay, sqrtf(ax*ax + az*az));
        
        // Calcular factor de velocidad basado en inclinación
        if (comando == SILLA_ADELANTE || comando == SILLA_ATRAS) {
            velocidad_factor = CalcularVelocidad(fabsf(pitch), 
                                                UMBRAL_INCLINACION_MIN, 
                                                UMBRAL_INCLINACION_MAX) / 255.0f;
        } else if (comando == SILLA_GIRO_IZQUIERDA || comando == SILLA_GIRO_DERECHA) {
            velocidad_factor = CalcularVelocidad(fabsf(roll), 
                                                UMBRAL_GIRO_MIN, 
                                                UMBRAL_GIRO_MAX) / 255.0f;
        }
    }
    
    switch (comando) {
        
        case SILLA_ADELANTE:
            silla.velocidad_motor_izq = VELOCIDAD_MIN + 
                                       (uint8_t)((VELOCIDAD_MAX - VELOCIDAD_MIN) * velocidad_factor);
            silla.velocidad_motor_der = silla.velocidad_motor_izq;
            L298N_Dual_Forward_PWM(silla.velocidad_motor_izq, silla.velocidad_motor_der);
            break;
            
        case SILLA_ATRAS:
            silla.velocidad_motor_izq = VELOCIDAD_MIN + 
                                       (uint8_t)((VELOCIDAD_MAX - VELOCIDAD_MIN) * velocidad_factor * 0.7f);
            silla.velocidad_motor_der = silla.velocidad_motor_izq;
            L298N_Dual_Backward_PWM(silla.velocidad_motor_izq, silla.velocidad_motor_der);
            break;
            
        case SILLA_GIRO_IZQUIERDA:
            silla.velocidad_motor_izq = VELOCIDAD_GIRO_BASE * velocidad_factor;
            silla.velocidad_motor_der = VELOCIDAD_GIRO_BASE * velocidad_factor;
            // Motor izquierdo atrás, derecho adelante
            L298N_Motor_Backward_PWM(MOTOR_A, silla.velocidad_motor_izq);
            L298N_Motor_Forward_PWM(MOTOR_B, silla.velocidad_motor_der);
            break;
            
        case SILLA_GIRO_DERECHA:
            silla.velocidad_motor_izq = VELOCIDAD_GIRO_BASE * velocidad_factor;
            silla.velocidad_motor_der = VELOCIDAD_GIRO_BASE * velocidad_factor;
            // Motor izquierdo adelante, derecho atrás
            L298N_Motor_Forward_PWM(MOTOR_A, silla.velocidad_motor_izq);
            L298N_Motor_Backward_PWM(MOTOR_B, silla.velocidad_motor_der);
            break;
            
        case SILLA_EMERGENCIA:
            silla.velocidad_motor_izq = 0;
            silla.velocidad_motor_der = 0;
            L298N_Dual_Stop();
            // Parpadeo rápido del LED
            for (int i = 0; i < 6; i++) {
                LED_Toggle();
                __delay_ms(100);
            }
            break;
            
        case SILLA_PARADA:
        default:
            silla.velocidad_motor_izq = 0;
            silla.velocidad_motor_der = 0;
            L298N_Dual_Stop();
            break;
    }
}

static uint8_t CalcularVelocidad(float inclinacion, float min, float max) {
    
    // Limitar inclinación a rango válido
    if (inclinacion < min) {
        return VELOCIDAD_MIN;
    }
    if (inclinacion > max) {
        return VELOCIDAD_MAX;
    }
    
    // Mapeo lineal de inclinación a velocidad
    float factor = (inclinacion - min) / (max - min);
    return VELOCIDAD_MIN + (uint8_t)((VELOCIDAD_MAX - VELOCIDAD_MIN) * factor);
}

/*******************************************************************************
 * IMPLEMENTACIÓN: SENSORES
 *******************************************************************************/

static bool LeerMPU6050(Vector3D_t *accel, Vector3D_t *gyro) {
    
    uint8_t datos[14];
    
    // Leer 14 bytes empezando desde ACCEL_XOUT_H
    if (SW_I2C_ReadBlock(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, datos, 14) != SW_I2C_SUCCESS) {
        return false;
    }
    
    // Convertir datos del acelerómetro (±4g = 8192 LSB/g)
    int16_t ax_raw = (int16_t)((datos[0] << 8) | datos[1]);
    int16_t ay_raw = (int16_t)((datos[2] << 8) | datos[3]);
    int16_t az_raw = (int16_t)((datos[4] << 8) | datos[5]);
    
    accel->x = ax_raw / 8192.0f;
    accel->y = ay_raw / 8192.0f;
    accel->z = az_raw / 8192.0f;
    
    // Convertir datos del giroscopio (±500°/s = 65.5 LSB/°/s)
    int16_t gx_raw = (int16_t)((datos[8] << 8) | datos[9]);
    int16_t gy_raw = (int16_t)((datos[10] << 8) | datos[11]);
    int16_t gz_raw = (int16_t)((datos[12] << 8) | datos[13]);
    
    gyro->x = gx_raw / 65.5f;
    gyro->y = gy_raw / 65.5f;
    gyro->z = gz_raw / 65.5f;
    
    return true;
}

static uint16_t LeerDistanciaTrasera(void) {
    
    uint16_t distancia = HC_SR04_Get_Distance_CM();
    
    // Si hay error, retornar distancia segura
    if (distancia == HC_SR04_ERROR_DISTANCE) {
        return 999;
    }
    
    return distancia;
}

/*******************************************************************************
 * IMPLEMENTACIÓN: COMUNICACIÓN Y DIAGNÓSTICO
 *******************************************************************************/

static void EnviarEstadoUART(void) {
    
    // Enviar estado actual
    sprintf(uart_buffer, "[ESTADO] %s | Vel_I:%d | Vel_D:%d | Dist:%dcm | Obs:%s",
            EstadoToString(silla.estado_actual),
            silla.velocidad_motor_izq,
            silla.velocidad_motor_der,
            silla.distancia_trasera_cm,
            silla.obstaculo_detectado ? "SI" : "NO");
    
    UART1_Println(uart_buffer);
}

static void ProcesarComandosUART(void) {
    
    char cmd;
    
    if (UART1_Read(&cmd)) {
        switch (cmd) {
            case 'S':
            case 's':
                // Parada de emergencia
                EjecutarMovimiento(SILLA_EMERGENCIA);
                UART1_Println("[CMD] Parada de emergencia activada");
                break;
                
            case 'R':
            case 'r':
                // Reiniciar sistema
                UART1_Println("[CMD] Reiniciando sistema...");
                __delay_ms(100);
                RESET();
                break;
                
            case 'C':
            case 'c':
                // Recalibrar
                UART1_Println("[CMD] Recalibrando - NO MOVER");
                L298N_Dual_Stop();
                CalibrarMPU6050();
                UART1_Println("[CMD] Calibración completada");
                break;
                
            case 'I':
            case 'i':
                // Información del sistema
                UART1_Println("\n=== INFORMACIÓN DEL SISTEMA ===");
                sprintf(uart_buffer, "Estado: %s", EstadoToString(silla.estado_actual));
                UART1_Println(uart_buffer);
                sprintf(uart_buffer, "Tiempo activo: %lu ms", tiempo_sistema_ms);
                UART1_Println(uart_buffer);
                sprintf(uart_buffer, "Distancia trasera: %d cm", silla.distancia_trasera_cm);
                UART1_Println(uart_buffer);
                UART1_Println("===============================\n");
                break;
                
            default:
                UART1_Println("[CMD] Comando no reconocido");
                UART1_Println("Comandos: S=Stop, R=Reset, C=Calibrar, I=Info");
                break;
        }
    }
}

static const char* EstadoToString(estado_silla_t estado) {
    switch (estado) {
        case SILLA_PARADA:          return "PARADO";
        case SILLA_ADELANTE:        return "ADELANTE";
        case SILLA_ATRAS:           return "ATRAS";
        case SILLA_GIRO_IZQUIERDA:  return "IZQUIERDA";
        case SILLA_GIRO_DERECHA:    return "DERECHA";
        case SILLA_EMERGENCIA:      return "EMERGENCIA";
        default:                    return "DESCONOCIDO";
    }
}

/*******************************************************************************
 * IMPLEMENTACIÓN: UTILIDADES
 *******************************************************************************/

static void ActualizarTiempo(void) {
    tiempo_sistema_ms++;
}

static void IndicadorLED(void) {
    if (silla.estado_actual == SILLA_EMERGENCIA) {
        LED_Toggle();
    } else if (silla.sistema_activo) {
        LED_ON();
    } else {
        LED_OFF();
    }
}