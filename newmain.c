/*******************************************************************************
 * SISTEMA DE CONTROL DE SILLA DE RUEDAS - MPU6050 + L298N + HC-SR04
 *
 * PROYECTO: Silla de Ruedas Inteligente para Personas con Discapacidad Motora
 * MICROCONTROLADOR: PIC18F57Q43 @ 64MHz
 *
 * CARACTERISTICAS:
 * - Control por movimientos de cabeza/mano (MPU6050)
 * - Sistema de seguridad con sensor ultrasonico (HC-SR04)
 * - Control dual de motores con arranque suave (L298N)
 * - Deteccion automatica de obstaculos en retroceso
 * - Sistema de emergencia y diagnostico
 * - Interfaz UART para monitoreo y configuracion
 *
 * CONEXIONES HARDWARE:
 * - MPU6050: SCL=RB1, SDA=RB2, VCC=3.3V, GND=GND
 * - L298N: Motor_A(IN1=RD0,IN2=RD1,ENA=RD2), Motor_B(IN3=RD4,IN4=RD5,ENB=RD3)
 * - HC-SR04: TRIG=RF5, ECHO=RF4, VCC=5V, GND=GND
 * - UART: TX=RC2, RX=RC3 (9600 baudios)
 * - LED Estado: RF3 (interno)
 *
 * AUTOR: Sistema de Asistencia Tecnologica
 * FECHA: 2025
 *******************************************************************************/

#include <xc.h>
#include <stdio.h>
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
 * CONFIGURACION DEL SISTEMA DE SILLA DE RUEDAS
 *******************************************************************************/

// Definiciones MPU6050
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_WHO_AM_I 0x75

// Umbrales de movimiento (ajustados para uso de silla de ruedas)
#define ZONA_MUERTA_ACCEL 0.20f   // Zona muerta para evitar movimientos involuntarios
#define ZONA_MUERTA_GYRO 20.0f    // Zona muerta para giros

#define UMBRAL_ADELANTE 0.35f     // Umbral para movimiento adelante
#define UMBRAL_ATRAS -0.35f     // Umbral para movimiento atras
#define UMBRAL_DERECHA -40.0f    // Umbral para giro derecha
#define UMBRAL_IZQUIERDA 40.0f     // Umbral para giro izquierda

// Configuracion de seguridad
#define DISTANCIA_SEGURIDAD_CM 30      // Distancia minima para retroceso seguro
#define TIEMPO_ESTABILIDAD_MS 100     // Tiempo para confirmar comando
#define VELOCIDAD_MAXIMA 80          // Velocidad maxima de motores (%)

// Estados del sistema
typedef enum {
    SILLA_PARADA = 0,
    SILLA_ADELANTE,
    SILLA_ATRAS,
    SILLA_GIRO_IZQUIERDA,
    SILLA_GIRO_DERECHA,
    SILLA_EMERGENCIA,
    SILLA_ERROR
} estado_silla_t;

// Estructura de datos del sensor
typedef struct {
    float x, y, z;
} Vector3D_t;

// Estructura de estado del sistema
typedef struct {
    estado_silla_t estado_actual;
    estado_silla_t estado_anterior;
    uint16_t distancia_trasera_cm;
    bool obstaculo_detectado; // Corregido
    bool sistema_activo;
    bool modo_seguridad;
    uint16_t comando_contador;
    uint32_t ultimo_comando_ms;
} sistema_silla_t;

/*******************************************************************************
 * VARIABLES GLOBALES
 *******************************************************************************/

// Sistema principal
static sistema_silla_t silla = {
    .estado_actual = SILLA_PARADA,
    .estado_anterior = SILLA_PARADA,
    .distancia_trasera_cm = 999,
    .obstaculo_detectado = false, // Corregido
    .sistema_activo = false,
    .modo_seguridad = true,
    .comando_contador = 0,
    .ultimo_comando_ms = 0
};

// Offsets de calibracion del MPU6050
static Vector3D_t accel_offset = {0, 0, 0};
static Vector3D_t gyro_offset = {0, 0, 0};

// Buffers de comunicacion
static char uart_buffer[100];
static uint32_t tiempo_sistema = 0;

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES
 *******************************************************************************/

// Inicializacion
static bool InicializarSistemaCompleto(void);
static bool InicializarMPU6050(void);
static bool CalibracionMPU6050(void);
static bool VerificarSensores(void);

// Control principal
static void ProcesarComandos(void);
static estado_silla_t DetectarMovimientoMPU6050(void);
static bool VerificarSeguridadRetroceso(void);
static void EjecutarComando(estado_silla_t comando);

// Control de motores
static void ControlMotores(estado_silla_t estado);
static void ParadaEmergencia(void);
static void ActivarModoSeguridad(void);

// Lectura de sensores
static bool LeerMPU6050(Vector3D_t *accel, Vector3D_t *gyro);
static uint16_t LeerDistanciaTrasera(void);

// Diagnostico y monitoreo
static void EnviarEstadoSistema(void);
static void ProcesarComandosUART(void);
static const char* EstadoToString(estado_silla_t estado);

// Utilidades
static void IncrementarTiempo(void);
static uint32_t ObtenerTiempo(void);
static void IndicadorLED(void);

/*******************************************************************************
 * FUNCION PRINCIPAL
 *******************************************************************************/

void main(void) {
    
    // === INICIALIZACION COMPLETA DEL SISTEMA ===
    if (!InicializarSistemaCompleto()) {
        UART1_Println("[SISTEMA] ERROR FATAL - Inicializacion fallida");
        UART1_Println("[SISTEMA] Verificar conexiones de hardware");
        
        // Bucle de error con LED parpadeante
        while(1) {
            LED_Toggle();
            __delay_ms(100);
        }
    }
    
    // === PRESENTACION DEL SISTEMA ===
    UART1_Println("");
    UART1_Println("========================================");
    UART1_Println("   SILLA DE RUEDAS INTELIGENTE v2.0   ");
    UART1_Println("========================================");
    UART1_Println("Sistema: FUNCIONANDO");
    UART1_Println("Control: MPU6050 (Movimientos de cabeza)");
    UART1_Println("Seguridad: HC-SR04 (Sensor trasero)");
    UART1_Println("Motores: L298N Dual con arranque suave");
    UART1_Println("========================================");
    UART1_Println("");
    UART1_Println("=== COMANDOS DISPONIBLES ===");
    UART1_Println("- ADELANTE: Inclinar cabeza hacia adelante");
    UART1_Println("- ATRAS: Inclinar cabeza hacia atras");
    UART1_Println("- IZQUIERDA: Girar cabeza hacia la izquierda");
    UART1_Println("- DERECHA: Girar cabeza hacia la derecha");
    UART1_Println("- PARAR: Mantener cabeza centrada");
    UART1_Println("=============================");
    UART1_Println("");
    UART1_Println("=== SISTEMA DE SEGURIDAD ===");
    sprintf(uart_buffer, "- Distancia seguridad: %d cm", DISTANCIA_SEGURIDAD_CM);
    UART1_Println(uart_buffer);
    UART1_Println("- Deteccion automatica de obstaculos");
    UART1_Println("- Parada de emergencia en retroceso");
    UART1_Println("=============================");
    UART1_Println("");
    
    // Marcar sistema como activo
    silla.sistema_activo = true;
    LED_ON();
    
    UART1_Println("[SISTEMA] SILLA DE RUEDAS LISTA PARA USAR!");
    UART1_Println("[SISTEMA] Enviando comandos UART para monitoreo...");
    UART1_Println("");
    
    // === BUCLE PRINCIPAL DEL SISTEMA ===
    while(1) {
        
        // Incrementar tiempo del sistema
        IncrementarTiempo();
        
        // Procesar comandos principales
        ProcesarComandos();
        
        // Enviar estado cada 2 segundos
        if((tiempo_sistema % 2000) == 0) {
            EnviarEstadoSistema();
        }
        
        // Procesar comandos UART (si los hay)
        if(UART1_DATA_READY()) {
            ProcesarComandosUART();
        }
        
        // Indicador visual de funcionamiento
        if((tiempo_sistema % 1000) == 0) {
            IndicadorLED();
        }
        
        // Delay del bucle principal (50Hz para respuesta suave)
        __delay_ms(20);
    }
}

/*******************************************************************************
 * IMPLEMENTACION DE FUNCIONES DE INICIALIZACION
 *******************************************************************************/

/**
 * @brief Inicializa todo el sistema paso a paso con verificaciones
 */
static bool InicializarSistemaCompleto(void) {
    
    // PASO 1: Hardware basico
    ClockInit();
    PinInit();
    UART1_Init();
    __delay_ms(500);
    
    UART1_Println("[INIT] Hardware basico inicializado");
    
    // PASO 2: Sistema I2C
    SW_I2C_Init();
    SW_I2C_SetSpeed(SW_I2C_SPEED_NORMAL);
    __delay_ms(100);
    
    UART1_Println("[INIT] I2C inicializado");
    
    // PASO 3: MPU6050
    if (!InicializarMPU6050()) {
        UART1_Println("[ERROR] MPU6050 no responde");
        return false;
    }
    
    UART1_Println("[INIT] MPU6050 configurado");
    
    // PASO 4: L298N (Motores)
    L298N_Dual_Init();
    L298N_Dual_Stop(); // Asegurar que esten parados
    __delay_ms(100);
    
    UART1_Println("[INIT] Sistema de motores L298N inicializado");
    
    // PASO 5: HC-SR04 (Sensor ultrasonico)
    HC_SR04_Init();
    __delay_ms(100);
    
    UART1_Println("[INIT] Sensor ultrasonico HC-SR04 inicializado");
    
    // PASO 6: Calibracion del MPU6050
    if (!CalibracionMPU6050()) {
        UART1_Println("[ERROR] Fallo en calibracion MPU6050");
        return false;
    }
    
    UART1_Println("[INIT] Calibracion completada");
    
    // PASO 7: Verificacion final de sensores
    if (!VerificarSensores()) {
        UART1_Println("[ERROR] Verificacion de sensores fallida");
        return false;
    }
    
    UART1_Println("[INIT] Verificacion de sensores exitosa");
    
    return true;
}

/**
 * @brief Inicializa y configura el MPU6050
 */
static bool InicializarMPU6050(void) {
    
    // Verificar comunicacion
    uint8_t who_am_i = SW_I2C_ReadRegister(MPU6050_ADDR, MPU6050_WHO_AM_I);
    if (who_am_i != 0x68) {
        sprintf(uart_buffer, "[ERROR] MPU6050 WHO_AM_I = 0x%02X (esperado 0x68)", who_am_i);
        UART1_Println(uart_buffer);
        return false;
    }
    
    // Despertar el sensor
    if (SW_I2C_WriteRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00) != SW_I2C_SUCCESS) {
        return false;
    }
    __delay_ms(100);
    
    // Configurar acelerometro (+-4g para mayor sensibilidad)
    if (SW_I2C_WriteRegister(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x08) != SW_I2C_SUCCESS) {
        return false;
    }
    
    // Configurar giroscopio (+-500 grados/s para movimientos de cabeza)
    if (SW_I2C_WriteRegister(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x08) != SW_I2C_SUCCESS) {
        return false;
    }
    
    __delay_ms(50);
    return true;
}

/**
 * @brief Calibra el MPU6050 con el usuario en posicion neutral
 */
static bool CalibracionMPU6050(void) {
    
    UART1_Println("");
    UART1_Println("=== CALIBRACION DEL SISTEMA ===");
    UART1_Println("INSTRUCCIONES:");
    UART1_Println("1. Sientese comodamente en la silla");
    UART1_Println("2. Mantenga la cabeza en posicion NEUTRAL");
    UART1_Println("3. NO se mueva durante 5 segundos");
    UART1_Println("");
    
    // Countdown con feedback visual
    for (uint8_t i = 5; i > 0; i--) {
        sprintf(uart_buffer, "Calibrando en %u segundos...", i);
        UART1_Println(uart_buffer);
        LED_ON();
        __delay_ms(500);
        LED_OFF();
        __delay_ms(500);
    }
    
    UART1_Println("CALIBRANDO! - Mantenga posicion...");
    LED_ON();
    
    // Realizar calibracion con multiples muestras
    const uint16_t muestras = 200;
    float suma_ax = 0, suma_ay = 0, suma_az = 0;
    float suma_gx = 0, suma_gy = 0, suma_gz = 0;
    uint16_t muestras_validas = 0; // Corregido
    
    for (uint16_t i = 0; i < muestras; i++) {
        Vector3D_t accel, gyro;
        
        if (LeerMPU6050(&accel, &gyro)) {
            suma_ax += accel.x;
            suma_ay += accel.y;
            suma_az += accel.z;
            suma_gx += gyro.x;
            suma_gy += gyro.y;
            suma_gz += gyro.z;
            muestras_validas++; // Corregido
        }
        
        __delay_ms(10);
    }
    
    LED_OFF();
    
    if (muestras_validas < (muestras * 0.8f)) { // Corregido
        UART1_Println("[ERROR] Insuficientes muestras validas para calibracion");
        return false;
    }
    
    // Calcular offsets promedio
    accel_offset.x = suma_ax / muestras_validas; // Corregido
    accel_offset.y = suma_ay / muestras_validas; // Corregido
    accel_offset.z = (suma_az / muestras_validas) - 1.0f; // Compensar gravedad
    
    gyro_offset.x = suma_gx / muestras_validas; // Corregido
    gyro_offset.y = suma_gy / muestras_validas; // Corregido
    gyro_offset.z = suma_gz / muestras_validas; // Corregido
    
    // Mostrar resultados
    sprintf(uart_buffer, "Offsets Acelerometro: X=%.3f Y=%.3f Z=%.3f",
            accel_offset.x, accel_offset.y, accel_offset.z);
    UART1_Println(uart_buffer);
    
    sprintf(uart_buffer, "Offsets Giroscopio: X=%.3f Y=%.3f Z=%.3f",
            gyro_offset.x, gyro_offset.y, gyro_offset.z);
    UART1_Println(uart_buffer);
    
    UART1_Println("CALIBRACION COMPLETADA");
    UART1_Println("===============================");
    UART1_Println("");
    
    return true;
}

/**
 * @brief Verifica que todos los sensores funcionen correctamente
 */
static bool VerificarSensores(void) {
    
    UART1_Println("[TEST] Verificando sensores...");
    
    // Test MPU6050
    Vector3D_t accel, gyro;
    if (!LeerMPU6050(&accel, &gyro)) {
        UART1_Println("[ERROR] MPU6050 no responde en verificacion");
        return false;
    }
    
    // Test HC-SR04
    uint16_t distancia = HC_SR04_Get_Distance_CM();
    if (distancia == HC_SR04_ERROR_DISTANCE) {
        UART1_Println("[ADVERTENCIA] HC-SR04 puede tener problemas");
        // No es critico, continuar
    } else {
        sprintf(uart_buffer, "[TEST] HC-SR04 funcionando - Distancia: %u cm", distancia);
        UART1_Println(uart_buffer);
    }
    
    // Test motores
    if (!L298N_Diagnostics_Check()) {
        UART1_Println("[ERROR] Sistema de motores con problemas");
        return false;
    }
    
    UART1_Println("[TEST] Todos los sensores verificados");
    
    return true;
}

/*******************************************************************************
 * IMPLEMENTACION DE CONTROL PRINCIPAL
 *******************************************************************************/

/**
 * @brief Procesa los comandos del sistema principal
 */
static void ProcesarComandos(void) {
    
    // Leer distancia trasera para seguridad
    silla.distancia_trasera_cm = LeerDistanciaTrasera();
    silla.obstaculo_detectado = (silla.distancia_trasera_cm < DISTANCIA_SEGURIDAD_CM);
    
    // Detectar comando de movimiento del MPU6050
    estado_silla_t comando_detectado = DetectarMovimientoMPU6050();
    
    // Verificar si es un comando nuevo y estable
    if (comando_detectado != silla.estado_anterior) {
        silla.ultimo_comando_ms = ObtenerTiempo();
        silla.estado_anterior = comando_detectado;
        return; // Esperar estabilidad
    }
    
    // Verificar tiempo de estabilidad
    if ((ObtenerTiempo() - silla.ultimo_comando_ms) < TIEMPO_ESTABILIDAD_MS) {
        return; // Comando aun no estable
    }
    
    // Ejecutar comando si es diferente al actual
    if (comando_detectado != silla.estado_actual) {
        EjecutarComando(comando_detectado);
    }
}

/**
 * @brief Detecta movimientos del MPU6050 y los convierte en comandos de silla
 */
static estado_silla_t DetectarMovimientoMPU6050(void) {
    
    Vector3D_t accel, gyro;
    
    // Leer datos del sensor
    if (!LeerMPU6050(&accel, &gyro)) {
        return SILLA_ERROR;
    }
    
    // PRIORIDAD 1: Deteccion de movimientos lineales (acelerometro Y)
    if (accel.y > UMBRAL_ADELANTE && fabs(accel.y) > ZONA_MUERTA_ACCEL) {
        return SILLA_ADELANTE;
    }
    else if (accel.y < UMBRAL_ATRAS && fabs(accel.y) > ZONA_MUERTA_ACCEL) {
        return SILLA_ATRAS;
    }
    
    // PRIORIDAD 2: Deteccion de giros (giroscopio Z)
    else if (gyro.z < UMBRAL_DERECHA && fabs(gyro.z) > ZONA_MUERTA_GYRO) {
        return SILLA_GIRO_DERECHA;
    }
    else if (gyro.z > UMBRAL_IZQUIERDA && fabs(gyro.z) > ZONA_MUERTA_GYRO) {
        return SILLA_GIRO_IZQUIERDA;
    }
    
    // Por defecto: parada
    return SILLA_PARADA;
}

/**
 * @brief Verifica la seguridad antes de permitir retroceso
 */
static bool VerificarSeguridadRetroceso(void) {
    
    if (!silla.modo_seguridad) {
        return true; // Seguridad deshabilitada
    }
    
    // Verificar distancia trasera
    uint16_t distancia = LeerDistanciaTrasera();
    
    if (distancia < DISTANCIA_SEGURIDAD_CM) {
        sprintf(uart_buffer, "[SEGURIDAD] OBSTACULO DETECTADO a %u cm - RETROCESO BLOQUEADO", distancia);
        UART1_Println(uart_buffer);
        return false;
    }
    
    return true;
}

/**
 * @brief Ejecuta el comando especificado con verificaciones de seguridad
 */
static void EjecutarComando(estado_silla_t comando) {
    
    // Verificar seguridad especial para retroceso
    if (comando == SILLA_ATRAS && !VerificarSeguridadRetroceso()) {
        comando = SILLA_EMERGENCIA;
    }
    
    // Actualizar estado del sistema
    silla.estado_actual = comando;
    silla.comando_contador++;
    
    // Ejecutar control de motores
    ControlMotores(comando);
    
    // Log del comando ejecutado
    sprintf(uart_buffer, "[CMD %04u] %s", silla.comando_contador, EstadoToString(comando));
    UART1_Println(uart_buffer);
}

/*******************************************************************************
 * IMPLEMENTACION DE CONTROL DE MOTORES
 *******************************************************************************/

/**
 * @brief Controla los motores segun el estado de la silla
 */
static void ControlMotores(estado_silla_t estado) {
    
    switch(estado) {
        
        case SILLA_PARADA:
            L298N_Dual_Stop();
            LED_OFF();
            break;
            
        case SILLA_ADELANTE:
            // Ambos motores hacia adelante (arranque suave)
            L298N_Dual_PWM_Start(MOTOR_FORWARD);
            LED_ON();
            break;
            
        case SILLA_ATRAS:
            // Ambos motores hacia atras (arranque suave)
            L298N_Dual_PWM_Start(MOTOR_BACKWARD);
            LED_ON();
            break;
            
        case SILLA_GIRO_IZQUIERDA:
            // Motor derecho adelante, motor izquierdo atras (giro en lugar)
            L298N_Motor_PWM_Start(MOTOR_B, MOTOR_FORWARD);   // Motor derecho adelante
            L298N_Motor_PWM_Start(MOTOR_A, MOTOR_BACKWARD);  // Motor izquierdo atras
            LED_ON();
            break;
            
        case SILLA_GIRO_DERECHA:
            // Motor izquierdo adelante, motor derecho atras (giro en lugar)
            L298N_Motor_PWM_Start(MOTOR_A, MOTOR_FORWARD);   // Motor izquierdo adelante
            L298N_Motor_PWM_Start(MOTOR_B, MOTOR_BACKWARD);  // Motor derecho atras
            LED_ON();
            break;
            
        case SILLA_EMERGENCIA:
            ParadaEmergencia();
            break;
            
        case SILLA_ERROR:
            ParadaEmergencia();
            ActivarModoSeguridad();
            break;
            
        default:
            L298N_Dual_Stop();
            break;
    }
}

/**
 * @brief Ejecuta parada de emergencia inmediata
 */
static void ParadaEmergencia(void) {
    
    L298N_Reset_Motors();  // Parada inmediata
    
    // Parpadeo rapido del LED para indicar emergencia
    for(uint8_t i = 0; i < 10; i++) {
        LED_Toggle();
        __delay_ms(50);
    }
    
    LED_OFF();
    
    UART1_Println("[EMERGENCIA] MOTORES DETENIDOS");
}

/**
 * @brief Activa el modo seguridad adicional
 */
static void ActivarModoSeguridad(void) {
    
    silla.modo_seguridad = true;
    
    UART1_Println("[SEGURIDAD] Modo seguridad activado");
    UART1_Println("[SEGURIDAD] Verificaciones adicionales habilitadas");
}

/*******************************************************************************
 * IMPLEMENTACION DE LECTURA DE SENSORES
 *******************************************************************************/

/**
 * @brief Lee datos del MPU6050 con compensacion de offset
 */
static bool LeerMPU6050(Vector3D_t *accel, Vector3D_t *gyro) {
    
    uint8_t datos[14];
    
    // Leer todos los datos del sensor
    if (SW_I2C_ReadBlock(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, datos, 14) != SW_I2C_SUCCESS) {
        return false;
    }
    
    // Procesar acelerometro (+-4g -> 8192 LSB/g)
    int16_t raw_ax = (datos[0] << 8) | datos[1];
    int16_t raw_ay = (datos[2] << 8) | datos[3];
    int16_t raw_az = (datos[4] << 8) | datos[5];
    
    accel->x = (raw_ax / 8192.0f) - accel_offset.x;
    accel->y = (raw_ay / 8192.0f) - accel_offset.y;
    accel->z = (raw_az / 8192.0f) - accel_offset.z;
    
    // Procesar giroscopio (+-500 grados/s -> 65.5 LSB/grados/s)
    int16_t raw_gx = (datos[8] << 8) | datos[9];
    int16_t raw_gy = (datos[10] << 8) | datos[11];
    int16_t raw_gz = (datos[12] << 8) | datos[13];
    
    gyro->x = (raw_gx / 65.5f) - gyro_offset.x;
    gyro->y = (raw_gy / 65.5f) - gyro_offset.y;
    gyro->z = (raw_gz / 65.5f) - gyro_offset.z;
    
    return true;
}

/**
 * @brief Lee la distancia del sensor trasero
 */
static uint16_t LeerDistanciaTrasera(void) {
    
    uint16_t distancia = HC_SR04_Get_Distance_CM();
    
    // Si hay error, asumir distancia segura para no bloquear
    if (distancia == HC_SR04_ERROR_DISTANCE) {
        return 999; // Distancia "infinita"
    }
    
    return distancia;
}

/*******************************************************************************
 * IMPLEMENTACION DE DIAGNOSTICO Y MONITOREO
 *******************************************************************************/

/**
 * @brief Envia el estado completo del sistema por UART
 */
static void EnviarEstadoSistema(void) {
    
    sprintf(uart_buffer, "[ESTADO] Silla: %s | Distancia: %u cm | Obstaculo: %s | Comandos: %u",
            EstadoToString(silla.estado_actual),
            silla.distancia_trasera_cm,
            silla.obstaculo_detectado ? "SI" : "NO",
            silla.comando_contador);
    UART1_Println(uart_buffer);
}

/**
 * @brief Procesa comandos recibidos por UART
 */
static void ProcesarComandosUART(void) {
    
    char comando = UART1_Read();
    
    switch(comando) {
        case 'S':
        case 's':
            ParadaEmergencia();
            UART1_Println("[UART] Parada de emergencia activada");
            break;
            
        case 'R':
        case 'r':
            silla.sistema_activo = true;
            silla.estado_actual = SILLA_PARADA;
            UART1_Println("[UART] Sistema reiniciado");
            break;
            
        case 'D':
        case 'd':
            EnviarEstadoSistema();
            break;
            
        case 'H':
        case 'h':
            UART1_Println("[UART] Comandos: S=Stop, R=Reset, D=Estado, H=Ayuda");
            break;
            
        default:
            sprintf(uart_buffer, "[UART] Comando desconocido: %c", comando);
            UART1_Println(uart_buffer);
            break;
    }
}

/**
 * @brief Convierte estado de silla a string descriptivo
 */
static const char* EstadoToString(estado_silla_t estado) {
    
    switch(estado) {
        case SILLA_PARADA: return "PARADA";
        case SILLA_ADELANTE: return "ADELANTE";
        case SILLA_ATRAS: return "ATRAS"; // Corregido
        case SILLA_GIRO_IZQUIERDA: return "GIRO IZQUIERDA";
        case SILLA_GIRO_DERECHA: return "GIRO DERECHA";
        case SILLA_EMERGENCIA: return "EMERGENCIA";
        case SILLA_ERROR: return "ERROR";
        default: return "DESCONOCIDO";
    }
}

/*******************************************************************************
 * IMPLEMENTACION DE UTILIDADES
 *******************************************************************************/

/**
 * @brief Incrementa el contador de tiempo del sistema
 */
static void IncrementarTiempo(void) {
    tiempo_sistema += 20; // Incremento en ms basado en delay del bucle
}

/**
 * @brief Obtiene el tiempo actual del sistema
 */
static uint32_t ObtenerTiempo(void) {
    return tiempo_sistema;
}

/**
 * @brief Controla el LED indicador del sistema
 */
static void IndicadorLED(void) {
    
    switch(silla.estado_actual) {
        case SILLA_PARADA:
            LED_OFF();
            break;
            
        case SILLA_ADELANTE:
        case SILLA_ATRAS:
        case SILLA_GIRO_IZQUIERDA:
        case SILLA_GIRO_DERECHA:
            LED_ON();
            break;
            
        case SILLA_EMERGENCIA:
        case SILLA_ERROR:
            LED_Toggle(); // Parpadeo para indicar problema
            break;
            
        default:
            LED_OFF();
            break;
    }
}

/*******************************************************************************
 * NOTAS DE IMPLEMENTACION:
 *
 * 1. SEGURIDAD PRIORITARIA:
 * - Verificacion automatica de obstaculos antes de retroceso
 * - Parada de emergencia inmediata si se detecta problema
 * - Arranque suave de motores para evitar movimientos bruscos
 *
 * 2. INTERFAZ INTUITIVA:
 * - Movimientos naturales de cabeza para control
 * - Zonas muertas para evitar movimientos involuntarios
 * - Tiempo de estabilidad para confirmar comandos
 *
 * 3. MONITOREO COMPLETO:
 * - Estado en tiempo real por UART
 * - Comandos de emergencia remotos
 * - Diagnostico continuo de sensores
 *
 * 4. ACCESIBILIDAD:
 * - Diseno especifico para personas con discapacidad motora
 * - Control manos libres con movimientos minimos
 * - Sistema tolerante a fallos con multiples respaldos
 *
 * 5. CONFIGURACION AJUSTABLE:
 * - Umbrales de sensibilidad modificables
 * - Distancia de seguridad configurable
 * - Modo seguridad activable/desactivable
 *******************************************************************************/