/*******************************************************************************
 * MAIN DE DIAGNÓSTICO DE MOTORES L298N - VERSIÓN CORREGIDA
 *
 * PROYECTO: Silla de Ruedas Inteligente - Módulo de Prueba de Motores
 * MICROCONTROLADOR: PIC18F57Q43 @ 64MHz
 *
 * OBJETIVO:
 * - Probar la conexión y alimentación del L298N de forma aislada.
 * - Permitir al usuario activar cada motor individualmente por Bluetooth.
 *******************************************************************************/

#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// === INCLUDES DEL PROYECTO ===
#include "config.h"
#include "UART_LIB.h"
#include "L298N_Dual.h"

// Prototipos
void InicializarDiagnostico(void);
void ProcesarComandosMotores(void);

/*******************************************************************************
 * FUNCIÓN PRINCIPAL
 *******************************************************************************/
void main(void) {
    InicializarDiagnostico();
    
    UART1_Println("\n=== DIAGNOSTICO DE MOTORES L298N ===");
    UART1_Println("Envie comandos para probar los motores:");
    UART1_Println(" 'A' -> Probar Motor A (2 seg)");
    UART1_Println(" 'B' -> Probar Motor B (2 seg)");
    UART1_Println(" 'S' -> Detener todo");
    UART1_Println("--------------------------------------");

    while (1) {
        if (UART1_DATA_READY()) {
            ProcesarComandosMotores();
        }
    }
}

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES DE DIAGNÓSTICO
 *******************************************************************************/

void InicializarDiagnostico(void) {
    // Inicializar hardware esencial
    ClockInit();
    PinInit();
    UART1_Init();
    __delay_ms(100);

    // Inicializar el controlador de motores
    L298N_Dual_Init();
    L298N_Dual_Stop(); // Asegurarse de que los motores estén detenidos al inicio
}

void ProcesarComandosMotores(void) {
    char comando = UART1_Read();

    switch (comando) {
        case 'A':
        case 'a':
            UART1_Println("[TEST] Activando Motor A...");
            L298N_Motor_SetSpeed(MOTOR_A, MOTOR_FORWARD, 80); // Activar Motor A al 80%
            __delay_ms(2000); // Mantenerlo encendido por 2 segundos
            L298N_Motor_Stop(MOTOR_A);
            UART1_Println("[TEST] Motor A detenido.");
            break;

        case 'B':
        case 'b':
            UART1_Println("[TEST] Activando Motor B...");
            L298N_Motor_SetSpeed(MOTOR_B, MOTOR_FORWARD, 80); // Activar Motor B al 80%
            __delay_ms(2000); // Mantenerlo encendido por 2 segundos
            L298N_Motor_Stop(MOTOR_B);
            UART1_Println("[TEST] Motor B detenido.");
            break;

        case 'S':
        case 's':
            UART1_Println("[TEST] Deteniendo todos los motores.");
            L298N_Dual_Stop();
            break;

        default:
            UART1_Println("[ERROR] Comando no reconocido.");
            break;
    }
}