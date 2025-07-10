/*******************************************************************************
 * MAIN DE DIAGNOSTICO L298N - VERSION COMPLETA Y CORREGIDA
 *
 * CORRECCIONES APLICADAS:
 * - Agregados todos los includes necesarios
 * - Declaradas todas las variables globales
 * - Corregidos caracteres especiales
 * - Agregadas funciones de inicializacion
 * - Version completa que compila sin errores
 *
 * CONEXIONES L298N:
 * RD0 ? IN1, RD1 ? IN2, RD2 ? ENA
 * RD3 ? ENB, RD4 ? IN3, RD5 ? IN4, RD6 ? LED Debug
 *
 * ALIMENTACION CRITICA:
 * VCC (Pin 9) ? 5V del PIC
 * VS (Pin 8)  ? 9V externo ? ¡MUY IMPORTANTE!
 * GND ? Comun PIC + 9V
 *******************************************************************************/

// =============================================================================
// INCLUDES NECESARIOS
// =============================================================================
#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Includes del proyecto
#include "config.h"
#include "UART_LIB.h"

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================
static char uart_buffer[100];  // Buffer para sprintf

// =============================================================================
// PROTOTIPOS DE FUNCIONES
// =============================================================================
static bool Inicializar_Sistema_Diagnostico(void);
static void Test_Pines_Individuales_CORREGIDO(void);
static void Diagnostico_L298N_Completo(void);
static void Test_Voltaje_Simultaneo(void);
static void Test_Manual_Motor_A(void);
static void Test_Manual_Motor_B(void);
static void Mostrar_Menu_Diagnostico(void);
static void Procesar_Comandos_Diagnostico(void);

// =============================================================================
// FUNCION PRINCIPAL
// =============================================================================
void main(void) {
    
    // === INICIALIZACION DEL SISTEMA ===
    if (!Inicializar_Sistema_Diagnostico()) {
        // Error critico - bucle infinito con LED parpadeante
        while(1) {
            LED_Toggle();
            __delay_ms(200);
        }
    }
    
    // === PRESENTACION DEL SISTEMA DE DIAGNOSTICO ===
    UART1_Println("");
    UART1_Println("##############################################");
    UART1_Println("#      DIAGNOSTICO COMPLETO L298N           #");
    UART1_Println("#           PIC18F57Q43 @ 64MHz             #");
    UART1_Println("##############################################");
    UART1_Println("");
    UART1_Println("ESTADO: Sistema de diagnostico inicializado");
    UART1_Println("HARDWARE: L298N + Motores DC");
    UART1_Println("COMUNICACION: UART 9600 baudios");
    UART1_Println("");
    
    Mostrar_Menu_Diagnostico();
    
    // === DIAGNOSTICO AUTOMATICO INICIAL ===
    UART1_Println("[SISTEMA] Ejecutando diagnostico automatico en 3 segundos...");
    UART1_Println("[SISTEMA] Presiona cualquier tecla para saltar");
    
    // Countdown con posibilidad de interrupcion
    for(uint8_t i = 3; i > 0; i--) {
        sprintf(uart_buffer, "[COUNTDOWN] %u...", i);
        UART1_Println(uart_buffer);
        
        for(uint8_t j = 0; j < 10; j++) {
            if(UART1_DATA_READY()) {
                char tecla = UART1_Read();
                UART1_Println("[SISTEMA] Saltando al modo manual");
                goto modo_manual;
            }
            __delay_ms(100);
        }
    }
    
    // === EJECUTAR DIAGNOSTICO AUTOMATICO ===
    Diagnostico_L298N_Completo();
    
    modo_manual:
    
    // === MODO MANUAL INTERACTIVO ===
    UART1_Println("");
    UART1_Println("[MODO MANUAL] Sistema listo para comandos");
    Mostrar_Menu_Diagnostico();
    
    while(1) {
        if(UART1_DATA_READY()) {
            Procesar_Comandos_Diagnostico();
        }
        
        // LED de estado
        LED_Toggle();
        __delay_ms(500);
    }
}

// =============================================================================
// IMPLEMENTACION DE FUNCIONES DE INICIALIZACION
// =============================================================================

/**
 * @brief Inicializa el sistema basico para diagnostico
 */
static bool Inicializar_Sistema_Diagnostico(void) {
    
    // PASO 1: Hardware basico
    ClockInit();
    PinInit();
    __delay_ms(100);
    
    // PASO 2: UART para comunicacion
    UART1_Init();
    __delay_ms(100);
    
    // PASO 3: Verificar que el Puerto D este configurado
    if(TRISD != 0x00) {
        UART1_Println("[ERROR] Puerto D mal configurado");
        sprintf(uart_buffer, "[ERROR] TRISD=0x%02X (debe ser 0x00)", TRISD);
        UART1_Println(uart_buffer);
        return false;
    }
    
    // PASO 4: Estado inicial seguro
    LATD = 0x00;  // Todos los motores parados
    
    // PASO 5: LED de estado
    LED_ON();
    
    return true;
}

// =============================================================================
// IMPLEMENTACION DE FUNCIONES DE DIAGNOSTICO
// =============================================================================

/**
 * @brief Test corregido de pines individuales sin interferencias
 */
static void Test_Pines_Individuales_CORREGIDO(void) {
    UART1_Println("");
    UART1_Println("=== TEST PINES INDIVIDUALES CORREGIDO ===");
    UART1_Println("Probando cada pin del Puerto D sin interferencias...");
    
    const char* nombres[] = {
        "RD0 (Motor A IN1)",
        "RD1 (Motor A IN2)", 
        "RD2 (Motor A ENA)",
        "RD3 (Motor B ENB)",
        "RD4 (Motor B IN3)",
        "RD5 (Motor B IN4)",
        "RD6 (LED Debug)"
    };
    
    // Parar todo primero
    LATD = 0x00;
    __delay_ms(500);
    
    for(uint8_t i = 0; i < 7; i++) {
        sprintf(uart_buffer, "Probando %s...", nombres[i]);
        UART1_Println(uart_buffer);
        
        // ? CORRECCIÓN: Activar SOLO este pin sin afectar otros
        LATD |= (1 << i);   // Activar pin actual SIN tocar otros
        __delay_ms(1000);
        
        // Verificar estado
        uint8_t estado = (PORTD >> i) & 1;
        sprintf(uart_buffer, "Estado: %s", estado ? "HIGH (5V)" : "LOW (0V)");
        UART1_Println(uart_buffer);
        
        // Desactivar SOLO este pin
        LATD &= ~(1 << i);  // Desactivar pin actual SIN tocar otros
        __delay_ms(300);
    }
    
    UART1_Println("=== FIN TEST PINES CORREGIDO ===");
}

/**
 * @brief Diagnostico completo del modulo L298N
 */
static void Diagnostico_L298N_Completo(void) {
    
    UART1_Println("");
    UART1_Println("##############################################");
    UART1_Println("#      DIAGNOSTICO COMPLETO L298N           #");
    UART1_Println("##############################################");
    
    // === PASO 1: Verificar configuracion de pines ===
    UART1_Println("");
    UART1_Println("=== PASO 1: CONFIGURACION DE PINES ===");
    
    if(TRISD != 0x00) {
        UART1_Println("ERROR: TRISD no esta configurado como salidas");
        sprintf(uart_buffer, "   TRISD actual: 0x%02X (debe ser 0x00)", TRISD);
        UART1_Println(uart_buffer);
        return;
    } else {
        UART1_Println("OK: TRISD configurado correctamente (0x00)");
    }
    
    // === PASO 2: Test de pines corregido ===
    UART1_Println("");
    UART1_Println("=== PASO 2: TEST DE PINES MEJORADO ===");
    Test_Pines_Individuales_CORREGIDO();
    
    // === PASO 3: Test de habilitacion de motores ===
    UART1_Println("");
    UART1_Println("=== PASO 3: TEST DE HABILITACION ===");
    
    // Parar todo
    LATD = 0x00;
    __delay_ms(100);
    
    // Test Motor A
    UART1_Println("Probando habilitacion Motor A (ENA=RD2)...");
    LATDbits.LATD2 = 1;  // Habilitar Motor A
    __delay_ms(500);
    uint8_t ena_state = PORTDbits.RD2;
    sprintf(uart_buffer, "ENA (RD2) estado: %s", ena_state ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    
    // Test Motor B  
    UART1_Println("Probando habilitacion Motor B (ENB=RD3)...");
    LATDbits.LATD3 = 1;  // Habilitar Motor B
    __delay_ms(500);
    uint8_t enb_state = PORTDbits.RD3;
    sprintf(uart_buffer, "ENB (RD3) estado: %s", enb_state ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    
    // Limpiar
    LATD = 0x00;
    
    // === PASO 4: Test de control direccional ===
    UART1_Println("");
    UART1_Println("=== PASO 4: TEST DIRECCIONAL ===");
    
    UART1_Println("Test Motor A - Configuracion ADELANTE:");
    LATDbits.LATD0 = 1;  // IN1 = HIGH
    LATDbits.LATD1 = 0;  // IN2 = LOW
    LATDbits.LATD2 = 1;  // ENA = HIGH
    __delay_ms(1000);
    
    sprintf(uart_buffer, "  IN1: %s, IN2: %s, ENA: %s", 
            PORTDbits.RD0 ? "HIGH" : "LOW",
            PORTDbits.RD1 ? "HIGH" : "LOW", 
            PORTDbits.RD2 ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    
    if(PORTDbits.RD0 && !PORTDbits.RD1 && PORTDbits.RD2) {
        UART1_Println("OK: Configuracion correcta para ADELANTE");
    } else {
        UART1_Println("ERROR: Configuracion incorrecta");
    }
    
    // Parar todo
    LATD = 0x00;
    __delay_ms(500);
    
    // === PASO 5: Verificaciones de alimentacion ===
    UART1_Println("");
    UART1_Println("=== PASO 5: VERIFICACION DE ALIMENTACION ===");
    UART1_Println("VERIFICAR MANUALMENTE:");
    UART1_Println("   1. VCC (Pin 9 L298N) -> 5V del PIC");
    UART1_Println("   2. VS (Pin 8 L298N) -> 7-35V EXTERNO <- CRITICO");
    UART1_Println("   3. GND -> Comun PIC + alimentacion externa");
    UART1_Println("   4. Motores conectados a OUT1/OUT2 y OUT3/OUT4");
    
    // === PASO 6: Test de carga de motores ===
    UART1_Println("");
    UART1_Println("=== PASO 6: TEST CON MOTORES REALES ===");
    UART1_Println("Activando Motor A por 3 segundos...");
    UART1_Println("OBSERVAR: Se mueve el motor fisicamente?");
    
    // Configurar Motor A adelante
    LATDbits.LATD0 = 1;  // IN1 = HIGH
    LATDbits.LATD1 = 0;  // IN2 = LOW  
    LATDbits.LATD2 = 1;  // ENA = HIGH
    
    // Countdown visual
    for(uint8_t i = 3; i > 0; i--) {
        sprintf(uart_buffer, "Motor A activo... %u segundos restantes", i);
        UART1_Println(uart_buffer);
        __delay_ms(1000);
    }
    
    // Parar motor
    LATD = 0x00;
    UART1_Println("Motor A parado");
    
    UART1_Println("");
    UART1_Println("##############################################");
    UART1_Println("#         DIAGNOSTICO COMPLETADO           #");
    UART1_Println("##############################################");
    UART1_Println("");
    UART1_Println("Si los motores NO se mueven fisicamente:");
    UART1_Println("1. Falta alimentacion VS (7-35V) en Pin 8");
    UART1_Println("2. Conexiones de motores incorrectas");
    UART1_Println("3. L298N defectuoso");
    UART1_Println("4. Motores danados");
    UART1_Println("");
}

/**
 * @brief Test de voltaje en todos los pines simultaneamente
 */
static void Test_Voltaje_Simultaneo(void) {
    
    UART1_Println("");
    UART1_Println("=== TEST DE VOLTAJE SIMULTANEO ===");
    UART1_Println("Activando TODOS los pines simultaneamente...");
    
    // Activar TODOS los pines del Puerto D
    LATD = 0x7F;  // Todos los bits de RD0 a RD6 en HIGH
    __delay_ms(1000);
    
    // Leer estados
    uint8_t portd_state = PORTD;
    
    UART1_Println("Estados de pines:");
    sprintf(uart_buffer, "RD0 (IN1): %s", (portd_state & 0x01) ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    sprintf(uart_buffer, "RD1 (IN2): %s", (portd_state & 0x02) ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    sprintf(uart_buffer, "RD2 (ENA): %s", (portd_state & 0x04) ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    sprintf(uart_buffer, "RD3 (ENB): %s", (portd_state & 0x08) ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    sprintf(uart_buffer, "RD4 (IN3): %s", (portd_state & 0x10) ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    sprintf(uart_buffer, "RD5 (IN4): %s", (portd_state & 0x20) ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    sprintf(uart_buffer, "RD6 (LED): %s", (portd_state & 0x40) ? "HIGH" : "LOW");
    UART1_Println(uart_buffer);
    
    sprintf(uart_buffer, "PORTD completo: 0x%02X", portd_state);
    UART1_Println(uart_buffer);
    
    // Limpiar
    LATD = 0x00;
    
    if(portd_state == 0x7F) {
        UART1_Println("OK: TODOS los pines funcionan correctamente");
    } else {
        UART1_Println("ERROR: Algunos pines NO funcionan");
        sprintf(uart_buffer, "   Esperado: 0x7F, Obtenido: 0x%02X", portd_state);
        UART1_Println(uart_buffer);
    }
}

/**
 * @brief Test manual del Motor A
 */
static void Test_Manual_Motor_A(void) {
    UART1_Println("=== TEST MANUAL MOTOR A ===");
    UART1_Println("Activando Motor A por 3 segundos...");
    
    LATDbits.LATD0 = 1;  // IN1 = HIGH
    LATDbits.LATD1 = 0;  // IN2 = LOW
    LATDbits.LATD2 = 1;  // ENA = HIGH
    
    for(uint8_t i = 3; i > 0; i--) {
        sprintf(uart_buffer, "Motor A girando... %u", i);
        UART1_Println(uart_buffer);
        __delay_ms(1000);
    }
    
    LATD = 0x00;
    UART1_Println("Motor A parado");
}

/**
 * @brief Test manual del Motor B
 */
static void Test_Manual_Motor_B(void) {
    UART1_Println("=== TEST MANUAL MOTOR B ===");
    UART1_Println("Activando Motor B por 3 segundos...");
    
    LATDbits.LATD4 = 1;  // IN3 = HIGH
    LATDbits.LATD5 = 0;  // IN4 = LOW
    LATDbits.LATD3 = 1;  // ENB = HIGH
    
    for(uint8_t i = 3; i > 0; i--) {
        sprintf(uart_buffer, "Motor B girando... %u", i);
        UART1_Println(uart_buffer);
        __delay_ms(1000);
    }
    
    LATD = 0x00;
    UART1_Println("Motor B parado");
}

/**
 * @brief Muestra el menu de comandos de diagnostico
 */
static void Mostrar_Menu_Diagnostico(void) {
    UART1_Println("=== COMANDOS DE DIAGNOSTICO ===");
    UART1_Println("");
    UART1_Println("D - Diagnostico completo L298N");
    UART1_Println("V - Test voltaje simultaneo");
    UART1_Println("T - Test pines individual corregido");
    UART1_Println("M - Test manual Motor A");
    UART1_Println("N - Test manual Motor B");
    UART1_Println("S - STOP todos los motores");
    UART1_Println("H - Mostrar esta ayuda");
    UART1_Println("");
    UART1_Println("===============================");
    UART1_Println("Enviar comando:");
}

/**
 * @brief Procesa comandos recibidos por UART
 */
static void Procesar_Comandos_Diagnostico(void) {
    
    char comando = UART1_Read();
    sprintf(uart_buffer, "[CMD] Recibido: '%c'", comando);
    UART1_Println(uart_buffer);
    
    switch(comando) {
        case 'D':
        case 'd':
            Diagnostico_L298N_Completo();
            break;
            
        case 'V':
        case 'v':
            Test_Voltaje_Simultaneo();
            break;
            
        case 'T':
        case 't':
            Test_Pines_Individuales_CORREGIDO();
            break;
            
        case 'M':
        case 'm':
            Test_Manual_Motor_A();
            break;
            
        case 'N':
        case 'n':
            Test_Manual_Motor_B();
            break;
            
        case 'S':
        case 's':
            UART1_Println("[STOP] Parando todos los motores");
            LATD = 0x00;
            break;
            
        case 'H':
        case 'h':
            Mostrar_Menu_Diagnostico();
            break;
            
        default:
            sprintf(uart_buffer, "[ERROR] Comando desconocido: '%c'", comando);
            UART1_Println(uart_buffer);
            UART1_Println("[AYUDA] Presiona 'H' para ver comandos disponibles");
            break;
    }
}

// FIN DEL ARCHIVO newmain1.c