#ifndef CONFIG_H
#define CONFIG_H

// Frecuencia del oscilador para la función __delay_ms()
#define _XTAL_FREQ 64000000UL

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// --- Definición de Pines de Hardware ---
// LED de Estado (en RD0)
#define LED_STATUS_TRIS     TRISDbits.TRISD0
#define LED_STATUS_LAT      LATDbits.LATD0

// Sensor Ultrasónico HC-SR04 (en RD1 y RD2)
#define HC_SR04_TRIG_TRIS   TRISDbits.TRISD1
#define HC_SR04_ECHO_TRIS   TRISDbits.TRISD2
#define HC_SR04_TRIG_LAT    LATDbits.LATD1
#define HC_SR04_ECHO_READ() (PORTDbits.RD2)

// --- Macros de Control ---
#define HC_SR04_TRIG_HIGH() (HC_SR04_TRIG_LAT = 1)
#define HC_SR04_TRIG_LOW()  (HC_SR04_TRIG_LAT = 0)
#define LED_ON()            (LED_STATUS_LAT = 1)
#define LED_OFF()           (LED_STATUS_LAT = 0)
#define LED_Toggle()        (LED_STATUS_LAT = ~LED_STATUS_LAT)

// --- Tipos de Datos Globales ---
// Estado de los motores (definición única para todo el proyecto)
typedef enum {
    S_STOP,
    S_FORWARD,
    S_BACKWARD,
    S_LEFT,
    S_RIGHT
} motor_state_t;

// Estructura principal de la silla de ruedas (definición única)
typedef struct {
    float angulo_cabeza;
    uint8_t velocidad_motor_izq;
    uint8_t velocidad_motor_der;
    uint16_t distancia_obstaculo;
    bool modo_seguridad_activo;
} SillaRuedas;

// --- Prototipos de Funciones Globales ---
void SYSTEM_Initialize(void);
void PinInit(void);
void ClockInit(void);

#endif /* CONFIG_H */