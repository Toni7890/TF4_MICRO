/*
 * ===============================================================================
 * DELAYS.H - SISTEMA DE DELAYS VARIABLES PARA SILLA DE RUEDAS
 * ===============================================================================
 * Proporciona funciones de delay variables necesarias para el HC_SR04
 * y otros componentes del sistema de silla de ruedas
 * ===============================================================================
 */

#ifndef DELAYS_H
#define DELAYS_H

#include <xc.h>
#include <stdint.h>

// Definir la frecuencia del oscilador si no está definida
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 64000000UL
#endif

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES DE DELAY
 *******************************************************************************/

/**
 * @brief Delay variable en milisegundos
 * @param ms Tiempo en milisegundos (1-65535)
 * @note Utiliza __delay_ms() internamente en bucle
 */
void delay_ms_variable(uint16_t ms);

/**
 * @brief Delay variable en microsegundos  
 * @param us Tiempo en microsegundos (1-65535)
 * @note Utiliza __delay_us() internamente en bucle
 */
void delay_us_variable(uint16_t us);

/**
 * @brief Delay preciso para tiempos cortos
 * @param cycles Número de ciclos de instrucción
 * @note Para delays muy precisos menores a 1us
 */
void delay_cycles(uint16_t cycles);

/*******************************************************************************
 * IMPLEMENTACIONES INLINE PARA DELAYS COMUNES
 *******************************************************************************/

// Delays comunes optimizados como macros
#define DELAY_1MS()     __delay_ms(1)
#define DELAY_5MS()     __delay_ms(5)
#define DELAY_10MS()    __delay_ms(10)
#define DELAY_50MS()    __delay_ms(50)
#define DELAY_100MS()   __delay_ms(100)
#define DELAY_500MS()   __delay_ms(500)
#define DELAY_1S()      __delay_ms(1000)

#define DELAY_1US()     __delay_us(1)
#define DELAY_5US()     __delay_us(5)
#define DELAY_10US()    __delay_us(10)
#define DELAY_50US()    __delay_us(50)
#define DELAY_100US()   __delay_us(100)

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES (INCLUIDA EN HEADER PARA OPTIMIZACIÓN)
 *******************************************************************************/

/**
 * @brief Implementación de delay variable en milisegundos
 */
inline void delay_ms_variable(uint16_t ms) {
    while(ms > 0) {
        if(ms >= 1000) {
            __delay_ms(1000);
            ms -= 1000;
        }
        else if(ms >= 100) {
            __delay_ms(100);
            ms -= 100;
        }
        else if(ms >= 10) {
            __delay_ms(10);
            ms -= 10;
        }
        else {
            __delay_ms(1);
            ms--;
        }
    }
}

/**
 * @brief Implementación de delay variable en microsegundos
 */
inline void delay_us_variable(uint16_t us) {
    while(us > 0) {
        if(us >= 1000) {
            __delay_us(1000);
            us -= 1000;
        }
        else if(us >= 100) {
            __delay_us(100);
            us -= 100;
        }
        else if(us >= 10) {
            __delay_us(10);
            us -= 10;
        }
        else {
            __delay_us(1);
            us--;
        }
    }
}

/**
 * @brief Implementación de delay en ciclos
 */
inline void delay_cycles(uint16_t cycles) {
    // Cada NOP toma 1 ciclo de instrucción
    while(cycles > 0) {
        __asm("NOP");
        cycles--;
    }
}

#endif /* DELAYS_H */