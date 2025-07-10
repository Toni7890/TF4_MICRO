#include "HC_SR04.h"
#include "config.h" // Se a�ade esta l�nea para que vea las macros

void HC_SR04_Init(void) {
    // La inicializaci�n de pines ya est� en config.c
}

uint16_t HC_SR04_GetDistance(void) {
    uint32_t time_us = 0;
    
    HC_SR04_TRIG_HIGH();
    __delay_us(10);
    HC_SR04_TRIG_LOW();
    
    uint16_t timeout = 0;
    while(!HC_SR04_ECHO_READ() && timeout < 50000) { timeout++; }

    while(HC_SR04_ECHO_READ() && timeout < 50000) {
        time_us++;
        __delay_us(1);
    }
    
    // F�rmula para cm: tiempo_us / 58.8
    return (uint16_t)(time_us / 58);
}