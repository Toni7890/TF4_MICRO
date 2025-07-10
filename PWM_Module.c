#include "PWM_Module.h"

void PWM_Init(void) {
    // Configurar pines ENA y ENB como salidas PWM
    TRISCbits.TRISC2 = 0; // ENA (CCP1)
    TRISEbits.TRISE0 = 0; // ENB (CCP2)

    // Mapear hardware PWM a los pines físicos usando PPS
    RC2PPS = 0x0F; // CCP1 -> RC2 (Motor Izquierdo)
    RE0PPS = 0x10; // CCP2 -> RE0 (Motor Derecho)

    // Configurar Timer2 como la base de tiempo para ambos PWM
    T2PR = 255;          // Período máximo para resolución de 8 bits
    T2CONbits.CKPS = 0b010; // Prescaler 1:4 -> Frec. PWM de ~15.6 kHz
    T2CONbits.ON = 1;    // Encender Timer2

    // Configurar CCP1 para el Motor Izquierdo
    CCP1CONbits.MODE = 0b1100;  // Modo PWM
    CCPTMRS0bits.C1TSEL = 0b01; // CCP1 usa Timer2
    CCPR1L = 0;                 // Velocidad inicial 0

    // Configurar CCP2 para el Motor Derecho
    CCP2CONbits.MODE = 0b1100;  // Modo PWM
    CCPTMRS0bits.C2TSEL = 0b01; // CCP2 usa Timer2
    CCPR2L = 0;                 // Velocidad inicial 0
}

void PWM_Set_Duty_Left(uint8_t dutyCycle) {
    CCPR1L = dutyCycle;
}

void PWM_Set_Duty_Right(uint8_t dutyCycle) {
    CCPR2L = dutyCycle;
}