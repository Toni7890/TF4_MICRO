#ifndef L298N_DUAL_H
#define L298N_DUAL_H

#include "config.h"

// ... (Definiciones de pines se mantienen igual) ...
// Pines Motor A (Izquierdo)
#define IN1_TRIS TRISAbits.TRISA1
#define IN2_TRIS TRISAbits.TRISA2
#define IN1_LAT  LATAbits.LATA1
#define IN2_LAT  LATAbits.LATA2

// Pines Motor B (Derecho)
#define IN3_TRIS TRISEbits.TRISE1
#define IN4_TRIS TRISEbits.TRISE2
#define IN3_LAT  LATEbits.LATE1
#define IN4_LAT  LATEbits.LATE2


// Prototipos
void L298N_Dual_Init(void);
void L298N_Dual_Set_Direction(motor_state_t direccion);
void L298N_Dual_Stop(void);
// CORRECCIÓN: Se añade el prototipo de la función de rampa
void L298N_Dual_Set_Speed_Ramped(uint8_t target_speed_left, uint8_t target_speed_right);

#endif