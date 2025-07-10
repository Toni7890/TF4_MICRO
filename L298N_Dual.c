#include "L298N_Dual.h"
#include "PWM_Module.h"

static uint8_t current_speed_left = 0;
static uint8_t current_speed_right = 0;

void L298N_Dual_Init(void) {
    IN1_TRIS = 0; IN2_TRIS = 0;
    IN3_TRIS = 0; IN4_TRIS = 0;
    PWM_Init();
    L298N_Dual_Stop();
}

// CORRECCIÓN: Se cambia el nombre de la función para que coincida con el .h y el main
void L298N_Dual_Set_Speed_Ramped(uint8_t target_speed_left, uint8_t target_speed_right) {
    // Rampa suave para motor izquierdo
    if (current_speed_left < target_speed_left) current_speed_left+=5;
    else if (current_speed_left > target_speed_left) current_speed_left-=5;
    
    // Rampa suave para motor derecho
    if (current_speed_right < target_speed_right) current_speed_right+=5;
    else if (current_speed_right > target_speed_right) current_speed_right-=5;
    
    PWM_Set_Duty_Left(current_speed_left);
    PWM_Set_Duty_Right(current_speed_right);
}

void L298N_Dual_Set_Direction(motor_state_t direccion) {
    switch (direccion) {
        case S_FORWARD:  IN1_LAT = 1; IN2_LAT = 0; IN3_LAT = 1; IN4_LAT = 0; break;
        case S_BACKWARD: IN1_LAT = 0; IN2_LAT = 1; IN3_LAT = 0; IN4_LAT = 1; break;
        case S_LEFT:     IN1_LAT = 0; IN2_LAT = 1; IN3_LAT = 1; IN4_LAT = 0; break;
        case S_RIGHT:    IN1_LAT = 1; IN2_LAT = 0; IN3_LAT = 0; IN4_LAT = 1; break;
        case S_STOP:     default: IN1_LAT = 0; IN2_LAT = 0; IN3_LAT = 0; IN4_LAT = 0; break;
    }
}

void L298N_Dual_Stop(void) {
    L298N_Dual_Set_Direction(S_STOP);
    current_speed_left = 0;
    current_speed_right = 0;
    PWM_Set_Duty_Left(0);
    PWM_Set_Duty_Right(0);
}