#include "SPEED_CONTROL.h"
#include "L298N_Dual.h"
#include "config.h" 
#include "newmain7.h" // <--- CORRECCIÓN IMPORTANTE

// Ya no se necesita 'extern SillaRuedas silla;' porque está en newmain7.h

void SpeedControl_ActualizarSistema(motor_state_t comando_usuario, uint8_t velocidad_objetivo) {
    
    L298N_Dual_Set_Direction(comando_usuario);
    L298N_Dual_Set_Speed(velocidad_objetivo, velocidad_objetivo);
}