#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "config.h"

/**
 * @brief Actualiza el estado y la velocidad de los motores basado en un comando.
 * @param comando_usuario La dirección deseada (S_FORWARD, S_STOP, etc.).
 * @param velocidad_objetivo La velocidad deseada (0-255).
 */
void SpeedControl_ActualizarSistema(motor_state_t comando_usuario, uint8_t velocidad_objetivo);

#endif /* SPEED_CONTROL_H */