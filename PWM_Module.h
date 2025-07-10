#ifndef PWM_MODULE_H
#define PWM_MODULE_H

#include "config.h"

void PWM_Init(void);
void PWM_Set_Duty_Left(uint8_t dutyCycle);
void PWM_Set_Duty_Right(uint8_t dutyCycle);

#endif /* PWM_MODULE_H */