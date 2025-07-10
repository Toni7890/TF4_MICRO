#include "config.h"
#include "newmain7.h" // Incluye su propia cabecera
#include "UART_LIB.h"
#include "L298N_Dual.h"
#include "PWM_Module.h" // Asegúrate de incluirlo
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Definición de la variable global (sin 'extern').
SillaRuedas silla;

void main(void) {
    SYSTEM_Initialize();
    UART1_Init(9600);
    L298N_Dual_Init();
    
    __delay_ms(100);
    
    UART1_Println("============================================");
    UART1_Println("    Control Remoto con Rampa de Velocidad   ");
    UART1_Println("============================================");
    UART1_Println("Formato: <letra><velocidad>. (Ej: w200.)");
    UART1_Println("--------------------------------------------");

    char command_buffer[10];
    uint8_t buffer_index = 0;
    motor_state_t target_direction = S_STOP;
    uint8_t target_speed = 0;

    while (1) {
        if (UART1_IsRxReady()) {
            char received_char = UART1_Receive();
            
            if (received_char != '.' && buffer_index < (sizeof(command_buffer) - 1)) {
                command_buffer[buffer_index++] = received_char;
            } else {
                command_buffer[buffer_index] = '\0';
                
                if (buffer_index > 0) {
                    char command_action = command_buffer[0];
                    uint16_t speed_value = (uint16_t)atoi(&command_buffer[1]);

                    if (speed_value > 255) speed_value = 255;
                    
                    UART1_Print("-> Comando: ["); UART1_Send(command_action);
                    UART1_Print("] Velocidad: [");
                    char speed_str[4];
                    sprintf(speed_str, "%u", speed_value);
                    UART1_Println(speed_str);
                    
                    switch (command_action) {
                        case 'w': case 'W': target_direction = S_FORWARD; break;
                        case 's': case 'S': target_direction = S_BACKWARD; break;
                        case 'a': case 'A': target_direction = S_LEFT; break;
                        case 'd': case 'D': target_direction = S_RIGHT; break;
                        default:            target_direction = S_STOP; break;
                    }
                    
                    target_speed = (command_action == 'q' || command_action == 'Q') ? 0 : (uint8_t)speed_value;
                    L298N_Dual_Set_Direction(target_direction);
                }
                buffer_index = 0;
            }
        }
        
        // CORRECCIÓN: Llamar a la función de rampa
        L298N_Dual_Set_Speed_Ramped(target_speed, target_speed);
        
        LED_Toggle();
        __delay_ms(20);
    }
}