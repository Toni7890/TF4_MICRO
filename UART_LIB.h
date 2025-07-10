#ifndef UART_LIB_H
#define UART_LIB_H

#include "config.h"

void UART1_Init(uint32_t baudrate);
void UART1_Println(const char* str);
void UART1_Print(const char* str);
void UART1_Send(uint8_t data);
uint8_t UART1_Receive(void);
bool UART1_IsTxReady(void); // Más robusto que IsTxDone
bool UART1_IsRxReady(void);

#endif /* UART_LIB_H */