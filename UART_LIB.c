#include "UART_LIB.h"

void UART1_Init(uint32_t baudrate) {
    // 1. Deshabilitar UART para configurar de forma segura
    U1CON1bits.ON = 0;

    // 2. Configurar los pines RC6 (TX) y RC7 (RX)
    TRISCbits.TRISC6 = 0; // RC6 como salida (TX)
    TRISCbits.TRISC7 = 1; // RC7 como entrada (RX)
    ANSELCbits.ANSELC6 = 0; // Deshabilitar análogo en RC6
    ANSELCbits.ANSELC7 = 0; // Deshabilitar análogo en RC7

    // 3. Configurar PPS (Peripheral Pin Select)
    RC6PPS = 0x20;   // Mapear U1TX a RC6
    U1RXPPS = 0x17;  // Mapear U1RX a RC7

    // 4. Configurar el generador de Baud Rate
    // Fórmula para modo asíncrono, 16 clocks por bit (BRGS=0)
    uint16_t brg_value = (uint16_t)((_XTAL_FREQ / (16UL * baudrate)) - 1);
    U1BRGL = (uint8_t)brg_value;
    U1BRGH = (uint8_t)(brg_value >> 8);
    
    // 5. Configurar los registros de control de la UART
    U1CON0bits.MODE = 0b0000; // Modo asíncrono de 8 bits
    U1CON0bits.TXEN = 1;     // Habilitar transmisor
    U1CON0bits.RXEN = 1;     // Habilitar receptor
    
    U1CON2 = 0x00;           // Polaridad normal, sin control de flujo
    
    // 6. Habilitar la UART como último paso
    U1CON1bits.ON = 1;
}

void UART1_Println(const char* str) {
    UART1_Print(str);
    UART1_Send('\r');
    UART1_Send('\n');
}

void UART1_Print(const char* str) {
    while (*str) {
        UART1_Send(*str++);
    }
}

void UART1_Send(uint8_t data) {
    while (!U1FIFObits.TXBE); // Esperar a que el buffer de transmisión tenga espacio
    U1TXB = data;
}

uint8_t UART1_Receive(void) {
    while (!U1FIFObits.RXBE); // Esperar a que haya datos en el buffer de recepción
    return U1RXB;
}

bool UART1_IsTxReady(void) {
    // Verdadero si el buffer está vacío y el registro de desplazamiento ha terminado de transmitir
    return (U1FIFObits.TXBE && U1ERRIRbits.TXMTIF);
}

bool UART1_IsRxReady(void) {
    return U1FIFObits.RXBE ? false : true;
}