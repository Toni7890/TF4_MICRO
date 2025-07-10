#include "config.h"

// --- CONFIGURACIÓN DE BITS DEL MICROCONTROLADOR ---
#pragma config FEXTOSC = OFF, RSTOSC = HFINTOSC_64MHZ
#pragma config CLKOUTEN = OFF, PR1WAY = ON, CSWEN = ON, FCMEN = ON
#pragma config MCLRE = EXTMCLR, PWRTS = PWRT_OFF, MVECEN = ON, IVT1WAY = ON, LPBOREN = OFF, BOREN = ON
#pragma config BORV = VBOR_1P9, ZCD = OFF, PPS1WAY = ON, STVREN = ON, LVP = ON, XINST = OFF
#pragma config WDTCPS = WDTCPS_31, WDTE = OFF
#pragma config WDTCWS = WDTCWS_7, WDTCCS = SC
#pragma config BBSIZE = BBSIZE_512, BBEN = OFF, SAFEN = OFF, DEBUG = OFF
#pragma config WRTB = OFF, WRTC = OFF, WRTD = OFF, WRTSAF = OFF, WRTAPP = OFF
#pragma config CP = OFF

void ClockInit(void) {
    OSCCON1 = 0x60; // NOSC HFINTOSC; NDIV 1;
    OSCFRQ = 0x08;  // HFFRQ 64_MHz;
}

void PinInit(void) {
    // Habilitar pull-ups débiles
    WPUA = 0xFF; WPUB = 0xFF; WPUC = 0xFF; WPUD = 0xFF; WPUE = 0xFF; WPUF = 0xFF;
    // Configurar todos los puertos como digitales
    ANSELA = 0x00; ANSELB = 0x00; ANSELC = 0x00; ANSELD = 0x00; ANSELE = 0x00; ANSELF = 0x00;
    
    // Configurar pines específicos
    LED_STATUS_TRIS = 0;
    HC_SR04_TRIG_TRIS = 0;
    HC_SR04_ECHO_TRIS = 1;
    
    LED_OFF();
    HC_SR04_TRIG_LAT = 0;
}

void SYSTEM_Initialize(void) {
    BORCONbits.SBOREN = 1; 
    ClockInit();
    PinInit();
}