/*
 * File:   maniLab9.c
 * Author: Amilcar Rodriguez
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// LIBRERIAS
#include <xc.h>
#include <stdio.h>

/*
 * Variables globales
 */
uint8_t potValue;
int cont = 0;
uint8_t botonPrevState;

/*
 * Constantes
 */
#define _XTAL_FREQ 4000000
#define dirEEPROM 0x04

/*
 * Prototipos de funcion
 */
void setup(void);
void writeToEEPROM(uint8_t data, uint8_t address);
uint8_t readFromEEPROM(uint8_t address);

/*
 * Interrupciones
 */
void __interrupt() isr(void)
{
    if(INTCONbits.RBIF)
    {
        PORTB = PORTB;
        INTCONbits.RBIF = 0;
    }
}

void main(void) {
    setup();
    ADCON0bits.GO = 1;
    while (1) {
        if (ADCON0bits.GO == 0){
            potValue = ADRESH;
            PORTD = potValue;
            __delay_us(50);
            ADCON0bits.GO = 1;
        }
        
        PORTC = readFromEEPROM(dirEEPROM);
        
        if (RB0 == 0)
            botonPrevState = 1;
        
        if (RB0 == 1 && botonPrevState == 1){
            writeToEEPROM(potValue, dirEEPROM);
            botonPrevState = 0;
        }
        
        if (RB1 == 1){
            PORTA = 0b00000000;
            INTCONbits.RBIF = 0;
            SLEEP();   
        }
        
        if (RB2 == 0){
            PORTA = 0b00000001;
        }
    }
}

void setup(void){
    // CONFIGURACION DE ENTRADAS Y SALIDAS
    // Pines digitales
    ANSEL = 0b00100000;
    ANSELH = 0x00;
    
    TRISA = 0x00; 
//    TRISB = 0x03; // set RB0 & RB1 como input 
    TRISB = 0b00000111;
    TRISC = 0x00;
    TRISD = 0x00;
    TRISE = 0x01; // set RE0 como input (pot)
    
    PORTA = 0x01;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;
    
    // Configuracion de pull ups
    OPTION_REGbits.nRBPU = 0;   // enable individual pull ups
    WPUBbits.WPUB0 = 1;  // enable Pull up de RB0 & RB1
    WPUBbits.WPUB1 = 1;
    
    //Configuracion de las interrupciones (SIN GIE)
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB2 = 1;
    
    // Configuracion oscilador interno
    OSCCONbits.IRCF = 0b110; // 1MHz
    OSCCONbits.SCS = 1;
    
    // Configuracion ADC
    ADCON1bits.ADFM = 0; //Justificado a la izquierda
    ADCON1bits.VCFG0 = 0; //Voltaje de ref. a GND
    ADCON1bits.VCFG1 = 0; //Voltaje de ref. a 5v
    
    ADCON0bits.ADCS = 1;    // ADC clock FOSC/8
    ADCON0bits.CHS = 5;     // Canal 05
    __delay_us(100);
    ADCON0bits.ADON = 1;    // Encender el modulo
}

/*
 * Funciones
 */

void writeToEEPROM(uint8_t data, uint8_t address){
    EEADR = address;
    EEDAT = data;
    
    EECON1bits.EEPGD = 0;   // Escribir a memoria de datos
    EECON1bits.WREN = 1;    // Habilitar escritura a EEPROM (datos)
    
    INTCONbits.GIE = 0;     // Deshabilitar interrupciones
    
    EECON2 = 0x55;          // Secuencia obligatoria
    EECON2 = 0xAA;
    EECON1bits.WR = 1;      // Habilitar escritura
    
    //INTCONbits.GIE = 1;     // Habilitar interrupciones
    EECON1bits.WREN = 0;    // Deshabilitar escritura de EEPROM
}

uint8_t readFromEEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0; // Escribir a memoria de datos
    EECON1bits.RD = 1; //leemos
    return EEDAT; //byte de datos leído
}