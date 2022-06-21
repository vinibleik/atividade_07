#ifndef INTERRUPTS_H
#define	INTERRUPTS_H

#include <xc.h>

typedef unsigned char u_char;
typedef unsigned short u_short;
typedef _Bool boolean;

#define ENABLE 1
#define DISABLE 0

// -----------------------------------------------------
// Prescalers do TMR0 (p. 127 do datasheet)
// -----------------------------------------------------
#define T0PS_256 0b111  // prescaler 256
#define T0PS_128 0b110  // prescaler 128
#define T0PS_64  0b101  // prescaler 64
#define T0PS_32  0b100  // prescaler 32
#define T0PS_16  0b011  // prescaler 16
#define T0PS_8   0b010  // prescaler 8
#define T0PS_4   0b001  // prescaler 4
#define T0PS_2   0b000  // prescaler 2

// -----------------------------------------------------
// Prescalers do TMR1 (p. 131 do datasheet)
// -----------------------------------------------------
#define T1CKPS_8 0b11   
#define T1CKPS_4 0b10
#define T1CKPS_2 0b01
#define T1CKPS_1 0b00

// -----------------------------------------------------
// Prescalers do TMR3 (p. 139 do datasheet)
// -----------------------------------------------------
#define T3CKPS_8 0b11   
#define T3CKPS_4 0b10
#define T3CKPS_2 0b01
#define T3CKPS_1 0b00


// -----------------------------------------------------
// Habilita interrupções globais (p. 101 do datasheet)
// -----------------------------------------------------
void enable_interrupts()
{
    RCONbits.IPEN = ENABLE;         // Habilita interrupções com prioridade
	INTCONbits.GIEH = ENABLE;		// Habilita todas interrupções de alta prioridade
	INTCONbits.GIEL = ENABLE;		// Habilita todas interrupções de baixa prioridade
}

// Definindo as INTn pin interrupts
#ifndef INT_INTERRUPTS
#define INT_INTERRUPTS

// -----------------------------------------------------
// Habilita a interrupção INT1 (p. 102 e 103 do datasheet)
// -----------------------------------------------------
void enable_INT1(boolean priority, boolean edge)
{
    INTCON3bits.INT1IE = ENABLE;         // Habilita a interrupção externa INT1
	INTCON3bits.INT1IP = priority;       // Habilita a interrupção externa INT1 como alta prioridade
	INTCON2bits.INTEDG1 = edge;          // Interrupção externa 1 na borda de descida
	INTCON3bits.INT1IF = DISABLE;        // Limpa o bit flag da interrupção INT1
}

// -----------------------------------------------------
// Desabilita a interrupção INT1 (p. 102 e 103 do datasheet)
// -----------------------------------------------------
void disable_INT1()
{
    INTCON3bits.INT1E = DISABLE;         // Desabilita a interrupção INT1 
}

// -----------------------------------------------------
// Habilita a interrupção INT2 (p. 102 e 103 do datasheet)
// -----------------------------------------------------
void enable_INT2(boolean priority, boolean edge)
{
    INTCON3bits.INT2IE = ENABLE;         // Habilita a interrupção externa INT2
	INTCON3bits.INT2IP = priority;		 // Habilita a interrupção externa INT2 como alta prioridade
	INTCON2bits.INTEDG2 = edge;          // Interrupção externa 2 na borda de descida
	INTCON3bits.INT2IF = DISABLE;        // Limpa o bit flag da interrupção INT2
}

// -----------------------------------------------------
// Desabilita a interrupção INT2 (p. 102 e 103 do datasheet)
// -----------------------------------------------------
void disable_INT2()
{
    INTCON3bits.INT2E = DISABLE;         // Desabilita a interrupção INT2 
}

#endif /* INT_INTERRUPTS */

// Definindo as TMRn interrupts
#ifndef TMR_INTERRUPTS
#define TMR_INTERRUPTS

/*
 * Para cálculo do tempo dos timers usar a seguinte fórmula
 * 
 *  T = T_cf * prescaler * (2^n - carga_inicial)
 *  
 *  Onde:
 *  T = tempo que o timer vai contar em segundos
 *  T_cf = 1 / f_cf 
 *  f_fc = f_cristal / 4
 *  n = bit mode do timer (se n=8 -> 256, se n=16 -> 65536)
 * 
 */

// -----------------------------------------------------
// Habilita a interrupção TMR0 (p. 127 e 129 do datasheet)
// -----------------------------------------------------
void enable_TMR0(boolean priority, boolean bit_set, boolean tosc, boolean tose,
        boolean psa, u_char pre_scaler, u_short value)
{
    INTCONbits.TMR0IE = ENABLE;          // Habilita a interrupção por overflow TMR0
    INTCON2bits.TMR0IP = priority;       // Prioridade da interrupção do TMR0
    T0CONbits.TMR0ON = ENABLE;           // Liga o timer 0
    T0CONbits.T08BIT = bit_set;          // Modo de operação de bits
    T0CONbits.T0CS = tosc;               // Clock source
    T0CONbits.T0SE = tose;               // Edge select bit
    T0CONbits.PSA = psa;                 // Habilita o prescaler
    T0CONbits.T0PS = pre_scaler;         // Seta o prescaler
    TMR0 = value;                        // Carga inicial
    INTCONbits.TMR0IF = DISABLE;         // Limpa o bit flag da interrupção TMR0
}

// -----------------------------------------------------
// Desabilita a interrupção TMR0 (p. 127 do datasheet)
// -----------------------------------------------------
void disable_TMR0()
{
    INTCONbits.TMR0IE = DISABLE;          // Desabilita a interrupção por overflow TMR0
    T0CONbits.TMR0ON = DISABLE;           // Desliga o timer 0
}

// -----------------------------------------------------
// Habilita a interrupção TMR1 (p. 131 e 135 do datasheet)
// -----------------------------------------------------
void enable_TMR1(boolean priority, boolean rd16, boolean run, u_char pre_scaler,
        boolean oscilator, boolean sync, boolean clock_source, u_short value)
{
    PIE1bits.TMR1IE = ENABLE;             // Habilita a interrupção por overflow do TMR1
    IPR1bits.TMR1IP = priority;           // Prioridade da interrupção TMR1
    PIR1bits.TMR1IF = DISABLE;            // Limpa o bit flag da interrupção TMR1
    T1CONbits.RD16 = rd16;                // 16-bit mode
    T1CONbits.T1RUN = run;                // System clock status bit
    T1CONbits.T1CKPS = pre_scaler;        // Prescaler do input clock
    T1CONbits.T1OSCEN = oscilator;        // Oscilator bit
    T1CONbits.T1SYNC = sync;              // Sincronização do clock externo
    T1CONbits.TMR1CS = clock_source;      // Fonte do clock
    TMR1 = value;                         // Carga inical
    T1CONbits.TMR1ON = ENABLE;            // Habilita o TMR1
}

// -----------------------------------------------------
// Desabilita a interrupção TMR1 (p. 131 e 106 do datasheet)
// -----------------------------------------------------
void disable_TMR1()
{
    PIE1bits.TMR1IE = DISABLE;          // Desabilita a interrupção TMR1
    T1CONbits.TMR1ON = DISABLE;         // Desliga o TMR1
}

// -----------------------------------------------------
// Habilita a interrupção TMR1 (p. 139 e 141 do datasheet)
// -----------------------------------------------------
void enable_TMR3(boolean priority, boolean rd16, boolean ccp1, boolean ccp2, boolean clock_source, 
        boolean sync, u_char pre_scaler, u_short value)
{
    PIE2bits.TMR3IE = ENABLE;           // Habilita a interrupção por overflow do TMR3
    IPR2bits.TMR3IP = priority;         // Prioridade da interrupção TMR3
    T3CONbits.RD16 = rd16;              // 16-bit mode
    T3CONbits.T3CCP1 = ccp1;                // CCPx do Timer 3 e Timer 1
    T3CONbits.T3CCP2 = ccp2;                // CCPx do Timer 3 e Timer 1
    T3CONbits.TMR3CS = clock_source;    // Fonte do clock
    T3CONbits.T3SYNC = sync;            // Sincronização do clock externo
    T3CONbits.T3CKPS = pre_scaler;      // Prescaler do input clock
    TMR3 = value;                       // Carga inical
    T3CONbits.TMR3ON = ENABLE;          // Habilita o TMR3
    
}

// -----------------------------------------------------
// Desabilita a interrupção TMR1 (p. 131 e 106 do datasheet)
// -----------------------------------------------------
void disable_TMR3()
{
    PIE2bits.TMR3IE = DISABLE;      // Desabilita a interrupção TMR3
    T3CONbits.TMR3ON = DISABLE;     // Desliga o TMR3
}

#endif /* TMR_INTERRUPTS */

#endif	/* INTERRUPTS_H */

