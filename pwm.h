#ifndef PWM_H_
#define PWM_H_
#include <xc.h> 

// -----------------------------------------------------
// Rotina de inicializa��o do m�dulo PWM
// -----------------------------------------------------
void inicializa_PWM(void){
   TRISCbits.TRISC2 = 0;    // Usando o m�dulo CCP1 (pino C2)
   T2CON = 0x07;            // Timer2 ON, Prescaler = 16 (p. 137 do datasheet)
   PR2 = 0xFF;              // Registrador que cont�m o per�odo do Timer2 (p. 137 do datasheet)
   CCP1CON = 0x0C;          // PWM mode active high com uma sa�da (P1A - Pino C2) (p. 151 do datasheet)
   CCPR1L = 0x00;           // Duty Cycle (junto com os bits 4 e 5 de CCP1CON para formar 10 bits)
}


// -----------------------------------------------------
// Rotina de configura��o do Periodo do PWM
// -----------------------------------------------------
// Para c�lculo do per�odo do PWM (p. 153 do datasheet):
// Periodo = [(PR2) + 1] * 4 * Tosc * (Timer2 Prescaler)
// -----------------------------------------------------
// Quando TMR2 � igual � PR2 acontece os seguinte eventos
// TMR2 � limpo
// O pino CCP1 � setado, somente se duty cycle != 0
// O duty cycle do PWM � copiado para CCPR1H
// -----------------------------------------------------
void periodo_PWM(unsigned char valor){
   T2CON = 0x07;         // Timer2 ON, Prescaler = 16 (p. 137 do datasheet)
   PR2 = valor;          // Registrador que cont�m o per�odo do Timer2 (p. 137 do datasheet)
}


// -----------------------------------------------------
// Rotina de configura��o de Duty Cycle do PWM
// -----------------------------------------------------
// Para c�lculo do Duty Cycle do PWM (p.154 do datasheet):
// DutyCycle = (CCPR1L:CCP1CON) * TOSC * (TMR2 Prescaler)
// -----------------------------------------------------
// O Valor s� ser� copiado para CCPR1H quando TMR2 = PR2
// -----------------------------------------------------
void dutyCycle_PWM(unsigned int valor){
   CCP1CONbits.DC1B0 = valor;
   CCP1CONbits.DC1B1 = valor >> 1;
   CCPR1L = valor >> 2;
}
#endif


