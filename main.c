// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <limits.h>	// Valor máximo de um u_char
#include <stdio.h>	// Função sprintf
#include "pwm.h"	// Módulo PWM
#include "nxlcd.h"	// LCD
#include "interrupts.h"

typedef unsigned int u_int;
typedef unsigned long u_long;

#define _XTAL_FREQ 20000000	//20MHz
#define PWM_MAX	0x3FF		// Valor máximo do DUTY CYCLE(10 bits) = 0011 1111 1111 = 1023_10
#define PWM_MIN	0x000		// Valor mínimo do DUTY CYCLE(10 bits) = 0000 0000 0000 = 0_10
#define CHAR_LCD 16		// Número de colunas do LCD
#define CLEAN_LCD 0x01		// Código para limpar o LCD e voltar o cursor
#define POS_1_1	0x80		// Primeira linha e coluna do LCD
#define POS_2_1	0xC0		// Segunda linha e primeira coluna do LCD
#define POS_1_5 0x84		// Primeira linha e quinta coluna do LCD
#define ON 1			// Monitoramento ligado
#define OFF 0			// Monitoramento desligado
#define PAS 9			// Número total de pás da ventoinha
#define TIME 0.001		// Tempo de cada estouro do TMR3 (1e-3 segundos)

void init_config(u_char, u_int);			// Configurações iniciais
void init_interrupts(void);				// Configurações das interrupções
void init_LCD(void);					// Configurações do LCD
void init_PWM(u_char, u_int);				// Configurações do módulo PWM
void config_INT1(void);					// Configurações da interrupção INT1
void config_INT2(void);					// Configurações da interrupção INT2
void __interrupt() ISHR(void);				// Rotina de tratamento das interrupções
void writeLCD(u_char, u_long);				// Escreve um u_int na segunda linha

boolean estado_atual = ON;	// Estado atual do monitoramento
u_long qtd_timer0 = 0;		// Quantidade de interrupções do TMR0
boolean volta_ventoinha = OFF;	// Se a ventoinha já completou uma volta
const u_char pos_LCD[2][16] = {
	{0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8F},
	{0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0x8C, 0xCD, 0xCF}
};				// Array com as posições do LCD

void main(void)
{
	
	u_long v_max = 70UL;				// Velocidade máxima
	u_long v_atual = 0UL;				// Velocidade atual
	u_char pwm_period = UCHAR_MAX;			// Período do módulo PWM
	u_int duty_cycle = PWM_MAX >> 1;		// Tamanho do duty cycle do PWM
	init_config(pwm_period, duty_cycle);
	
	while(1) 
	{	
		// Verifica se o monitoramento ligado
		if (estado_atual == ON) {	// Se sim calcula a velocidade e liga o LED "MON"
			if (!PORTDbits.RD0)	// Verifica se já não estava ligado anteriormente
			{
				PORTDbits.RD0 = 1;	// Liga o LED de monitoramento
				WriteCmdXLCD(CLEAN_LCD);
				writeLCD(pos_LCD[0][0], v_max);
				writeLCD(pos_LCD[1][0], v_atual);
			}
			
			// Pisca o LED V_CTRL case o limite for ultrapassado
			PORTDbits.RD2 = (v_atual > v_max) ? ~PORTDbits.RD2 : 0;
			
		} else { // Se não, desliga o LED
			if (PORTDbits.RD0)	// Verifica se já não estava desligado anteriormente
			{
				PORTDbits.RD0 = PORTDbits.RD2 = 0;	// Desliga o LED do monitoramento e o LED de controle de velocidade
				writeLCD(pos_LCD[0][4], 0);
			}
		}
		
		// Verifica se houve alteração na velocidade do motor
		if (!PORTBbits.RB3) {		// Diminui o duty cycle em 2x
			duty_cycle >>= 1;
			dutyCycle_PWM(duty_cycle);
			
		} else if (!PORTBbits.RB4) {	// Aumenta o duty cycle em 2x
			duty_cycle = ((duty_cycle << 1) | 1) > PWM_MAX ? PWM_MAX : ((duty_cycle << 1) | 1);
			dutyCycle_PWM(duty_cycle);
		}

		// Verifica se a ventoinha completou uma volta
		if (volta_ventoinha)
		{
			v_atual = (u_long)((60.0 / (qtd_timer0 * TIME) ) + 0.5); // RPM
			writeLCD(pos_LCD[1][0], v_atual);
			qtd_timer0 = 0;						// Zera contagem para a próxima medição
			volta_ventoinha = OFF;
		}
		
		__delay_ms(250);
	}
		
	return;
}

// -----------------------------------------------------
// Configurações iniciais
// -----------------------------------------------------
void init_config(u_char period, u_int duty_cycle)
{
	RBPU = 0;		// Resistor de pull-up
	TRISBbits.RB1 = 1;	// Interrupção INT1 (STOP)
	TRISBbits.RB2 = 1;	// Interrupção INT2 (START)
	TRISBbits.RB3 = 1;	// Entrada de redução da velocidade
	TRISBbits.RB4 = 1;	// Entrada de diminuição da velocidade
	TRISDbits.RD0 = 0;	// Saída do LED de monitoramento
	TRISDbits.RD2 = 0;	// Saída do LED de controle da velocidade
	PORTDbits.RD0 = OFF;	// Monitoramento desligado
	PORTDbits.RD2 = OFF;	// Alerta de velocidade desligado
	init_interrupts();
	init_LCD();
	init_PWM(period, duty_cycle);
}

// -----------------------------------------------------
// Configurações iniciais das interrupções usadas
// -----------------------------------------------------
void init_interrupts(void)
{
	enable_interrupts();
	enable_INT1(ENABLE, DISABLE); // STOP
	enable_INT2(ENABLE, DISABLE); // START
	enable_TMR3(ENABLE, DISABLE, DISABLE, DISABLE, DISABLE, DISABLE, T3CKPS_1, 0xEC78); // Vai temporizar 0.001 segundos;
	enable_TMR1(ENABLE, DISABLE, DISABLE, T1CKPS_1, DISABLE, ENABLE, ENABLE, 0xFFF7); // Vai contar 9
}

// -----------------------------------------------------
// Rotina de tratamento das interrupções
// -----------------------------------------------------
void __interrupt() ISHR(void)
{
	if (INTCON3bits.INT1IF)  // Interrupção INT1 (STOP)
	{
		estado_atual = OFF;		// Desliga o monitoramento
		INTCON3bits.INT1IF = 0;		// Limpa o bit flag da interrupção INT2
	}
	
	if (INTCON3bits.INT2IF) // Interrupção INT2 (START)
	{
		estado_atual = ON;		// Liga o monitoramento
		INTCON3bits.INT2IF = 0;		// Limpa o bit flag da interrupção INT2
	}
	
	if(PIR2bits.TMR3IF) // Interrução TMR3
	{
		++qtd_timer0;		// Incrementa a quantidade de estouro do TMR3
		TMR3 = 0xEC78;		// Recarrega a carga inicial
		PIR2bits.TMR3IF = 0;	// Limpa o bit flag da interrupção TMR3
	}
	
	if (PIR1bits.TMR1IF) // Interrupção TMR1
	{
		volta_ventoinha = ON;		// Completou uma volta
		TMR1 = 0xFFF7;			// Recarrega a carga inicial
		PIR1bits.TMR1IF = 0;		// Limpa o bit flag da interrupção TMR1
	}
}

// -----------------------------------------------------
// Inicia o LCD
// -----------------------------------------------------
void init_LCD()
{
	OpenXLCD(FOUR_BIT & LINES_5X7);		// Modo 4 bits de dados e caracteres 5x7
}

// -----------------------------------------------------
// Inicia o módulo PWM
// -----------------------------------------------------
void init_PWM(u_char period, u_int duty_cycle)
{	
	inicializa_PWM();
	periodo_PWM(period);
	dutyCycle_PWM(duty_cycle);
}

// -----------------------------------------------------
// Escreve o valor no LCD com a devida mensagem
// -----------------------------------------------------
void writeLCD(u_char pos, u_long value)
{
	char str[CHAR_LCD];	// Mensagem para escrever no LCD
	switch(pos)
	{
		case POS_1_1:
			sprintf(str, "V MAX= %4lu RPM", value);
			break;
		case POS_2_1:
			sprintf(str, "V ATL= %4lu RPM", value);
			break;
		case POS_1_5:
			WriteCmdXLCD(CLEAN_LCD);
			sprintf(str, "DESLIGADO");
			break;
		default:
			WriteCmdXLCD(CLEAN_LCD);
			sprintf(str, "ERROR!");
	}
	WriteCmdXLCD(pos);
	__delay_ms(15);
	putsXLCD(str);
}
