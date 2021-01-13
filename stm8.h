#ifndef STM8_H
#define STM8_H

// CLOCK
#define CLK_CKDIVR  (*(unsigned char *)0x50C6) // CLK SPEED DIVIDER

// PORT B
#define PB_ODR      (*(unsigned char *)0x5005) // PORT B OUTPUT DATA REGISTER
#define PB_IDR      (*(unsigned char *)0x5006) // PORT B INPUT DATA REGISTER
#define PB_DDR      (*(unsigned char *)0x5007) // PORT B DATA DIRECTION
#define PB_CR1      (*(unsigned char *)0x5008) // PORT B CONTROL REGISTER 1
#define PB_CR2      (*(unsigned char *)0x5009) // PORT B CONTROL REGISTER 2
#define PORTB_SET(pin, value) ( value ? (PB_ODR |= (1<<pin)) : (PB_ODR &= (~(1<<pin))) )
#define PORTB_READ(pin) ( PB_IDR & (1<<pin) )
#define PORTB_TOGGLE(pin) (PB_ODR ^= 1<<pin)
#define PORTB_PINMODE(pin, mode) ( mode ? (PB_DDR |= (1<<pin)) : (PB_DDR &= (~(1<<pin))) )

// PORT A
#define PA_ODR      (*(unsigned char *)0x5000) // PORT A OUTPUT DATA REGISTER
#define PA_IDR      (*(unsigned char *)0x5001) // PORT A INPUT DATA REGISTER
#define PA_DDR      (*(unsigned char *)0x5002) // PORT A DATA DIRECTION
#define PA_CR1      (*(unsigned char *)0x5003) // PORT A CONTROL REGISTER 1
#define PORTA_SET(pin, value) ( value ? (PA_ODR |= (1<<pin)) : (PA_ODR &= (~(1<<pin))) ) 
#define PORTA_READ(pin) ( PA_IDR & (1<<pin) )
#define PORTA_TOGGLE(pin) (PA_ODR ^= 1<<pin)
#define PORTA_PINMODE(pin, mode) ( mode ? (PA_DDR |= (1<<pin)) : (PA_DDR &= (~(1<<pin))) )

// PORT C
#define PC_ODR      (*(unsigned char *)0x500A) // PORT C OUTPUT DATA REGISTER
#define PC_IDR      (*(unsigned char *)0x500B) // PORT C INPUT DATA REGISTER
#define PC_DDR      (*(unsigned char *)0x500C) // PORT C DATA DIRECTION
#define PC_CR1      (*(unsigned char *)0x500D) // PORT C CONTROL REGISTER 1
#define PORTC_SET(pin, value) ( value ? (PC_ODR |= (1<<pin)) : (PC_ODR &= (~(1<<pin))) )
#define PORTC_READ(pin) ( PC_IDR & (1<<pin) )
#define PORTC_TOGGLE(pin) (PC_ODR ^= 1<<pin)
#define PORTC_PINMODE(pin, mode) ( mode ? (PC_DDR |= (1<<pin)) : (PC_DDR &= (~(1<<pin))) )

// PORT D
#define PD_ODR      (*(unsigned char *)0x500F) // PORT D OUTPUT DATA REGISTER
#define PD_IDR      (*(unsigned char *)0x5010) // PORT D INPUT DATA REGISTER
#define PD_DDR      (*(unsigned char *)0x5011) // PORT D DATA DIRECTION
#define PD_CR1      (*(unsigned char *)0x5012) // PORT D CONTROL REGISTER 1
#define PORTD_SET(pin, value) ( value ? (PD_ODR |= (1<<pin)) : (PD_ODR &= (~(1<<pin))) )
#define PORTD_READ(pin) ( PD_IDR & (1<<pin) )
#define PORTD_TOGGLE(pin) (PD_ODR ^= 1<<pin)
#define PORTD_PINMODE(pin, mode) ( mode ? (PD_DDR |= (1<<pin)) : (PD_DDR &= (~(1<<pin))) )

// GPIO
#define ALL_OFF 0x00


// TIMER 2

#define TIM2_CR1    (*(unsigned char *)0x5300) // TIMER 2 control register
#define TIM2_IER    (*(unsigned char *)0x5303) // TIMER 2 Interrupt enable register
#define TIM2_SR1    (*(unsigned char *)0x5304) // TIMER 2 status register 1
#define TIM2_SR2    (*(unsigned char *)0x5305) // TIMER 2 status register 2
#define TIM2_PSCR   (*(unsigned char *)0x530E) // TIMER 2 prescaler register

// TIMER 4

#define TIM4_CR1    (*(unsigned char *)0x5340) // TIMER 4 control register
#define TIM4_IER    (*(unsigned char *)0x5343) // TIMER 4 Interrupt enable register
#define TIM4_SR     (*(unsigned char *)0x5344) // TIMER 4 status register 1
#define TIM4_EGR    (*(unsigned char *)0x5345) // TIMER 4 event registration register
#define TIM4_CNTR   (*(unsigned char *)0x5346) // TIMER 4 counter
#define TIM4_PSCR   (*(unsigned char *)0x5347) // TIMER 4 prescaler register
#define TIM4_ARR    (*(unsigned char *)0x5348) // TIMER 4 auto-reload register 0xFF

/*

Interrupts are defined at stm8s_it.h

Interrupt commands 
#define enableInterrupts()    {__asm__("rim\n");}   enable interrupts 
#define disableInterrupts()   {__asm__("sim\n");}   disable interrupts 
#define rim()                 {__asm__("rim\n");}   enable interrupts 
#define sim()                 {__asm__("sim\n");}   disable interrupts 
#define nop()                 {__asm__("nop\n");}   No Operation 
#define trap()                {__asm__("trap\n");}  Trap (soft IT) 
#define wfi()                 {__asm__("wfi\n");}   Wait For Interrupt 
#define halt()                {__asm__("halt\n");}  Halt 

Interrupts:
0 TLI
1 AWU Auto Wake up from Halt
2 CLK Clock controller
3 EXTI0 Port A external interrupts
4 EXTI1 Port B external interrupts
5 EXTI2 Port C external interrupts
6 EXTI3 Port D external interrupts
7 EXTI4 Port E external interrupts
8 CAN CAN RX interrupt
9 CAN CAN TX/ER/SC interrupt
10 SPI End of Transfer
11 TIM1 Update /Overflow/Underflow/Trigger/Break
12 TIM1 Capture/Compare
13 TIM2 Update /Overflow
14 TIM2 Capture/Compare
15 TIM3 Update /Overflow
16 TIM3 Capture/Compare
17 UART1 Tx complete
18 UART1 Receive Register DATA FULL
19 I2C I2C interrupt
20 UART2/3 Tx complete
21 UART2/3 Receive Register DATA FULL
22 ADC End of Conversion
23 TIM4 Update/Overflow
24 FLASH EOP/WR_PG_DIS

TLI 0
AWU 1
CLK 2
EXTI_PORTA 3
EXTI_PORTB 4
EXTI_PORTC
EXTI_PORTD
EXTI_PORTE
CAN_RX
CAN_TX
SPI
TIM1_UPD_OVF_TRG_BRK
TIM1_CAP_COM
TIM2_UPD_OVF_BRK
TIM2_CAP_COM
TIM3_UPD_OVF_BRK
TIM3_CAP_COM
UART1_TX
UART1_RX
I2C 19
ADC1 22
TIM4_UPD_OVF 23
EEPROM_EEC 24
*/


#endif // STM8_H
