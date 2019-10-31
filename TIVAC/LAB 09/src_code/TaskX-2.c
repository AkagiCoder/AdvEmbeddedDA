//*****************************************************************************
//
// pwm_teste.c - PWM example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_gpio.h"
#include "driverlib/pwm.h"

//#include <inc/tm4c123gh6pm.h>

//*****************************************************************************
//
//! \addtogroup pwm_examples_list
//! <h1>PWM Reload Interrupt (reload_interrupt)</h1>
//!
//! This example shows how to setup an interrupt on PWM0.  This example
//! demonstrates how to setup an interrupt on the PWM when the PWM timer is
//! equal to the configurable PWM0LOAD register.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - GPIO Port F peripheral (for PWM5 pin)
//! - PWM5 - PF1
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of the
//! PWM.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! PWM 1 Generator 2- PWM1_GEN2_IntHandler
//
//*****************************************************************************

#define RED_LED             GPIO_PIN_1
#define BLUE_LED            GPIO_PIN_2
#define GREEN_LED           GPIO_PIN_3
#define SW1                 GPIO_PIN_4    //PF4
#define SW2                 GPIO_PIN_0    //PF0
#define atraso_1000ms       50000000/3
#define atraso_500ms        50000000/6
#define atraso_100ms        50000000/30
#define atraso_200ms        50000000/15

struct  acionamentos
    {
    unsigned int led1:1;
    unsigned int led2:1;
    unsigned int led3:1;
    unsigned int chave1:1;
    unsigned int chave2:1;
    unsigned int inverter:1;
    } estado;

// ou struct acionamentos estado;


uint32_t var_temp, pwm_width;
uint8_t cont_leds = 0;
volatile uint32_t teste;


//*****************************************************************************
//
// Interrupção por borda de descida no PF4 (SW1) e PF0 (SW2)
//
//*****************************************************************************
void GPIOF_ISR(void){
    cont_leds++;
    var_temp = GPIOIntStatus(GPIO_PORTF_BASE,SW1|SW2);

    if (var_temp & SW1){
        if (cont_leds < 5){     // acende LED a cada 5 interrupcoes
            GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED);
        }
        else{
            GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, 0);
            cont_leds = 0;
            }
        }
    else if (var_temp & SW2){
        if (cont_leds < 5){     // acende LED a cada 5 interrupcoes
            GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, GREEN_LED);
        }
        else{
            GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, 0);
            cont_leds = 0;
            }
        }
    SysCtlDelay(atraso_200ms);
    GPIOIntClear(GPIO_PORTF_BASE, SW1|SW2);
}



//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif




//*****************************************************************************
//
// The interrupt handler for the for PWM1 interrupts.
//
//*****************************************************************************
void
PWM1_GEN2_IntHandler(void)
{
    //
    // Clear the PWM0 LOAD interrupt flag.  This flag gets set when the PWM
    // counter gets reloaded.
    //
    PWMGenIntClear(PWM1_BASE, PWM_GEN_2, PWM_INT_CNT_LOAD);

    //
    // If the duty cycle is less or equal to 75% then add 0.01% to the duty
    // cycle.  Else, reset the duty cycle to 0.1% cycles.  Note that 50 is
    // 0.01% of the period (50000 cycles).
    //

    if((PWMPulseWidthGet(PWM1_BASE, PWM_OUT_5) + 50) <=
       ((PWMGenPeriodGet(PWM1_BASE, PWM_GEN_2) * 3) / 4))
    {
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,
                         PWMPulseWidthGet(PWM1_BASE, PWM_OUT_5) + 50);
    }
    else
    {
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 50);
    }
}


//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    estado.led1 ^= 1;

    if (estado.led1)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    else
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

    UARTprintf("Interrupçao do Timer 0\n");

}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void
Timer1IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Use the flags to Toggle the LED for this timer
    //
    estado.led2 ^= 1;

    if (estado.led2)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    else
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    UARTprintf("Interrupçao do Timer 1\n");

}

//*****************************************************************************
//
// Prints out 5x "." with a second delay after each print.  This function will
// then backspace, clear the previously printed dots, backspace again so you
// continuously printout on the same line.  The purpose of this function is to
// indicate to the user that the program is running.
//
//*****************************************************************************
void
PrintRunningDots(void)
{
    UARTprintf(". ");
    SysCtlDelay(SysCtlClockGet() / 3);
    UARTprintf(". ");
    SysCtlDelay(SysCtlClockGet() / 3);
    UARTprintf(". ");
    SysCtlDelay(SysCtlClockGet() / 3);
    UARTprintf(". ");
    SysCtlDelay(SysCtlClockGet() / 3);
    UARTprintf(". ");
    SysCtlDelay(SysCtlClockGet() / 3);
    UARTprintf("\b\b\b\b\b\b\b\b\b\b");
    UARTprintf("          ");
    UARTprintf("\b\b\b\b\b\b\b\b\b\b");
    SysCtlDelay(SysCtlClockGet() / 3);
}


//*****************************************************************************
//
// Configure the UART and its pins. This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Exemplo de inicializado do Timer 0 com interrupção da cada 0,5s e
// Timer 1 com interrupção a cada 4s. Incluir os protótipos no arquivo com os
// vetores de interrupção.
//
//*****************************************************************************

void Init_Timer(){
    // Enable the peripherals used by this example.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Enable processor interrupts.
    ROM_IntMasterEnable();

    // Configure the two 32-bit periodic timers.
    //
    //TimerConfigure
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() / 2);
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet() * 4);

    // Setup the interrupts for the timer timeouts.
    ROM_IntEnable(INT_TIMER0A);
    ROM_IntEnable(INT_TIMER1A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    ROM_TimerEnable(TIMER1_BASE, TIMER_A);
}


//*****************************************************************************
//
// Exemplo de inicializado do PWM 1, gerador 2, saída PWM5 do pino PF1 (LED
// vermelho) com recarga periódica. A frequencia do módulo foi divida por 4
// para gerar um PWM com frequencia de 250 Hz e duty cycle variando de 0,01% a
// 75% com inclemento de 0,01% ().
//
//*****************************************************************************

void Init_PWM (void){
    //
    // Set the PWM clock to the system clock.
    //
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4); //50/4 = 12,5MHz

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Configure the GPIO pin muxing to select PWM00 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    GPIOPinConfigure(GPIO_PF1_M1PWM5);

    // Configure the PWM function for this pin.
    // Consult the data sheet to see which functions are allocated per pin.
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Configure the PWM1 to count down without synchronization.
    //
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the PWM period to 250 Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * SysClk.  Where N is the
    // function parameter, f is the desired frequency, and SysClk is the
    // system clock frequency.
    // In this case you get: (1 / 250 Hz) * 12.5MHz =  cycles.  Note that
    // the maximum period you can set is 2^16.
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 50000);

    // For this example the PWM1 duty cycle will be variable.  The duty cycle
    // will start at 0.1% (0.01 * 50000 cycles = 500 cycles) and will increase
    // to 75% (0.75 * 50000 cycles = 37500 cycles).  After a duty cycle of 75%
    // is reached, it is reset to 0.1%.  This dynamic adjustment of the pulse
    // width is done in the PWM1 load interrupt, which increases the duty
    // cycle by 0.1% everytime the reload interrupt is received.

     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 50);

     //
     // Enable processor interrupts.
     //
     IntMasterEnable();

     //
     // Allow PWM1 generated interrupts.  This configuration is done to
     // differentiate fault interrupts from other PWM0 related interrupts.
     //
     PWMIntEnable(PWM1_BASE, PWM_INT_GEN_2);

     //
     // Enable the PWM0 LOAD interrupt on PWM0.
     //
     PWMGenIntTrigEnable(PWM1_BASE, PWM_GEN_2, PWM_INT_CNT_LOAD);

     //
     // Enable the PWM0 interrupts on the processor (NVIC).
     //
     IntEnable(INT_PWM1_2);

     //
     // Enable the PWM0 output signal (PD0).
     //
     PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);

     //
     // Enables the PWM generator block.
     //
     PWMGenEnable(PWM1_BASE, PWM_GEN_2);

     //
     // Loop forever while the PWM signals are generated and PWM0 interrupts
     // get received.
     //
}

//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    // Habilita clock geral do sistema para rodar em 50 MHz a partir do PLL com cristal
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                     SYSCTL_OSC_MAIN);

    //SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
      //                SYSCTL_XTAL_16MHZ);

    // Habilita e espera o acesso ao PORTF
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}

    // Desbloqueia o PF0
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // Configura o GPIOF para operação com LEDs
    //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);

    // Configura os dois pinos para leitura do estado das chaves SW1 e SW2
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, SW1|SW2);

    // Configura a força para 2 mA e resistor fraco de pull-up
    GPIOPadConfigSet(GPIO_PORTF_BASE, SW1|SW2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //Configuração de Interrupção no Port F e Pino 4 (SW1) por borda de descida
    //GPIOIntTypeSet(GPIO_PORTF_BASE, SW1|SW2, GPIO_FALLING_EDGE);
    //GPIOIntEnable(GPIO_PORTF_BASE, SW1|SW2);
    //IntEnable(INT_GPIOF);

    // Initialize the UART and write status.
    ConfigureUART();

    UARTprintf("PWM ->\n");
    UARTprintf("  Modulo: PWM1\n");
    UARTprintf("  Pino: PF1\n");
    UARTprintf("  Duty Cycle: Variavel -> ");
    UARTprintf("0,1%% a 75%% com incremento de 0,1%%.\n");
    UARTprintf("  Apresentando: ");
    UARTprintf("PWM variavel usando interrupcao de recarga.\n\n");
    UARTprintf("Gerando PWM no PWM1 (GPIO_PF1_M1PWM5) -> ");

    // Init_Timer();

    Init_PWM();

    estado.inverter = 0;

    while(1)
    {
        //
        // Print out indication on the console that the program is running.
        //
        PrintRunningDots();

    }
}
