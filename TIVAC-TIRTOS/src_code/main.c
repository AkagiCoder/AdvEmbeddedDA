//---------------------------------------------------------------------------------
// Project: Blink TM4C BIOS Using Swi (STARTER)
// Author: Eric Wilbur
// Date: June 2014
//
// Note: The function call TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT) HAS
//       to be in the ISR. This fxn clears the TIMER's interrupt flag coming
//       from the peripheral - it does NOT clear the CPU interrupt flag - that
//       is done by hardware. The author struggled figuring this part out - hence
//       the note. And, in the Swi lab, this fxn must be placed in the
//       Timer_ISR fxn because it will be the new ISR.
//
// Follow these steps to create this project in CCSv6.0:
// 1. Project -> New CCS Project
// 2. Select Template:
//    - TI-RTOS for Tiva-C -> Driver Examples -> EK-TM4C123 LP -> Example Projects ->
//      Empty Project
//    - Empty Project contains full instrumentation (UIA, RTOS Analyzer) and
//      paths set up for the TI-RTOS version of MSP430Ware
// 3. Delete the following files:
//    - Board.h, empty.c, EK_TM4C123GXL.c/h, empty_readme.txt
// 4. Add main.c from TI-RTOS Workshop Solution file for this lab
// 5. Edit empty.cfg as needed (to add/subtract) BIOS services, delete given Task
// 6. Build, load, run...
//----------------------------------------------------------------------------------


//----------------------------------------
// BIOS header files
//----------------------------------------
#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles


//------------------------------------------
// TivaWare Header Files
//------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "inc/hw_gpio.h"

//----------------------------------------
// Constants
//----------------------------------------
#define PWM_FREQUENCY 55        // Frequency of the PWM

//----------------------------------------
// Prototypes
//----------------------------------------
void hardware_init(void);
void Timer_ISR(void);
void ledToggle(void);
void potADC(void);
void UARTDisplay(void);
void PWMSwitch(void);
void printString(char *);

//---------------------------------------
// Globals
//---------------------------------------
volatile int16_t i16ToggleCount = 0;
volatile int8_t i8TaskTime = 0;
volatile int32_t ui32PotValue;
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint16_t ui16Adjust = 1136;
uint32_t ui32ADC1Value[4];

//---------------------------------------------------------------------------
// main()
//---------------------------------------------------------------------------
void main(void)
{
   hardware_init();							// init hardware via Xware
   BIOS_start();
}

//---------------------------------------------------------------------------
// hardware_init()
//
// inits GPIO pins for toggling the LED
//---------------------------------------------------------------------------
void hardware_init(void)
{
	uint32_t ui32Period;

	//Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// ADD Tiva-C GPIO setup - enables port, sets pins 1-3 (RGB) pins for output
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	// Timer 2 setup code
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);			// enable Timer 2 periph clks
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);		// cfg Timer 2 mode - periodic
	ui32Period = (SysCtlClockGet() / 1000);					// period = CPU clk div 1000 (1 ms)
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);			// set Timer 2 period
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);		// enables Timer 2 to interrupt CPU
	TimerEnable(TIMER2_BASE, TIMER_A);						// enable Timer 2

	// Setup ADC1 Channel 3
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC1_BASE, 1, 2, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC1_BASE, 1, 3, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 1);

    // Setup PE0 as an ADC input to ADC1 Channel 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);            // Enable GPIOE
    GPIOADCTriggerEnable(GPIO_PORTE_BASE, GPIO_PIN_0);      // PE0 is ADC input

    // Setup UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);            // Enable GPIOA
    GPIOPinConfigure(GPIO_PA0_U0RX);                        // Configure the PA0 and PA1 for UART
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Set the PWM clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enables PWM1, GPIOD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // Configure PF1 as a PWM
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);

    // Unlock PF0 for SW
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // Configure pull-up for PF0 and PF4
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Configure PWM clock
    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ui32Load);

    // Adjust width and enable PWM
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ui16Adjust);
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
}

// TASK #1
// Read the ADC value at PE0
// Value ranges from 0 < x < 4096
void potADC(void)
{
    while(1)
    {
        Semaphore_pend(potSem, BIOS_WAIT_FOREVER);
        ADCIntClear(ADC1_BASE, 1);
        ADCProcessorTrigger(ADC1_BASE, 1);
        // Poll for the ADC flag
        while(!ADCIntStatus(ADC1_BASE, 1, false));
        ADCSequenceDataGet(ADC1_BASE, 1, ui32ADC1Value);
        // Average out the sampled analog signal
        ui32PotValue = (ui32ADC1Value[0] + ui32ADC1Value[1] + ui32ADC1Value[2] + ui32ADC1Value[3] + 2)/4;
    }
}

// Task #2
// Display the ADC value on the terminal
void UARTDisplay(void)
{
    char buffer[4];
    while(1)
    {
        Semaphore_pend(UARTSem, BIOS_WAIT_FOREVER);
        ltoa(ui32PotValue, buffer);                          // Convert the potentiometer value into a string
        printString("Read Potentiometer Value: ");
        printString(buffer);
        UARTCharPut(UART0_BASE, '\n');
        UARTCharPut(UART0_BASE, '\r');
    }
}

// Task #3
// Adjust the DC of the PWM using the two switches.
// Changing the DC will adjust the red LED's brightness.
void PWMSwitch(void)
{
    while(1)
    {
        Semaphore_pend(PWMSem, BIOS_WAIT_FOREVER);
        // Decrease DC
        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
        {
            ui16Adjust = ui16Adjust - 100;
            // Min DC = 10%
            if (ui16Adjust < 1136)
            {
                ui16Adjust = 1136;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ui16Adjust);
        }
        // Increase DC
        if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
        {
            ui16Adjust = ui16Adjust + 100;
            // Max DC = 90%
            if (ui16Adjust > 10225)
            {
                ui16Adjust = 10225;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ui16Adjust);
        }
    }
}

// Function to service the 1 ms HWI_TIMER2
// Timeline:
//  10 ms - Task #1
//  20 ms - Task #2
//  30 ms - Task #3
void Timer_ISR(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);         // must clear timer flag FROM timer
    i8TaskTime++;                                           // Update counter
    switch(i8TaskTime)
    {
    // At 10 ms, post the potentiometer [task #1]
    case 10:
        Semaphore_post(potSem);
        break;
    // At 20 ms, post the UART [task #2]
    case 20:
        Semaphore_post(UARTSem);
        break;
    // At 30 ms, post the PWM [task #3]
    case 30:
        Semaphore_post(PWMSem);
        // Reset the counter
        i8TaskTime = 0;
    }
}

// Function to print a string to the terminal
void printString(char *string)
{
    while(*string)
    {
        UARTCharPut(UART0_BASE, *string);
        string++;
    }
}
