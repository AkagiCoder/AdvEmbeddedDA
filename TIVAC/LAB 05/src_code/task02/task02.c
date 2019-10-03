#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "inc/tm4c123gh6pm.h"
#define TARGET_IS_BLIZZARD_RB1
#include "driverlib/rom.h"


#ifdef DEBUG
void__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// Variables
uint32_t ui32ADC0Value[4];
uint32_t ui32Period;
volatile uint32_t ui32TempAvg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;

void main(void)
{
    // f = 40 MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // Enables the ADC
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 32);      // Average at 32 samples

    // ADC0, sample sequencer 1, processor triggers, highest priority
    ROM_ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
    ROM_ADCSequenceEnable(ADC0_BASE, 1);

    // Set up TIMER1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    // (40 MHz / 0.5 Hz)
    ui32Period = ROM_SysCtlClockGet() * 0.5;
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period - 1);

    // Enable Timer1A interrupt
    ROM_IntEnable(INT_TIMER1A);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    ROM_IntMasterEnable();

    // Start TIMER1A
    ROM_TimerEnable(TIMER1_BASE, TIMER_A);

    while(1)
    {
    }
}

// Timer 1A ISR
void Timer1IntHandler(void)
{
    // Reset the count on Timer1
    ROM_TimerDisable(TIMER1_BASE, TIMER_A);                 // Disable Timer1
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);     // (40 MHz / 0.5 Hz)
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);     // Clear Timer1A flag
    // Perform ADC conversion
    ROM_ADCIntClear(ADC0_BASE, 1);                          // Clear ADC interrupt flag
    ROM_ADCProcessorTrigger(ADC0_BASE, 1);                  // Trigger the ADC conversion
    while(!ROM_ADCIntStatus(ADC0_BASE, 1, false));          // Poll until conversion completes
    ROM_ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);    // Store the temperature value
    ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1]      // Average the sampled temperatures
                   + ui32ADC0Value[2] + ui32ADC0Value[3]
                   + 2) / 4;
    // Convert to Celsius
    ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
    // Convert to Fahrenheit
    ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;

    ROM_TimerEnable(TIMER1_BASE, TIMER_A);                  // Re-enable Timer1
}
