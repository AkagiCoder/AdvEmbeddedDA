#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#define TARGET_IS_BLIZZARD_RB1
#include "driverlib/rom.h"

#ifdef DEBUG
void__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void main(void)
{
    // Variables
    uint32_t ui32ADC0Value[4];
    volatile uint32_t ui32TempAvg;
    volatile uint32_t ui32TempValueC;
    volatile uint32_t ui32TempValueF;

    // f = 40 MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // Enables the ADC and GPIOF
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 64);

    // ADC0, sample sequencer 2, processor triggers, highest priority
    // Typo on Assignment PDF instruction for Task 01
    // Task 00 already used SS1 whereas "Task01 should being using SS2"
    ROM_ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_TS);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_TS);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_TS);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
    ROM_ADCSequenceEnable(ADC0_BASE, 2);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);     // Enable PF2

    while(1)
    {
        ROM_ADCIntClear(ADC0_BASE, 2);                          // Clear ADC interrupt flag
        ROM_ADCProcessorTrigger(ADC0_BASE, 2);                  // Trigger the ADC conversion
        while(!ROM_ADCIntStatus(ADC0_BASE, 2, false));          // Poll until conversion completes
        ROM_ADCSequenceDataGet(ADC0_BASE, 2, ui32ADC0Value);    // Store the temperature value
        ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1]      // Average the sampled temperatures
                       + ui32ADC0Value[2] + ui32ADC0Value[3]
                       + 2) / 4;
        // Convert to Celsius
        ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
        // Convert to Fahrenheit
        ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;

        // Turn on PF2 LED if temp > 72 F
        if(ui32TempValueF > 72)
            ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
        else
            ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    }
}
