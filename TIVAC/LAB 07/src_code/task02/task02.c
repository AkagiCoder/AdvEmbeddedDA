#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#define TARGET_IS_BLIZZARD_RA1
#include "driverlib/rom.h"

// Variables
uint32_t ui32ADC0Value[4];
uint32_t ui32Period;
volatile uint32_t ui32TempAvg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;
volatile bool temp;
char buffer[4];

// Function to print a string to the terminal
void printString(char *string)
{
    while(*string)
    {
        UARTCharPut(UART0_BASE, *string);
        string++;
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
    ltoa(ui32TempValueF, buffer);                           // Convert the temperature into a string
    // Check to display the temperature on the terminal
    if(temp)
    {
        // Prints the temperature
        UARTCharPut(UART0_BASE, 0x0C);    // Clear the terminal
        printString("Temperature: ");
        printString(buffer);
        printString(" F");
        UARTCharPut(UART0_BASE, '\n');
        UARTCharPut(UART0_BASE, '\r');
    }
    else
        UARTCharPut(UART0_BASE, 0x0C);    // Clear the terminal
    ROM_TimerEnable(TIMER1_BASE, TIMER_A);                  // Re-enable Timer1
}

void UARTIntHandler(void)
{
    char input;
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true);   //get interrupt status
    UARTIntClear(UART0_BASE, ui32Status);           //clear the asserted interrupts
    while(UARTCharsAvail(UART0_BASE))               //loop while there are chars
    {
        input = UARTCharGetNonBlocking(UART0_BASE); // Grab the inputs from the keyboard

        // Check the inputs to execute the commands
        switch(input)
        {
        // Red LED
        case 'R':
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            break;
        case 'r':
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            break;
        // Blue LED
        case 'B':
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            break;
        case 'b':
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            break;
        // Green LED
        case 'G':
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
            break;
        case 'g':
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
            break;
        // Temperature
        case 'T':
            temp = true;
            break;
        case 't':
            temp = false;
            break;
        }
    }
}

int main(void) {
    // f = 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Configure the UART, GPIOA, and GPIOF
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

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

    // (50 MHz * 0.5 Hz)
    ui32Period = ROM_SysCtlClockGet() * 0.5;
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period - 1);

    // Enable Timer1A interrupt
    ROM_IntEnable(INT_TIMER1A);
    ROM_IntEnable(INT_UART0);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    ROM_IntMasterEnable();

    // Start TIMER1A
    TimerEnable(TIMER1_BASE, TIMER_A);

    while (1);
}


