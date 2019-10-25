#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/adc.h"

#define NUM_SSI_DATA 2

int main(void)
{
    uint32_t pui32DataTx[NUM_SSI_DATA];
    uint32_t pui32DataRx[NUM_SSI_DATA];
    uint32_t ui32Index;

    // ADC Variables
    uint32_t ui32ADC0Value[4];
    volatile uint32_t ui32TempAvg;
    volatile uint32_t ui32TempValueC;
    volatile uint32_t ui32TempValueF;

    // System clock at 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // The SSI0 peripheral and port A must be enabled for use.
    // Enable the SSI0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    // The SSI0 peripheral is on Port A and pins 2,3,4 and 5.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // This function/s configures the pin muxing on port A pins 2,3,4 and 5
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_4,GPIO_PIN_4);

    // Configure and enable the SSI port for SPI master mode.
    SSIClockSourceSet(SSI0_BASE,SSI_CLOCK_SYSTEM);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(SSI0_BASE);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);

    // Enable the ADC module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);

    // ADC0, sample sequencer 2, processor triggers, highest priority
    // Typo on Assignment PDF instruction for Task 01
    // Task 00 already used SS1 whereas "Task01 should being using SS2"
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);

    while(1)
    {
        ADCIntClear(ADC0_BASE, 2);                          // Clear ADC interrupt flag
        ADCProcessorTrigger(ADC0_BASE, 2);                  // Trigger the ADC conversion
        while(!ADCIntStatus(ADC0_BASE, 2, false));          // Poll until conversion completes
        ADCSequenceDataGet(ADC0_BASE, 2, ui32ADC0Value);    // Store the temperature value
        ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1]      // Average the sampled temperatures
                       + ui32ADC0Value[2] + ui32ADC0Value[3]
                       + 2) / 4;
        // Convert to Celsius
        ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
        // Convert to Fahrenheit
        ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;

        // Print the temperature to the terminal
        UARTprintf("\033[2J");
        UARTprintf("Measured temp: %d F; %d C\n", ui32TempValueF, ui32TempValueC);

        // Place the two temperatures into an array that's to be transmitted
        pui32DataTx[0] = ui32TempValueF;
        pui32DataTx[1] = ui32TempValueC;

        UARTprintf("SSI ->\n");
        UARTprintf(" Mode: SPI\n");
        UARTprintf(" Data: 8-bit\n\n");
        UARTprintf("Sent:\n ");

        // Transmission process
        while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0]));
        for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
        {
            // Display the data that SSI is transferring.
            UARTprintf(" %d ", pui32DataTx[ui32Index]);
            SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
        }
        while(SSIBusy(SSI0_BASE));
        // Receiving the data
        UARTprintf("\nReceived:\n ");
        for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
        {
            SSIDataGet(SSI0_BASE, &pui32DataRx[ui32Index]);
            // Since we are using 8-bit data, mask off the MSB.
            pui32DataRx[ui32Index] &= 0x00FF;
            // Display the data that SSI0 received.
            UARTprintf(" %d ", pui32DataRx[ui32Index]);
        }
    }
    return 0;
}
