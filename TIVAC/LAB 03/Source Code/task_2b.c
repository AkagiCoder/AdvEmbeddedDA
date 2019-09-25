#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

uint8_t ui8PinData = 0;
uint8_t seqCount = 1;

void main(void)
{
    // Task 2
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Part B - Blink R, G, B, RG, RB, GB, RGB, R, G,...
    while(1)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, ui8PinData);
        SysCtlDelay(2000000);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
        SysCtlDelay(2000000);

        // Determines the current sequence
        switch(seqCount)
        {
        case 1:
            if(ui8PinData == 0)
                ui8PinData = 2;     // Set to RED
            else
            {
                ui8PinData = 0;
                seqCount++;
            }
            break;
        case 2:
            if(ui8PinData == 0)
                ui8PinData = 8;     // Set to GREEN
            else
            {
                ui8PinData = 0;
                seqCount++;
            }
            break;
        case 3:
            if(ui8PinData == 0)
                ui8PinData = 4;     // Set to BLUE
            else
            {
                ui8PinData = 0;
                seqCount++;
            }
            break;
        case 4:
            if(ui8PinData == 0)
                ui8PinData = 2;     // Set to RED
            else if(ui8PinData == 2)
                ui8PinData = 8;     // Set to GREEN
            else
            {
                ui8PinData = 0;
                seqCount++;
            }
            break;
        case 5:
            if(ui8PinData == 0)
                ui8PinData = 2;     // Set to RED
            else if(ui8PinData == 2)
                ui8PinData = 4;     // Set to BLUE
            else
            {
                ui8PinData = 0;
                seqCount++;
            }
            break;
        case 6:
            if(ui8PinData == 0)
                ui8PinData = 8;     // Set to GREEN
            else if(ui8PinData == 8)
                ui8PinData = 4;     // Set to BLUE
            else
            {
                ui8PinData = 0;
                seqCount++;
            }
            break;
        case 7:
            if(ui8PinData == 0)
                ui8PinData = 2;     // Set to RED
            else if(ui8PinData == 2)
                ui8PinData = 8;     // Set to GREEN
            else if(ui8PinData == 8)
                ui8PinData = 4;     // Set to BLUE
            else
            {
                ui8PinData = 0;
                seqCount = 1;
            }
            break;
        default:
            ui8PinData = 0;         // Clear LED
            seqCount = 1;           // Reset sequence
        }
    }
}
