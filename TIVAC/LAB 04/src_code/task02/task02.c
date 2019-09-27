#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

// Timer1 delay function where delayT is the number of seconds to delay.
void timer1A_delay(int delayT)
{
    int i;
    TimerDisable(TIMER1_BASE, TIMER_A);                 // Disable Timer1A
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);    // Configure Timer1 to be periodic
    TimerLoadSet(TIMER1_BASE, TIMER_A, 3999999);        // 1 Hz
    TIMER1_ICR_R = 0x1;                                 // Clear the Timer1A timeout flag
    TimerEnable(TIMER1_BASE, TIMER_A);                  // Enable Timer1A

    for(i = 0; i < delayT; i++)
    {
        while((TIMER1_RIS_R & 0x1) == 0);               // Check for timer-out
        TIMER1_ICR_R = 0x1;                             // Clear the Timer1A timeout flag
    }
}


void main(void)
{
    uint32_t ui32Period = 0;

    // Set up the clock f = 40 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Set up the GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                                // Configure the PF GPIO_Reg

    // Unlock and lock the GPIO_PF0
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);   // PF1, PF2, PF3 set to output for LEDS
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);                          // Configure PF0 as input for SW2
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);                 // Set PF0 as a weak pull-up
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0);                             // Set up interrupt for PF0 [SW2]
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);              // Detect input on rising edge


    // Set up TIMER0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Set up TIMER1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // (40 MHz / 10 Hz) * 43% DC
    ui32Period = (SysCtlClockGet() / 10) * 0.43;
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);

    // Enable interrupts
    IntEnable(INT_TIMER0A);                             // TIMER0A
    IntEnable(INT_GPIOF);                               // Enable the GPIOF interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    // Start TIMER0A
    TimerEnable(TIMER0_BASE, TIMER_A);

    while(1)
    {
    }
}

// Timer0 ISR
// Toggles the the LED for f= 10 Hz with DC = 43%
void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Read the current state of the GPIO pin and
    // write back the opposite state
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
    }
}

// SW2 ISR
// 1) Suspend Timer0A to prevent LED from toggling
// 2) Use Timer1 to turn on the LED for 1 sec
// 3) Re-enable Timer0 ISR
void SW2IntHandler(void)
{
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0);                          // Clear the GPIO interrupt
    TimerDisable(TIMER0_BASE, TIMER_A);                                     // Disable Timer0A
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 4); // Turn on the LED
    timer1A_delay(1);                                                       // Call Timer1 delay function for 1 second
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0); // Turn off the LED
    TimerEnable(TIMER0_BASE, TIMER_A);                                      // Re-enable Timer0A
}

