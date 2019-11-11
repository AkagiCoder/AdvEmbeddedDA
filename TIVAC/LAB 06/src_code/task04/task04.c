#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/qei.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"


#define PWM_FREQUENCY 55

volatile int qeiPosition;
volatile int qeiVelocity;

void main(void)
{
    uint32_t ui32ADC0Value[4];
    volatile uint32_t ui32PotValue;
    volatile uint32_t ui32Load;
    volatile uint32_t ui32PWMClock;
    volatile uint16_t ui16Adjust = 0;

    // Sets up the system clock and PWM clock
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enable PE2 [ADC input], PWM0 [PE4 is the PWM], PA2, and PA3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);        // ADC input and PWM0
    // PA2/PA3 controls direction of motor
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOADCTriggerEnable(GPIO_PORTE_BASE, GPIO_PIN_2);  // PE2 is ADC input
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);         // Enable PWM0 for PE4

    // Configure PE4 as a PWM
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);

    // Configure PWM clock
    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load);

    // Adjust width and enable PWM
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ui16Adjust);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    // Enable ADC0 [Code from lab 5]
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);

    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    //Unlock GPIOD7 - Like PF0 its used for NMI -
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7.
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    //DISable peripheral and int before configuration
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER |
    QEI_INTINDEX);

    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET
    | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1000);
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet());

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);

    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, 500);
    QEIVelocityEnable(QEI0_BASE);

    // Set up GPIOA for UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);

    while(1)
    {
        ADCIntClear(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC0_BASE, 1);
        // Poll for the ADC flag
        while(!ADCIntStatus(ADC0_BASE, 1, false));
        ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
        // Average out the sampled analog signal
        ui32PotValue = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
        // Adjust the speed of the motor by multiplying the pot value by 3
        ui16Adjust = ui32PotValue * 3;

        // Motor is off
        if(ui16Adjust < 1136)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0);
            ui16Adjust = 1136;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ui16Adjust);
        }

        // Motor is on
        else
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);
            // Adjust the speed from 10% DC to 90%
            if(ui16Adjust < 10225)
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ui16Adjust);
            else
                // Maximum DC of the PWM is 90%
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 10225);
        }

        // Read the position and speed of the motor using QEI
        qeiPosition = QEIPositionGet(QEI0_BASE);
        qeiVelocity = QEIVelocityGet(QEI0_BASE);
        UARTprintf("QEI Velocity: %d\n", qeiVelocity);
        SysCtlDelay(10000);

    }
}
