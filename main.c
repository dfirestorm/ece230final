/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 * Author: 
*******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#define A2TIMER_PERIOD  30000  //drives servos at 50 Hz (period of 20 ms)
//angle servo macros
#define MIN_ANGLE       1133
#define MAX_ANGLE       3396
#define CENTER_ANGLE    2250
//#define ANGLE_STEP      187

//continuous servo macros
#define MAX_CW          //???
#define NO_ROTATION     2250
#define MAX_CCW         //???

/* Global Variables */
//state defining
bool write = false;
bool jsRead = false;

//data storage
int joystickX = 0;
int joystickY = 0;
int servoAngle = 2250;
int servoSpeed = 2250;


//first: get servos responding to joystick correctly

//TA2: controls servo pulse width
Timer_A_PWMConfig pwmConfigA2 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_2,          //3/2 = 1.5 MHz
        A2TIMER_PERIOD,                         //signal at 50 Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        CENTER_ANGLE                            //starts at 0 degrees
};

//TA1: 50 ms timer
Timer_A_UpModeConfig upConfigA1 = {
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source (3MHz)
        TIMER_A_CLOCKSOURCE_DIVIDER_4,          // SMCLK/4 = 75000 Hz
        7500,                                   // 50 ms
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
        };

//configures ADC and Timer A1, which controls sampling
void initializeADC(void){
    /* Setting Flash wait state */
        MAP_FlashCtl_setWaitState(FLASH_BANK0, 1);
        MAP_FlashCtl_setWaitState(FLASH_BANK1, 1);

        /* Setting DCO to 48MHz  */
        MAP_PCM_setPowerState(PCM_AM_LDO_VCORE1);
        MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

        /* Enabling the FPU for floating point operation */
        MAP_FPU_enableModule();
        MAP_FPU_enableLazyStacking();

        //![Single Sample Mode Configure]
        /* Initializing ADC (MCLK/1/4) */
        MAP_ADC14_enableModule();
        MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4,
                             0);
        MAP_ADC14_setResolution(ADC_14BIT);

        /* Configuring GPIOs (5.0 A5 JS-Y) and (5.1 A4 JS-X) and (5.2 A3 JS-X)*/
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN0,
        GPIO_TERTIARY_MODULE_FUNCTION);
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN1,
        GPIO_TERTIARY_MODULE_FUNCTION);
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN2,
                GPIO_TERTIARY_MODULE_FUNCTION);

        /* Configuring ADC Memory */
        MAP_ADC14_configureMultiSequenceMode(ADC_MEM3, ADC_MEM5, true);
        // MAP_ADC14_configureSingleSampleMode(ADC_MEM15, true);
        MAP_ADC14_configureConversionMemory(ADC_MEM5, ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A5,
                                            false);
        MAP_ADC14_configureConversionMemory(ADC_MEM4, ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A4,
                                            false);
        MAP_ADC14_configureConversionMemory(ADC_MEM3, ADC_VREFPOS_AVCC_VREFNEG_VSS,
                ADC_INPUT_A3,
                                                    false);

        /* Configuring Sample Timer */
        MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

        //congifure Timer A1 in upMode
          MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfigA1);
        //  MAP_Interrupt_enableSleepOnIsrExit();
          MAP_Interrupt_enableInterrupt(INT_TA1_0);
          MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

          /* Enabling/Toggling Conversion */
          MAP_ADC14_enableConversion();
          MAP_ADC14_toggleConversionTrigger();

          /* Enabling interrupts */
          MAP_ADC14_enableInterrupt(ADC_INT5);
          MAP_ADC14_enableInterrupt(ADC_INT4);
          MAP_ADC14_enableInterrupt(ADC_INT3);
          MAP_Interrupt_enableInterrupt(INT_ADC14);
          MAP_Interrupt_enableMaster();
}

void initializeServo(void){
    //configure pin 5.6 as output (angle servo pin)
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6   ,
               GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigA2);
}

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    /*Configure Fancy Clock */
       MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
                  GPIO_PORT_PJ,
                  GPIO_PIN3 | GPIO_PIN2,
                  GPIO_PRIMARY_MODULE_FUNCTION);
          //MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
          /* Just in case the user wants to use the getACLK, getMCLK, etc. functions,
           * Set HFXT clock frequency to 48MHz
           */

       CS_setExternalClockSourceFrequency(32000, 48000000);

       MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
       MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
       MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
       CS_startHFXT(false);

       /* Initializing MCLK and SMCLK to HFXT with a 16:1 prescaler (3 MHz)  */
       MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16);
       MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16);

       //Configure LCD Display

       initializeADC(); //configure ADC
       initializeServo();


    while(1)
    {
       //testing commit part 2
        if(jsRead){
            jsRead=false;
            if(joystickX > 16000){
                servoAngle -=25;
                if(servoAngle < MIN_ANGLE){
                    servoAngle = MIN_ANGLE;
                }
                Timer_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, servoAngle);
            } else if(joystickX < 50){
                servoAngle +=25;
                if(servoAngle > MAX_ANGLE){
                    servoAngle = MAX_ANGLE;
                }
                Timer_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, servoAngle);
            }
        }
    }
}

void TA1_0_IRQHandler(void)
{
    jsRead = true;
    MAP_ADC14_toggleConversionTrigger();
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
    TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

void ADC14_IRQHandler(void)
{
    uint64_t status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if (ADC_INT5 & status)
    {
        joystickX = MAP_ADC14_getResult(ADC_MEM5);
       // voltPotResult15 = (digiPotResult15 * 3.3) / 16384;
    }

    if (ADC_INT4 & status)
    {
        joystickY = MAP_ADC14_getResult(ADC_MEM4);

    }

    if(ADC_INT3 & status){
        //temperature sensor stuff
    }

}
