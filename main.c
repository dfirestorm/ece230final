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
#include <stdio.h>
#include "lcd.h"
#include "delays.h"
#include <string.h>

#define A2TIMER_PERIOD  30000  //drives servos at 50 Hz (period of 20 ms)
//angle servo macros
#define MIN_ANGLE       1125
#define MAX_ANGLE       3375
#define CENTER_ANGLE    2250
//#define ANGLE_STEP      187

//continuous servo macros min = 1 ms max = 2ms stopped = 1.5ms pulse width
#define MAX_SPEED          3000
#define NO_SPEED         2250
#define MAX_CCW         1500

/* Global Variables */
//state defining
bool write = true;
bool jsRead = false;
int speedLimit = MAX_SPEED;
int currentState = 0;
int dataState = 0;

//data storage
int joystickX = 0;
int joystickY = 0;
int servoAngle = 2250;
int servoSpeed = 2250;
int digitalTempValue = 0;
float celsiusTempValue = 0;
float fahrenheitTempValue = 0;
int16_t accel_xL, accel_yL, accel_zL;
int16_t gyro_xL, gyro_yL, gyro_zL;

int16_t accel_xR, accel_yR, accel_zR;
int16_t gyro_xR, gyro_yR, gyro_zR;

//i2C stuff
#define SLAVE_0       0x68
#define SLAVE_1       0x69

#define NUM_OF_REC_BYTES    14
#define ACCEL_BASE     0x3B
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define PWR_MGMT       0x6B

/* Variables */
const uint8_t TXData[] = { 0x04 };
static uint8_t RXData[NUM_OF_REC_BYTES];
static volatile uint32_t xferIndex;
static volatile bool stopSent;
bool currentSlave = true;

//first: get servos responding to joystick correctly - done
//second: get I2C reading different values - done
//third: get LCD printing data in units desired.

//TA2: controls anglular servo pulse width
Timer_A_PWMConfig pwmConfigA2AServo =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_2,          //3/2 = 1.5 MHz
        A2TIMER_PERIOD,                         //signal at 50 Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        CENTER_ANGLE                            //starts at 0 degrees
};

//controls continuous servo pulse width
Timer_A_PWMConfig pwmConfigA2CServo =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_2,          //3/2 = 1.5 MHz
        A2TIMER_PERIOD,                         //signal at 50 Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        CENTER_ANGLE                            //starts at 0 speed
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

const eUSCI_I2C_MasterConfig i2cConfig = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
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
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN6   ,
               GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigA2AServo);
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigA2CServo);
}

void initializeLCD(){
    configLCD(GPIO_PORT_P6, GPIO_PIN1, GPIO_PORT_P6, GPIO_PIN4, GPIO_PORT_P4);
    initDelayTimer(CS_getMCLK());
    initLCD();
    printChar('h');
    printChar('o');
    printChar('w');
    printChar('d');
    printChar('y');
    printChar('!');
    printChar(' ');
}

void initializeI2C(){
    /* Select Port 1 for I2C - Set Pin 6, 7 to input Primary Module Function,
     *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
     */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1,
            GPIO_PIN6 + GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);
    stopSent = false;
    memset(RXData, 0x00, NUM_OF_REC_BYTES);

    /* Initializing I2C Master to SMCLK at 100khz with no autostop */
    MAP_I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);

    /* Enable I2C Module to start operations */
    MAP_I2C_enableModule(EUSCI_B0_BASE);
    MAP_Interrupt_enableInterrupt(INT_EUSCIB0);

    //configure Slave 0 (Left GY-521)
    /* Specify slave address for slave0 */
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_0);

    while (I2C_masterIsStopSent(EUSCI_B0_BASE));
    /* Send Start, address frame, and the first byte of write. */
    I2C_masterSendMultiByteStart(EUSCI_B0_BASE, PWR_MGMT);
    /* Send final byte of write, and Stop   */
    I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, 0);

    while (I2C_masterIsStopSent(EUSCI_B0_BASE));
    /* Send Start, address frame, and the first byte of write. */
    I2C_masterSendMultiByteStart(EUSCI_B0_BASE, ACCEL_CONFIG);
    /* Send final byte of write, and Stop   */
    I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, 0b10000);

    while (I2C_masterIsStopSent(EUSCI_B0_BASE));
    /* Send Start, address frame, and the first byte of write. */
    I2C_masterSendMultiByteStart(EUSCI_B0_BASE, GYRO_CONFIG);
    /* Send final byte of write, and Stop   */
    I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, 0b00001000);

    //configure Slave 1 (Right GY-521)
    /* Specify slave address */
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_1);

    while (I2C_masterIsStopSent(EUSCI_B0_BASE));
    /* Send Start, address frame, and the first byte of write. */
    I2C_masterSendMultiByteStart(EUSCI_B0_BASE, PWR_MGMT);
    /* Send final byte of write, and Stop   */
    I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, 0);

    while (I2C_masterIsStopSent(EUSCI_B0_BASE));
    /* Send Start, address frame, and the first byte of write. */
    I2C_masterSendMultiByteStart(EUSCI_B0_BASE, ACCEL_CONFIG);
    /* Send final byte of write, and Stop   */
    I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, 0b10000);

    while (I2C_masterIsStopSent(EUSCI_B0_BASE));
    /* Send Start, address frame, and the first byte of write. */
    I2C_masterSendMultiByteStart(EUSCI_B0_BASE, GYRO_CONFIG);
    /* Send final byte of write, and Stop   */
    I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, 0b00001000);

    //read from Slave 0 first
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_0);
}

void initializeSwitches(){
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    MAP_Interrupt_enableInterrupt(INT_PORT1);
}

void printLAccelData(){
    //this is test code
    if(write){
        write = false;
           if(currentSlave){
               displayStart();
               char zLArray[8];
               sprintf(zLArray, "% ld   ", accel_zL);
               printString(zLArray, 8);
               printChar(' ');
               printChar(' ');
               displayLine2();
               char zRArray[5];
               sprintf(zRArray, "% ld", accel_yL);
               printString(zRArray, 5);
               printChar(' ');
               printChar(' ');

           }

           /* Send start and the first byte of the transmit buffer. */
           MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, ACCEL_BASE);

           /* Sent the first byte, now we need to initiate the read */
           xferIndex = 0;
           MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
           MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    }

}

void printLGyroData(){
    if(write){
        write = false;
           if(currentSlave){
               displayStart();
               char zLArray[8];
               sprintf(zLArray, "% ld   ", gyro_zL);
               printString(zLArray, 8);
               printChar(' ');
               printChar(' ');
               displayLine2();
               char zRArray[5];
               sprintf(zRArray, "% ld", gyro_yL);
               printString(zRArray, 5);
               printChar(' ');
               printChar(' ');

           }

           /* Send start and the first byte of the transmit buffer. */
           MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, ACCEL_BASE);

           /* Sent the first byte, now we need to initiate the read */
           xferIndex = 0;
           MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
           MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    }

}

void printRAccelData(){
    if(write){
        write = false;
           if(!currentSlave){
               displayStart();
               char zLArray[8];
               sprintf(zLArray, "% ld   ", accel_zR);
               printString(zLArray, 8);
               printChar(' ');
               printChar(' ');
               displayLine2();
               char zRArray[5];
               sprintf(zRArray, "% ld", accel_yR);
               printString(zRArray, 5);
               printChar(' ');
               printChar(' ');

           }

           /* Send start and the first byte of the transmit buffer. */
           MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, ACCEL_BASE);

           /* Sent the first byte, now we need to initiate the read */
           xferIndex = 0;
           MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
           MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    }

}

void printRGyroData(){
    if(write){
        write = false;
           if(!currentSlave){
               displayStart();
               char zLArray[8];
               sprintf(zLArray, "% ld   ", gyro_zR);
               printString(zLArray, 8);
               printChar(' ');
               printChar(' ');
               displayLine2();
               char zRArray[5];
               sprintf(zRArray, "% ld", gyro_yR);
               printString(zRArray, 5);
               printChar(' ');
               printChar(' ');

           }

           /* Send start and the first byte of the transmit buffer. */
           MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, ACCEL_BASE);

           /* Sent the first byte, now we need to initiate the read */
           xferIndex = 0;
           MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
           MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    }

}

void printTempData(){
    if(write){
        clearScreen();
        write = false;
        printString("Temp: 85", 8);
    }

}

void printServoData(){
    if(write){
        clearScreen();
        write = false;
        printString("Servo: 90", 9);
    }

}


void normalState(){
    speedLimit = MAX_SPEED;
    switch(dataState){
        case 0:
            printLAccelData();
            break;
        case 1:
            printLGyroData();
            break;
        case 2:
            printRAccelData();
            break;
        case 3:
            printRGyroData();
            break;
        case 4:
            printTempData();
            break;
        case 5:
            printServoData();
            break;
        default:
            dataState = 0;
    }

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
       initializeLCD();
       initializeSwitches();

       MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
       TIMER32_PERIODIC_MODE);
       MAP_Timer32_setCount(TIMER32_0_BASE, 1500000);
       MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);
       MAP_Timer32_enableInterrupt(TIMER32_0_BASE);
       MAP_Interrupt_enableInterrupt(INT_T32_INT1);
       MAP_Timer32_startTimer(TIMER32_0_BASE, false);

       initializeI2C();


    while(1)
    {
        /* Making sure the last transaction has been completely sent out */
        while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE));

        if(currentSlave==true){
              accel_xL = (((uint16_t)RXData[0]) << 8) + (RXData[1]);
              accel_yL = (((uint16_t)RXData[2]) << 8) + (RXData[3]);
              accel_zL = (((uint16_t)RXData[4]) << 8) + (RXData[5]);

              gyro_xL = (((uint16_t)RXData[8]) << 8) + (RXData[9]);
              gyro_yL = (((uint16_t)RXData[10]) << 8) + (RXData[11]);
              gyro_zL = (((uint16_t)RXData[12]) << 8) + (RXData[13]);
         }else{
              accel_xR = (((uint16_t)RXData[0]) << 8) + (RXData[1]);
              accel_yR = (((uint16_t)RXData[2]) << 8) + (RXData[3]);
              accel_zR = (((uint16_t)RXData[4]) << 8) + (RXData[5]);

              gyro_xR = (((uint16_t)RXData[8]) << 8) + (RXData[9]);
              gyro_yR = (((uint16_t)RXData[10]) << 8) + (RXData[11]);
              gyro_zR = (((uint16_t)RXData[12]) << 8) + (RXData[13]);
         }

       //joystick and servo control
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

            if(joystickY < 50){
                servoSpeed +=1;
                if(servoSpeed > speedLimit){
                    servoSpeed = speedLimit;
                }
                Timer_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, servoSpeed);
            } else if(joystickY > 16000){
                servoSpeed -=1;
                if(servoSpeed < NO_SPEED){
                    servoSpeed = NO_SPEED;
                }
                Timer_A_setCompareValue(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, servoSpeed);
            }

        }

        if(currentState == 0){
            normalState();
        }else if(currentState == 1){
           // hotEngine();
        }else if(currentState == 2){
           // lowSpeed();
        }else{
            currentState = 0;
        }

  //      MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
        //MAP_PCM_gotoLPM0InterruptSafe();
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
        digitalTempValue = MAP_ADC14_getResult(ADC_MEM3);
        celsiusTempValue = 100 * (digitalTempValue * 3.3) / 16384;
        fahrenheitTempValue = celsiusTempValue*(9/5) +32;
    }

}

void EUSCIB0_IRQHandler(void)
{
    uint_fast16_t status;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B0_BASE);
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);

    /* Receives bytes into the receive buffer. If we have received all bytes,
     * send a STOP condition */
    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {

        if (xferIndex == NUM_OF_REC_BYTES - 2)
        {
            MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_STOP_INTERRUPT);

            /*
             * Switch order so that stop is being set during reception of last
             * byte read byte so that next byte can be read.
             */
            MAP_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
            RXData[xferIndex++] = MAP_I2C_masterReceiveMultiByteNext(
            EUSCI_B0_BASE);
        }
        else if (xferIndex < NUM_OF_REC_BYTES)
        {
            RXData[xferIndex++] = MAP_I2C_masterReceiveMultiByteNext(
            EUSCI_B0_BASE);
        }
    }
    else if (status & EUSCI_B_I2C_STOP_INTERRUPT)
    {
        MAP_Interrupt_disableSleepOnIsrExit();
        MAP_I2C_disableInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_STOP_INTERRUPT);
    }

    if(currentSlave == true){
        currentSlave = false;
        MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_1);
    } else{
        currentSlave = true;
        MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_0);
    }
}

//triggers sampling and writing once per second
void T32_INT1_IRQHandler(void)
{
    uint64_t status = Timer32_getInterruptStatus(TIMER32_0_BASE);
    write = true;
    MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);

}

void PORT1_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    int i=0;
    for(i =0; i < 15000; i++);

    //change flags
    if (status & GPIO_PIN1)
    {
        dataState--;
        if(dataState<0){
            dataState = 0;
        }
    }

    if (status & GPIO_PIN4)
    {
        dataState++;
        if(dataState > 5){
            dataState ==5;
        }
    }

}
