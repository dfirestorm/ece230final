/*!
 * lcd.c
 *
 *      Description: Helper file for LCD library. For Hitachi HD44780 parallel LCD
 *               in 8-bit mode.
 *
 *      Author: ece230
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "lcd.h"
#include "delays.h"

#define NONHOME_MASK        0xFC

#define LONG_INSTR_DELAY    2000
#define SHORT_INSTR_DELAY   50

uint_fast8_t RS_Port, EN_Port, DB_Port;
uint_fast16_t RS_Pin, EN_Pin;

void configLCD(uint_fast8_t rsPort, uint_fast16_t rsPin,
                    uint_fast8_t enPort, uint_fast16_t enPin,
                    uint_fast8_t dbPort) {
    GPIO_setOutputLowOnPin(enPort, enPin);

    GPIO_setAsOutputPin(rsPort, rsPin);
    GPIO_setAsOutputPin(enPort, enPin);
    GPIO_setAsOutputPin(dbPort, PIN_ALL8);

    RS_Port = rsPort;
    EN_Port = enPort;
    DB_Port = dbPort;
    RS_Pin = rsPin;
    EN_Pin = enPin;
}

/*!
 * Delay method based on instruction execution time.
 *   Execution times from Table 6 of HD44780 data sheet, with buffer.
 *
 * \param mode RS mode selection
 * \param instruction Instruction/data to write to LCD
 *
 * \return None
 */
void instructionDelay(uint8_t mode, uint8_t instruction) {
    if ((mode == DATA_MODE) || (instruction & NONHOME_MASK)) {
        delayMicroSec(SHORT_INSTR_DELAY);
    }
    else {
        delayMicroSec(LONG_INSTR_DELAY);
    }
}

/*!
 * Function to write instruction/data to LCD.
 *
 * \param mode          Write mode: 0 - control, 1 - data
 * \param instruction   Instruction/data to write to LCD
 *
 * \return None
 */
void writeInstruction(uint8_t mode, uint8_t instruction) {
    GPIO_setOutputLowOnPin(DB_Port, PIN_ALL8);
    if (mode == DATA_MODE) {
        GPIO_setOutputHighOnPin(RS_Port, RS_Pin);
    } else {
        GPIO_setOutputLowOnPin(RS_Port, RS_Pin);
    }
    GPIO_setOutputHighOnPin(EN_Port, EN_Pin);
    GPIO_setOutputHighOnPin(DB_Port, instruction);
    delayMicroSec(1);
    GPIO_setOutputLowOnPin(EN_Port, EN_Pin);
    instructionDelay(mode, instruction);
}

/*!
 * Function to write command instruction to LCD.
 *
 * \param command Command instruction to write to LCD
 *
 * \return None
 */
void commandInstruction(uint8_t command) {
    writeInstruction(CTRL_MODE, command);
}

void commandInstruction4bit(uint8_t command){
    writeInstruction(CTRL_MODE, command);
    delayMicroSec(5);
    writeInstruction(CTRL_MODE, command << 4);
}
/*!
 * Function to write data instruction to LCD.
 *
 * \param data ASCII value/data to write to LCD
 *
 * \return None
 */
void dataInstruction(uint8_t data) {
    writeInstruction(DATA_MODE, data);
    delayMicroSec(5);
    writeInstruction(DATA_MODE, data << 4);
}

void initLCD(void) {
    // DONE complete command instructions for initialization
    delayMilliSec(40);
    commandInstruction(FUNCTION_SET_MASK | DL_FLAG_MASK);
    delayMilliSec(5);
    commandInstruction(FUNCTION_SET_MASK | DL_FLAG_MASK);
    delayMicroSec(150);
    commandInstruction(FUNCTION_SET_MASK | DL_FLAG_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction(FUNCTION_SET_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction4bit(FUNCTION_SET_MASK | N_FLAG_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction4bit(DISPLAY_CTRL_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction4bit(CLEAR_DISPLAY_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction4bit(ENTRY_MODE_MASK | ID_FLAG_MASK);
    // Initialization complete, turn ON display
    delayMicroSec(LONG_INSTR_DELAY);
    commandInstruction4bit(DISPLAY_CTRL_MASK | D_FLAG_MASK);
}

void printChar(char character) {
    dataInstruction(character);
}
void printString(char string[], int length){
    int i;
    for(i=0; i<length; i++){
        if(string[i] != 0){
            printChar(string[i]);
        }
        else{
            printChar(' ');
        }
    }
}
void clearScreen(){
    commandInstruction4bit(CLEAR_DISPLAY_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction4bit(ENTRY_MODE_MASK | ID_FLAG_MASK);
    // Initialization complete, turn ON display
    delayMicroSec(LONG_INSTR_DELAY);
    commandInstruction4bit(DISPLAY_CTRL_MASK | D_FLAG_MASK);
}
void displayStart(){
    commandInstruction4bit(0x02);
}
void displayLine2(){
    commandInstruction4bit(0xc0);
}
