/*
 * File:   newmain.c
 * Author: ext_mmt
 *
 * Created on 2025/06/11, 14:38
 */

#define F_CPU 3333333UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <avr/wdt.h>

#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
#define MAX_INPUT_LEN 57
#define CONTINUE_SENDING_FLAG_ADDR 0
#define EEPROM_DATA_START_ADDR 1 
#define CRC8_POLYNOMIAL 0x31

void USART0_init(void);
void BOD_init(void);
void WDT_init(void);
void USART0_sendChar(char c);
void USART0_sendString(char *str);
void USART0_sendHexByte(uint8_t byte);
char USART0_readCharLow(void);
char USART0_readCharHigh(void);
void readEEPROMAndSend(void);
void readEEPROMAndVerifyCRC(void);
void readEEPROMAndSendLOG(void);
void initializeEEPROM(void);
void writeToEEPROM(char *input);
void setContinueSendingState(bool state);
bool getContinueSendingState(void);
void sendNumber(uint8_t number);
void logEEPROMOperation(uint8_t address, char value);
void enterSleepMode(void);
void enterPowerDown(void);
uint8_t crc8(const uint8_t *data, size_t length);
void stopAllFunctions(void);
void sendChar(char *input);
void executeCallsign(char *input);

void USART0_init(void)
{
    cli();
    PORTB.OUT |= PIN2_bm;
    PORTB.DIR &= ~PIN3_bm; // RX
    PORTB.DIR |= PIN2_bm;  // TX 
    USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600);
    USART0.CTRLC |= USART_PMODE_ODD_gc;
    USART0.CTRLB |= USART_TXEN_bm | USART_RXEN_bm; 
    USART0.CTRLB |= USART_SFDEN_bm; 
    USART0.CTRLA |= USART_RXCIE_bm;
    CCL.CTRLA |= CCL_RUNSTDBY_bm;
    sei();
}

void BOD_init(void)
{
    BOD.CTRLA = BOD_ACTIVE_ENABLED_gc; 
    BOD.CTRLB = BOD_LVL_BODLEVEL7_gc;
    
    BOD.VLMCTRLA = BOD_VLMLVL_5ABOVE_gc; 
    
    BOD.INTCTRL |= BOD_VLMIE_bm;
}

void WDT_init(void)
{
    _PROTECTED_WRITE(WDT.CTRLA, 0x00);
    _PROTECTED_WRITE(WDT.CTRLA, 0x8A);
}

void USART0_sendChar(char c)
{
    while (!(USART0.STATUS & USART_DREIF_bm))
    {
        ;
    }        
    USART0.TXDATAL = c; // Send character
}

void USART0_sendString(char *str)
{
    for(size_t i = 0; i < strlen(str); i++)
    {
        USART0_sendChar(str[i]);
    }
}

void USART0_sendHexByte(uint8_t byte)
{
    const char hex[] = "0123456789ABCDEF";
    USART0_sendChar(hex[(byte >> 4) & 0x0F]);
    USART0_sendChar(hex[byte & 0x0F]);
}

char USART0_readCharLow(void)
{
    while (!(USART0.STATUS & USART_RXCIF_bm))
    {
        ;
    }
    return USART0.RXDATAL; // Read received character
}

char USART0_readCharHigh(void)
{
    while (!(USART0.STATUS & USART_RXCIF_bm))
    {
        ;
    }
    return USART0.RXDATAH;
}

void readEEPROMAndSend(void)
{
    char eepromData[EEPROM_SIZE];
    eeprom_read_block(eepromData, (const void *)EEPROM_DATA_START_ADDR, EEPROM_SIZE - EEPROM_DATA_START_ADDR);
    for (uint8_t i = 0; i < EEPROM_SIZE - EEPROM_DATA_START_ADDR; i++)
    {
        if (eepromData[i] == '\0') // Stop if null character is found
        {
            break;
        }
        USART0_sendChar(eepromData[i]);
    }
    uint8_t crc = crc8((const uint8_t *)eepromData, strlen(eepromData));
    USART0_sendString(" CRC: ");
    USART0_sendHexByte(crc);
    USART0_sendChar('\n'); // Send a newline after sending all characters
}

void readEEPROMAndVerifyCRC(void) 
{
    char eepromData[EEPROM_SIZE];
    eeprom_read_block(eepromData, (const void *)EEPROM_DATA_START_ADDR, EEPROM_SIZE - EEPROM_DATA_START_ADDR);
    
    uint8_t stored_crc = eeprom_read_byte((const void *)(EEPROM_DATA_START_ADDR + strlen(eepromData) + 1));
    
    uint8_t calculated_crc = crc8((const uint8_t *)eepromData, strlen(eepromData));
    
    if (calculated_crc == stored_crc)
    {
        USART0_sendString("Data is valid. \n");
    }
    else 
    {
        USART0_sendString("Error: CRC mismatch! \n");
    }
}

void readEEPROMAndSendLOG(void)
{
    char eepromData[EEPROM_SIZE];
    eeprom_read_block(eepromData, (const void *)EEPROM_DATA_START_ADDR, EEPROM_SIZE - EEPROM_DATA_START_ADDR);
    for (uint8_t i = 0; i < EEPROM_SIZE - EEPROM_DATA_START_ADDR; i++)
    {
        if (eepromData[i] == '\0') // Stop if null character is found
        {
            break;
        }
        logEEPROMOperation(EEPROM_DATA_START_ADDR + i, eepromData[i]);
    }
    USART0_sendChar('\n'); // Send a newline after sending all characters
}

void initializeEEPROM(void)
{
    char emptyData[EEPROM_SIZE] = {0}; // Initialize with null characters
    eeprom_write_block(emptyData + EEPROM_DATA_START_ADDR, (void *)EEPROM_DATA_START_ADDR, EEPROM_SIZE - EEPROM_DATA_START_ADDR); // Clear EEPROM
    setContinueSendingState(false); // Reset sending state
}

void writeToEEPROM(char *input)
{
    eeprom_write_block(input, (void *)EEPROM_DATA_START_ADDR, strlen(input) + 1); 
    
    uint8_t crc = crc8((const uint8_t *)input, strlen(input));
    
    eeprom_write_byte((void *)(EEPROM_DATA_START_ADDR + strlen(input) + 1), crc);
}

void setContinueSendingState(bool state)
{
    eeprom_write_byte((uint8_t *)CONTINUE_SENDING_FLAG_ADDR, state ? 1 : 0); 
}

bool getContinueSendingState(void)
{
    return eeprom_read_byte((const uint8_t *)CONTINUE_SENDING_FLAG_ADDR) == 1; 
}

void sendNumber(uint8_t number)
{
    if (number >= 10)
    {
        sendNumber(number / 10);
    }
    USART0_sendChar((number % 10) + '0');
}

void logEEPROMOperation(uint8_t address, char value)
{
    USART0_sendString("EEPROM[");
    sendNumber(address);
    USART0_sendString("]; ");
    USART0_sendChar(value);
    USART0_sendChar('\n');
}

void enterSleepMode(void)
{
    SLPCTRL.CTRLA = SLEEP_MODE_STANDBY;
    SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}

void enterPowerDown(void)
{
    SLPCTRL.CTRLA = SLEEP_MODE_PWR_DOWN;
    SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}

uint8_t crc8(const uint8_t *data, size_t length) 
{
    uint8_t crc = 0x00; 
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            }
            else 
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void stopAllFunctions(void)
{
    USART0.CTRLB &= ~(USART_TXEN_bm | USART_RXEN_bm);
    USART0.CTRLA &= ~USART_RXCIE_bm; 
    
    PORTB.OUT &= ~PIN2_bm;
    PORTB.DIR &= ~PIN3_bm;
    
    SLPCTRL.CTRLA = SLEEP_MODE_PWR_DOWN;
    SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;
    sleep_enable();
    sleep_cpu();
}

ISR(BOD_VLM_vect)
{
    USART0_sendString("Shutting down; insufficient voltage detected. Please reconnect with 5V VCC.");
    _delay_ms(500);
    stopAllFunctions();
}

void sendChar(char *input)
{
    if (strcmp(input, "info") == 0)
    {
        readEEPROMAndSend();
        readEEPROMAndVerifyCRC();
        _delay_ms(500);
    } 
    else if (strcmp(input, "MODE") == 0)
    {
        USART0_sendString("Customer Mode\n");
        _delay_ms(500);
    }
    else if (strcmp(input, "RESET") == 0)
    {
        setContinueSendingState(false); 
        USART0_sendString("Reset to factory mode. Disconnect and reconnect, then refer to documentation for instructions. \n");
        _delay_ms(500);
    }
    else if (strcmp(input, "LOG") == 0)
    {
        USART0_sendString("Current bytes & respective positions: \n");
        readEEPROMAndSendLOG();
        _delay_ms(500);
    }
    else if (strcmp(input, "OFF") == 0)
    {
        USART0_sendString("MCU disabled.\n");
        _delay_ms(500);
        enterPowerDown();
    }
    else if (strlen(input) < 6)
    {
        USART0_sendString("Error: Invalid Command\n");
        _delay_ms(500);
    }
    else 
    {
        USART0_sendString("Error: Length error\n");
        _delay_ms(500);
    }
}

void executeCallsign(char *input)
{
    if(strcmp(input, "INIT") == 0)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            readEEPROMAndSend();
            readEEPROMAndVerifyCRC();
            _delay_ms(500);
        }
    }
    else if (strcmp(input, "MODE") == 0)
    {
        USART0_sendString("Factory Mode\n");
        _delay_ms(500);
    }
    else if (strcmp(input, "FINAL") == 0)
    {
        setContinueSendingState(true);
        USART0_sendString("Entered customer mode. Disconnect and reconnect, then input 'info' for shunt characteristics. \n");
        _delay_ms(500);
    }
    else if (strcmp(input, "LOG") == 0)
    {
        USART0_sendString("Current bytes & respective positions: \n");
        readEEPROMAndSendLOG();
        _delay_ms(500);
    }
    else if (strlen(input) == 56)
    {
        USART0_sendString(input);
        USART0_sendChar('\n');
        writeToEEPROM(input);
        _delay_ms(500);
    }
    else if (strlen(input) < 6)
    {
        USART0_sendString("Error: Invalid Command\n");
        _delay_ms(500);
    }
    else 
    {
        USART0_sendString("Error: Length error\n");
        _delay_ms(500);
    }
}

int main(void)
{
    char input[MAX_INPUT_LEN];
    uint8_t index = 0;
    char c;
    
    BOD_init();
    USART0_init();
    WDT_init();
    
    if (eeprom_read_byte((const uint8_t *)CONTINUE_SENDING_FLAG_ADDR) == 0xFF) 
    {
        initializeEEPROM(); 
    }
    
    if (getContinueSendingState())
    {
        while (1)
        {
            wdt_reset();
            enterSleepMode();
            
            uint8_t meta = USART0_readCharHigh();
            c = USART0_readCharLow();
            
            
            if (meta & USART_PERR_bm)
            {
                USART0_sendString("USART Error: Parity\n");
                break;
            }
            
            if (meta & USART_FERR_bm)
            {
                USART0_sendString("USART Error: Frame\n");
                break;
            }
            
            
            if (c != '\n' && c != '\r')
            {
                if (index < MAX_INPUT_LEN - 1)
                {
                    input[index++] = c;
                }
                else 
                {
                    index = 0; // Reset index if input exceeds max length
                }
            }
            if (c == '\n')
            {
                input[index] = '\0'; 
                index = 0; 
                sendChar(input);
            }
        }
    }
    
    while (1) 
    {
        wdt_reset();
        enterSleepMode();
        
        uint8_t meta = USART0_readCharHigh();
        c = USART0_readCharLow();
        
        if (meta & USART_PERR_bm)
        {
            USART0_sendString("USART Error: Parity\n");
            break;
        }
         
        if (meta & USART_FERR_bm)
        {
            USART0_sendString("USART Error: Frame\n");
            break;
        }
        
        if (c != '\n' && c != '\r')
        {
            if (index < MAX_INPUT_LEN - 1)
            {
                input[index++] = c;
            }
            else 
            {
                index = 0; // Reset index if input exceeds max length
            }
        }
        if (c == '\n')
        {
            input[index] = '\0'; 
            index = 0; 
            executeCallsign(input);
        }
    }
    
    return 0;
}

