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

const uint8_t crc_table[256] = {
0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53,   0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,   0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4,   0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19,   0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,   0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D,   0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7,   0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,   0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75,   0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8,   0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,   0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F,   0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66,   0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,   0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1,   0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C,   0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

void USART0_init(void);
void BOD_init(void);
void WDT_init(void);
void USART0_sendChar(char c);
void USART0_sendString(char *str);
void USART0_sendHexByte(uint8_t byte);
char USART0_readCharLow(void);
char USART0_readCharHigh(void);
void readEEPROMAndSend(void);
void readEEPROMAndSendLOG(void);
void initializeEEPROM(void);
void writeToEEPROM(char *input);
void setContinueSendingState(bool state);
bool getContinueSendingState(void);
void sendNumber(uint8_t number);
void logEEPROMOperation(uint8_t address, char value);
void enterSleepMode(void);
void enterPowerDown(void);
uint8_t crc8_table_calc(const uint8_t *data, size_t length);
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
    char eepromData[EEPROM_SIZE - EEPROM_DATA_START_ADDR];
    uint8_t len = 0;
    
    eeprom_read_block(eepromData, (const void *)EEPROM_DATA_START_ADDR, sizeof(eepromData));
    
    for (uint8_t i = 0; i < sizeof(eepromData); i++)
    {
        if (eepromData[i] == '\0') // Stop if null character is found
        {
            break;
        }
        USART0_sendChar(eepromData[i]);
        len++;
    }
    
    uint8_t crc = crc8_table_calc((const uint8_t *)eepromData, len);
    
    USART0_sendString(" CRC: ");
    USART0_sendHexByte(crc);
    USART0_sendChar('\n'); // Send a newline after sending all characters
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
    size_t len = strlen(input);
    
    if (len > 0 && input[len - 1] == '\n')
    {
        input[len - 1] = '\0';
        len--;
    }
    
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

uint8_t crc8_table_calc(const uint8_t *data, size_t length)
{
    uint8_t crc = 0xFF;
    
    for (size_t i = 0; i < length; i++)
    {
        crc = crc_table[crc ^ data[i]];
    }
    
    return crc ^ 0xFF;
}

uint8_t crc8(const uint8_t *data, size_t length) 
{
    uint8_t crc = 0xFF;
    const uint8_t poly = 0x1D;
    
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ poly;
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

