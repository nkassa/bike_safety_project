/*

Created by:
Team #4 (Natan K., Sujit P., and Franco C.)
Spring 2024
SignalCycle Solutions

*/


#include "lcd.h"
#include <avr/io.h>
#include <util/delay.h>

// LCD control signals
#define LCD_RS PC5
#define LCD_EN PC4

// LCD data signals
#define LCD_D4 PD4
#define LCD_D5 PD5
#define LCD_D6 PD6
#define LCD_D7 PD7


void lcd_writenibble(unsigned char);

#define DATA_BITS ((1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7))
#define CTRL_BITS ((1 << LCD_RS) | (1 << LCD_EN))


void lcd_setup(void) {

	// Set data pins as outputs
    DDRD |= DATA_BITS;
    // Set control pins as outputs
    DDRC |= CTRL_BITS;
    _delay_ms(15);

	lcd_writenibble(0x30);      // Use lcd_writenibble to send 0b0011
    _delay_ms(5);               // Delay at least 4msec

    lcd_writenibble(0x30);      // Use lcd_writenibble to send 0b0011
    _delay_us(120);             // Delay at least 100usec

    lcd_writenibble(0x30);      // Use lcd_writenibble to send 0b0011, no delay needed

    lcd_writenibble(0x20);      // Use lcd_writenibble to send 0b0010
    _delay_ms(2);               // Delay at least 2ms
    
    lcd_command(0x28);     // Function Set: 4-bit interface, 2 lines

    lcd_command(0x0f);     // Display and cursor on

}

void lcd_moveto(unsigned char row, unsigned char col)
{
    unsigned char pos;
    if(row == 0) {
        pos = 0x80 + col;       // 1st row locations start at 0x80
    }
    else {
        pos = 0xc0 + col;       // 2nd row locations start at 0xc0
    }
    lcd_command(pos);      // Send command
}

void lcd_print(char *message) {
    int i = 0;
    while (message[i] != '\0') {    // Loop until next charater is NULL byte
        lcd_data(message[i]);  // Send the character
        i++;
    }
}

void lcd_command(unsigned char command) {

    // RS low for command mode
    PORTC &= ~(1 << LCD_RS);
    
    lcd_writenibble(command);
    
    lcd_writenibble((command << 4));
    
    _delay_ms(2);
}

void lcd_data(unsigned char data) {

    // RS high for data mode
    PORTC |= (1 << LCD_RS);
    
    lcd_writenibble(data);
    
    lcd_writenibble((data << 4));
    
    _delay_ms(2);
    

}

void lcd_writenibble(unsigned char lcdbits)
{
    /* Load PORTD, bits 7-4 with bits 7-4 of "lcdbits" */
    
    PORTD &= ~DATA_BITS;
    PORTD |= ( lcdbits & DATA_BITS);
	
    /* Make E signal (PB1) go to 1 and back to 0 */
    PORTC |= (1 << LCD_EN);
    PORTC |= (1 << LCD_EN);
    PORTC &= ~(1 << LCD_EN);
    
}

