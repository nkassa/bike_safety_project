/*

Created by:
Team #4 (Natan K., Sujit P., and Franco C.)
Spring 2024
SignalCycle Solutions

*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include <stdlib.h>



#define TRIG_PIN PC3
#define ECHO_PIN_1 PC2
#define ECHO_PIN_2 PB0
#define ECHO_PIN_3 PB7
#define ECHO_PIN_4 PC0


#define DECODE_MODE 0x09
#define INTENSITY 0x0A
#define SCAN_LIMIT 0x0B
#define SHUTDOWN 0x0C
#define DISPLAY_TEST 0x0F


// encoder variables 
volatile long encoder0Pos = 0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
long vel;
volatile int count = 0;
volatile char old_state = 0;
volatile char new_state = 0;
volatile char changed = 0;
#define DISTANCE_PER_COUNT  2.042 // Distance per count of the encoder (in inches)


//flag 
int x = 0;
int safe_left = 0;
int safe_right = 0;


// Function prototype
void send2bytes(uint8_t, uint8_t); 
void millis_init();
unsigned long millis_time();
void sendPulse(void);


int main(void)
{

    // turn signal initialization 
    uint8_t reg, value, i;
    uint8_t leds_on = 0; // Variable to track LED state (0 for off, 1 for on)
    // Button setup
    DDRD &= ~(1 << PD0) | ~(1 << PD3) | ~(1 << PD1) | ~(1 << PD2); // Set PD0 and PD3 as inputs (push buttons)
    PORTD |= (1 << PD0) | (1 << PD3) | (1 << PD1) | (1 << PD2); // Enable pull-up resistors on PD0 and PD3
    // SPI pins setup
    DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2) | (1 << PB1); // Set MOSI, SCLK, SS (CS), and LOAD for output
    PORTB |= (1 << PB2) | (1 << PB1); // Pull SS (CS) pins high initially (inactive)
    // Enable SPI, Master mode, and set clock rate
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);



    // Set TRIG pin as output
    DDRC |= (1 << TRIG_PIN);
    // Set ECHO pin as input
    DDRC &= ~(1 << ECHO_PIN_1);
    // Set ECHO pin as input
    DDRB &= ~(1 << ECHO_PIN_2);
	// Set ECHO pin as input
    DDRB &= ~(1 << ECHO_PIN_3);
    // Set ECHO pin as input
    DDRC &= ~(1 << ECHO_PIN_4);
    
    // rotary encoder initialization 
    // Set PC1 as input
    DDRC &= ~(1<<1);
    // Enable pull-up resistor on PC1
    PORTC |= (1<<1);
    // Enable Pin Change Interrupt for PCINT8-PCINT14 (PCIE1 for ATmega328P or similar)
    PCICR |= (1<<PCIE1);
    // Enable Pin Change Interrupt for PC1
    PCMSK1 |= (1<<PCINT9); // Adjust the mask for PC1 specifically
    // Globally enable interrupts
    sei();
    // Initialize millis function for time tracking
    millis_init();


	// max7219 clearing
    // Clear the array to all off
    for (i = 0; i < 16; i++) {
        reg = i + 1;
        send2bytes(reg, 0);
    }
    
    
    _delay_ms(500); // Initial delay to stabilize
    
    
    //LCD setup and initialization 
    lcd_setup();
    //Write splash screen
    lcd_command(0x01); //clear screen
    _delay_ms(1000);
    lcd_moveto(0,2); 
    lcd_print("Hello Rider");
    lcd_moveto(1,2);
    lcd_print("Ready To Go!");
    _delay_ms(20000); //Allows splash screen to display for 1 sec
    lcd_command(0x01); //clear screen
    lcd_moveto(0, 6); // Move cursor to the desired position on the LCD
    
    while (1)
    {
    
    	// print speed on LCD
        char buffer[10]; // Buffer to store the speed as a string
		encoder0Pos = count;
        newposition = encoder0Pos;
        newtime = millis_time();
        vel = (newposition - oldposition) * DISTANCE_PER_COUNT * 1000 / (newtime - oldtime);
        oldposition = newposition;
        oldtime = newtime;
        itoa(vel, buffer, 10);
        
        // deal with how LCD prints out speed
        int numDigits = strlen(buffer); // Get the number of digits in the speed
		if(numDigits == 1)
		{
			lcd_moveto(0, 6);
			// Print the speed
			lcd_print(" ");
			lcd_moveto(0, 7);
			lcd_print(buffer);
			_delay_ms(1000);
		}
		else
		{
			lcd_moveto(0, 6);
			// Print the speed
			lcd_print(buffer);
			_delay_ms(1000);
		}
		_delay_ms(2000); 
        
        
        
        // Check if the button is pressed
        if (!(PIND & (1 << PD0)))
        {
            _delay_ms(50); // Debounce delay
            if (!(PIND & (1 << PD0)))
            { // Confirm button press
                // Toggle LED state
                leds_on = !leds_on;
                x = 1;
                if (leds_on) {
                    // Initialize LED module for display sequence
                    send2bytes(DISPLAY_TEST, 0); // Turn off display test mode

                    send2bytes(DECODE_MODE, 0x00); // Decode-Mode set for no decode
                    send2bytes(INTENSITY, 0x08);   // Use medium intensity
                    send2bytes(SCAN_LIMIT, 0x0f);  // Scan-Limit set for all 8 digits
                    send2bytes(SHUTDOWN, 0x01);    // Shutdown set for normal operation

                    while(leds_on) {
                        // Turn on LEDs
                        for (i = 0; i < 8; i++)
                        {
                            reg = i + 1;
                            if (i == 7)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 6)
                            {
                                value = 0b00111100; // Both corners are lit
                            }
                            else if (i == 5)
                            {
                                value = 0b01111110; // Both corners are lit
                            }
                            else if (i == 4)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 3)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 2)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 1)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else
                            {
                                value = 0b00011000; // No other LEDs are lit
                            }
                            send2bytes(reg, value);
                        }
                        _delay_ms(2000); // Blink interval
                        // Turn off LEDs
                        for (i = 0; i < 8; i++)
                        {
                            reg = i + 1;
                            send2bytes(reg, 0);
                        }
                        _delay_ms(2000); // Blink interval
                        // Check if the button is pressed to turn off the LEDs
                        if (!(PIND & (1 << PD0))) {
                            _delay_ms(50); // Debounce delay
                            if (!(PIND & (1 << PD0))) 
                            {
                            	leds_on = 0; // Turn off LEDs
                                x = 0;
                            }
                        }
                    }
                } else {                    
                	// Turn off LEDs
                    for (i = 0; i < 8; i++)
                    {
                        reg = i + 1;
                        send2bytes(reg, 0);
                    }
                }

                // Wait for button release
                while (!(PIND & (1 << PD0)));
            }
        }

        // Check if the button is pressed
        if (!(PIND & (1 << PD3)))
        {
            _delay_ms(50); // Debounce delay
            if (!(PIND & (1 << PD3)))
            { // Confirm button press
                // Toggle LED state
                leds_on = !leds_on;
                x = 2;
                if (leds_on) {
                    // Initialize LED module for display sequence
                    send2bytes(DISPLAY_TEST, 0); // Turn off display test mode

                    send2bytes(DECODE_MODE, 0x00); // Decode-Mode set for no decode
                    send2bytes(INTENSITY, 0x08);   // Use medium intensity
                    send2bytes(SCAN_LIMIT, 0x0f);  // Scan-Limit set for all 8 digits
                    send2bytes(SHUTDOWN, 0x01);    // Shutdown set for normal operation

                    while(leds_on) {
                        // Turn on LEDs
                        for (i = 0; i < 8; i++)
                        {
                            reg = i + 1;
                            if (i == 0 )
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 1)
                            {
                                value = 0b00111100; // Both corners are lit
                            }
                            else if (i == 2)
                            {
                                value = 0b01111110; // Both corners are lit
                            }
                            else if (i == 3)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 4)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 5)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 6)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else
                            {
                                value = 0b00011000; // No other LEDs are lit
                            }
                            send2bytes(reg, value);
                        }
                        _delay_ms(2000); // Blink interval
                        // Turn off LEDs
                        for (i = 0; i < 8; i++)
                        {
                            reg = i + 1;
                            send2bytes(reg, 0);
                        }
                        _delay_ms(2000); // Blink interval
                        // Check if the button is pressed to turn off the LEDs
                        if (!(PIND & (1 << PD3))) {
                            _delay_ms(50); // Debounce delay
                            if (!(PIND & (1 << PD3))) {
                                leds_on = 0; // Turn off LEDs
                                x = 0;
                            }
                        }
                    }
                } else {
                    // Turn off LEDs
                    for (i = 0; i < 8; i++)
                    {
                        reg = i + 1;
                        send2bytes(reg, 0);
                    }
                }

                // Wait for button release
                while (!(PIND & (1 << PD3)));
            }
        }
        // Check if the button is pressed
        if (!(PIND & (1 << PD1)))
        {
            _delay_ms(50); // Debounce delay
            if (!(PIND & (1 << PD1)))
            { // Confirm button press
                // Toggle LED state
                leds_on = !leds_on;
                x = 1;
                if (leds_on) {
                    // Initialize LED module for display sequence
                    send2bytes(DISPLAY_TEST, 0); // Turn off display test mode

                    send2bytes(DECODE_MODE, 0x00); // Decode-Mode set for no decode
                    send2bytes(INTENSITY, 0x08);   // Use medium intensity
                    send2bytes(SCAN_LIMIT, 0x0f);  // Scan-Limit set for all 8 digits
                    send2bytes(SHUTDOWN, 0x01);    // Shutdown set for normal operation

                    while(leds_on && safe_left) {
                        // Turn on LEDs
                        for (i = 0; i < 8; i++)
                        {
                            reg = i + 1;
                            if (i == 7)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 6)
                            {
                                value = 0b00111100; // Both corners are lit
                            }
                            else if (i == 5)
                            {
                                value = 0b01111110; // Both corners are lit
                            }
                            else if (i == 4)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 3)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 2)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 1)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else
                            {
                                value = 0b00011000; // No other LEDs are lit
                            }
                            send2bytes(reg, value);
                        }
                        _delay_ms(2000); // Blink interval
                        // Turn off LEDs
                        for (i = 0; i < 8; i++)
                        {
                            reg = i + 1;
                            send2bytes(reg, 0);
                        }
                        _delay_ms(2000); // Blink interval
                        // Check if the button is pressed to turn off the LEDs
                        if (!(PIND & (1 << PD1))) {
                            _delay_ms(50); // Debounce delay
                            if (!(PIND & (1 << PD1))) 
                            {
                            	leds_on = 0; // Turn off LEDs
                                x = 0;
                            }
                        }
                    }
                } else {                    
                	// Turn off LEDs
                    for (i = 0; i < 8; i++)
                    {
                        reg = i + 1;
                        send2bytes(reg, 0);
                    }
                }

                // Wait for button release
                while (!(PIND & (1 << PD1)));
            }
        }
        
		// Check if the button is pressed
        if (!(PIND & (1 << PD2)))
        {
            _delay_ms(50); // Debounce delay
            if (!(PIND & (1 << PD2)))
            { // Confirm button press
                // Toggle LED state
                leds_on = !leds_on;
                x = 2;
                if (leds_on) {
                    // Initialize LED module for display sequence
                    send2bytes(DISPLAY_TEST, 0); // Turn off display test mode

                    send2bytes(DECODE_MODE, 0x00); // Decode-Mode set for no decode
                    send2bytes(INTENSITY, 0x08);   // Use medium intensity
                    send2bytes(SCAN_LIMIT, 0x0f);  // Scan-Limit set for all 8 digits
                    send2bytes(SHUTDOWN, 0x01);    // Shutdown set for normal operation

                    while(leds_on && safe_right) {
                        // Turn on LEDs
                        for (i = 0; i < 8; i++)
                        {
                            reg = i + 1;
                            if (i == 0 )
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 1)
                            {
                                value = 0b00111100; // Both corners are lit
                            }
                            else if (i == 2)
                            {
                                value = 0b01111110; // Both corners are lit
                            }
                            else if (i == 3)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 4)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 5)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else if (i == 6)
                            {
                                value = 0b00011000; // Both corners are lit
                            }
                            else
                            {
                                value = 0b00011000; // No other LEDs are lit
                            }
                            send2bytes(reg, value);
                        }
                        _delay_ms(2000); // Blink interval
                        // Turn off LEDs
                        for (i = 0; i < 8; i++)
                        {
                            reg = i + 1;
                            send2bytes(reg, 0);
                        }
                        _delay_ms(2000); // Blink interval
                        // Check if the button is pressed to turn off the LEDs
                        if (!(PIND & (1 << PD2))) {
                            _delay_ms(50); // Debounce delay
                            if (!(PIND & (1 << PD2))) {
                                leds_on = 0; // Turn off LEDs
                                x = 0;
                            }
                        }
                    }
                } else {
                    // Turn off LEDs
                    for (i = 0; i < 8; i++)
                    {
                        reg = i + 1;
                        send2bytes(reg, 0);
                    }
                }

                // Wait for button release
                while (!(PIND & (1 << PD2)));
            }
        }
		sendPulse();
        // Measure pulse duration_1
        int duration_1 = 0;
        while (!(PINC & (1 << ECHO_PIN_1)));
        while (PINC & (1 << ECHO_PIN_1))
        {
            _delay_us(1);
            duration_1++;
        }
        
        sendPulse();
        // Measure pulse duration_2
        int duration_2 = 0;
        while (!(PINB & (1 << ECHO_PIN_2)));
        while (PINB & (1 << ECHO_PIN_2))
        {
            _delay_us(1);
            duration_2++;
        }
        
        sendPulse();
        // Measure pulse duration_3
        int duration_3 = 0;
        while (!(PINB & (1 << ECHO_PIN_3)));
        while (PINB & (1 << ECHO_PIN_3))
        {
            _delay_us(1);
            duration_3++;
        }
        
        sendPulse();
        // Measure pulse duration_4
        int duration_4 = 0;
        while (!(PINC & (1 << ECHO_PIN_4)));
        while (PINC & (1 << ECHO_PIN_4))
        {
            _delay_us(1);
            duration_4++;
        }

        // Convert duration_1 to distance
        int distance_1 = (duration_1 / 2) / 29.1;
        
        //Convert duration_2 to distance
        int distance_2 = (duration_2 / 2) / 29.1;
        
        //Convert duration_2 to distance
        int distance_3 = (duration_3 / 2) / 29.1;
        
        //Convert duration_2 to distance
        int distance_4 = (duration_4 / 2) / 29.1;

        // If distance is less than or equal to 30 cm, turn on LED
        if ((distance_1 <= 2 || distance_2 <= 2 ) && (distance_3 <= 2 || distance_4 <= 2))
        {
        	safe_right = 0;
        	safe_left = 0;
            lcd_moveto(1, 1);
            // deal with how LCD prints out speed
            lcd_print("Both Not Safe");
            _delay_ms(2000);
        }
        // If distance is less than or equal to 30 cm, turn on LED
        else if (distance_1 <= 2 || distance_2 <= 2 )
        {
        	safe_right = 0;
        	safe_left = 1;
            lcd_moveto(1, 1);
            // deal with how LCD prints out speed
            lcd_print("Right Not Safe");
            _delay_ms(2000);
        }
        // If distance is less than or equal to 30 cm, turn on LED
        else if (distance_3 <= 2 || distance_4 <= 2)
        {
        	safe_right = 1;
        	safe_left = 0;
            lcd_moveto(1, 1);
            // deal with how LCD prints out speed
            lcd_print("Left Not Safe");
            _delay_ms(2000);
        }
        else
        {
        	safe_right = 1;
        	safe_left = 1;
            lcd_moveto(1, 1);
            lcd_print("     Safe     ");
            _delay_ms(2000);
        }

        // Delay before next measurement
        _delay_ms(500);
    }

    return 0;
    
}


// max7219 function 
void send2bytes(uint8_t addr, uint8_t data)
{
    if(x == 1)
    {
    
        PORTB &= ~(1 << PB1); // Make SS (CS) low for the second MAX7219
        SPDR = addr;          
        // Send address byte
        while (!(SPSR & (1 << SPIF)));        
        // Wait for send complete
        SPDR = data; // Send data byte
        while (!(SPSR & (1 << SPIF)));                
        // Wait for send complete
        PORTB |= (1 << PB1); // Make SS (CS) high for the second MAX7219
    }
    else
    {
        PORTB &= ~(1 << PB2); // Make SS (CS) low for the first MAX7219
        SPDR = addr;          
        // Send address byte
        while (!(SPSR & (1 << SPIF)));        
        // Wait for send complete
        SPDR = data; // Send data byte
        while (!(SPSR & (1 << SPIF)));                
        // Wait for send complete
        PORTB |= (1 << PB2); // Make SS (CS) high for the first MAX7219
    }
    
}

void sendPulse(void) {
    // Send ultrasonic pulse
    PORTC |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTC &= ~(1 << TRIG_PIN);
}

// encoder counter function 
ISR(PCINT1_vect){
    unsigned char temp = PINC;
    unsigned char a = (temp & (1<<1)); // Adjust for PC1
  
    if (old_state == 0) {
        if (a != 0){
            new_state = 1;
            count++;  
        }
    }
    else if (old_state == 1) {
        if (a == 0){
            new_state = 0;
            count++;   
        }
    }

    if (new_state != old_state) {
        changed = 1;
        old_state = new_state;
    }
}


// time keeping functions
void millis_init()
{
    // Timer0 setup for millis_time()
    TCCR0A = 0;             // Normal mode
    TCCR0B |= (1 << CS00);  // Prescaler 1
    OCR0A = 249;            // Set compare value for 1ms
    TIMSK0 |= (1 << OCIE0A);// Enable Timer0 compare interrupt
    sei();                  // Enable global interrupts
}


unsigned long timer_ticks = 0;
ISR(TIMER0_COMPA_vect)
{
    timer_ticks++;
}


unsigned long millis_time()
{
    unsigned long ms;
    cli();  // Disable interrupts to read timer_ticks atomically
    ms = timer_ticks;
    sei();  // Enable interrupts back
    return ms;
}
