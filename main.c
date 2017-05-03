/*
 * main.c
 * 
 * Copyright 2016 shuu01 <shuu01@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

// Global defines
#define F_CPU 8000000L // CPU clock speed 16 MHz
#define F_SCL 100000L // I2C clock speed 100 kHz
#define ClearBit(x, y) x &= ~_BV(y) // equivalent to cbi(x,y)
#define SetBit(x, y) x |= _BV(y) // equivalent to sbi(x,y) 

// Includes
#include <avr/io.h> // deal with port register
#include <util/delay.h> // used for _delay_ms function
#include <string.h> // string manipulation
#include <stdlib.h>
#include <math.h> //include libm

#define PORT_LCD PORTC
#define DDR_LCD DDRC

// Typedefs
typedef uint8_t byte;
typedef int8_t sbyte;

// Misc routines

void avr_init()
{
    DDR_LCD = 0x6F; // 0110.1111; set X0-X3, X5-X6 as output 
}

void ms_delay(int delay)
{
    for (int i = 0; i < delay; i++)
        _delay_ms(1);
}
//-------------
// lcd4_init   initializes LCD controller with 4 data lines
// lcd_cmd     sends LCD controller command
// lcd_char    sends single ascii character
// lcd_clear   clears display
// lcd_home    homes cursor
// lcd_goto    puts cursor at position (x,y)
// lcd_line    puts cursor at start of line
// lcd_hex     displays a hexadecimal value
// lcd_integer displays a integer value
// lcd_string  displays a string
//
// The LCD module requires 6 I/O pins: 2 control lines & 4 data lines.
// PortX is used for data communications with the HD44780 LCD.
// The following defines specify which port pins connect to the
// controller

#define LCD_RS 5
#define LCD_E  6
#define DAT4   0
#define DAT5   1
#define DAT6   2
#define DAT7   3

#define CLEARDISPLAY 0x01
#define SETCURSOR 0x80

void pulse_enable_line()
{
    SetBit(PORT_LCD, LCD_E);
    _delay_us(30);
    ClearBit(PORT_LCD, LCD_E);
}

void send_nibble(byte data)
{
    PORT_LCD &= 0x60;  // 0110.0000 clear 4 data lines
    if (data & _BV(4)) SetBit(PORT_LCD, DAT4);
    if (data & _BV(5)) SetBit(PORT_LCD, DAT5);
    if (data & _BV(6)) SetBit(PORT_LCD, DAT6);
    if (data & _BV(7)) SetBit(PORT_LCD, DAT7);
    pulse_enable_line();
}

void send_byte(byte data)
{
    send_nibble(data); // send upper 4 bits
    send_nibble(data << 4); // send lower 4 bits
}

void lcd_cmd(byte cmd)
{
    ClearBit(PORT_LCD, LCD_RS); // R/S line 0 = command data
    send_byte(cmd);
}

void lcd_char(byte ch)
{
    SetBit(PORT_LCD, LCD_RS); // R/S line 1 = character data
    send_byte(ch);
}

void lcd_clear()
{
    lcd_cmd(CLEARDISPLAY);
    _delay_ms(3);
}

void lcd4_init()
{
    _delay_ms(100);
    lcd_cmd(0x30); // Special case of 'Function Set'
    _delay_ms(5);
    lcd_cmd(0x30); // Special case of 'Function Set'
    _delay_us(100);
    lcd_cmd(0x30); // Special case of 'Function Set'
    _delay_us(100);
    lcd_cmd(0x20); // Initial 'Function Set' to change interface
    _delay_us(100);
    lcd_cmd(0x28); // 2 line, 5x7 matrix
    _delay_us(40);
    lcd_cmd(0x08); // turn display off
    lcd_clear();
    lcd_cmd(0x0C); // display on, cursor off (0x0E to enable)
    lcd_cmd(0x06); // cursor direction = right
    lcd_clear();
}

void lcd_home()
{
    lcd_cmd(SETCURSOR);
}

/// put LCD cursor on specified line
void lcd_goto(byte x, byte y)
{
    byte addr = 0; // line 0 begins at addr 0x00
    switch(y)
    {
        case 1: addr = 0x40; break; // line 1 begins at addr 0x40
        case 2: addr = 0x14; break;
        case 3: addr = 0x54; break;
    }
    lcd_cmd(SETCURSOR + addr + x); // update cursor with x,y position
}

/// put cursor on specified line
void lcd_line(byte row)
{
    lcd_goto(0, row);
}

/// display string on LCD
void lcd_string(const char *text)
{
    while (*text) // do until /0 character
        lcd_char(*text++); // send char & update char pointer
}

/// displays the hex value of DATA at current LCD cursor position
void lcd_hex(int data)
{
    char st[8] = ""; // save enough space for result
    itoa(data, st, 16); // convert to ascii hex
    //LCD_Message("0x"); // add prefix "0x" if desired
    lcd_string(st); // display it on LCD
}

/// displays the integer value of DATA at current LCD cursor position
void lcd_integer(int data)
{
    char st[8] = ""; // save enough space for result
    itoa(data, st, 10); // convert to ascii
    lcd_string(st); // display in on LCD
}

#define ADC_REFRES 1024 //reference resolution used for conversions 10 bit

//--------
// adc_init   initializes ADC
// adc_read   read value
//--------

void adc_init()
{
    ADMUX = (0 << REFS1) | (1 << REFS0); // For Aref = AVcc;
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1<<ADPS0);
    // prescaler div factor 128
}

uint16_t adc_read(uint8_t ch)
{
    //Select ADC Channel ch must be 0-7
    ch = ch & 0b00000111;
    ADMUX |= ch;

    //Start Single conversion
    ADCSRA |= (1 << ADSC);

    //Wait for conversion to complete
    while( !(ADCSRA & (1 << ADIF)));

    //Clear ADIF by writing one to it
    //Note you may be wondering why we have write one to clear it
    //This is standard way of clearing bits in io as said in datasheets.
    //The code writes '1' but it result in setting bit to '0' !!!

    ADCSRA |= (1 << ADIF);

    return(ADC);
}

#define MQ135_PIN 0
#define MQ135_DEFAULTPPM 415 //default ppm of CO2 for calibration
//#define MQ135_MAXRSRO 2.428 //for CO2
//#define MQ135_MINRSRO 0.358 //for CO2
#define MQ135_RLOAD 10000 //define mq135 pulldown resistor
#define MQ135_RZERO 41763 //default Ro for MQ135_DEFAULTPPM ppm of CO2
//#define MQ135_RZERO 68550 //default Ro for mq135 default ppm of CO2

/// parameters for calculating ppm from sensor resistance
#define PARA 116.6020682 //CO2 curve
#define PARB -2.769034857 //CO2 curve
//float CO2Curve[2] = {113.7105289, -3.019713765}; //MQ135

/// parameters to model temperature and humidity dependence
#define CORA 0.00035
#define CORB 0.02718
#define CORC 1.39538
#define CORD 0.0018

/// convert an adc value to a resistance value
long mq135_get_resistance()
{
    uint16_t adcread = adc_read(MQ135_PIN);
	
    if(adcread == 0)
		return 0;
	else
		return (((long)MQ135_RLOAD * ADC_REFRES)/adcread - MQ135_RLOAD);
}

/// get the ppm concentration
long mq135_get_ppm(long resvalue) 
{
    return (long)(PARA * pow(((float)resvalue/MQ135_RZERO), PARB));
}

/// get the calibrated rzero based upon read resistance, and a known ppm
long mq135_get_rzero(long resvalue, long ppm) 
{
	return (long)(resvalue * exp(log(PARA/ppm)/PARB));
}

/*!
@brief  Get the correction factor to correct for temperature and humidity
@param[in] t  The ambient air temperature
@param[in] h  The relative humidity
@return The calculated correction factor
*/
float mq135_get_cor_factor(float t, float h) 
{
    return CORA * t * t - CORB * t + CORC - (h - 33.) * CORD;
}

float mq135_get_cor_resistance(float t, float h) 
{
    return mq135_get_resistance()/mq135_get_cor_factor(t, h);
}

float mq135_get_cor_ppm(float t, float h) 
{
    return PARA * pow((mq135_get_cor_resistance(t, h)/MQ135_RZERO), -PARB);
}

float mq135_get_cor_rzero(float t, float h) 
{
    return mq135_get_cor_resistance(t, h) * 
        pow(((float)MQ135_DEFAULTPPM/PARA), (1./PARB));
}

int main(void)
{
    long res, rzero, ppm = 0;
    char printbuff[10];
    
    avr_init();
    lcd4_init();
    lcd_string("Ready.");
    _delay_ms(2000);
    adc_init();
    while (1)
    {
        lcd_clear();
        res = mq135_get_resistance();
        rzero = mq135_get_rzero(res, MQ135_DEFAULTPPM);
        ppm = mq135_get_ppm(res) + 130;
        ltoa(ppm, printbuff, 10);
        lcd_string(printbuff);
        lcd_string(" ppm");
        
        lcd_line(1);
        ltoa(rzero, printbuff, 10);
        lcd_string(printbuff);
        lcd_string(" ohm");

        _delay_ms(1000);
    }
}
