/********************************************************************
; Filename: LCD-C.C													*	
; Function:	A sample of LCD module control/display in C format.		*
; Author: Steve Hsiung												*
; Date: 9-14-2023													*
;********************************************************************/
#include <xc.h> 		//uses non-legacy pic16f877a.h

__CONFIG(FOSC_HS & WDTE_OFF & PWRTE_OFF & LVP_OFF & CP_OFF); //use for XC8 or Hi-Tech C


#define	LCD_RS RC0		//LCD register select on RC0 (when set to '0', command is sent to LCD and when set to '1' data is sent to LCD )
#define LCD_EN RC1		//latches LCD to commands or data
#define LCD_DATA PORTB	//LCD 8-bit parallel data on PORTB

#ifndef _XTAL_FREQ
 // Unless specified elsewhere, 20MHz system frequency is assumed
 #define _XTAL_FREQ 20000000
#endif


// toggles the enable bit on to send command or data
void toggle_e()
	{
		PORTC,RC1=1;		//set register select and enable pins to 1
		__delay_us(20);		//delay for 20 microseconds
		LCD_EN=0;			//set enable pin to 0
		__delay_us(20);		//delay for 20 microseconds
		LCD_EN=1;			//set enable pin to 1
	}

//write command or data to LCD
void lcd_write(unsigned char c)
	{
		LCD_DATA = (c);		//pass command or data to 8-bit pins of PortB
		toggle_e();			//toggle enable pin to 1
		__delay_ms(20);		//delay for 20 microseconds
	}

//clear the LCD and home the cursor
void lcd_clear(void)
	{
		LCD_RS = 0;			//instructs that commands are being passed to LCD
		lcd_write(0x01);	//hex value clears the display according to the LCD datasheet
		__delay_ms(20);		//delays for 20 microseconds
	}

//write a string of chars to the LCD
void lcd_puts(const char * s)
	{
		LCD_RS = 1;			//write data (characters)
		while(*s)			//while there is data at pointer address
		lcd_write(*s++);	//write data to LCD and then increment pointer address
		__delay_ms(20);		//wait 20 microseconds
	}

//write one character to the LCD
void lcd_putch(char c)
	{
		LCD_RS = 1;			//write data (characters)
		lcd_write( c );		//write data to LCD
	}

//go to the specified position
void lcd_goto(unsigned char pos)
	{
		LCD_RS = 0;			//write instruction
		lcd_write(0x80+pos);//Force cursor to the beginning ( 1st line) and then offset the cursor to the specified position (pos) on the LCD
	}
	
//initialise the LCD - put into 8 bit mode
void lcd_init()
	{
		LCD_RS = 0;			//write instructions 
		lcd_write(0X30);	
		lcd_write(0X30);
		lcd_write(0X30);
		lcd_write(0X38);	//2 lines and 5×7 matrix
		lcd_write(0X0C);	//Display on, cursor off
		lcd_write(0X01);	//Clear display
		lcd_write(0X06);	//Sets Entry Mode to auto-increment cursor and disable shift mode
		lcd_write(0X02);	//send the cursor to the home position
	}




void main(void)
{
	TRISB=0;				//Set all pins of PORTC to outputs (data lines)
	TRISC=0;				//Set all pins of PORTC to outputs (For RS and E pins)

	lcd_init();						//initialize LCD
	lcd_puts("Time is scarce in abundance; cherish it");	//display text

	for(;;);				//prevent program from terminating
}
 
