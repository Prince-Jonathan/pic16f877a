/********************************************************************
; Filename: LCD-C.C													*	
; Function:	A sample of LCD module control/display in C format.		*
; Author: Prince Jonathan Appau Nkansah												*
; Date: 9-22-2023													*
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

long unsigned int ADC_Reading; 	//declaring 4 byte memory allocation for ADC reading
unsigned char TMP @ 0X30, TMP2 @ 0X31, TMP3 @ 0X32, TMP4 @ 0X33, TMP5 @ 0X34, TMP6 @0X111, TMP7 @ 0XC4;		//6 bit precision for reading on LCD
unsigned char *ptr = &TMP;
unsigned char i;				//iterator variable

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

void BCD_conv(long unsigned int hex_code){
	//reset word_bits					
	for (i=0; i<6;i++){
		ptr[i] = 0x0;
	}
	while (hex_code >= 0XA){
		//subtract 100,000 from hex code
		if (hex_code >= 0X186A0){
			hex_code -= 0X186A0; //increment 5th bit on every subtraction by 100,000 decimal
			ptr[0]++;
		}
		//subtract 10,000 from hex code
		else if (hex_code >= 0X2710){
			hex_code -= 0X2710;
			ptr[1]++;
		}
		//subtract 1,000 from hex code
		else if (hex_code >= 0X3E8){
			hex_code -= 0X3E8;
			ptr[2]++;
		}
		//subtract 100 from hex code
		else if (hex_code >= 0X64){
			hex_code -= 0X64;
			ptr[3]++;
		}
		//subtract 10 from hex code
		else if (hex_code >= 0XA){
			hex_code -= 0XA;
			ptr[4]++;
		}
	}	
	ptr[5] = hex_code ; //conserve remainder as final value to 10 decimal word
}

//custom multiplication function
long unsigned int mult(long unsigned int word, unsigned char fact){
	long unsigned int res = 0x0;
	for (i=0; i<fact;i++){
		res+=word;
	}
	return res;
}

//custom division function
long unsigned int div(long unsigned int word, long unsigned int fact){
	unsigned char word_bit = 0X0;

	while (fact<=word){
		word = word - fact;
		word_bit++;
	}
	return word_bit;
}

//convert hex code to ASCII code
unsigned int ASCII(unsigned int hex_code){
	return (hex_code+0X30);
}


void main(void)
{
	TRISA = 0X01;				//Set all pin 0 of Port A to input
	TRISB = 0X00;				//Set all pins of PORTB to outputs (data lines)
	TRISC = 0X00;				//Set all pins of PORTC to outputs (For RS and E pins)
	ADCON1 = 0X8F;				//configuring right justify and AN0 as Analogue with AN3 and AN2 as +Vref and -Vref respectively
	CHS0, CHS1, CHS2 = 0;		//selecting AN0 as input channel
	
	//select clock Fosc/32 while ADCS2 = 0
	ADCS1 = 1;
	ADCS0 = 0;			

	ADON = 1;					//turn on A/D module 
	__delay_us(20);				//allowing for 20 microseconds acquisition time

	lcd_init();						//initialize LCD
	lcd_puts("Voltage Reading:");	//display text

	while(1){
		GO_nDONE = 1;							//start conversion
		while (GO_nDONE);						//wait to conversion is done, as this bit is cleared
		ADC_Reading = (ADRESH  << 8) | ADRESL;	//pass value of low byte A/D Result to ADC_Reading
		lcd_goto(40);							//skip to next line on LCD Screen
		BCD_conv(ADC_Reading*0X1E9);			//Convert ADC reading to decimal bits. Multiply conversion value by 100,000 = 186A0
	
		//display ADC readings in decimal
		for (i = 0; i < 6; i++){
			lcd_putch(ASCII(ptr[i]));
			if (i == 0){
				lcd_putch(0x2E);			//display decimal point
			}	
		}
		lcd_putch(0X56);						//append 'V'
		//lcd_putch(ASCII(BCD_conv(div(mult(mult(ADC_Reading,0x05), 0X186A0),0X3FF))));	
		
	}

//	for(;;);				//prevent program from terminating
}
 
