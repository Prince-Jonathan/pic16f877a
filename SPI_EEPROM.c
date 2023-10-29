/********************************************************************
; Filename: SPI EEPROM														
; Function:	Samples, displays, stores and retrieves potentiometer 
			readings. Using MCU, LCD and SPI EEPROM modules.
; Author: Prince Jonathan Appau Nkansah												
; Date: 10-13-2023													
;********************************************************************/
#include <xc.h> 		//uses non-legacy pic16f877a.h

__CONFIG(FOSC_HS & WDTE_OFF & PWRTE_OFF & LVP_OFF & CP_OFF); //use for XC8 or Hi-Tech C


#define	LCD_RS RC0		//LCD register select on RC0 (when set to '0', command is sent to LCD and when set to '1' data is sent to LCD )
#define LCD_EN RC1		//latches LCD to commands or data
#define LCD_DATA PORTB	//LCD 8-bit parallel data on PORTB
#define CS RC2

#ifndef _XTAL_FREQ
 // Unless specified elsewhere, 20MHz system frequency is assumed
 #define _XTAL_FREQ 20000000
#endif

long unsigned int ADC_Reading; 	//declaring 4 byte memory allocation for ADC reading
unsigned char *ptr;
unsigned char *ptr2;
unsigned char mcu_start;						//store the next byte start address in MCU
unsigned char ee_start;							//EEPROM starting address for storage
unsigned char num_readings;						//number of analogue readings to sample
unsigned char total_bytes;		//total number of bytes to be stored: is a multiple of the number of digits from the BCD conversion
unsigned char ee_end;	//last address in the EEPROM to be accessed for storing readings
unsigned char num_page1_writes;					//number of writes within inital EEPROM Page
unsigned char num_page2_writes; 				//number of writes within next EEPROM page where there is an overflow to next page

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
		lcd_write(0X38);	//2 lines and 5�7 matrix
		lcd_write(0X0C);	//Display on, cursor off
		lcd_write(0X01);	//Clear display
		lcd_write(0X06);	//Sets Entry Mode to auto-increment cursor and disable shift mode
		lcd_write(0X02);	//send the cursor to the home position
	}

void BCD_conv(long unsigned int hex_code, unsigned char i){
	//reset word_bits
	for (unsigned char j=0; j<4;j++){
		ptr[i+j] = 0x0;
	}					
	while (hex_code >= 0XA){
		//subtract 1,000 from hex code
		if (hex_code >= 0X3E8){
			hex_code -= 0X3E8;
			ptr[i]++;
		}
		//subtract 100 from hex code
		else if (hex_code >= 0X64){
			hex_code -= 0X64;
			ptr[i+1]++;
		}
		//subtract 10 from hex code
		else if (hex_code >= 0XA){
			hex_code -= 0XA;
			ptr[i+2]++;
		}
	}	
	ptr[i+3] = hex_code ; //conserve remainder as final value to 10 decimal word
}

//convert hex code to ASCII code
unsigned int ASCII(unsigned int hex_code){
	return (hex_code+0X30);		//return ASCII code of hexcode passed
}

//configure ADC module bits
void adc_init(){
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
}
//read 10-bit resolution analogue input into memory
void read_analog(){
	GO_nDONE = 1;							//start conversion
	while (GO_nDONE);						//wait to conversion is done, as this bit is cleared
	ADC_Reading = (ADRESH  << 8) | ADRESL;	//pass value of low byte A/D Result to ADC_Reading	
}
void probe(unsigned char param_){
	/*
	display param_ on LCD screen and pause program execution
	Parameters:
		-param_:value to be displayed
	*/
	lcd_putch(ASCII(param_));
	for(;;);
}
unsigned char SPI_write(unsigned char data){
	/*
	Transmit and return data between master and slave
	Parameters:
		-data: data to be sent to slave.
	Returns:
		-data: data from slave that is stored in the buffer
	*/
	SSPBUF = data;		//pass on data to be transmitted to buffer
	while (!BF);		//poll buffer full status until transmit buffer is full, which sets BF to 1
	data = SSPBUF;		//pass data from slave or read from buffer to toggle BF back to zero for next read function-usecase
	return data; 		
}
void write_enable(){
	//Enable write to the EEPROM
	CS = 0;				//pull chip select low to access EEPROM
	SPI_write(0x06);	//set the write enable latch
	CS = 1;
}
void write_disable(){
	//Enable write to the EEPROM
	CS = 0;				//pull chip select low to access EEPROM
	SPI_write(0x04);	//set the write enable latch
	CS = 1;
}
unsigned char read_sr(){
	/*
	Read the Status Register byte from the EEPROM
	Returns:
		-data: Status register byte
	*/
	unsigned char data;
	CS = 0;					//pull chip select low to access EEPROM
	SPI_write(0x05);		//send the read status register instruction 
	data = SPI_write(0xFF);	//send dummy from master to allow slave return register byte on same running clock
	CS = 1;
	return data;
}
void wip_poll(){
	//Ensure that there is no EEPROM write in progress 
	unsigned char wip;
	do{
		wip = read_sr() & 0x01;	//extract WIP bit from status register byte
	}
	while(wip);
}
void write_sr(unsigned char data){
	/*
	Write data to EEPROM Status Register
	Parameters:
		-data: byte to write to EEPROM
	*/
	CS = 0;			//pull chip select low to access EEPROM
	SPI_write(0x01);//send write to status register instruction
	SPI_write(data);//send data byte to EEPROM
	CS = 1;
	wip_poll();		//confirm no EEPROM write is in progress
}
void write_eebyte(unsigned char data, unsigned int address){
	/*
	Write data byte(s) to EEPROM page
	Parameters:
		-data: byte to write to EEPROM
		-address: 16-bit EEPROM page address
	*/
	CS = 0;					//pull chip select low to access EEPROM
	SPI_write(0x02);		//send write byte sequence instruction
	SPI_write(address>>8);	//send high byte of 16-bit address with MSB being a 'don't care' bit
	SPI_write(address);		//send low byte of 16-bit address
	SPI_write(data);
	CS = 1;			
	wip_poll();		//confirm no EEPROM write is in progress
}
void write_eebytes(unsigned char cycles, unsigned int ee_start){
	/*
	Write data bytes to EEPROM page from MCU address range
	Parameters:
		-start_addr: pointer to MCU starting address with data to be sent
		-cycles: number of byte data to be sent
		-ee_start: starting address of EEPROM page 
	*/
	CS = 0;
	SPI_write(0x02);
	SPI_write(ee_start>>8);
	SPI_write(ee_start);
	for (unsigned char i=0; i<cycles; i++){
		SPI_write(*ptr2++);
	}
	CS = 1;
	wip_poll();
}
void read_eebytes(unsigned int start_addr, unsigned int end_addr){
	/*
	Read data byte from EEPROM Page
	Parameters:
		-start_addr: 16-bit EEPROM starting page address
		-end_addr: 16-bit EEPROM ending page address
	*/
	write_disable();		//clear Write Enabled bit (WEL)
	CS = 0;					//pull chip select low to access EEPROM
	SPI_write(0x03);		//send read byte sequence instruction
	SPI_write(start_addr>>8);	//send high byte of 16-bit address with MSB being a 'don't care' bit
	SPI_write(start_addr);		//send low byte of 16-bit address
	
	for (unsigned int i=0; i<=end_addr-start_addr; i++){
		ptr[i] = SPI_write(0xFF);	//send dummy from master to allow slave return register byte on same running clock			
	}
	CS = 1;			
}
void read_eebyte(unsigned int addr){
	/*
	Read data byte from EEPROM Page
	Parameters:
		-addr: 16-bit EEPROM starting page address
	*/
	read_eebytes(addr, addr);
}
void interrupt spi_interrupt(){
	/*
	Set interrupts: 
		-Synchronous Serial Port Interrupt Flag; to check for transmission complete
		-Write Collision; to flag when write to buffer command is issued while transmission buffer is full
	*/
	if(SSPIF){
		//lcd_puts("Transmission completed");
		//lcd_clear();
		SSPIF = 0;
	}
	if(WCOL){
		lcd_puts("Write Collision Occured");
		__delay_ms(2000);
		lcd_clear();
		WCOL = 0;
	}
}
void main(void)
{
	ptr = 0x120;
	//Enable interrupt Service Routine
	SSPIE = 1;	//enabling Synchronous Serial Port Interrupt
	GIE = 1;	//enable all unmasked interrupts
	PEIE = 1;	//enable all unmasked peripheral interrupts
	

	adc_init();	//initialize Analogue Digital Converter Module
	lcd_init();	

	SSPSTAT = 0x80;			//sample input data at the end of data output time and transmit from idle to active clock state
	SSPCON = 0x32;			//set SPI Master Mode, clock = Fosc/64, set clock idle state to high
	TRISC = 0x10;			//configure RC2,RC3 and RC5 as output for chip select, clock and serial data output, respectively and configure RC4 as serial data input

	/*
	read mode: RD0 = 0;
	write mode: RD0 = 1;
	*/
	TRISD = 1;				//configure pin 0 of PORT D as input for toggling between read and write modes				

	ee_start = 0x7FBA;				//EEPROM starting address for storage
	num_readings = 0xA;				//number of analogue readings to sample
	total_bytes = num_readings*4;		//total number of bytes to be stored: is a multiple of the number of digits from the BCD conversion
	ee_end = ee_start + total_bytes;	//last address in the EEPROM to be accessed for storing readings

	switch(RD0){
	case 0:
		lcd_clear();
		lcd_puts("Logging Voltage: ");
		for (unsigned char i=0; i<num_readings; i++){
			read_analog();							//read 10-bit resolution analogue input into pointer locations 0x34 and 0x35
			mcu_start = 4*i;						//offset mcu address for every ADC reading
			BCD_conv(ADC_Reading*0X5, mcu_start);	//Convert ADC reading to decimal bits. Multiply conversion value by 100,000 = 186A0
			//display ADC readings in decimal
			if(i==9){
					lcd_clear();
					lcd_puts("Logging Voltage: ");
			}
			for (unsigned char i = 0; i < 4; i++){
				lcd_putch(ASCII(ptr[mcu_start+i]));	//display digits of ADC reading
				if (i == 0){
					lcd_putch(0x2E);	//display decimal point
				}	
			}
			lcd_putch(0X56);	//append 'V'
			lcd_putch(0x20);	//append space
			__delay_ms(2000);
		}

		//calculate number of writes for EEPROM start address within page
		ptr2 = ptr;				
		while(ee_start<ee_end){
			//calculate number of writes from EEPROM start address within page
			num_page1_writes = 0x40-(ee_start % 0x40);	
			num_page2_writes = total_bytes - num_page1_writes;

			write_enable();
			write_eebytes(num_page1_writes, ee_start);	//write bytes to EEPROM starting from ee_start, for num_page1_writes times
			
			ee_start += num_page1_writes;				//update next write start address
			
			//check if remaining bytes to write do not exceed next page
			if (num_page2_writes<0x40){
				write_enable();
				write_eebytes(num_page2_writes, ee_start);
				ee_start += num_page2_writes;
			}else {
				total_bytes = num_page2_writes; 		//pass remaining bytes to total_bytes and restart loop
			}
		}
		lcd_clear();
		lcd_puts("write complete");
		break;
	case 1:
		while(1){
			lcd_goto(0);
			lcd_puts("Retrieved Voltages:     ");
			read_eebytes(ee_start, ee_start+total_bytes);	//providing EE start and end address reads from EEPROM
			for(unsigned char i=0; i<num_readings; i++){
				mcu_start = 4*i;
				//show next page of display on LCD 
				if(i==8){
					__delay_ms(2000);
					lcd_clear();
					lcd_puts("Retrieved Voltages:     ");
				}
				for (unsigned char j=0; j<4; j++){
					lcd_putch(ASCII(ptr[mcu_start+j]));	//display digits of ADC reading
					if (j == 0){
						lcd_putch(0x2E);		//display decimal point
					}					
				}
			
				lcd_putch(0X56);	//append 'V'
				lcd_putch(0x20);	//append space
			}
			__delay_ms(1000);
		}
		break;
	default:
		break;
	}

	//asm("nop");
	for(;;);				//prevent program from terminating
}

 
