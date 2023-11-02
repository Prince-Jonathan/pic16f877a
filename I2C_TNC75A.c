/********************************************************************
; Filename: I2C_TEST-C.C											*	
; Function:	A framework for reading  TCN75A temp sensor	
;	with I2C and writing to EEPROM with SPI
; Author: Prince Jonathan Appau Nkansah
; Date: 10-26-2023													*
;********************************************************************/

//PORTC: 	SDI	RC5
//		SDA/SDO RC4
//		SCL/SCK RC3
//		RTCCS	RC2
//		TmpCS	RC1



//'1' on TCN75 is a read operation command
//'0' on TCN75 is a write operation command
//Delay of 1 is 50ns
//Delay of 1000 is 50us
//Delay of 1,000,000 is 50ms//


#include <xc.h>   //uses non-legacy pic16f877a.h 

#define S 3
#define P 4
#define	LCD_RS RC0		//LCD register select on RC0 (when set to '0', command is sent to LCD and when set to '1' data is sent to LCD )
#define LCD_EN RC1		//latches LCD to commands or data
#define LCD_DATA PORTB	//LCD 8-bit parallel data on PORTB
#define CS RC2

#ifndef _XTAL_FREQ
 // Unless specified elsewhere, 20MHz system frequency is assumed
 #define _XTAL_FREQ 20000000
#endif

/* __CONFIG (0X03F32); */
__CONFIG(FOSC_HS & WDTE_OFF & PWRTE_ON & BOREN_OFF & LVP_OFF & CP_OFF); //this is the non-legacy version 

long unsigned int ADC_Reading;
unsigned char *ptr;
unsigned char *ptr2;
 
void IC_ON(void);
void IC_OFF(void);
void IC_START(void);
void IC_REST(void);
void IC_STOP(void);
void POLL_BF_C(void);
void POLL_BF_S(void);
unsigned char I2R();
void IC_WB(unsigned char byte);
void MS_ACK();
void MS_NAK();
unsigned char MS_RECEIVE();

/* oggles the enable bit on to send command or data */
void toggle_e()
	{
		PORTC,RC1=1;		//set register select and enable pins to 1
		__delay_us(20);		//delay for 20 microseconds
		LCD_EN=0;			//set enable pin to 0
		__delay_us(20);		//delay for 20 microseconds
		LCD_EN=1;			//set enable pin to 1
	}

/*
Write command or data to LCD
Parameters:
	-c: byte to write PortB
*/
void lcd_write(unsigned char c)
	{
		LCD_DATA = (c);		//pass command or data to 8-bit pins of PortB
		toggle_e();			//toggle enable pin to 1
		__delay_ms(20);		//delay for 20 microseconds
	}

/* Clear the LCD and home the cursor */
void lcd_clear(void)
	{
		LCD_RS = 0;			//instructs that commands are being passed to LCD
		lcd_write(0x01);	//hex value clears the display according to the LCD datasheet
		__delay_ms(20);		//delays for 20 microseconds
	}

/*
Write a string of chars to the LCD
Parameters:
	-s: pointer to first character in string
*/
void lcd_puts(const char * s)
	{
		LCD_RS = 1;			//write data (characters)
		while(*s)			//while there is data at pointer address
		lcd_write(*s++);	//write data to LCD and then increment pointer address
		__delay_ms(20);		//wait 20 microseconds
	}

/*
Write one character to the LCD
Parameters:
	-c: byte data to be passed to PORTB
*/
void lcd_putch(char c)
	{
		LCD_RS = 1;			//write data (characters)
		lcd_write( c );		//write data to LCD
	}

/*
Go to the specified position
Parameters:
	-pos: specified position
*/
void lcd_goto(unsigned char pos)
	{
		LCD_RS = 0;			//write instruction
		lcd_write(0x80+pos);//Force cursor to the beginning ( 1st line) and then offset the cursor to the specified position (pos) on the LCD
	}
	
/* Initialise the LCD - put into 8 bit mode */
void lcd_init()
	{
		TRISB = 0x0;		//Set tristate of PORTB Pins to output
		TRISC0 = 0;			//Set tristate of RS [RC0 pin] to output
		TRISC1 = 0;			//Set tristate of E [RC1 pin] to output
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

/*
Convert hex to decimal
Parameters:
	-hex_code: hex code to be converted to a decimal equivalent, still in hex format
	-char: offset number for stacking conversion results
*/
void BCD_conv(long unsigned int hex_code, unsigned char i){
	//reset word_bits
	for (unsigned char j=0; j<3;j++){
		ptr[i+j] = 0x0;
	}					
	while (hex_code >= 0XA){
		//subtract 100 from hex code
		if (hex_code >= 0X64){
			hex_code -= 0X64;
			ptr[i]++;
		}
		//subtract 10 from hex code
		else if (hex_code >= 0XA){
			hex_code -= 0XA;
			ptr[i+1]++;
		}
	}	
	ptr[i+2] = hex_code ; //conserve remainder as final value to 10 decimal word
}

/*
Convert hex code to ASCII code
Parameters:
	-hex_code: hex to be converted
Return:
	-ASCII code
*/
unsigned int ASCII(unsigned int hex_code){
	return (hex_code+0X30);		//return ASCII code of hexcode passed
}

/* Configure ADC module bits */
void adc_init(){
	TRISA0 |= 0X01;				//Set pin 0 of Port A to input
	ADCON1 = 0X8F;				//configuring right justify and AN0 as Analogue with AN3 and AN2 as +Vref and -Vref respectively
	CHS0, CHS1, CHS2 = 0;		//selecting AN0 as input channel
	
	//select clock Fosc/32 while ADCS2 = 0
	ADCS1 = 1;
	ADCS0 = 0;			

	ADON = 1;					//turn on A/D module 
	__delay_us(20);				//allowing for 20 microseconds acquisition time	
}

/* Read 10-bit resolution analogue input into memory */
void read_analog(){
	GO_nDONE = 1;							//start conversion
	while (GO_nDONE);						//wait to conversion is done, as this bit is cleared
	ADC_Reading = (ADRESH  << 8) | ADRESL;	//pass value of low byte A/D Result to ADC_Reading	
}

/*
Display param_ on LCD screen and pause program execution
Parameters:
	-param_:value to be displayed
*/
void probe(unsigned char param_){

	lcd_putch(ASCII(param_));
	for(;;);
}

/* Configuration for SPI communication */
void SPI_ON(void){
	SSPSTAT = 0x80;			//sample input data at the end of data output time and transmit from idle to active clock state
	SSPCON = 0x32;			//set SPI Master Mode, clock = Fosc/64, set clock idle state to high
	TRISC = 0x10;			//configure RC2,RC3 and RC5 as output for chip select, clock and serial data output, respectively and configure RC4 as serial data input
}

/*
Transmit and return data between master and slave
Parameters:
	-data: data to be sent to slave.
Returns:
	-data: data from slave that is stored in the buffer
*/
unsigned char SPI_write(unsigned char data){
	
	SSPBUF = data;		//pass on data to be transmitted to buffer
	while (!BF);		//poll buffer full status until transmit buffer is full, which sets BF to 1
	data = SSPBUF;		//pass data from slave or read from buffer to toggle BF back to zero for next read function-usecase
	return data; 		
}

/* Enable write to the EEPROM */
void write_enable(){
	CS = 0;				//pull chip select low to access EEPROM
	SPI_write(0x06);	//set the write enable latch
	CS = 1;
}

/* Enable write to the EEPROM */
void write_disable(){
	
	CS = 0;				//pull chip select low to access EEPROM
	SPI_write(0x04);	//set the write enable latch
	CS = 1;
}

/*
Read the Status Register byte from the EEPROM
Returns:
	-data: Status register byte
*/
unsigned char read_sr(){
	
	unsigned char data;
	CS = 0;					//pull chip select low to access EEPROM
	SPI_write(0x05);		//send the read status register instruction 
	data = SPI_write(0xFF);	//send dummy from master to allow slave return register byte on same running clock
	CS = 1;
	return data;
}

/* Ensure that there is no EEPROM write in progress  */
void wip_poll(){
	unsigned char wip;
	do{
		wip = read_sr() & 0x01;	//extract WIP bit from status register byte
	}
	while(wip);
}

/*
Write data to EEPROM Status Register
Parameters:
	-data: byte to write to EEPROM
*/
void write_sr(unsigned char data){
	CS = 0;			//pull chip select low to access EEPROM
	SPI_write(0x01);//send write to status register instruction
	SPI_write(data);//send data byte to EEPROM
	CS = 1;
	wip_poll();		//confirm no EEPROM write is in progress
}

/*
Write data byte(s) to EEPROM page
Parameters:
	-data: byte to write to EEPROM
	-address: 16-bit EEPROM page address
*/
void write_eebyte(unsigned char data, unsigned int address){
	CS = 0;					//pull chip select low to access EEPROM
	SPI_write(0x02);		//send write byte sequence instruction
	SPI_write(address>>8);	//send high byte of 16-bit address with MSB being a 'don't care' bit
	SPI_write(address);		//send low byte of 16-bit address
	SPI_write(data);
	CS = 1;			
	wip_poll();		//confirm no EEPROM write is in progress
}

/*
Write data bytes to EEPROM page from MCU address range
Parameters:
	-start_addr: pointer to MCU starting address with data to be sent
	-cycles: number of byte data to be sent
	-ee_start: starting address of EEPROM page 
*/
void write_eebytes(unsigned char cycles, unsigned int ee_start){
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

/*
Read data byte from EEPROM Page
Parameters:
	-start_addr: 16-bit EEPROM starting page address
	-end_addr: 16-bit EEPROM ending page address
*/
void read_eebytes(unsigned int start_addr, unsigned int end_addr){
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

/*
Read data byte from EEPROM Page
Parameters:
	-addr: 16-bit EEPROM starting page address
*/
void read_eebyte(unsigned int addr){
	read_eebytes(addr, addr);
}

/*
Set interrupts: 
	-Synchronous Serial Port Interrupt Flag; to check for transmission complete
	-Write Collision; to flag when write to buffer command is issued while transmission buffer is full
*/
void interrupt spi_interrupt(){
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

/******************************************
Function: Setup I2C SDA, SCL, and Master Mode
This is to be used before START
********************************************/
void IC_ON(void)
	{
		TRISC,RC3 = 0;
		TRISC,RC4 = 0;
		SSPSTAT = 0XC0;
		SSPCON = 0X28;
		SSPADD = 0X19;
	}

/******************************************
Function: Disable I2C function: SDA, SCL, and 
Master Mode. This is to be used after the STOP
********************************************/
void IC_OFF(void)
	{
		TRISC,RC3 = 0;
		TRISC,RC4 = 0;
		SSPSTAT = 0X00;
		SSPCON = 0X00;
	}

/******************************************
Function: Polls for BF clear.
********************************************/
void POLL_BF_C(void)
	{
		unsigned char dummy;
		while(SSPSTAT,BF == 1)
			{
			}			//Wait till BF clear
		dummy = SSPBUF;	//Dummy read to clear flag
	}

/******************************************
Function: Polls for BF set.
********************************************/
void POLL_BF_S(void)
	{
		unsigned char dummy;
		while(SSPSTAT,BF == 0)
			{
			}			//Wait till BF set		
		dummy = SSPBUF;	//Dummy read to clear flag
	}

/******************************************
Function: Polls for SSPIF set.
********************************************/
void POLL_SSPIF_S(void)
	{
		while(PIR1,SSPIF == 0)
			{
			}			//Wait till BF set		
	}

/******************************************
Function: Polls for SSPIF clear.
********************************************/
void POLL_SSPIF_C(void)
	{
		while(PIR1,SSPIF == 1)
			{
			}			//Wait till BF set		
	}

/******************************************
Function: Generate a START and POLL THE S BIT 
IN SSPSTAT and poll R_W till not in transmission
********************************************/
void IC_START(void)
	{
		unsigned char dummy;
		SSPCON2,SEN = 1;
		while(SSPSTAT,S == 0)
			{
			}			//Loop here till P bit set/START generated
		while(SSPCON2,SEN == 1)
			{
			}			//Wait till clear/no transmission
		POLL_SSPIF_S();
		PIR1,SSPIF = 0;
	}

/******************************************
Function: Generate a START and POLL THE S BIT 
IN SSPSTAT and poll R_W till not in transmission
********************************************/
void CHK_WCOL(void)
	{
		while(SSPCON,WCOL == 1)
			{
			}	//wait till no collision
	}

/******************************************
Function: I2C Write/transmit a byte, Poll the ACK 
after write to the SLAVE
********************************************/
void IC_WB(unsigned char byte)
	{
		POLL_BF_C();
		PIR1,SSPIF = 0;
		SSPBUF = byte;		//Write a byte to I2C Slave
		while(SSPSTAT,R_W == 1)
			{
			}				//Wait till no transmission
		POLL_BF_C();
	}

/******************************************
Function: Poll ACKSTAT bit that must come from
Slave acknowledge
********************************************/
void POLL_ACK(void)
	{
		TRISC,RC4 = 1;
		while(SSPCON2,ACKSTAT == 1)
			{
			}				//Loop in here until clear where Slave ACKed
		TRISC,RC4 = 0;
		POLL_SSPIF_S();
		PIR1,SSPIF = 0;
	}

/******************************************
Function: Generate a STOP, Poll the P BIT IN SSPCON2
till set/STOP generated
********************************************/
void IC_STOP(void)
	{
		unsigned char dummy;
		PIR1,SSPIF = 0;
		SSPCON2,PEN = 1;
		while(SSPSTAT,P == 0)
			{
			}				//Loop in here until set
		while(SSPCON2,PEN == 1)
			{
			}				//Loop in here until PEN clear
		POLL_SSPIF_S();
		PIR1,SSPIF = 0;
		POLL_BF_C();
	}

/******************************************
Function: POLL THE BF bit in SSPSTAT after a read
argument: Polls the buffer bit to ensure it is
set. If set the buffer is full and we have
successfully received all the info, if not
then the information from the read never arrived.
********************************************/
unsigned char I2R()
	{
		unsigned char dummy;
		SSPCON2,ACKEN = 1;	//Generate an ACK to the Slave
		while (SSPCON2,ACKDT == 1)
			{
			}				//Wait till ACKDT clear where ACK sent
		dummy = SSPBUF;		//Dummy read to clear flag and the read byte recived into dummy
		return dummy;
	}

/******************************************
Function: Generate a RESTART, poll the S bit in
SSPSTAT
********************************************/
void IC_REST(void)
	{
		SSPCON2,RSEN = 1;	//Generates the restart to be able to read
		while(SSPSTAT,S == 0)
			{
			}				//Wait until set S=1/RESTART generated
		while(SSPCON2,RSEN == 1)
			{
			}				//Wait till no transmission
		POLL_SSPIF_S();
		PIR1,SSPIF = 0;
	}

/******************************************
Function: Master receive mode setup
********************************************/
unsigned char MS_RECEIVE(void)
	{
		unsigned char RDBYTE;
		SSPCON2,RCEN = 1;	//Enable Master receive
		while(SSPCON2,RCEN == 1)
			{
			}				//Wait until RECEN clear
		POLL_SSPIF_S();
		PIR1,SSPIF = 0;
		POLL_BF_S();
		RDBYTE = SSPBUF;
//		SSPCON,SSPOV = 0;
		POLL_BF_C();
		return RDBYTE;
	}

/**********************************************************
Function: Master sends an ACK to Save
***********************************************************/
void MS_ACK()
	{
		SSPCON2,ACKDT = 0;
		SSPCON2,ACKEN = 1;	//Generate an ACK to the Slave
		while (SSPCON2,ACKEN == 1)
			{
			}				//Wait till ACKEN clear where ACK sent
		POLL_SSPIF_S();
		PIR1,SSPIF = 0;
	}

/******************************************
Function: Master sends a NACK to Slave
********************************************/
void MS_NAK()
	{
		SSPCON2,ACKDT = 1;
		SSPCON2,ACKEN = 1;	//Generate an NACK to the Slave
		while (SSPCON2,ACKEN == 1)
			{
			}				//Wait till ACKEN clear where ACK sent
		POLL_SSPIF_S();
		PIR1,SSPIF = 0;
	}

void main(void)
{	
	ptr = 0x120;
	adc_init();	//initialize Analogue Digital Converter Module
	lcd_init();
	unsigned char high_byte;
	unsigned char low_byte;
	unsigned int _low_byte;
	unsigned char sign_;
	unsigned char mcu_start;						//store the next byte start address in MCU
	unsigned char num_readings = 0xA;				//number of analogue readings to sample
	unsigned char CONFIG_reg;

	unsigned char ee_start = 0x7FBA;				//EEPROM starting address for storage
	unsigned char total_bytes = num_readings*6;		//total number of bytes to be stored: is a multiple of the number of digits from the BCD conversion
	unsigned char ee_end = ee_start + total_bytes;	//last address in the EEPROM to be accessed for storing readings
	unsigned char num_page1_writes;					//number of writes within inital EEPROM Page
	unsigned char num_page2_writes; 				//number of writes within next EEPROM page where there is an overflow to next page

	TRISD = 1;										//configure pin 0 of PORT D as input for toggling between read and write modes	

	switch(RD0){
		case 0:
			
			IC_ON();		//configure MCU for I2C
		
		
			//configuring TCN75A for 11-bit resolution, 4 time fault queue, comparator mode and shut down enable
			IC_START();		//Initiate Start condition on SDA and SCL pins. Poll THE S BIT IN SSPSTAT and poll R_W till not in transmission
			CHK_WCOL();		//ensure no collision
			IC_WB(0x98);	//write slave address with write-bit byte
			POLL_ACK();		//wait for slave to acknowlege 
			IC_WB(0x01);	//write the configuration register pointer
			POLL_ACK();		//wait for slave to acknowledge
			IC_WB(0x40);	//write to the configuration register: 11-bit resolution, 4 time fault queue, comparator mode and shut down disable
			POLL_ACK();		//wait for slave to acknowledge
			IC_STOP();		//Generate a STOP, Poll the P BIT IN SSPCON2 till set/STOP generated
		
	/*	
			//read from TCN75A configuration register
			IC_START();		//Initiate Start condition on SDA and SCL pins. Poll THE S BIT IN SSPSTAT and poll R_W till not in transmission
			CHK_WCOL();		//ensure no collision
			IC_WB(0x98);	//write slave address with write-bit byte
			POLL_ACK();		//wait for slave to acknowlege 
			IC_WB(0x01);	//write the configuration register pointer
			POLL_ACK();		//wait for slave to acknowlege 
			IC_REST();		//initiate repeated start
			IC_WB(0x99);	//write slave address byte with read bit
			POLL_ACK();		//wait for slave to acknowledge
			CONFIG_reg =  MS_RECEIVE(); 	//set master receive bit and pass Ta MSB to data_MSB
			MS_NAK();									//Master Not Acknowledges
			IC_STOP();									//end I2C to free bus
	*/	
			lcd_puts("Logging Tmp in C: ");
			for (unsigned i=0; i<num_readings;i++){		
				//read ambient temperature
				mcu_start = 6*i;//offset mcu address for every TNC75A reading
				IC_START();		//Initiate Start condition on SDA and SCL pins. Poll THE S BIT IN SSPSTAT and poll R_W till not in transmission
				CHK_WCOL();		//ensure no collision
				IC_WB(0x98);	//write slave address with write-bit byte
				POLL_ACK();		//wait for slave to acknowlege 
				IC_WB(0x00);	//write the ambient temperature register pointer
				POLL_ACK();		//wait for slave to acknowlege 
				IC_REST();		//initiate repeated start
				IC_WB(0x99);	//write slave address byte with read bit
				POLL_ACK();		//wait for slave to acknowledge
				high_byte =  MS_RECEIVE(); 	//set master receive bit and pass Ta MSB to data_MSB
				MS_ACK();					//Master sends an ACK to Save
				low_byte =  MS_RECEIVE(); 	//set master receive bit and pass Ta LSB to data_MSB
				MS_NAK();					//Master Not Acknowledges
				IC_STOP();					//end I2C to free bus

				//handle two's complement in case of negative temperature readings
				sign_ = high_byte >>7;	//assign MSB to sign_
				if (sign_){
					high_byte = ~(high_byte -1);
					lcd_putch(0x2D);				//display negative sign
				}

				//convert high byte and low byte to decimal equivalent
				BCD_conv(high_byte, mcu_start);				//convert high_byte to decimal

				//process low_byte for decimal values
				_low_byte = 0;
				if(low_byte){
					if(low_byte>>7){
						//add 500
						_low_byte += 0x1F4;
					}
					if((low_byte>>6) & 1){
						//add 250
						_low_byte += 0xFA;
					}
					if((low_byte>>5) & 1){
						//add 125
						_low_byte += 0x7D;
					}
				}
				BCD_conv(_low_byte, mcu_start+3);	//multiply decimal code by 1000 and then divide by 256 and convert to decimal digits
				
				//format LCD display of readings
				if (ptr[mcu_start]!=0){
					lcd_putch(ASCII(ptr[mcu_start]));		//display first number if non-zero number
				}
				for (unsigned char i=1;i<6;i++){
					lcd_putch(ASCII(ptr[mcu_start+i]));		//display digits of reading
					if (i==2){
						lcd_putch(0x2E);			//display decimal point
					}
				}
				if (i==7){
					lcd_clear();
					lcd_puts("Logging Tmp in C: ");		
				}else if(i!=num_readings-1){
					lcd_puts(", ");
					__delay_ms(5000);
				}else{
					__delay_ms(2000);
				}
			}
			IC_OFF();		//Disable I2C function: SDA, SCL, and Master Mode. This is to be used after the STOP
			SPI_ON();		//setup registers and ports for SPI communication with EEPROM
			ptr2 = ptr;	

			//calculate number of writes from EEPROM start address within page
			while(ee_start<ee_end){
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
			lcd_puts("Write complete");
			break;
		case 1:
			SPI_ON(); 											//setup registers and ports for SPI communication with EEPROM
			//Retrieve and continiously display temperature readings from EEPROM
			while(1){
				lcd_goto(0);
				lcd_puts("Retrieved Tmp in C: ");
				read_eebytes(ee_start, ee_start+total_bytes);	//providing EE start and end address reads from EEPROM
				for(unsigned char i=0; i<num_readings; i++){
					mcu_start = 6*i;
					if (ptr[mcu_start]!=0){
						lcd_putch(ASCII(ptr[mcu_start]));		//display first number if non-zero number
					}
					for (unsigned char i=1;i<6;i++){
						lcd_putch(ASCII(ptr[mcu_start+i]));		//display digits of reading
						if (i==2){
							lcd_putch(0x2E);					//display decimal point
						}
					}
					
					if (i==6){
						lcd_clear();
						lcd_puts("Retrieved Tmp in C: ");		
					}else if(i!=num_readings-1){
						lcd_puts(", ");
						__delay_ms(2000);
					}else{
						__delay_ms(2000);
						lcd_clear();
					}	
				}
			}
			break;
		default:
			break;
}

	for(;;);	
}

