/**
 * Names: Kyle Ritchie, Emmett Hitz, Daniel Centore, Adam Stanczyk
 * Section: 4A
 * Date: 2015
 * Filename: lab4.c
 * Description: 
 */

#include <c8051_SDCC.h>// include files. This file is available online
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
#define PW_UR_MIN 2027
#define PW_UR_MAX 3502
#define PW_UR_NEUT 2765
#define PW_EC_MIN 2275
#define PW_EC_MAX 3295
#define PW_EC_NEUT 2805
#define PCA_START 28672

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void Interrupt_Init(void); 	
void PCA_Init (void);
void XBR0_Init(void);
void SMB0_Init(void);
void ADC_Init(void);

void Adjust_Wheels(void);
void Drive_Motor(void);

unsigned int Read_Compass(void);
unsigned int Read_Ranger(void);

unsigned char speed_from_ADC=0;
unsigned char battery_from_ADC=0;

unsigned char read_AD_input(unsigned char n);
void ping_ranger(void);
void PCA_ISR(void) __interrupt 9;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned int UR_PW = PW_UR_NEUT;
unsigned int EC_PW = PW_EC_NEuT;

unsigned int current_range = 0;

unsigned char wait = 0;

unsigned char new_range_flag = 1;
unsigned char new_heading_flag = 1; // flag for count of compass timing

int compass_adj = 0; // correction value from compass
int range_adj = 0; // correction value from ranger

unsigned char r_count = 0; // overflow count for range
unsigned char h_count = 0; // overflow count for heading

unsigned int current_heading = 0;
unsigned int desired_heading = 900;


__sbit __at 0xB6 RUN;

/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/

void main()
{
	Sys_Init(); // Initialize the C8051 board
	putchar(' '); // Required for output to terminal
	Port_Init(); 
	Interrupt_Init();
	PCA_Init();
	ADC_Init();
	XBR0_Init();
	SMB0_Init();
	
	while (wait < 50);
	
	while (1)
	{
		run_stop = 0;

		// stay in loop until switch is in run position
 		while (!RUN)
		{
 			if (run_stop == 0)
 			{
 				UR_PW = PW_UR_NEUT;
				EC_PW = EC_PW_NEUT;
				PCA0CP0 = 0xFFFF - EC_PW;
				PCA0CP2 = 0xFFFF - UR_PW;
 				desired_heading = pick_heading();
				desired_range = pick_range();
				run_stop = 1: // only try to update desired heading once
 			}
 		}
		if (new_heading_flag)
		{
			// If there's a new heading available, read it and update the current value
			Steering_Servo();
			current_heading = ReadCompass();
			printf("Degrees: %i\r\nServo pulsewidth:%u\r\n", current_heading,EC_PW);
			new_heading_flag = 0;

		if (new_range_flag)
		{
			Drive_Motor();
			current_range = Read_Ranger();
			new_range_flag = 0;
			ping_ranger();
			printf("Range: %u\r\nDrive pulsewidth: %u\r\n", current_range, UR_PW);
		}
	}
}


void Port_Init(void)
{
	P0MDOUT = 0xFF;
	P1MDOUT = 0xFC; //set output pin for CEX2 in push-pull mode

	P3MDOUT = 0x00;

	P3 = 0xFF;
}


void Interrupt_Init(void)
{
	EA = 1;
	EIE1 |=0x08;
}


void PCA_Init (void)
{
	PCA0CPM2 = 0xC2;
	PCA0MD = 0x81;
	PCA0CN = 0x40;
}


void XBR0_Init(void)
{
	XBR0 = 0x27;
}

void SMB0_Init(void)
{
	SMB0CR = 0x93;
	ENSMB = 1;
}

void ADC_Init(void)
{
	REF0CN 	 =0x03;
	ADC1CF	|=0x00;
	ADC1CN	 =0x80;
}

void Adjust_Wheels(void)
{
	// Figure out the current error based on the desired and current heading
	signed int error = (signed int) desired_heading - (signed int) current_heading;	
	// Shift the error to be between -1800 and 1800
	if (error > 1800)
		error -= 3600;
	else if (error < -1800)
		error += 3600;	
	// Figure out our desired pulsewidth using our value for k_p
	EC_PW = EC_PW_NEUT + 5 * error / 12;
	
	// Make sure our desired pulsewidth is within the maximum bounds of the servo
	if (EC_PW > EC_PW_MAX)
		EC_PW = EC_PW_MAX;
	else if (EC_PW < EC_PW_MIN)
		EC_PW = EC_PW_MIN;
	
	// Update PCA pulsewidth
    PCA0CP0 = 0xFFFF - EC_PW;
}

void Drive_Motor(void)
{
	if (current_range <= 10)
	{
		UR_PW=PW_UR_MIN;
	}
	else if (current_range >= 40 && current_range <= 50)
	{
		UR_PW=PW_UR_NEUT;
	}
	else if (current_range>=90)
	{
		UR_PW=PW_UR_MAX;
	}
	else
	{
		UR_PW = PW_UR_MIN + 18 * current_range;
	}

	PCA0CP2 = 0xFFFF - UR_PW;
}

unsigned int Read_Compass(void)
{
	unsigned char addr = 0xC0; // the address of the sensor, 0xC0 for the compass
	unsigned char Data[2]; // Data is an array with a length of 2
	unsigned int heading; // the heading returned in degrees between 0 and 3599
	i2c_read_data(addr, 2, Data, 2); // read two byte, starting at reg 2
	heading =(((unsigned int)Data[0] << 8) | Data[1]); //combine the two values
	//heading has units of 1/10 of a degree 
	return heading; // the heading returned in degrees between 0 and 3599
}

unsigned int Read_Ranger(void)
{
	unsigned char Data[2];
	unsigned int range =0;
	// the address of the ranger is 0xE0
	unsigned char addr=0xE0;
	i2c_read_data(addr, 2, Data, 2); // read two bytes, starting at reg 2
	range = (((unsigned int)Data[0] << 8) | Data[1]);
	return range;
}

void ping_ranger(void)
{	
	// write 0x51 to reg 0 of the ranger:
	unsigned char Data[1];
	unsigned char addr=0xE0;
	Data[0]=0x51;
	i2c_write_data(addr, 0, Data , 1) ; // write one byte of data to reg 0 at addr
}

unsigned char read_AD_input(unsigned char n)
{
	AMX1SL = n; // Set P1.n as the analog input for ADC1
	ADC1CN = ADC1CN & ~0x20; // Clear the “Conversion Completed” flag
	ADC1CN = ADC1CN | 0x10; // Initiate A/D conversion
	while ((ADC1CN & 0x20) == 0x00);// Wait for conversion to complete
	return ADC1; // Return digital value in ADC1 register
}

void PCA_ISR(void)__interrupt 9
{
	wait++;
	if (CF)
	{
		CF = 0;
		PCA0 = PCA_START;
		
		h_count++;
		if (h_count >= 2)
		{
			new_heading_flag = 1;
			h_count = 0;
		}
		
		r_count++;
		if (r_count >= 4)
		{
			new_range_flag = 1;
			r_count = 0;
		}
	}

	PCA0CN &= 0xC0;

}
