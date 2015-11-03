/*  Names: Kyle Ritchie and Emmett Hitz
		   Daniel Centore and Adam Stancyzk
    Section: 4A
    Date: 11/3/15
    File name: Lab4
    Description: FULL ON CAR MODE
    KANYE CODE: COLLEGE DROPOUT SON
*/
/*
  read range from ranger and use it to control the motor
*/

#include <c8051_SDCC.h>// include files. This file is available online
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
#define PW_UR_MIN 2027
#define PW_UR_MAX 3502
#define PW_UR_NEUT 2765
#define PW_EC_MIN xxx
#define PW_EC_MAX xxx
#define PW_EC_NEUT xxx
#define PCA_START 28672

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void Interrupt_Init(void); 	
void PCA_Init (void);
void XBR0_Init(void);
void SMB0_Init(void);
void Drive_Motor(void);
unsigned int Read_Ranger(void);
void ping(void);
void PCA_ISR(void)__interrupt 9;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned int UR_PW=PW_UR_NEUT;
unsigned int current_range=0;
unsigned char wait,counter_80ms=0;
unsigned char new_range_flag=1;

__sbit __at 0xB6 SS;

/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/

main()
{
	Sys_Init(); // Initialize the C8051 board
	putchar(' '); // Required for output to terminal
	Port_Init(); 
	Interrupt_Init();
	PCA_Init();
	XBR0_Init();
	SMB0_Init();
	while(wait<50);
	while (1)
	{
		if (!SS)
		{
			UR_PW=PW_UR_NEUT;
			PCA0CP2 = 0xFFFF - UR_PW;
			while(!SS);
		}

		if (new_range_flag)
		{
			Drive_Motor();
			current_range=Read_Ranger();
			new_range_flag=0;
			ping();
			printf("Range: %u\r\nDrive pulsewidth: %u\r\n",current_range,UR_PW);
		}
	}
}


void Port_Init(void)
{
	P0MDOUT = 0xFF;
	P1MDOUT = 0xFF;//set output pin for CEX2 in push-pull mode
	P3MDOUT = 0x00;

	P3		= 0xFF;
}


void Interrupt_Init(void)
{
	EA=1;
	EIE1 |=0x08;
}


void PCA_Init (void)
{
	PCA0CPM2=0xC2;
	PCA0MD=0x81;
	PCA0CN=0x40;
}


void XBR0_Init(void)
{
	XBR0 = 0x27;
}

void SMB0_Init(void)
{
	SMB0CR=0x93;
	ENSMB=1;
}


void Drive_Motor(void)
{
	if(current_range<=10) 								UR_PW=PW_UR_MAX;
	else if (current_range>=40 && current_range <=50) 	UR_PW=PW_UR_NEUT;
	else if (current_range>=90) 						UR_PW=PW_UR_MIN;
	else 												UR_PW=PW_UR_MAX-18*current_range;

	PCA0CP2 = 0xFFFF - UR_PW;
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

void ping(void)
{	
	// write 0x51 to reg 0 of the ranger:
	unsigned char Data[1];
	unsigned char addr=0xE0;
	Data[0]=0x51;
	i2c_write_data(addr, 0, Data , 1) ; // write one byte of data to reg 0 at addr
}


void PCA_ISR(void)__interrupt 9
{
	wait++;
	counter_80ms++;
	if(CF)
	{
		CF=0;
		PCA0=PCA_START;
		if(counter_80ms>=4)
		{
			counter_80ms=0;
			new_range_flag=1; // 4 overflows is about 80 ms
		}
	}

	PCA0CN &= 0xC0;

}