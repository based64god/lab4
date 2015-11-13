/**
 * Names: Kyle Ritchie, Emmett Hitz, Daniel Centore, Adam Stanczyk
 * Section: 4A
 * Date: 2015
 * Filename: lab4.c
 * Description: TODO
 */

#include <c8051_SDCC.h>// include files. This file is available online
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
#define DRIVE_PW_MIN 2027
#define DRIVE_PW_MAX 3502
#define DRIVE_PW_NEUT 2765
#define STEER_PW_MIN 2275
#define STEER_PW_MAX 3295
#define STEER_PW_NEUT 2785
#define PCA_START 28672

#define DIST_MAX (50+18)				// Distance to begin steering at
#define DIST_AVOID_MIN (20+18)		// Distance before giving up at max steering
#define DIST_STOP (12+18)				// Distance to stop under any conditions

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void Interrupt_Init(void); 	
void PCA_Init (void);
void XBR0_Init(void);
void SMB0_Init(void);
void ADC_Init(void);

void Pick_Heading(void);

void Adjust_Wheels(void);
void Drive_Motor(void);

unsigned int Read_Compass(void);
unsigned int Read_Ranger(void);

unsigned char read_AD_input(unsigned char n);
void Ping_Ranger(void);
void PCA_ISR(void) __interrupt 9;

void Update_Battery(void);
void Pick_S_Gain(void);
void Update_Speed(void);
void Update_Speed(void);
void Process(void);
//void Steering_Servo(void);
unsigned int Read_Compass(void);
void Paused_LCD(void);
void Update_LCD(void);
void Steering_Goal(void);
void printDebug(void);



//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

unsigned int speed_from_pot = 0;
unsigned char battery_level = 0;

unsigned int drive_pw = DRIVE_PW_NEUT;
unsigned int steer_pw = STEER_PW_NEUT;

unsigned int current_range = 0;

unsigned int wait = 0;

unsigned char new_range_flag = 0;
unsigned char new_heading_flag = 0; // flag for count of compass timing
unsigned char new_battery_flag = 0;
unsigned char new_LCD_flag = 0;

int compass_adj = 0; // correction value from compass
int range_adj = 0; // correction value from ranger

unsigned char r_count = 0; // overflow count for range
unsigned char h_count = 0; // overflow count for heading
unsigned char b_count = 0; // overflow count for battery reading
unsigned char l_count = 0; // overflow count for LCD reading

signed int current_heading = 0;
signed int desired_heading = 900;

unsigned char steering_gain = 0;

__sbit __at 0xB7 RUN;

/********************************************************************/

void main()
{
	Sys_Init(); // Initialize the C8051 board
	putchar(' '); // Required for output to terminal
	Port_Init(); 
	PCA_Init();
	ADC_Init();
	XBR0_Init();
	SMB0_Init();
	
	printf("\r\nSTART\r\n");
	
	while (wait < 50);
	
	//Update_Battery();
	
	printf("Looking for heading...\r\n");
	Pick_Heading();
	Pick_S_Gain();
	
	Update_Speed();
	
	while (1)
		Process();
}

void Process()
{
	// Wait in neutral until switch is in run position
	if (!RUN)
	{
		drive_pw = DRIVE_PW_NEUT;
		steer_pw = STEER_PW_NEUT;
		PCA0CP0 = 0xFFFF - steer_pw;
		PCA0CP2 = 0xFFFF - drive_pw;
		while (!RUN)
		{
			Paused_LCD();
		}
	}

	if (new_heading_flag)
	{
		// If there's a new heading available, read it and update the current value
		//Steering_Servo();
		
		//printf("READING HEADING\r\n");
		current_heading = Read_Compass();
		//printf("DONE READING HEADING\r\n");
		
		//printf("Degrees: %i\r\nServo pulsewidth:%u\r\n", current_heading, steer_pw);
		new_heading_flag = 0;
	}
	
	if (new_range_flag)
	{
		Drive_Motor();
		
		current_range = Read_Ranger();
		new_range_flag = 0;
		Ping_Ranger();
		//printf("Range: %u\r\nDrive pulsewidth: %u\r\n", current_range, drive_pw);
	}
	
	if (new_battery_flag)
	{
		Update_Battery();
		Update_Speed();
		
		new_battery_flag = 0;
	}
	
	if (new_LCD_flag)
	{
		Update_LCD();
		
		new_LCD_flag = 0;
	}
}

void Paused_LCD(void)
{
	lcd_clear();
	lcd_print("Car Ready\n");
}

void Update_LCD(void)
{
	lcd_clear();
	lcd_print("Heading: %d\n", current_heading / 10);
	lcd_print("Range: %d\n", current_range);
	lcd_print("Steer: %ld%%\n", ((signed long) 100 * (signed long) ((signed long) steer_pw - (signed long) STEER_PW_NEUT)) / (signed long) (STEER_PW_MAX - STEER_PW_NEUT));
	lcd_print("Battery: %d\n", 15 * battery_level / 244);
}

void Pick_Heading(void)
{
	do {
		lcd_clear();
		lcd_print("Heading (0-360):");
			
		desired_heading = kpd_input(1) * 10;
	} while (desired_heading > 3600);
	
}

void Pick_S_Gain(void)
{
	lcd_clear();
	lcd_print("Steering Gain (~24):");
		
	steering_gain = kpd_input(1);
}

void Drive_Motor(void)
{
	unsigned int temp_steer_pw;
	
	if (current_range >= DIST_MAX)
	{
		// Going toward heading
		drive_pw = speed_from_pot;
		
		Steering_Goal();
	}
	else if (current_range <= DIST_STOP || (current_range <= DIST_AVOID_MIN && steer_pw == STEER_PW_MIN))
	{
		// Stopping conditions
		drive_pw = DRIVE_PW_NEUT;
	}
	else
	{
		// Steering around obstacle
		drive_pw = speed_from_pot;
		
		// Calculate steering to go around obstacle
		temp_steer_pw = (STEER_PW_NEUT - steering_gain * (DIST_MAX - current_range));
		
		// Calculate steering toward heading for comparison
		Steering_Goal();
		
		// Only adjust the steering for the obstacle if the steering to go around the obstacle is
		//   more extreme than the steering to just continue toward the heading
		if (temp_steer_pw < steer_pw)
			steer_pw = temp_steer_pw;
	}
	
	PCA0CP2 = 0xFFFF - drive_pw;
	
	// Make sure our desired steering pulsewidth is within the maximum bounds of the servo
	if (steer_pw > STEER_PW_MAX)
		steer_pw = STEER_PW_MAX;
	else if (steer_pw < STEER_PW_MIN)
		steer_pw = STEER_PW_MIN;
	
	// Update PCA pulsewidth
    PCA0CP0 = 0xFFFF - steer_pw;
    
    // Print debug information to the console
    printDebug();
}

void printDebug(void)
{
	// Figure out the current error based on the desired and current heading
	signed int steer_error = (signed int) desired_heading - (signed int) current_heading;	
	// Shift the error to be between -1800 and 1800
	if (steer_error > 1800)
		steer_error -= 3600;
	else if (steer_error < -1800)
		steer_error += 3600;
		
	printf("%d, %d, %d, %d, %d, %u\r\n", wait*20+((PCA0 - PCA_START)/(65535 - PCA_START))*20 , steer_error, current_range, current_heading
			, (100 * (steer_pw - STEER_PW_NEUT)) / (STEER_PW_MAX - STEER_PW_NEUT), drive_pw
		);
}

void Steering_Goal(void)
{
	// Figure out the current error based on the desired and current heading
	signed int error = (signed int) desired_heading - (signed int) current_heading;	
	// Shift the error to be between -1800 and 1800
	if (error > 1800)
		error -= 3600;
	else if (error < -1800)
		error += 3600;
	
	// Figure out our desired pulsewidth using our value for k_p
	steer_pw = STEER_PW_NEUT + 5 * error / 12;
}

void Port_Init(void)
{
	P0MDOUT = 0xFF;
	
	P1MDOUT = 0x0F;
	P1 |= ~0x0F;
	P1MDIN = 0x3F;

	P3MDOUT = 0x00;
	P3 = 0xFF;
}

void PCA_Init (void)
{
	PCA0MD = 0x81;
	PCA0CPM0 = 0xC2; // 16 bit, enable compare, enable PWM
	PCA0CPM2 = 0xC2;
	EIE1 = 0x08;
	PCA0CN |= 0x40;
	EA = 1;
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
	REF0CN = 0x03;
	ADC1CF |= 0x01;
	ADC1CN = 0x80;
}

unsigned int Read_Compass(void)
{
	unsigned char addr = 0xC0; // the address of the sensor, 0xC0 for the compass
	unsigned char Data[2]; // Data is an array with a length of 2
	unsigned int heading; // the heading returned in degrees between 0 and 3599
	i2c_read_data(addr, 2, Data, 2); // read two byte, starting at reg 2
	heading =(((unsigned int) Data[0] << 8) | Data[1]); //combine the two values
	//heading has units of 1/10 of a degree 
	return heading; // the heading returned in degrees between 0 and 3599
}

unsigned int Read_Ranger(void)
{
	unsigned char Data[2];
	unsigned int range = 0;
	// the address of the ranger is 0xE0
	unsigned char addr = 0xE0;
	i2c_read_data(addr, 2, Data, 2); // read two bytes, starting at reg 2
	range = (((unsigned int) Data[0] << 8) | Data[1]);
	
	//return 90;
	return range;
}

void Ping_Ranger(void)
{	
	// write 0x51 to reg 0 of the ranger:
	unsigned char Data[1];
	unsigned char addr = 0xE0;
	Data[0] = 0x51;
	i2c_write_data(addr, 0, Data , 1) ; // write one byte of data to reg 0 at addr
}

void Update_Speed(void)
{
	AMX1SL = 5; // Set P1.n as the analog input for ADC1
	
	printf("# SWITCHING TO POT CHANNEL...\r\n");
	//printf("# SWITCHING TO POT CHANNEL...\r\n");
	
	ADC1CN = ADC1CN & ~0x20; // Clear the “Conversion Completed” flag
	ADC1CN = ADC1CN | 0x10; // Initiate A/D conversion
	while ((ADC1CN & 0x20) == 0x00);// Wait for conversion to complete
	//speed_from_pot = ((DRIVE_PW_MAX-DRIVE_PW_MIN)/255)*ADC1+DRIVE_PW_MIN; 
	//printf("ADC1: %d\r\n", ADC1);
	speed_from_pot = ((unsigned long) (3502 - 2028) * (unsigned long) ADC1) / (unsigned long) 255 + (unsigned long) 2028;
	//speed_from_pot = ADC1;
}

void Update_Battery(void)
{
	AMX1SL = 7; // Set P1.n as the analog input for ADC1
	
	printf("# SWITCHING TO BATTERY CHANNEL...\r\n");
	//printf("# SWITCHING TO BATTERY CHANNEL...\r\n");

	ADC1CN = ADC1CN & ~0x20; // Clear the “Conversion Completed” flag
	ADC1CN = ADC1CN | 0x10; // Initiate A/D conversion
	while ((ADC1CN & 0x20) == 0x00);// Wait for conversion to complete
	battery_level = ADC1; // Return digital value in ADC1 register
	
	printf("# Battery level: %u\r\n", battery_level);
}

void PCA_ISR(void) __interrupt 9
{
	wait++;
	
	if (CF)
	{
		CF = 0;
		PCA0 = PCA_START;
		
		++h_count;
		if (h_count >= 2)		// 40 ms
		{
			new_heading_flag = 1;
			h_count = 0;
		}
		
		++r_count;
		if (r_count >= 4)		// 80 ms
		{
			new_range_flag = 1;
			r_count = 0;
		}
		
		++l_count;
		if (l_count >= 20)		// 400 ms
		{
			new_LCD_flag = 1;
			l_count = 0;
		}
		
		++b_count;
		if (b_count >= 50)		// 1 second
		{
			new_battery_flag = 1;
			b_count = 0;
		}
	}

	PCA0CN &= 0xC0;
}
