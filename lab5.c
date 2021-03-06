/**
 * Names: Kyle Ritchie, Emmett Hitz, Daniel Centore, Adam Stanczyk
 * Section: 4A
 * Date: 2015
 * Filename: lab5.c
 * Description: In lab 5, an accelerometer was utilized to provide the feedback
 *              signal instead of the ranger and compass. The car drove down a
 *              ramp until it was on level ground and once it was there, it stopped.
 */

#include <c8051_SDCC.h>// include files. This file is available online
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

// Pulsewidth constants
#define DRIVE_PW_MIN 2027
#define DRIVE_PW_NEUT 2765
#define DRIVE_PW_MAX 3502
#define STEER_PW_MIN 2275
#define STEER_PW_NEUT 2785
#define STEER_PW_MAX 3295

// Constants for the average X and Y accelerometer readings when on flat ground
#define NEUTRAL_X (28)
#define NEUTRAL_Y (-8)

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
void Pick_S_Gain(void);
void Pick_Heading(void);
void Adjust_Wheels(void);
void Drive_Motor(void);
void Read_Accel(void);
void PCA_ISR(void) __interrupt 9;
void Paused_LCD(void);
void Update_LCD(void);
void Update_Battery(void);
void set_servo_PWM(void);
void set_drive_PWM(void);
void Process(void);
void Steering_Goal(void);
void printDebug(void);
void setGains(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

unsigned char battery_level = 0;          // Battery voltage in volts

unsigned int drive_pw = DRIVE_PW_NEUT;    // The actual pulsewidth we are driving at
unsigned int steer_pw = STEER_PW_NEUT;    // The actual pulsewidth we are steering at

unsigned int wait = 0;                    // Elapsed 20ms ticks

unsigned char new_battery_flag = 0;       // Flag to indicate new battery voltage (1s)
unsigned char new_LCD_flag = 0;           // Flag to indicate we should update the LCD (400ms)
unsigned char new_accel_flag = 0;         // Flag to indicate we need to read from the accelerometer (20ms)
unsigned char new_debug_flag = 0;         // Flag to indicate we need to print to debug (60ms)

unsigned char b_count = 0;                // overflow count for battery reading
unsigned char l_count = 0;                // overflow count for LCD reading
unsigned char d_count = 0;                // overflow count for printing to debug

unsigned char i_accel = 0;                // The current x_accel and y_accel array positions
signed int x_accel[5];                    // Last 5 accelerometer horizontal tilt readings
signed int y_accel[5];                    // Last 5 accelerometer front-back pitch readings
signed int avg_X;                         // Current average horizontal tilt reading
signed int avg_Y;                         // 

unsigned char steering_gain = 0;          // The steering gain (from keypad)
unsigned char drive_x_gain = 0;           // The steering gain (from keypad)
unsigned char drive_y_gain = 0;           // The steering gain (from keypad)

__sbit __at 0xB7 RUN;                     // Run switch

/********************************************************************/

void main()
{
	unsigned char i = 0;
	
	Sys_Init(); // Initialize the C8051 board
	putchar(' '); // Required for output to terminal
	Port_Init(); 
	PCA_Init();
	ADC_Init();
	XBR0_Init();
	SMB0_Init();
	Accel_Init();
	
	printf("\r\nSTART\r\n");
	
	// Initialize default acceleration values to 0
	for (i = 0; i < 4; ++i)
	{
		x_accel[i] = 0;
		y_accel[i] = 0;
	}
	
	// Wait for ADC and motors to be ready
	while (wait < 50);
	
	// Select gains from keypad
	setGains();
	
	// Print current gains for reference
	printf("Gains: S: %d Dx: %d Dy: %d\r\n", steering_gain, drive_x_gain, drive_y_gain);
	
	// Run main loop
	while (1)
		Process();
}

void Process()
{
	// Wait in neutral until switch is in run position
	if (!RUN)
	{
		// Set drive and steering to neutral
		
		drive_pw = DRIVE_PW_NEUT;
		steer_pw = STEER_PW_NEUT;
		PCA0CP0 = 0xFFFF - steer_pw;
		PCA0CP2 = 0xFFFF - drive_pw;
		
		while (!RUN)
		{
			// Display "Car Ready" on the LCD
			if (new_LCD_flag)
			{
				Paused_LCD();
				new_LCD_flag = 0;
			}
		}
	}
	
	// Read from accelerometer and update motor and drive pwm (every 20ms)
	if (new_accel_flag)
	{
		Read_Accel();
		
		set_servo_PWM();
		set_drive_PWM();
		
		new_accel_flag = 0;
	}
	
	// Update battery and speed from ADC (every 1s)
	if (new_battery_flag)
	{
		Update_Battery();
		new_battery_flag = 0;
	}
	
	// Update LCD info (every 400ms)
	if (new_LCD_flag)
	{
		Update_LCD();
		new_LCD_flag = 0;
	}
	
	// Print debug info (every 60ms)
	if (new_debug_flag)
	{
		printDebug();
		new_debug_flag = 0;
	}
}

// Sets the PWM of the servo based on horizontal tilt
void set_servo_PWM(void)
{
	long temp;
	
	// Calculate the pulsewidth given the horizontal tilt
	temp = (long) STEER_PW_NEUT + (long) steering_gain * (long) avg_X;
	
	// Make sure the pulsewidth is within the bounds
	if (temp > STEER_PW_MAX)
		temp = STEER_PW_MAX;
	if (temp < STEER_PW_MIN)
		temp = STEER_PW_MIN;
		
	// If servo pulsewidth is close to neutral, set it to neutral
	if (temp < STEER_PW_NEUT * 11 / 10 && temp > STEER_PW_NEUT * 9 / 10)
		temp = STEER_PW_NEUT;
	
	steer_pw = temp;
	
	// Update PCA pulsewidth steering
    PCA0CP0 = 0xFFFF - steer_pw;
}

// Sets the PWM of the drive motor based on horizontal and vertical tilts
void set_drive_PWM(void)
{
	long temp;
	
	// Calculate the pulsewidth given the horizontal and vertical tilts
	temp = (long) DRIVE_PW_NEUT + (long) drive_y_gain * avg_Y - (long) drive_x_gain * abs(avg_X);
	
	// Make sure the pulsewidth is within the bounds
	if (temp > DRIVE_PW_MAX)
		temp = DRIVE_PW_MAX;
	if (temp < DRIVE_PW_MIN)
		temp = DRIVE_PW_MIN;
	
	drive_pw = temp;
	
	// Actually update drive pw
	PCA0CP2 = 0xFFFF - drive_pw;
}

// Print "Car ready" to the LCD
void Paused_LCD(void)
{
	lcd_clear();
	lcd_print("Car Ready\n");
}

// Update LCD with current info
void Update_LCD(void)
{
	lcd_clear();
	lcd_print("gX:  %d\n", avg_X);
	lcd_print("gY:  %d\n", avg_Y);
	lcd_print("sPW: %d\n", steer_pw);
	lcd_print("dPW: %d%%\n", drive_pw);
}

// Asks user for the 3 gains
void setGains(void)
{
	lcd_clear();
	lcd_print("Steering Gain (~3):");
		
	steering_gain = kpd_input(1);
	
	lcd_clear();
	lcd_print("Drive X Gain (~3):");
		
	drive_x_gain = kpd_input(1);
	
	lcd_clear();
	lcd_print("Drive Y Gain (~3):");
		
	drive_y_gain = kpd_input(1);
}

// Prints out some info to the console
void printDebug(void)
{
	printf("%d, %d, %d, %d, %d\r\n"
			, wait * 20
			, avg_X
			, avg_Y
			, steer_pw
			, drive_pw
		);
}

// Initialize ports
void Port_Init(void)
{
	P0MDOUT = 0xFF;
	
	P1MDOUT = 0x0F;
	P1 |= ~0x0F;
	P1MDIN = 0x3F;

	P3MDOUT = 0x00;
	P3 = 0xFF;
}

// Initialize PCS
void PCA_Init (void)
{
	PCA0MD = 0x81;
	PCA0CPM0 = 0xC2; // 16 bit, enable compare, enable PWM
	PCA0CPM2 = 0xC2;
	EIE1 = 0x08;
	PCA0CN |= 0x40;
	EA = 1;
}

// Initialize crossbar
void XBR0_Init(void)
{
	XBR0 = 0x27;
}

// Initialize SMB
void SMB0_Init(void)
{
	SMB0CR = 0x93;
	ENSMB = 1;
}

// Initialize ADC
void ADC_Init(void)
{
	REF0CN = 0x03;
	ADC1CF |= 0x01;
	ADC1CN = 0x80;
}

// Reads current battery 0-255 from ADC
void Update_Battery(void)
{
	AMX1SL = 7; // Set P1.n as the analog input for ADC1
	
	printf("# SWITCHING TO BATTERY CHANNEL...\r\n");

	ADC1CN = ADC1CN & ~0x20; // Clear the “Conversion Completed” flag
	ADC1CN = ADC1CN | 0x10; // Initiate A/D conversion
	while ((ADC1CN & 0x20) == 0x00);// Wait for conversion to complete
	battery_level = ADC1; // Return digital value in ADC1 register
	
	printf("# Battery level: %u\r\n", battery_level);
}

// Reads the next values from the accelerometer and updates the current averages
void Read_Accel(void)
{
	unsigned char addr = 0x30;
	unsigned char Data[4];
	
	// See if new accelerometer readings are available
	i2c_read_data(addr, 0x27, Data, 1);
	
	// If not, exit
	if ((Data[0] & 0x03) != 0x03)
		return;
	
	// Read new accelerometer values
	i2c_read_data(addr, 0x28 | 0x80, Data, 4);
	
	// Proceed to the next position in the arrays
	++i_accel;
	if (i_accel >= 4)
		i_accel = 0;
	
	// Set the values of the next positions in the arrays to the normalized X and Y accelerations
	x_accel[i_accel] = ((Data[1] << 8) >> 4) - NEUTRAL_X;
	y_accel[i_accel] = ((Data[3] << 8) >> 4) - NEUTRAL_Y;
	
	// Set avg_X and avg_Y to the mean of the last 5 x and y accelerations, respectively
	avg_X = (x_accel[0] + x_accel[1] + x_accel[2] + x_accel[3] + x_accel[4]) / 5;
	avg_Y = (y_accel[0] + y_accel[1] + y_accel[2] + y_accel[3] + y_accel[4]) / 5;
}

void PCA_ISR(void) __interrupt 9
{
	++wait;
	
	if (CF)
	{
		CF = 0;
		PCA0 = PCA_START;
		
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
		
		++d_count;
		if (d_count >= 3)		// 60 ms
		{
			new_debug_flag = 1;
			d_count = 0;
		}

		new_accel_flag = 1;		// 20 ms
	}

	PCA0CN &= 0xC0;
}
