/*3PI template code for NYU "Intro to Robotics" course. 
  Yann LeCun, 02/2009. 
  This program was modified from an example program from Pololu.  */ 
// The 3pi include file must be at the beginning of any program that 
// uses the Pololu AVR library and 3pi. 
//#include <pololu/orangutan.h> 
#include <pololu/3pi.h> 
#include "3pi_kinematics.h"
#include <avr/pgmspace.h> 
#include <math.h> 
#include "calibrate.h"
 
// This include file allows data to be stored in program space.  The 
// ATmega168 has 16k of program space compared to 1k of RAM, so large 
// pieces of static data should be stored in program space. 
 
// speed of the robot 
int speed = 30; 
int run=0; 
 
// Introductory messages.  The "PROGMEM" identifier  
// causes the data to go into program space. 
const char hello[] PROGMEM = "TACHIKOMA"; 
 
/*Notes 
 *1k of RAM 8k of flash.  PROGMEM places on flash?  More space on flash. 
 *16 bits per int 
 *long is 32 bits for our robot. 
 *FPU - floating point unit - calculates floating point arithmetic. 
 *fixed-point instead of floating points, we use ints. 
 * multiply 1.35, e.g., by 1000 get 1350. 
*/ 
  
// Data for generating the characters used in load_custom_characters 
// and display_readings.  By reading levels[] starting at various 
// offsets, we can generate all of the 7 extra characters needed for a 
// bargraph.  This is also stored in program space. 

const char levels[] PROGMEM = { 
	0b00000, 
	0b00000, 
	0b00000, 
	0b00000, 
	0b00000, 
	0b00000, 
	0b00000, 
	0b11111, 
	0b11111, 
	0b11111, 
	0b11111, 
	0b11111, 
	0b11111, 
	0b11111 
};


char display_characters[9] = { ' ', 0, 1, 2, 3, 4, 5, 6, 255 }; 
 
// This function loads custom characters into the LCD.  Up to 8 
// characters can be loaded; we use them for 7 levels of a bar graph. 
void load_custom_characters() 
{ 
	lcd_load_custom_character(levels+0,0); // no offset, e.g. one bar 
	lcd_load_custom_character(levels+1,1); // two bars 
	lcd_load_custom_character(levels+2,2); // etc... 
	lcd_load_custom_character(levels+3,3); 
	lcd_load_custom_character(levels+4,4); 
	lcd_load_custom_character(levels+5,5); 
	lcd_load_custom_character(levels+6,6); 
	clear(); // the LCD must be cleared for the characters to take effect 
} 
 
// This function displays the sensor readings using a bar graph. 
void display_bars(const unsigned int *s, const unsigned int *minv, const unsigned int* maxv) { 
	// Initialize the array of characters that we will use for the 
	// graph.  Using the space, and character 255 (a full black box). 
	unsigned char i; 
	for (i=0;i<5;i++) { 
		int c = ((int)s[i]-(int)minv[i])*9/((int)maxv[i]-(int)minv[i]); 
		c = (c<0)?0:(c>8)?8:c; 
		// if (i==0) {print_long(s[0]); print_long(c); } 
		print_character(display_characters[c]); 
	} 
} 
 
void update_bounds(const unsigned int *s, unsigned int *minv, unsigned int *maxv) { 
	int i; 
	for (i=0; i<5; i++) {  
		if (s[i]<minv[i]) minv[i] = s[i]; 
		if (s[i]>maxv[i]) maxv[i] = s[i]; 
	} 
} 

struct coords{
  int x;
  int y;
};

typedef struct coords coords;
