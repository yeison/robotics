#ifndef CALIBRATE_A_H_INCLUDED
#define CALIBRATE_A_H_INCLUDED

//#include "3pi_kinematics.h"
//#include <pololu/3pi.h>

const char tocalib[] PROGMEM = "To calib";
const char calibrate[] PROGMEM = "Calibrate";
const char calibrateA[] PROGMEM = "Hold A"; 
const char calibrateB[] PROGMEM = "Press B";
const char calibrateC[] PROGMEM = "Press C";
void calibrate_m2angle(){
  int counter;
  long delT;
  long angle;
  while(!button_is_pressed(BUTTON_B)){
    lcd_goto_xy(0, 0);
    print_from_program_space(calibrate);    
    lcd_goto_xy(0, 1);
    print_from_program_space(calibrateB);
  }

  delay(100);
  delT = millis();
  set_motors(40, -40);
  angle = motor2angle(40, -40);
  for(counter=0; counter < 160; counter++){
    delay_ms(10);
  }
  clear();
  print_long(angle);

  delT = millis() - delT;
  lcd_goto_xy(0,1 );
  print_long(delT);


  set_motors(0, 0);
  delay_ms(4000);

  while(!button_is_pressed(BUTTON_C)){
    lcd_goto_xy(0, 1);
    print_from_program_space(calibrateC);    
  }

  delay_ms(200);

  delT = millis();
  set_motors(80,-80);
  angle = motor2angle(80, -80);
  for(counter=0; counter < 160; counter++){
    delay_ms(10);
  }
  clear();
  print_long(angle);

  delT = millis() - delT;
  lcd_goto_xy(0, 1);
  print_long(delT);

	
  set_motors(0,0);
  while(!button_is_pressed(BUTTON_A));
}

#endif /* FILE_B_H_INCLUDED */
