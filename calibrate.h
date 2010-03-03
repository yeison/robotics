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
  
  while(!button_is_pressed(BUTTON_B)){
    lcd_goto_xy(0, 0);
    print_from_program_space(calibrate);    
    lcd_goto_xy(0, 1);
    print_from_program_space(calibrateB);
  }

  delay(200);

  show_spin(40, 160);

  delay_ms(4000);

  while(!button_is_pressed(BUTTON_C)){
    lcd_goto_xy(0, 1);
    print_from_program_space(calibrateC);    
  }

  delay_ms(200);

  show_spin(80, 80);

  while(!button_is_pressed(BUTTON_A));
}

void show_spin(int speed, int duration){
  int counter;
  long delT;
  long angle;  

  delT = millis();
  set_motors(speed, -speed);
  angle = motor2angle(speed, -speed);
  for(counter=0; counter < duration; counter++){
    delay_ms(10);
  }
  set_motors(0,0);

  clear();
  print_long(angle);

  delT = millis() - delT;
  lcd_goto_xy(0, 1);
  print_long(delT);

}


#endif /* FILE_B_H_INCLUDED */
