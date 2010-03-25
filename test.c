/*I moved the stuff that used to be up here to header.h*/
#include "header.h"
 
// return line position 
long line_position(unsigned int *s, unsigned int *minv, unsigned int *maxv) { 
  int i;
  long position;
  long numSum = 0;
  long denSum = 0;
  long sense[] = {0, 0, 0, 0, 0};
  
  for(i = 0; i < 5; i+=1){
    sense[i] = (100*((long)s[i]-(long)minv[i]))/((long)maxv[i]-(long)minv[i]);
    numSum += sense[i]*(i-2)*100;
    denSum += sense[i];
  }

  if(denSum == 0)
    denSum += 1;

  position = numSum/denSum;
  return position;
}


int endOfLine(unsigned int *s, unsigned int *minv, unsigned int *maxv) 
{
  int i, k = 0;
  long sense[] = {0, 0, 0, 0, 0};

  for(i = 0; i < 5; i+=1)
    sense[i] = (100*((long)s[i]-(long)minv[i]))/((long)maxv[i]-(long)minv[i]);

  for(i = 0; i < 5; i+=1)
    if( sense[i]>20 )
      k=1;
  return k;
}


void nposition(long lm, long rm, long *dt, long *x, long *y, long *theta){

	
  long new_theta = (motor2angle(lm, rm)*(*dt)) + *theta;
  clear();
  lcd_goto_xy(0,1);
  print_long(new_theta);

  long avg_speed = (lm + rm)/2;
  long avg_theta = (*theta + new_theta)/(2*1000);
  *x = *x + (motor2speed(avg_speed)*(Cos(avg_theta))*(*dt))/(1000);
  *y = *y + (motor2speed(avg_speed)*(Sin(avg_theta))*(*dt))/(1000);
  
  *theta = new_theta;

}

void spin_by(long millimeters){
  //rate of rotation (ror) in (1/10)mm/s
  long ror = motor2angle-(speed, speed);

  //Duration in milliseconds
  long durationInMS = (millimeters*100)/(ror);

  if(durationInMS < 0){
    speed = -speed;
    durationInMS = -durationInMS;
  }


  //Maybe it should be set to (-speed, speed)
  clear();
  print_long(durationInMS);
  lcd_goto_xy(0,1);
  print_long(millimeters);
  
  set_motors(-speed, speed);
  delay_ms(durationInMS);

  set_motors(0, 0);

  delay_ms(500);

}

void goBack(){
  

}


// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
int initialize()
{
  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  pololu_3pi_init(2000);
  load_custom_characters(); // load the custom characters
  // display message
  print_from_program_space(hello);
  lcd_goto_xy(0, 1);
  print("Press B");
  while (!button_is_pressed(BUTTON_B));
  return 0;
}


// This is the main function, where the code starts.  All C programs
// must have a main() function defined somewhere.
int main()
{
  // global array to hold sensor values
  unsigned int sensors[5];
  // global arrays to hold min and max sensor values
  // for calibration
  unsigned int minv[5], maxv[5];
  // line position relative to center
  int i;
  int eol = 0;
  long left; 
  long right;
  long x = 0;
  long y = 0;
  long theta = 0;
  long oldposition = 0;
  long position = 0;
  int kp = 80; //40
  int ut = 0;
  long dt = millis();
  long old_time = millis();
  long running_time = millis();


  // set up the 3pi, and wait for B button to be pressed
  initialize();

  read_line_sensors(sensors,IR_EMITTERS_ON);
  for (i=0; i<5; i++) { minv[i] = maxv[i] = sensors[i]; }    
    
  
  dance(sensors, minv, maxv);

  dt = millis();
  old_time = millis();
  running_time = millis();
  while(1) {
    delay_ms(5);

    //compute delta t (time it takes for the loop to iterate once)
    dt = millis() - old_time;
    old_time = millis();
    
    // Read the line sensor values
    read_line_sensors(sensors, IR_EMITTERS_ON);

    // compute line positon
    oldposition = position;
    position = line_position(sensors, minv, maxv);
    ut = (position*kp)/100;
    
    if (run) {

      left = speed + ut;
      right = speed - ut;
	
      set_motors(left, right);

      nposition(left, right, &dt, &x, &y, &theta);
      
      //      clear();
      //      lcd_goto_xy(0, 0);  
      //      print_long(x);

      //      lcd_goto_xy(0, 1);
      //      print_long(theta);
     
    }
    
    run = endOfLine(sensors, minv, maxv);

    if(!run){
      set_motors(0, 0);
      //      clear();
      //      print_long(millis() - running_time);
      //      lcd_goto_xy(0, 1);
      //      print_long(motor2speed(speed));
      //      calibrate_m2angle();
      //      delay(10000);
      spin_by(theta);
      delay(10000);
    }    

  }
      
}
