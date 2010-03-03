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

// Make a little dance: Turn left and right
void dance(unsigned int *s, unsigned int *minv, unsigned int *maxv) {
	int counter;
	for(counter=0;counter<80;counter++){
	  read_line_sensors(s, IR_EMITTERS_ON);
	  update_bounds(s, minv, maxv);
		if(counter < 20 || counter >= 60)
			set_motors(40,-40);
		else
			set_motors(-40,40);
		// Since our counter runs to 80, the total delay will be
		// 80*20 = 1600 ms.
		delay_ms(20);
	}
	
	set_motors(0,0);
}


long theta;
long theta_i;

long theta_new(long dt, long left, long right){
  return theta + motor2angle(left, right) * dt;
}

long alpha(){
  //Remember maybe we might need to subtract alpha from 90
  return (theta + theta_i)/2;
}

long theta = 0;
long theta_i = 0;

coords robot_position(dt, x_i, y_i){
  coords robot_pos;
  robot_pos.x = motor2speed(speed)*dt*Sin(alpha()) + x_i;
  robot_pos.y = motor2speed(speed)*dt*Cos(alpha()) + y_i;
  return robot_pos;
}

coords runIt(long val, long dt) {
  coords position;
  position.x = 0;
  position.y = 0;
  long speed_left;
  long speed_right;
  long alphav;

  speed_left = speed + val;
  speed_right = speed - val;
  set_motors(speed_left, speed_right);
  theta = motor2angle(speed_left, speed_right);
  theta_i = theta_new(dt, speed_left, speed_right);
  alphav = alpha();
  
  position = robot_position(dt, position.x, position.y);
  return position;
  
}


int EndOfLine(unsigned int *s, unsigned int *minv, unsigned int *maxv) 
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



// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
void initialize()
{
  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  pololu_3pi_init(2000);
  load_custom_characters(); // load the custom characters
  // display message
  print_from_program_space(hello);
  lcd_goto_xy(0,1);
  print("Press B");
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
  int difference = 0;
  int oldposition = 0;
  int position = 0;
  long integral = 0;
  int kc = 85; //Kc seems to be 60 using current values.
  int kp = 144; //40
  int kd = 0; //10
  int ki = 0; //100
  int ut = 0;
  int dt = 0;
  int *editable = &kd;
  coords robsp;

  // set up the 3pi, and wait for B button to be pressed
  initialize();

  read_line_sensors(sensors,IR_EMITTERS_ON);
  for (i=0; i<5; i++) { minv[i] = maxv[i] = sensors[i]; }    
    

  // Display calibrated sensor values as a bar graph.
  while (!button_is_pressed(BUTTON_B)){
    clear();
    print_from_program_space(tocalib);
    lcd_goto_xy(0,1);
    print_from_program_space(calibrateA);    
    delay(50);
    if(button_is_pressed(BUTTON_A))
      calibrate_m2angle();
    read_line_sensors(sensors,IR_EMITTERS_ON);
    position = line_position(sensors, minv, maxv);
    //      lcd_goto_xy(4,0);
    //      print_long(minv[0]);
    //display_bars(sensors, minv, maxv);
    delay(50);
  }
  
  delay(50);  
  dance(sensors, minv, maxv);


  while(1) {
    if (button_is_pressed(BUTTON_B)) {
      delay(100);
      run = 1-run; 
      ut = 0;
      difference = 0;
      oldposition = 0;
      position = 0;
      integral = 0;
      dt = millis();
      delay(200);
    }

    if (button_is_pressed(BUTTON_A)) { 
      //speed -= 10; delay(100);
      *editable = *editable - 1;
      delay(100);
    }

    if (button_is_pressed(BUTTON_C)) { 
      *editable = *editable + 1;
      //speed += 10; 
      delay(100);
      // play_switch += 1;
    }
    //compute delta t (time it takes for the loop to iterate once)
    dt = millis() - dt;
    
    // Read the line sensor values
    read_line_sensors(sensors, IR_EMITTERS_ON);
    // update minv and maxv values,
    // and put normalized values in v
    //update_bounds(sensors, minv, maxv);

    //update_bounds(sensors, minv, maxv);

    // compute line positon
    oldposition = position;
    position = line_position(sensors, minv, maxv);
    difference = (position - oldposition)/dt;
    
    if (run) {
      integral = integral + (((position + oldposition)/2)*dt);
      ut = (position*kp + difference*kd + integral/ki)/100;
      robsp = runIt(ut, dt);
      clear();
      lcd_goto_xy(0, 0);  
      print_long(robsp.x);
      lcd_goto_xy(4, 0);
      print_long(robsp.y);
    }

    if(!run) {
      set_motors(0, 0);
      // display bargraph
      //clear();
      //print_long(*editable);
      //lcd_goto_xy(4, 0);  
      //print_long(ut);
      lcd_goto_xy(0,1);
      for (i=0; i<8; i++) { print_character(display_characters[i]); }
      display_bars(sensors, minv, maxv);
    }    
   
  }
}
