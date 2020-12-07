 //Description----------------------------------------------------------------------------------------------------
 
/* 
 * University of Cape Town
 * Department of Mechanical Engineering
 * 
 * MEC4110W - Final Year Project
 * 
 * Project: Wheeled robot for testing stabilized platforms
 * 
 * Student: Andrew Court - CRTAND010
 * Supervisor: Hennie Mouton
 * 
 * Code version: V_15.1.2 
 */

//Defines and Includes-------------------------------------------------------------------------------------------

//Global Variabes------------------------------------------------------------------------------------------------

//kill switch
int ks = 0; //variable to read value from reviever channel 5 connected to switch D on controller

//read speed
int32_t rotation_counter = 0; //count how many rotations have happened, used to get speed reading 
int32_t time_new = 0; //used for speed reading
int32_t time_old = 0; //used for speed reading 
int32_t rpm = 0; //varible describing the vehicle speed 
int dir = 0; //variable to tell the rotational direction of the motor

//read reciever
int16_t rc2 = 0; //variable to store teh signal from chanal 2 on the reciever
int16_t speed_set_point = 0; //variable that describes the desired speed of the vehicle

//PID
float control_signal = 0; //variable to store teh control signal 
int32_t sum_error = 0; //sum of error for control_signal control
int32_t last_error = 0; //previous error for control_signal controll



//Function Declerations------------------------------------------------------------------------------------------

int main(void);
void radio_setup(void);
void initTimer(void);
void speed_rpm(void);
void PID_control(void);
void init_encoder(void);
void read_reciever(void);
void output_inteprator(void);
void read_kill_switch(void);



//Main Function--------------------------------------------------------------------------------------------------

void setup(){}

int main(void) {

  
  init(); 
  radio_setup(); 
  initTimer();
  init_encoder();
  
  
  while(true){
    read_kill_switch();
    read_reciever(); //call functin to find the set point
    speed_rpm(); //call function to find out the rpm 
    PID_control(); //impliment PID control
    output_inteprator(); //inteprate control signal and send to motor controller
  }
  loop();//have to have this 
  return 0; 
}

//Other Functions-----------------------------------------------------------------------------------------------

//Setups--------------------------------------------------------------------------------------------------------

void radio_setup(void) { 
  pinMode(3, INPUT); //setpoint input 
  pinMode(12, OUTPUT); //H-bridge motor direction 1
  pinMode(13, OUTPUT); //H-bridge motor direction 2
  pinMode(6, OUTPUT); //PWM for motor output
  pinMode(5, INPUT); //kill swich input 
  Serial.begin(115200); //begin serial coms if testing code segmants. Comment out for faster code. 
}

void initTimer(void) { //Initalize Timer 1 for input capture  
  TCCR1A = 0;
  TCCR1B = (0<<ICNC1) | (1<<ICES1) | (1<<CS10);
  TCCR1C = 0;
  TIFR1 = (1<<ICF1) | (1<<TOV1);    
  TIMSK1 = (1<<ICIE1) | (1<<TOIE1); 

  pinMode(8, INPUT);
  digitalWrite(8, 0);     
}

void init_encoder(void){
  pinMode(7, INPUT); //set pin 7 as the one input for checking the direction of the encoder wheel. 
}

//Operation Function-------------------------------------------------------------------------

void read_kill_switch(void){
  int16_t switch_D = pulseIn(5, HIGH, 25000);
  if (switch_D > 2030 & switch_D < 2090){
    ks = 1;
  }
  else if (switch_D > 1020 & switch_D < 1080){
    ks = 0;
  }
  Serial.print("ks: ");
  Serial.println(ks);
  Serial.print("switch D: ");
  Serial.println(switch_D);
}

void speed_rpm(void){//calculate actual speed of the motor
  time_new = micros(); //Returns the number of microseconds since the Arduino board began running the current program. 
  rpm = (rotation_counter*60000000)/((time_new - time_old)*4); //*4 for the 4 slots in the disk.
  time_old = time_new;
  rotation_counter = 0;  
  //Serial.print("actual rotational speed: ");
  //Serial.println(rpm);
  //Serial.print("Direaction, [1 = CW] :");
  //Serial.println(dir);
  if (dir == 0){ // CCW
    rpm = -rpm; //acount for direction
  }
  delay(30);  
}

void read_reciever(void){ //function reads the signal from the reciever and maps it to the correct RPM set point 
  rc2 = pulseIn(3, HIGH, 25000); //read in data from reciever
  //Serial.print("rc2: ");
  //Serial.println(rc2);
  if (rc2 >= 910 & rc2 <= 2100){ //simple filter
    speed_set_point = map(rc2, 930, 2050, 5000, -5000); //map controller input from max forwards RPM ot max backwards rpm 
    Serial.print("desired speed: ");
    Serial.println(speed_set_point);
  }
}

void PID_control(void){//function to implement PID controll
  float kp = 0.1; //the proportional constant
  float ki = 0.07; //the integral constant
  float kd = 0; //the differential constant
  
  int16_t error = speed_set_point - rpm; //find the eror between the input and the actual speed
  //Serial.print("error: ");
  //Serial.println(error);
  control_signal = (error*kp + sum_error*ki +(error - last_error)*kd); //implement PID 
  //Serial.print("control_signal: ");
  //Serial.println(control_signal);
  last_error = error; //update last error
  sum_error += error; //sum errors
  if (sum_error > 30000){ //wind up prevention
    sum_error = 30000;
  }
  if (sum_error < -30000){
    sum_error = -30000;  
  }
  //Serial.print("sum error: ");
  //Serial.println(sum_error);
}

void output_inteprator(void){ //inteprate the PID control signal and sends it to the H-bridge 
  int in1 = 12;
  int in2 = 13;
  int enable1 = 6; 

  int16_t enable_out = map(control_signal, -5000, 5000, -255, 255); //maps the control signal from a speed in RPM to the appropriat enable value
  Serial.print("Enable_out: ");
  Serial.println(enable_out);

  if(speed_set_point > -500 & speed_set_point < 500 | rc2 == 0){ //sets up the dead zone on the controller and forces the car to stop in the event of no RC cntroller signal. 
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enable1, 0);
    sum_error = 0; 
  }

  else if (ks == 0){ //used to break the motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    analogWrite(enable1, 0);
    sum_error = 0; 
  }

  else{
  
  if (enable_out < 255 & enable_out >= 0){ //control signal for forwards if the control signal is not max forwards
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enable1, enable_out);
    //Serial.println("forwards");
  }
  else if (enable_out >= 255){ //control_signal for max forwards
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enable1, 255);
    //Serial.println("max forwards");
  }
  else if (enable_out < 0 & enable_out > -255){ //control signal backwards if the signal is not max backwards
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enable1, -enable_out);
    //Serial.println("backwards");    
  }
  else if (enable_out <= -255){//control_signal backwards max 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enable1, 255);
    //Serial.println("max backwards");
  }
  }
}



//Interrupt Handelers------------------------------------------------------------------------------------------

ISR(TIMER1_CAPT_vect) { //input cpture interupt handeler 
  rotation_counter ++; //incriment counter
  int encoder_1_value = digitalRead(8); //code to tell the direction of the motor. 
  int encoder_2_value = digitalRead(7);
  if (encoder_1_value == encoder_2_value){
    dir = 0;
    //Serial.println("Rotating CCW");
  }
  else{
    dir = 1; 
    //Serial.println("Rotating CW");
  }
}


ISR(TIMER1_OVF_vect) {
}

