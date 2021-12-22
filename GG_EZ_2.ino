//Main changes from the L298 test code: updated and commented certain variables for clarity (please comment your code, folks), added line follower logic to keep the robot on-course



#include <Encoder.h>
#include <Servo.h>
//Encoder backRight(2,29);
//Encoder backLeft(3,31);
Encoder frontRight(18,17);
Encoder frontLeft(19,20);

float enc_br = 1;
float enc_bl = 0.84;
float enc_fr = 0.8;
float enc_fl = 1;

int mpl = 255; //Left Motor Power
int mpr = 255; //Right Motor Power
int kpfor; //Proportional control variable for the forward motion of the robot

int left_speed = 0; //Output variable to control the speed of the left motors
int right_speed = 0; //Output variable to control the speed of the right motors
int left_back_speed = 0;
int right_back_speed = 0;
int LimitHit;

int error = 0; // Error variable. When this value is 0, the robot is centered on the line. This value becomes greater than 0 when the robot is too far to the right and becomes less than 0 when the robot is too far to the left.
int shooterPower = 0;
int diag2trigger = 0;
int backDiagTrigger = 0;

//Define all states. Note that certain numbers such as 107 are unused - these were from obsolete or otherwise unused states.
const int HOME = 101; //All motors stopped, ignore all sensors
const int FORWSIDE = 102; //Move sideways, switch to FORWARD upon hitting center line
const int FORWLINE = 103;  //Move forward along centerline, adjusting bearing as needed. Switch to STOP upon hitting half-court line
const int BACKLINE = 106; //Move the robot back to the home position to be reset
const int BACKUP = 108; //Back the robot up from the half-court line until the left and right sensors no longer detect a line. This is to make sure no part of the robot is sticking over the side of the half-court line
const int STOP = 109; //Stop all movement and prepare the launcher
const int FORWSIDE2 = 110; //Same as FORWSIDE but for the opposite court

int balls = 0; //iterative value to count the number of balls launched during a given "SHOOT" state


Servo LoadServo;

int state = 101;
//PushButton Pins (Ultimately, these pins were not used. We opted to instead modify the code and upload)
//int LeftCourt = 16, RightCourt = 17, LeftCourt_in, RightCourt_in;

//Line Follower Sensors Pins
int LSense0pin = 39, LSense1pin = 35, LSense2pin = 37; //LS0 is center-left, LS1 is center, LS2 is center-right (tentative)
int LSense3pin = 33; //LS3 is far left sensor
int LSenseRpin = 31; //LSR is far right sensor

int LS[5] = {0,0,0,0,0}; //Overall line sensor data array

int BRpin = 10; //Pin going to back right motor
int FRpin = 5; //Pin going to front right motor
int BLpin = 11; //Pin going to back left motor
int FLpin = 6; //Pin going to front left motor
int FR1pin = 22; //L298 pin for the front right motor. Set to HIGH, this should cause the motor to spin forwards
int FR2pin = 24; //L298 pin for the front right motor. Set to HIGH, this should cause the motor to spin backwards
int FL1pin = 26; //L298 pin for the front left motor. Set to HIGH, this should cause the motor to spin forwards
int FL2pin = 28; //L298 pin for the front left motor. Set to HIGH, this should cause the motor to spin backwards
int BR1pin = 44; //L298 pin for the back right motor. Set to HIGH, this should cause the motor to spin forwards
int BR2pin = 46; //L298 pin for the back right motor. Set to HIGH, this should cause the motor to spin backwards
int BL1pin = 48; //L298 pin for the back left motor. Set to HIGH, this should cause the motor to spin forwards
int BL2pin = 50; //L298 pin for the back left motor. Set to HIGH, this should cause the motor to spin backwards

int LimitSwitchPin = 13; 

//void PID_Control()
//{
//  
//}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(BRpin, OUTPUT); //Back Right PWM 
  pinMode(FRpin, OUTPUT); //Front Right PWM
  pinMode(BLpin, OUTPUT);  //Back Left PWM
  pinMode(FLpin, OUTPUT);  //Front Left PWM


  //pinMode(28, OUTPUT);
  //pinMode(30, OUTPUT);

  
}

int count[3];
int power = 0;

void loop() {
  // put your main code here, to run repeatedly:

  // Line Sensor Readings
  LS[0] = digitalRead(LSense0pin);
  LS[1] = digitalRead(LSense1pin);
  LS[2] = digitalRead(LSense2pin);
  LS[3] = digitalRead(LSense3pin); //Left-most
  LS[4] = digitalRead(LSenseRpin); //Right-most

  //PushButton Readings (Ultimately ended up being unused; we simply changed the values and reuploaded to reflect our assigned court)
//  LeftCourt_in  = digitalRead(LeftCourt);
//  RightCourt_in = digitalRead(RightCourt);
    int LeftCourt_in = 0;
    int RightCourt_in = 1;
    

  

//NEW AND IMPROVED Error sensor code (now 90% sure to work)
if (LS[0] == 0 && LS[1] == 0 && LS[2] == 1){
  error = 2;
}
else if (LS[0] == 0 && LS[1] == 1 && LS[2] == 1){
  error = 1;
}
//else if (LS[0] == 0 && LS[1] == 1 && LS[2] == 0 && LS[3] == 1){
//  error = 0;
//}
else if (LS[0] == 0 && LS[1] == 1 && LS[2] == 0){
  error = 0;
}
else if (LS[0] == 1 && LS[1] == 1 && LS[2] == 0){
  error = -1;
}
else if (LS[0] == 1 && LS[1] == 0 && LS[2] == 0){
  error = -2;
}

//End of Error sensor code
  //Proportional Control
  mpl = 170; //Left Motor Power. This speed is further modified depending on the error, seen below.
  mpr = 170; //Right Motor Power
  kpfor = 30; //kp value for when the robot moved forward
  left_speed = mpl + kpfor*error; //Tentative line follower fix: if the left sensor goes off (error = -1), make the right motors faster to course-correct
  right_speed = mpr - kpfor*error;

  //Further modifications to the motor speed depending on how off-course the robot currently is
  if(error == -1) {
    left_back_speed = mpl*0.75;
    right_back_speed = 0;
  }
  else if(error == 1)  {
    left_back_speed = 0;
    right_back_speed = mpr*0.75;
  }
  else if(error > 1)  {
    left_back_speed = 0;
    right_back_speed = mpr * 1.25;
  }
  else if(error < 1)  {
    left_back_speed = mpl * 1.25;
    right_back_speed = 0;
  }
  else  {
    left_back_speed = mpl;
    right_back_speed = mpr;
  }
  
//Debug information regarding the backwards movement of the robot
//  Serial.print("LS: ");
//  Serial.print(left_back_speed);
//  Serial.print("RS :");

//Ensure all motors are moving forward
  digitalWrite(FR1pin, HIGH); //Right In1
  digitalWrite(FR2pin, LOW); //Right In2

  digitalWrite(FL1pin, HIGH); //Right In3
  digitalWrite(FL2pin, LOW); //Right In4

  digitalWrite(BR2pin, LOW); //Left In2
  digitalWrite(BR1pin, HIGH); //Left In1
  
  digitalWrite(BL1pin, HIGH); //Left In3
  digitalWrite(BL2pin, LOW); //Left In4
  




  //State Machine 


  switch(state){

    //All Motors are off in State 0. Robot is in Home position
    case HOME: //101
    analogWrite(BRpin,0);
    analogWrite(FRpin,0);
    analogWrite(BLpin,0);
    analogWrite(FLpin,0);

    if(RightCourt_in == 1){
      state = FORWSIDE2; //Move sideways to the left
      }
    else if(LeftCourt_in == 1) {
      state = FORWSIDE; //Move sideways to the right
    }
    else {
      state = HOME; //We messed something up if this happens
      }
     break;
     
    //The robot stops and begins launching balls
    case STOP: //109
    //Disable the four motors
    analogWrite(BRpin,0);
    analogWrite(FRpin,0);
    analogWrite(BLpin,0);
    analogWrite(FLpin,0);

    //Initialize the ball-loading servo
    LoadServo.attach(9);  

    //Initialize the launcher motor and set the motor speed, while keeping the motor disabled
    digitalWrite(3,LOW);   
    shooterPower = 90; 
    analogWrite(2,shooterPower);

    //Fully extend the loader servo (maximum angle is 180°) to prevent balls from entering the launcher prematurely
    LoadServo.write(180);

    //Loop to give the STOP state an end-case.
    while (balls < 5) {
    
      analogWrite(2, shooterPower); 

      //Allow the launcher to reach full speed by delaying for two seconds
      delay(2000);    

      //After the two-second spinning up, rotate the loader servo such that a ball rolls into the launcher and hold that position for 0.75 seconds to prevent the ball from getting pinched
      LoadServo.write(90);
      delay(750); 

      //After 0.75 seconds, close the launching servo to prevent more than one ball from entering the launcher and hold for an extra second to allow the ball to reach the shooter and get launched
      LoadServo.write(180);
      delay(1000);

      //Disable the motor to reset it. This is done since the motor would steadily lose power each time a ball was introduced unless its power was cycled for each shot.
      analogWrite(2, 0);
      delay(2000);

      //Increment the loop variable
      balls = balls + 1;
    }

    //Determine when no more balls are present on the robot
    if(balls >= 5)  {
      //Disable the launcher motor and send to the next state
      analogWrite(2,0);
      state = BACKLINE;
      frontRight.write(0);
      frontLeft.write(0);
    }
      
     

    
    //analogWrite(9,255);
    
    break;
    
    case FORWSIDE: //102; Move robot to the right until it reaches the centerline

    //Set the correct motor direction for each motor. This allows the mechanum wheels to slide the robot sideways without requiring a turn.
    digitalWrite(FR1pin, LOW); //Right In1
    digitalWrite(FR2pin, HIGH); //Right In2

    digitalWrite(FL1pin, HIGH); //Right In3
    digitalWrite(FL2pin, LOW); //Right In4
  
    digitalWrite(BR2pin, LOW); //Left In2
    digitalWrite(BR1pin, HIGH); //Left In1
    
    digitalWrite(BL1pin, LOW); //Left In3
    digitalWrite(BL2pin, HIGH); //Left In4
  
    //Write the speed of the motors. Due to a combination of weight imbalance and the right-side motors being different from the left-side motors, different speeds needed to be written to each side
    analogWrite(BRpin,200);
    analogWrite(FRpin,200);
    analogWrite(BLpin,255);//229.5
    analogWrite(FLpin,255);//242.25

    //If the center line sensor detects the centerline, switch to the FORWLINE state. Otherwise, maintain current state
    if(LS[1] == 1){
      state = FORWLINE;
    }
    else  {
      state = FORWSIDE;
    }
    

    break;

    case FORWSIDE2: //110; Similar concept as the FORWSIDE state, though the robot is instructed to move forward for a short amount of time to avoid a patch of uneven surface present in the right court.
      if(diag2trigger == 0) {

       //Ensure all motors are moving forward
        digitalWrite(FR1pin, HIGH); //Right In1
        digitalWrite(FR2pin, LOW); //Right In2
      
        digitalWrite(FL1pin, HIGH); //Right In3
        digitalWrite(FL2pin, LOW); //Right In4
      
        digitalWrite(BR2pin, LOW); //Left In2
        digitalWrite(BR1pin, HIGH); //Left In1
        
        digitalWrite(BL1pin, HIGH); //Left In3
        digitalWrite(BL2pin, LOW); //Left In4

        //Move the robot forward out of the home position for a short time. This was done since the surface directly to the side of the home position caused major slipping when the robot tried to move sideways.
        analogWrite(BRpin,125);
        analogWrite(FRpin,125);
        analogWrite(BLpin,125);
        analogWrite(FLpin,125);
    
        delay(4000);
        diag2trigger = 1;
      }
      //Set the correct motor direction for each motor. This allows the mechanum wheels to slide the robot sideways without requiring a turn.
     digitalWrite(FR1pin, HIGH); //Right In1
     digitalWrite(FR2pin, LOW); //Right In2

     digitalWrite(FL1pin, LOW); //Right In3
     digitalWrite(FL2pin, HIGH); //Right In4
  
     digitalWrite(BR2pin, HIGH); //Left In2
     digitalWrite(BR1pin, LOW); //Left In1
    
     digitalWrite(BL1pin, HIGH); //Left In3
     digitalWrite(BL2pin, LOW); //Left In4
    

  
    //WRITE TO MOTORS HERE
    analogWrite(BRpin,175);
    analogWrite(FRpin,175);
    analogWrite(BLpin,230);
    analogWrite(FLpin,230);

    if(LS[1] == 1){
      state = FORWLINE;
    }
    else  {
      state = FORWSIDE2;
    }
    

    break;

    case FORWLINE: //103; this is the forward line-following state. The robot is to follow the centerline until reaching the half-court line, at which point it backs up slightly and beings shooting.

    digitalWrite(FR1pin, HIGH); //Right In1
    digitalWrite(FR2pin, LOW); //Right In2
  
    digitalWrite(FL1pin, HIGH); //Right In3
    digitalWrite(FL2pin, LOW); //Right In4
  
    digitalWrite(BR2pin, LOW); //Left In2
    digitalWrite(BR1pin, HIGH); //Left In1
    
    digitalWrite(BL1pin, HIGH); //Left In3
    digitalWrite(BL2pin, LOW); //Left In4

    
    //WRITE TO MOTORS HERE
    //Instead of writing constant values to the motor speed, we use variables that are modified by the line-following code above.
    analogWrite(BRpin,right_speed);
    analogWrite(FRpin,right_speed);
    analogWrite(BLpin,left_speed);
    analogWrite(FLpin,left_speed);
       
    //If all five line sensors are activated (Only possible if the robot reached the half-court line), move to the BACKUP state.
    if (LS[0] == 1 && LS[1] == 1 && LS[2] == 1 && LS[3] == 1 && LS[4] == 1)  {
      state = BACKUP; 
    }
    else  {
      state = FORWLINE;
    }
    
    break;

    case BACKUP: //108; Robot tends to overshoot the half-court line slightly. This state inches it back over the line.
    
    //Reverse L298 motor directions to go backwards
    digitalWrite(FR1pin, LOW); //Right In1
    digitalWrite(FR2pin, HIGH); //Right In2

    digitalWrite(FL1pin, LOW); //Right In3
    digitalWrite(FL2pin, HIGH); //Right In4

    digitalWrite(BR2pin, HIGH); //Left In2
    digitalWrite(BR1pin, LOW); //Left In1
  
    digitalWrite(BL1pin, LOW); //Left In3
    digitalWrite(BL2pin, HIGH); //Left In4
    
    //WRITE TO MOTORS HERE
    analogWrite(BRpin,125);
    analogWrite(FRpin,125);
    analogWrite(BLpin,125);
    analogWrite(FLpin,125);

    //After half a second of travel, begin launching
    delay(500); 
    state = STOP;   
    
    break;
    

 

  
  

    case BACKLINE: //106
    //INPUT TO MOTORS TO MAKE IT BACK TO REAR OF COURT
    //GO TO BACKDIAG ONCE THE LIMITSWITCH IS HIT

    digitalWrite(FR1pin, LOW); //Right In1
    digitalWrite(FR2pin, HIGH); //Right In2

    digitalWrite(FL1pin, LOW); //Right In3
    digitalWrite(FL2pin, HIGH); //Right In4
  
    digitalWrite(BR2pin, HIGH); //Left In2
    digitalWrite(BR1pin, LOW); //Left In1
    
    digitalWrite(BL1pin, LOW); //Left In3
    digitalWrite(BL2pin, HIGH); //Left In4     
      
      //This code tells the robot to turn by a preset amount before going straight backwards, depending on which court it's in.
      //Encoder counts were used in lieu of line-following due to the placement of the line sensors at the front making line-following difficult while moving backwards.
      if(backDiagTrigger == 0 && RightCourt_in == 1) {
          analogWrite(BLpin,255*.84);
          analogWrite(FLpin,255*.84);
//          Serial.println(frontLeft.read());
          if(frontLeft.read() < -1200) {
            backDiagTrigger = 1;
            frontRight.write(0);
            frontLeft.write(0);
          }
      } else if(backDiagTrigger == 0 && LeftCourt_in == 1) {
          analogWrite(BRpin,255);
          analogWrite(FRpin,255);
//          Serial.println(frontRight.read());
          if(frontRight.read() < -3000) {
            backDiagTrigger = 1;
            frontRight.write(0);
            frontLeft.write(0);
          }        
      } else if(frontLeft.read() < -9000 && RightCourt_in == 1) {
          analogWrite(BRpin,255);
          analogWrite(FRpin,255); 
          analogWrite(BLpin,0);
          analogWrite(FLpin,0);
      } else if(frontRight.read() < -16000 && LeftCourt_in == 1) {
          analogWrite(BRpin,0);
          analogWrite(FRpin,0); 
          analogWrite(BLpin,255*.84);
          analogWrite(FLpin,255*.84);
      } else if(frontRight.read()/75 < frontLeft.read()/45) {
          analogWrite(BRpin,100);
          analogWrite(FRpin,100);
          analogWrite(BLpin,125*.84);
          analogWrite(FLpin,125*.84);
      } else if(frontRight.read()/75 > frontLeft.read()/45) {
          analogWrite(BRpin,125);
          analogWrite(FRpin,125);
          analogWrite(BLpin,100*.84);
          analogWrite(FLpin,100*.84);
      } else if(frontRight.read()/75 == frontLeft.read()/45){
          analogWrite(BRpin,125);
          analogWrite(FRpin,125);
          analogWrite(BLpin,125*.84);
          analogWrite(FLpin,125*.84);          
      }
    //Debug information regarding the motor encoder values
    Serial.print("FR: ");
    Serial.println(frontRight.read());
    Serial.print("FL: ");
    Serial.println(frontLeft.read());
    
    //Stop motors upon activation of the limit switch on the robot's rear
    LimitHit = digitalRead(LimitSwitchPin);

    if(LimitHit == 0) {
      //state = BACKDIAG2;
      analogWrite(BRpin,0);
      analogWrite(FRpin,0);
      analogWrite(BLpin,0);
      analogWrite(FLpin,0);
    }
    else{
      state = BACKLINE;
    }

    break;
    
    }
    //Debugging information regarding the current state of the robot
    Serial.println(state);
    }
    
