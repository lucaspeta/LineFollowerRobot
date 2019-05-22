#include <Ultrasonic.h>

// Infrared sensors
#define LINE_FOLLOW_SENSOR_0    24
#define LINE_FOLLOW_SENSOR_1    25
#define LINE_FOLLOW_SENSOR_2    26

// DC motors
#define LEFT_MOTOR_PWM    2
#define LEFT_MOTOR2       3
#define LEFT_MOTOR1       4               
#define RIGHT_MOTOR2      5
#define RIGHT_MOTOR1      6
#define RIGHT_MOTOR_PWM   7

// Ultrasonic sensors
#define US_BOX_TRIGGER        28
#define US_BOX_ECHO           29
#define US_OBSMID_TRIGGER     8
#define US_OBSMID_ECHO        9
#define US_OBSLEF_TRIGGER     12
#define US_OBSLEF_ECHO        13
#define US_OBSRIG_TRIGGER     10
#define US_OBSRIG_ECHO        11

// Speed 
#define MAX_SPEED 255
#define MIN_SPEED 100
#define MID_SPEED 180

Ultrasonic ultrasonic_box          (US_BOX_TRIGGER   ,   US_BOX_ECHO );
Ultrasonic ultrasonic_obstacle_mid (US_OBSMID_TRIGGER,   US_OBSMID_ECHO);
Ultrasonic ultrasonic_obstacle_lef (US_OBSLEF_TRIGGER,   US_OBSLEF_ECHO);
Ultrasonic ultrasonic_obstacle_rig (US_OBSRIG_TRIGGER,   US_OBSRIG_ECHO);

// Line follower modes
#define STOPPED              0
#define FOLLOWING_LINE       1
#define FOLLOWING_LINE_LEFT  2
#define FOLLOWING_LINE_RIGHT 3
#define NO_LINE              4

// Triggers
bool box_detected, obstacle_detected_mid, obstacle_detected_lef, waitForArm, tryToFindTheLine;

int mode = 0;

// Erros vector
int   LFSensor[3]       = { 0, 0, 0 };  
float error             = 0;
float Proportion        = 0;


//Distances
int box_distance, obstacle_distance, obstacle_dmid, obstacle_dlef, obstacle_drig;

bool contR = false;

// Function that verifies each sensor and return the mode state and the error *****LINE FOLLOWER
void readLFsensors()
{
  //reads the infrared sensor one by one with a delay 
  LFSensor[0] = digitalRead(LINE_FOLLOW_SENSOR_0);
  delay(50);
  LFSensor[1] = digitalRead(LINE_FOLLOW_SENSOR_1);
  delay(50);
  LFSensor[2] = digitalRead(LINE_FOLLOW_SENSOR_2);
  delay(50);

/*
  Serial.print("Sensor E -> ");  Serial.println(LFSensor[0]);
  Serial.print("Sensor M -> ");  Serial.println(LFSensor[1]);
  Serial.print("Sensor D -> ");  Serial.println(LFSensor[2]);
  Serial.println("");
*/

  if((     LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 ) || (LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 ))  { mode = FOLLOWING_LINE_RIGHT; error =  -1; }
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 ) || (LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 ))  { mode = FOLLOWING_LINE_LEFT; error =  1;  }
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 ))  { mode = FOLLOWING_LINE; error =  0;  }
  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 ))  { mode = STOPPED;    error =  0; }
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 ))  { mode = NO_LINE;    error =  0; }
}

/*
  [int] del -> delay
  [int] val -> pwm value to both motors
*/
void forward(int del, int val){
    digitalWrite(LEFT_MOTOR1 , LOW);
    digitalWrite(LEFT_MOTOR2 , HIGH);
    digitalWrite(RIGHT_MOTOR1, HIGH);
    digitalWrite(RIGHT_MOTOR2, LOW);

    analogWrite(LEFT_MOTOR_PWM , val);
    analogWrite(RIGHT_MOTOR_PWM, val);

    delay(del);
}

/*
  [int] del -> delay
  [int] val -> pwm value to both motors
*/
void backward(int del, int val){
    digitalWrite(LEFT_MOTOR1 , HIGH);
    digitalWrite(LEFT_MOTOR2 , LOW);
    digitalWrite(RIGHT_MOTOR1, LOW);
    digitalWrite(RIGHT_MOTOR2, HIGH);

    analogWrite(LEFT_MOTOR_PWM , val);
    analogWrite(RIGHT_MOTOR_PWM, val);

    delay(del);
}

/*
  [int] del -> delay
  [int] maxspd -> speed to the right motor
  [int] minspd -> speed to the left motor
*/
void go_right(int del, int maxspd, int minspd){
    digitalWrite(LEFT_MOTOR1 , LOW);
    digitalWrite(LEFT_MOTOR2 , HIGH);
    digitalWrite(RIGHT_MOTOR1, LOW);
    digitalWrite(RIGHT_MOTOR2, HIGH);
  
    analogWrite(LEFT_MOTOR_PWM , maxspd);
    analogWrite(RIGHT_MOTOR_PWM, minspd);

    delay(del);
}

/*
  [int] del -> delay
  [int] maxspd -> speed to the left motor
  [int] minspd -> speed to the right motor
*/
void go_left(int del, int maxspd, int minspd){
    digitalWrite(LEFT_MOTOR1 , HIGH);
    digitalWrite(LEFT_MOTOR2 , LOW);
    digitalWrite(RIGHT_MOTOR1, HIGH);
    digitalWrite(RIGHT_MOTOR2, LOW);
  
    analogWrite(LEFT_MOTOR_PWM , maxspd);
    analogWrite(RIGHT_MOTOR_PWM, minspd);

    delay(del);
}

/*
  [int] del -> delay
*/
void stopMotors(int del){
    digitalWrite(LEFT_MOTOR1 , HIGH);
    digitalWrite(LEFT_MOTOR2 , LOW);
    digitalWrite(RIGHT_MOTOR1, LOW);
    digitalWrite(RIGHT_MOTOR2, HIGH);
     
    //Alta
    analogWrite(LEFT_MOTOR_PWM ,0);
    analogWrite(RIGHT_MOTOR_PWM,0);

    delay(del);
}

// Function that detects if the box is on the spot and triggers the robot
void boxDetector(){ 
  box_distance = ultrasonic_box.read();
  //Serial.println(box_distance);
  // if there is an object
  if(box_distance > 0 && box_distance < 5){
    box_detected = true;
  }
  else{
    box_detected = false;
  }
}

// Function that detects obstacles
void obstacleDetector(){  
  obstacle_dmid   = ultrasonic_obstacle_mid.read();
  obstacle_dlef   = ultrasonic_obstacle_lef.read();
  obstacle_drig   = ultrasonic_obstacle_rig.read();

/*
  Serial.println("mid");
  Serial.println(obstacle_dmid);
  Serial.println("lef");
  Serial.println(obstacle_dlef);
  Serial.println("rig");
  Serial.println(obstacle_drig);
  Serial.println("");
*/

  // if there is an obstacle
  if(obstacle_dmid > 0 && obstacle_dmid < 30)
  {
	  obstacle_detected_mid = true;
    
    //detects that the obstacle in the left is closer, turn to right
    if(obstacle_drig > obstacle_dlef)
    { 
      obstacle_detected_lef = true;  
    }
    //detects that the obstacle in the right is closer, turn left
    else{ 
      obstacle_detected_lef = false;
    }
  }
  else{
    obstacle_detected_lef = false;
    obstacle_detected_mid = false;
  }
}

// Pre Settings
void setup()
{
  Serial.begin(9600);

  // Infrared sensors 
  pinMode(LINE_FOLLOW_SENSOR_0, INPUT  );
  pinMode(LINE_FOLLOW_SENSOR_1, INPUT  );
  pinMode(LINE_FOLLOW_SENSOR_2, INPUT  );

  // Ultrasonic sensors
  pinMode(US_BOX_TRIGGER    , OUTPUT );
  pinMode(US_BOX_ECHO       , INPUT  );
  pinMode(US_OBSMID_TRIGGER , OUTPUT );
  pinMode(US_OBSMID_ECHO    , INPUT  );
  pinMode(US_OBSLEF_TRIGGER , OUTPUT );
  pinMode(US_OBSLEF_ECHO    , INPUT  );
  pinMode(US_OBSRIG_TRIGGER , OUTPUT );
  pinMode(US_OBSRIG_ECHO    , INPUT  );
  
  // DC motors
  pinMode(LEFT_MOTOR1      , OUTPUT );
  pinMode(LEFT_MOTOR2      , OUTPUT );
  pinMode(LEFT_MOTOR_PWM   , OUTPUT );
  pinMode(RIGHT_MOTOR1     , OUTPUT );
  pinMode(RIGHT_MOTOR2     , OUTPUT );
  pinMode(RIGHT_MOTOR_PWM  , OUTPUT );
  
  // Pre set Motors
  mode = STOPPED; 
}
 
//Main
void loop(){     
  boxDetector();
  // check if there is an object to carry on
  if(box_detected == true){
    // check if there is any object in the front
    obstacleDetector();

    if(obstacle_detected_mid){ // obstacle ahead          
        stopMotors(500);
        
        backward(1500, MAX_SPEED);
            
        if(obstacle_detected_lef){    
           // Turn to left
          go_right(2500, MAX_SPEED, MIN_SPEED);
          
          // Go forward
          forward(4000, MAX_SPEED); 
          
          // Straighten right
          go_left(1000, MAX_SPEED, MIN_SPEED);
  
          // Go forward
          forward(1500, MAX_SPEED);
  
          // Return to the line (right) 
          go_left(1400, MAX_SPEED, MIN_SPEED);
  
          // Go forward
          forward(500, MAX_SPEED);
  
          // Return to the line right
          go_left(2000, MAX_SPEED, MIN_SPEED);

          forward(3000, MAX_SPEED);
        }
        else{
          // Turn to left
          go_left(2500, MAX_SPEED, MIN_SPEED);
          
          // Go forward
          forward(4000, MAX_SPEED); 
          
          // Straighten right
          go_right(1000, MAX_SPEED, MIN_SPEED);
  
          // Go forward
          forward(1500, MAX_SPEED);
  
          // Return to the line (right) 
          go_right(1400, MAX_SPEED, MIN_SPEED);
  
          // Go forward
          forward(500, MAX_SPEED);
  
          // Return to the line right
          go_right(2000, MAX_SPEED, MIN_SPEED);

          forward(3000, MAX_SPEED);
        }
  
      obstacle_detected_mid = false;
    }
    
    // Read the sensors to know what is the next step to do
    readLFsensors(); 
  
    switch (mode)
    {
      //111 
      case STOPPED: 
        stopMotors(20);
        break; 
      //010
      case FOLLOWING_LINE: 
        forward(20, 200);
        tryToFindTheLine = true;

        break; 
      //100 OR 110
      case FOLLOWING_LINE_LEFT:
        go_left(80, MAX_SPEED, 150);
        tryToFindTheLine = true;

        break;
      //001 OR 011 
      case FOLLOWING_LINE_RIGHT:
        go_right(20, MAX_SPEED, 150);
        tryToFindTheLine = true;

        break;
      //000 THIS MODE SHOULD STOP THE CAR, CHANGE THIS SWITCH IF THE IR POSITION CHANGE
      case NO_LINE:
        if(tryToFindTheLine){
          forward(200, MAX_SPEED);
          tryToFindTheLine = false;
        }
        
        break;
    }
  }
  else{
      stopMotors(0);
  }

  delay(100);
}
