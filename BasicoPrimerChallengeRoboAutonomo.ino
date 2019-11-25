/*

 by ahmed Azouz
  https://www.instructables.com/id/How-to-Make-Ardu...

 Download the lib from here first
  https://github.com/ErickSimoes/Ultrasonic/blob/ma...

*/

#include <Ultrasonic.h>
#include <Servo.h>

Servo dirServo;                   // define servo to control turning of smart car
int dirServoPin = 2;              // define pin for signal line of the last servo
float dirServoOffset = 0;         // define a variable for deviation(degree) of the servo
int dirServoDegree;


Ultrasonic ultrasonic(0,1);
const int IN1=7;     // define pin used to control rotational direction of motor A or  dirAPin
const int IN2=6;     // define pin for PWM used to control rotational speed of motor A  or pwmAPin
const int IN3=4;     // define pin used to control rotational direction of motor B  or dirBPin
const int IN4=5;    // define pin for PWM used to control rotational speed of motor B or pwmBPin

const int WHITE=0;
const int BLACK=1;

#define IR_LEFT_FRONT 9     
#define IR_RIGHT_FRONT 8
#define IR_RIGHT_BACK 10    
#define IR_LEFT_BACK 11


int distance ;
int IR_left_front;
int IR_right_front;
int IR_left_back;
int IR_right_back;


void setup() {
   Serial.begin(9600);
   dirServo.attach(dirServoPin);  // attaches the servo on servoDirPin to the servo object
   //IR SENSOR INITIALIZATION
   pinMode(IR_LEFT_FRONT, INPUT);   
   pinMode(IR_RIGHT_FRONT, INPUT);  
   pinMode(IR_RIGHT_BACK, INPUT);   
   pinMode(IR_LEFT_BACK, INPUT); 
   
   pinMode(IN1, OUTPUT);   // set dirAPin to output mode
   pinMode(IN2, OUTPUT);   // set pwmAPin to output mode
   pinMode(IN3, OUTPUT);   // set dirBPin to output mode
   pinMode(IN4, OUTPUT);   // set pwmBPin to output mode
   delay (5000); // as per sumo compat roles
}

void loop(){
     read_IR_sensor_values();
     distance = ultrasonic.read();
     ROTATE(200); // start rotete if (distance < 10){
     Stop();
     dirServoDegree = 90;
    
    while (distance < 10 ) {
        FORWARD(255); 
        distance = ultrasonic.read();
        read_IR_sensor_values();
        if ((IR_left_front==BLACK && IR_right_front ==BLACK) || (IR_left_back==BLACK && IR_right_back==BLACK) ){
           break;
        }
        delay(10);
    }
    
    if (IR_left_front==WHITE || IR_right_front == WHITE){
        Stop();
        delay (50);
        BACKWARD(255);
        delay (500);
    }

    if (IR_left_back==WHITE || IR_right_back == WHITE){
       Stop();
       delay (50);
       FORWARD(255);
       delay (500);
    }
    // ----------- debugging ----------------
    DEBUG_CODE( IR_left_front,  IR_right_front,  IR_left_back,  IR_right_back , distance, dirServoDegree);
} 


void read_IR_sensor_values()
{
  IR_left_front = digitalRead(IR_LEFT_FRONT);
  IR_right_front = digitalRead(IR_RIGHT_FRONT);
  IR_left_back = digitalRead(IR_LEFT_BACK);
  IR_right_back = digitalRead(IR_RIGHT_BACK);
}

void DEBUG_CODE( int IR_left_front, int IR_right_front, int IR_left_back, int IR_right_back , int distance, int dirServoDegree ){
    Serial.print("distance : " );
    Serial.print(distance);
    Serial.println("IR_left_front :");
    Serial.println(IR_left_front); 

    Serial.println("IR_right_front :");
    Serial.println(IR_right_front); 

    Serial.println("IR_left_back :");
    Serial.println(IR_left_back); 

    Serial.println("IR_right_back :");
    Serial.println(IR_right_back); 

    Serial.print("servo Distance : " ); 
    Serial.println(dirServoDegree);
}

//--------------------------------------------
void FORWARD (int Speed){
  //When we want to let Motor To move forward,
  // just void this part on the loop section .
  analogWrite(IN1,Speed);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,Speed);
}//--------------------------------------------
void BACKWARD (int Speed){
  //When we want to let Motor To move forward,
  // just void this part on the loop section .
  analogWrite(IN1,0);
  analogWrite(IN2,Speed);
  analogWrite(IN3,Speed);
  analogWrite(IN4,0);
}//--------------------------------------------
void ROTATE (int Speed)
{
  //When we want to let Motor To Rotate ,
  // just void this part on the loop section .
  analogWrite(IN1,Speed);
  analogWrite(IN2,0);
  analogWrite(IN3,Speed);
  analogWrite(IN4,0);
}//--------------------------------------------
void Stop(){
  //When we want to  Motor To stop ,
  // just void this part on the loop section .
  analogWrite(IN1,0);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,0);
}

void  rotationServoAngle(byte dirServoDegree) {
  dirServo.write(dirServoDegree + dirServoOffset); 
}
