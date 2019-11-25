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
const int dirAPin=7;     // define pin used to control rotational direction of motor A or  dirAPin
const int pwmAPin=6;     // define pin for PWM used to control rotational speed of motor A  or pwmAPin
const int dirBPin=4;     // define pin used to control rotational direction of motor B  or dirBPin
const int pwmBPin=5;    // define pin for PWM used to control rotational speed of motor B or pwmBPin

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
   
   pinMode(dirAPin, OUTPUT);   // set dirAPin to output mode
   pinMode(pwmAPin, OUTPUT);   // set pwmAPin to output mode
   pinMode(dirBPin, OUTPUT);   // set dirBPin to output mode
   pinMode(pwmBPin, OUTPUT);   // set pwmBPin to output mode
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
void FORWARD (byte motorSpd){
  //When we want to let Motor To move forward,
  // just void this part on the loop section .
  analogWrite(dirAPin,motorSpd);
  analogWrite(pwmAPin,0);
  analogWrite(dirBPin,0);
  analogWrite(pwmBPin,motorSpd);

  /*digitalWrite(dirAPin, motorDir);
  digitalWrite(dirBPin, motorDir);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
  */
  
}//--------------------------------------------
void BACKWARD (int Speed){
  //When we want to let Motor To move forward,
  // just void this part on the loop section .
  analogWrite(dirAPin,0);
  analogWrite(pwmAPin,Speed);
  analogWrite(dirBPin,Speed);
  analogWrite(pwmBPin,0);
}//--------------------------------------------
void ROTATE (int Speed)
{
  //When we want to let Motor To Rotate ,
  // just void this part on the loop section .
  analogWrite(dirAPin,Speed);
  analogWrite(pwmAPin,0);
  analogWrite(dirBPin,Speed);
  analogWrite(pwmBPin,0);
}//--------------------------------------------
void Stop(){
  //When we want to  Motor To stop ,
  // just void this part on the loop section .
  analogWrite(dirAPin,0);
  analogWrite(pwmAPin,0);
  analogWrite(dirBPin,0);
  analogWrite(pwmBPin,0);
}

void  rotationServoAngle(byte dirServoDegree) {
  dirServo.write(dirServoDegree + dirServoOffset); 
}
