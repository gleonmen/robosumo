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

#define IR_sensor_front A0 // front sensor
#define IR_sensor_back A1 // rear senson
int distance ;

void setup() {
   Serial.begin(9600);
   dirServo.attach(dirServoPin);  // attaches the servo on servoDirPin to the servo object
   pinMode(IN1, OUTPUT);   // set dirAPin to output mode
   pinMode(IN2, OUTPUT);   // set pwmAPin to output mode
   pinMode(IN3, OUTPUT);   // set dirBPin to output mode
   pinMode(IN4, OUTPUT);   // set pwmBPin to output mode
   delay (5000); // as per sumo compat roles
}

void loop(){
    int IR_front = analogRead(IR_sensor_front);
    int IR_back = analogRead(IR_sensor_back);
    distance = ultrasonic.read();
    ROTATE(200); // start rotete if (distance < 10){
    Stop();
    dirServoDegree = 90;
    
    while (distance < 10 ) {
        FORWARD(255); 
        distance = ultrasonic.read();
        IR_front = analogRead(IR_sensor_front);
        IR_back = analogRead(IR_sensor_back);
        if ( IR_front > 650 || IR_back > 650 ) { break;}
          delay(10);
    }
    //< 650 means white line
    if (IR_front < 650 ) {  
        Stop();
        delay (50);
        BACKWARD(255);
        delay (500);
    } 
    if (IR_back < 650 ){
        Stop();
        delay (50);
        FORWARD(255);
        delay (500);
    }
    // ----------- debugging ----------------
    DEBUG_CODE(IR_front, IR_back, distance, dirServoDegree);
} 

void DEBUG_CODE( int IR_front, int IR_back, int distance, int dirServoDegree ){
    Serial.print(distance);
    Serial.println("cm");
    Serial.println("IR front :");
    Serial.println(IR_front); 
    Serial.println("IR back :");
    Serial.println(IR_back);
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
