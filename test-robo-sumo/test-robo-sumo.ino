#include <Ultrasonic.h>
#include <Servo.h>

//Servo llantas delanteras
Servo dirServo;                   // define servo to control turning of smart car
int dirServoPin = 2;              // define pin for signal line of the last servo
float dirServoOffset = 20;         // define a variable for deviation(degree) of the servo
int dirServoDegree;

//Servomotor ultrasonido
Servo ultrasonicServo;            // define servo to control turning of ultrasonic sensor
int ultrasonicPin = 3;            // define pin for signal line of the last servo
float ultrasonicServoOffset = -5; // define a variable for deviation(degree) of the servo
int ultrasonicServoDegree;

const int dirAPin = 7;    // define pin used to control rotational direction of motor A
const int pwmAPin = 6;    // define pin for PWM used to control rotational speed of motor A
const int dirBPin = 4;    // define pin used to control rotational direction of motor B
const int pwmBPin = 5;    // define pin for PWM used to control rotational speed of motor B

//Ultrasonido
Ultrasonic ultrasonic(12,13);
int distance ;

//Sensors
#define LFSensor_front 10 //sensor1
#define LFSensor_back 11 //sensor2

#define FORWARD HIGH
#define BACKWARD LOW

int sensor_front;
int sensor_back;

void setup() {
  dirServo.attach(dirServoPin);  // attaches the servo on servoDirPin to the servo object
  ultrasonicServo.attach(ultrasonicPin);  // attaches the servo on ultrasonicPin to the servo object
  pinMode(dirAPin, OUTPUT);   // set dirAPin to output mode
  pinMode(pwmAPin, OUTPUT);   // set pwmAPin to output mode
  pinMode(dirBPin, OUTPUT);   // set dirBPin to output mode
  pinMode(pwmBPin, OUTPUT);   // set pwmBPin to output mode
  pinMode(LFSensor_front,INPUT);
  pinMode(LFSensor_back,INPUT); 
  Serial.begin(9600);
}
void loop()
{
     distance = ultrasonic.read();
     Serial.print("distance : " );
     Serial.println(distance);
     read_sensor_values();
     Serial.print("sensor_front : " );
     Serial.println(sensor_front);
     Serial.print("sensor_back : " );
     Serial.println(sensor_back);
     delay(2000);
//     
//     forward(255);
//     delay(1000);
//     stopcar();
//     
//     rigth(200);
//     delay(1000);
//     stopcar();
//
//     left(200);
//     delay(1000);
//     stopcar();
//
//     back(255);
//     delay(1000);
//     stopcar();
//     
//     delay(1000);
}

void rigth(byte motorSpd) {
  dirServo.write(45 + dirServoOffset);
  ultrasonicServo.write(45 + ultrasonicServoOffset);
  digitalWrite(dirAPin, BACKWARD);
  digitalWrite(dirBPin, FORWARD);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
  Serial.println("Rigth");
}

void left(byte motorSpd) {
  dirServo.write(135 + dirServoOffset);
  ultrasonicServo.write(135 + ultrasonicServoOffset);
  digitalWrite(dirAPin, FORWARD);
  digitalWrite(dirBPin, BACKWARD);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
  Serial.println("Left");
}

void forward( byte motorSpd) {
  dirServo.write(90 + dirServoOffset);
  ultrasonicServo.write(90 + ultrasonicServoOffset);
  digitalWrite(dirAPin, FORWARD);
  digitalWrite(dirBPin, FORWARD);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
  Serial.println("Forward");
}

void back( byte motorSpd) {
  dirServo.write(90 + dirServoOffset);
  ultrasonicServo.write(90 + ultrasonicServoOffset);
  digitalWrite(dirAPin, BACKWARD);
  digitalWrite(dirBPin, BACKWARD);
  analogWrite(pwmAPin, motorSpd);
  analogWrite(pwmBPin, motorSpd);
  Serial.println("Back");
}

void stopcar() {
  analogWrite(pwmAPin, 0);
  analogWrite(pwmBPin, 0);
  Serial.println("Stop");
}

void read_sensor_values()
{
  sensor_front=digitalRead(LFSensor_front);
  sensor_back=digitalRead(LFSensor_back);
}
