#include <BMI160.h>
#include <CurieIMU.h>

#include <Servo.h>

int aileron_l_in;  //assign pin numbers for ailerons
int aileron_l_out;
int aileron_r_in;
int aileron_r_out;

int elevator_in; // assign pin numbers for elevators
int elevator_out;

int rudder_in; // assign pin number for rudder
int rudder_out;

int throttle_in; // assign pin number for throttle
int throttle_out;

int auto_pilot; // assign pin numbers for auto functions
int flip;
int barrel_roll;


// declare all nesicary functions

volatile bool check_auto(); //returns true/false if auto pilot is engaged
void manual(); //declare inturupt function

float gyro(); //returns usable gyro values
float gyro.x();
float gyro.y();
float gyro.z();

float accel(); // returns usable acceleration values in m/s
float accel.x();
float accel.y();
float accel.z();

float velocity(); // returns current speed

//declare auto pilot functions
void barrle_roll();
void land();
void takeoff();
void flip(); //backwards loop

float altitude(): // returns altitude with ultrasonc sensor. Range = 2-3 meters

void stabalize();


  void setup() {
  Serial.begin(9600);

  //assign pins
  pinMode(aileron_l_in, INPUT);
  pinMode(aileron_l_out, OUTPUT);

  pinMode(aileron_r_in, INPUT);
  pinMode(aileron_r_out, OUTPUT);

  pinMode(elevator_in, INPUT);
  pinMode(elevator_out, OUTPUT);

  pinMode(rudder_in, INPUT);
  pinMode(rudder_out, OUTPUT);

  pinMode(throttle_in, INPUT);
  pinMode(throttle_out, OUTPUT);

  pinMode(autopilot, INPUT);
  pinMode(flip, INPUT);
  pinMode(barrel_roll, INPUT);

  attachInterrupt(0,manual,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void manual(){
  analogWrite(aileron_l_out, analogRead(aileron_l_in));
  analogWrite(aileron_r_out, analogRead(aileron_r_in));
  analogWrite(elevator_out, analogRead(elevator_in));
  analogWrite(rudder_out, analogRead(rudder_in));
  analogWrite(throttle_out, analogRead(throttle_in));

       
}

bool check_autopilot() {
}

float altitude() {
}

array float gyro() {
}

float gyro.x() {
}
float gyro.y() {
}
float gyro.z() {
}

array float accel() {
} // returns usable acceleration values in m/s
float accel.x() {
}
float accel.y() {
}
float accel.z() {
}

//returns current speed
float velocity() {
}

//autopiolot funcitions

void stabalize() {
}
void barrle_roll() {
}

void land() {
}

void takeoff() {
}

//backwards loop
void flip() {
}
