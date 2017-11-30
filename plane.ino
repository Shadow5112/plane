#include <BMI160.h>
#include <CurieIMU.h>
#include <Servo.h>

/*
   all servvo inputs should be on anolog pins
   all servo outputs MUST be on PMW pins
*/

int trig_pin; //assign pins for ultrasonic sensor
int echo_pin;

int aileron_l_in = 5;  //assign pin numbers for ailerons
int aileron_l_out = 3;
int aileron_r_in;
int aileron_r_out;

int elevator_in; // assign pin numbers for elevators
int elevator_out;

int rudder_in; // assign pin number for rudder
int rudder_out;

int throttle_in; // assign pin number for throttle
int throttle_out;

int auto_pilot_pin; // assign pin nuber for auto pilot inturupt function - must be digital pin
int flip_pin;
int barrel_roll_pin;
int land_pin;
int takeoff_pin;

//declare servo variables
Servo  aileron_l_servo,
       aileron_r_servo,
       elevator_servo,
       rudder_servo,
       throttle_servo;

// declare initial possition of each servo 0-180 degrees
int aileron_start;
int elevator_start;
int rudder_start;
int throttle_start;

//declare maximum and minimum servo angles
int aileron_max;
int aileron_min;
int elevator_max;
int elevator_min;
int rudder_max;
int rudder_min;
int throttle_max;
int throttle_min;

// declare all nesicary functions
void manual(); //declare inturupt function

void gyro_update(); //returns usable gyro values
float gyro_x;
float gyro_y;
float gyro_z;

void velocity_update(); // updates current velocity
float velocity;

//declare auto pilot functions
void barrel_roll();
void land();
void takeoff();
void flip(); //backwards loop

float altitude(); // returns altitude with ultrasonc sensor. Range = 4.5 meters

void stabalize();

void setup() {
  Serial.begin(9600);

  //assign pins and declare if they're input or output
  pinMode(trig_pin, OUTPUT); 
  pinMode(echo_pin, INPUT);

  
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

  pinMode(auto_pilot_pin, INPUT);
  pinMode(flip_pin, INPUT);
  pinMode(barrel_roll_pin, INPUT);

  //attach servos to pins
  aileron_l_servo.attach(aileron_l_out);
  aileron_r_servo.attach(aileron_r_out);
  elevator_servo.attach(elevator_out);
  rudder_servo.attach(elevator_out);
  throttle_servo.attach(throttle_out);

  //set servos to initial possitions
  aileron_l_servo.write(aileron_start);
  aileron_r_servo.write(aileron_start);
  elevator_servo.write(elevator_start);
  rudder_servo.write(rudder_start);
  throttle_servo.write(throttle_start);

  attachInterrupt(digitalPinToInterrupt(auto_pilot_pin), manual, HIGH);
}

void loop() {
  
  if(digitalRead(flip_pin) == HIGH){
    flip();
  }
  if (digitalRead(barrel_roll_pin) == HIGH){
    barrel_roll();
  }
  if (digitalRead(takeoff_pin) == HIGH and altitude() < 450){
    takeoff();
  }
  if (digitalRead(land_pin) == HIGH and altitude() < 450){
    land();
  }
  
  gyro_update();
  velocity_update();

}

void manual() {
    analogWrite(aileron_l_out, analogRead(aileron_l_in));
    analogWrite(aileron_r_out, analogRead(aileron_r_in));
    analogWrite(elevator_out, analogRead(elevator_in));
    analogWrite(rudder_out, analogRead(rudder_in));
    analogWrite(throttle_out, analogRead(throttle_in));
}

/*
 * uses ultrasonic sensor to determine altitude plane MUST be level for accurate reading
 * returns altitude in cm range from 0 - 450cm
 */
float altitude() {
  long duration;
  long alt;

  /*
   * write low to ensure a clean high then triger sensor with a 10 microsecond high
   */
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  duration = pulseIn(echo_pin, HIGH);
  alt = duration / 29 / 2;
  /*
  speed of sound is 340m/s or 29 microseconds/cm then
  /2 to compensate for the there and back again of the sound
  */

    return alt;
}

void gyro_update() {
  
}

//returns current speed
void velocity_update() {

}

//autopiolot funcitions

void stabalize() {

}
void barrel_roll() {

}

void land() {

}

void takeoff() {

}

//backwards loop
void flip() {
  if (altitude() < 0 ) // check if plane is above ultrasonic sensor range
  {
    float level =  gyro_x; // raise elevator to maximum angle unti the plane returns to its origaonl angle 
    while (!level) {   
      elevator_servo.write(elevator_max);
    }
    stabalize();
  }
  else {
    return;
  }
}
