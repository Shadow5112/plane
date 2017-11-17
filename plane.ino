#include <BMI160.h>
#include <CurieIMU.h>

#include <Servo.h>

int trig_pin; //assign pins for ultrasonic sensor
int echo_pin;

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

int auto_pilot; // assign pin nuber for auto pilot inturupt function
int flip_input;
int barrel_roll;

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

// declare all nesicary functions
void manual(); //declare inturupt function

float gyro(); //returns usable gyro values
float gyro_x();
float gyro_y();
float gyro_z();

float accel(); // returns usable acceleration values in m/s
float accel_x();
float accel_y();
float accel_z();

float velocity(); // returns current speed

//declare auto pilot functions
void barrle_roll();
void land();
void takeoff();
void flip(); //backwards loop

float altitude(); // returns altitude with ultrasonc sensor. Range = 2-3 meters

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

  pinMode(auto_pilot, INPUT);
  pinMode(flip_input, INPUT);
  pinMode(barrel_roll, INPUT);
  
  
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

  attachInterrupt(digitalPinToInterrupt(auto_pilot),manual,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void manual(){
  while(digitalRead(auto_pilot) != LOW){
  analogWrite(aileron_l_out, analogRead(aileron_l_in));
  analogWrite(aileron_r_out, analogRead(aileron_r_in));
  analogWrite(elevator_out, analogRead(elevator_in));
  analogWrite(rudder_out, analogRead(rudder_in));
  analogWrite(throttle_out, analogRead(throttle_in));
  }
  
}

float altitude() {
 long duration;

  /*
   * write low to ensure a clean high then triger sensor with a 10 microsecond high
   */
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  duration = pulseIn(echo_pin, HIGH);

  return duration / 29 / 2; /*
  speed of sound is 340m/s or 29 microseconds/cm then
  /2 to compensate for the there and back again of the sound
  */
}

float gyro() {
return 0;
}

float gyro_x() {
return 0;
}
float gyro_y() {
return 0;
}
float gyro_z() {
return 0;
}

float accel() {
return 0;
} // returns usable acceleration values in m/s
float accel_x() {
return 0;
}
float accel_y() {
return 0;
}
float accel_z() {
return 0;
}

//returns current speed
float velocity() {
return 0;
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
  if (altitude() < 0 ) // check if plane is above ultrasonic sensor range
  {
    float level =  gyro_x(); // raise elevator to maximum angle unti the plane returns to its origaonl angle 
    while (!level) {   
      elevator_servo.write(elevator_max);
    }
    stabalize();
  }
  else {
    return;
  }
}
