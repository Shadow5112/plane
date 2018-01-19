#include <Wire.h>

#include <Servo.h>

int aileron_l_in;  //assign pin numbers for ailerons
int aileron_l_out = 3;
int aileron_r_in;
int aileron_r_out;

int elevator_in; // assign pin numbers for elevators
int elevator_out;

int rudder_in; // assign pin number for rudder
int rudder_out;

int auto_pilot_pin; // assign pin nuber for auto pilot inturupt function - must be digital pin
int flip_pin;
int barrel_roll_pin;
int land_pin;
int takeoff_pin;

//declare servo variables
Servo  aileron_l_servo,
       aileron_r_servo,
       elevator_servo,
       rudder_servo;

// declare initial possition of each servo 0-180 degrees
int aileron_start = 90;
int elevator_start;
int rudder_start;


//declare maximum and minimum servo angles
int aileron_max;
int aileron_min;
int elevator_max;
int elevator_min;
int rudder_max;
int rudder_min;


void barrel_roll();
void land();
void takeoff();
void flip(); //backwards loop

void stabalize();

void read_command();#include <Wire.h>

#include <Servo.h>

int aileron_l_in;  //assign pin numbers for ailerons
int aileron_l_out = 3;
int aileron_r_in;
int aileron_r_out;

int elevator_in; // assign pin numbers for elevators
int elevator_out;

int rudder_in; // assign pin number for rudder
int rudder_out;

int auto_pilot_pin; // assign pin nuber for auto pilot inturupt function - must be digital pin
int flip_pin;
int barrel_roll_pin;
int land_pin;
int takeoff_pin;

//declare servo variables
Servo  aileron_l_servo,
       aileron_r_servo,
       elevator_servo,
       rudder_servo;

// declare initial possition of each servo 0-180 degrees
int aileron_start = 90;
int elevator_start;
int rudder_start;


//declare maximum and minimum servo angles
int aileron_max;
int aileron_min;
int elevator_max;
int elevator_min;
int rudder_max;
int rudder_min;


void barrel_roll();
void land();
void takeoff();
void flip(); //backwards loop

void stabalize();

void read_command();
void led_blink();

int led = 13;
byte x = 0;
byte command_finished = true;

void setup() {
  Serial.begin(9600);
  while(!Serial){}
  Serial.println("serial port open");
  
  Wire.begin(9);
  Wire.onReceive(reveive_event);
  Wire.onRequest(request_event);

  pinMode(led , OUTPUT);

  pinMode(aileron_l_in, INPUT);
  pinMode(aileron_l_out, OUTPUT);

  pinMode(aileron_r_in, INPUT);
  pinMode(aileron_r_out, OUTPUT);

  pinMode(elevator_in, INPUT);
  pinMode(elevator_out, OUTPUT);

  pinMode(rudder_in, INPUT);
  pinMode(rudder_out, OUTPUT);

  pinMode(auto_pilot_pin, INPUT);
  pinMode(flip_pin, INPUT);
  pinMode(barrel_roll_pin, INPUT);

  //attach servos to pins
  aileron_l_servo.attach(aileron_l_out);
  aileron_r_servo.attach(aileron_r_out);
  elevator_servo.attach(elevator_out);
  rudder_servo.attach(elevator_out);

  //set servos to initial possitions
  aileron_l_servo.write(aileron_start);
  aileron_r_servo.write(aileron_start);
  elevator_servo.write(elevator_start);
  rudder_servo.write(rudder_start);

}

void reveive_event(int bytes) {
  
  command_finished = false;
  x =  Wire.read();
  Serial.print("commmand received: ");
  Serial.println(x);
  read_command(x);
}

void request_event() {
  
    while(command_finished == false){
    }
    
    Wire.write(0);
    Serial.println("ack sent"); 

  
}
void loop() {

  //  Serial.println("no command");

}

//interprit command from master
void read_command(byte x) {
  switch (x)
  {
    case 1:
      led_blink();
      break;
    case 2:
      aileron_l_servo.write(70);
      break;
    case 3:
      aileron_l_servo.write(20);
      break;
    default:
      Serial.print("invalid command: ");
      Serial.println(x);
      break;
  }
   command_finished = true;
}

void led_blink() {

  digitalWrite(led, HIGH);
  delay(300);
  digitalWrite(led, LOW);
  delay(300);

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
/*
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
*/
void led_blink();

int led = 13;
byte x = 0;
byte command_finished = true;

void setup() {
  Serial.begin(9600);
  while(!Serial){}
  Serial.println("serial port open");
  
  Wire.begin(9);
  Wire.onReceive(reveive_event);
  Wire.onRequest(request_event);

  pinMode(led , OUTPUT);

  pinMode(aileron_l_in, INPUT);
  pinMode(aileron_l_out, OUTPUT);

  pinMode(aileron_r_in, INPUT);
  pinMode(aileron_r_out, OUTPUT);

  pinMode(elevator_in, INPUT);
  pinMode(elevator_out, OUTPUT);

  pinMode(rudder_in, INPUT);
  pinMode(rudder_out, OUTPUT);

  pinMode(auto_pilot_pin, INPUT);
  pinMode(flip_pin, INPUT);
  pinMode(barrel_roll_pin, INPUT);

  //attach servos to pins
  aileron_l_servo.attach(aileron_l_out);
  aileron_r_servo.attach(aileron_r_out);
  elevator_servo.attach(elevator_out);
  rudder_servo.attach(elevator_out);

  //set servos to initial possitions
  aileron_l_servo.write(aileron_start);
  aileron_r_servo.write(aileron_start);
  elevator_servo.write(elevator_start);
  rudder_servo.write(rudder_start);

}

void reveive_event(int bytes) {
  
  command_finished = false;
  x =  Wire.read();
  Serial.print("commmand received: ");
  Serial.println(x);
  read_command(x);
}

void request_event() {
  
    while(command_finished == false){
      read_command(x);
    }

    Wire.write(0);
    Serial.println("ack sent"); 

  
}
void loop() {

  //  Serial.println("no command");

}

//interprit command from master
void read_command(byte x) {
  switch (x)
  {
    case 1:
      led_blink();
      break;
    case 2:
      aileron_l_servo.write(70);
      break;
    case 3:
      aileron_l_servo.write(20);
      break;
    default:
      Serial.print("invalid command: ");
      Serial.println(x);
      break;
  }
  command_finished = true;
}

void led_blink() {

  digitalWrite(led, HIGH);
  delay(300);
  digitalWrite(led, LOW);
  delay(300);

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
/*
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
*/
