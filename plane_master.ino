#include <SD.h>

#include <BMI160.h>
#include <CurieIMU.h>
#include <Wire.h>
#include <Servo.h>

File flight_data;

void led_blink(); // for trouble shooting
/*
   all servvo inputs should be on anolog pins
   all servo outputs MUST be on PMW pins
*/

int slave_chanel = 9;


//Chrono timer(Chrono::MICROS);
int rudder_in; // assign pin number for rudder
int rudder_out;


int throttle_in; // assign pin number for throttle
int throttle_out;

int auto_pilot_pin = 12; // assign pin nuber for auto pilot inturupt function - must be digital pin
int flip_pin = 7;
int barrel_roll_pin = 8;
int land_pin;
int takeoff_pin = 10;

//declare servo variables
Servo  rudder_servo,
       throttle_servo;

// declare initial possition of each servo in degrees

int rudder_start;
int throttle_start;

//declare maximum and minimum servo angles in degrees


int rudder_max;
int rudder_min;

int throttle_max;
int throttle_min;


// declare all nesicary functions
void manual(); //declare inturupt function

void gyro_update(); //returns usable gyro values
float gyro_roll;
float gyro_pitch;
float gyro_yaw;

void velocity_update(); // updates current velocity
float velocity;

//declare auto pilot functions#include <SD.h>

#include <BMI160.h>
#include <CurieIMU.h>
#include <Wire.h>
#include <Servo.h>

File flight_data;
/*
   Connect VCC with 5V in the Arduino.
  Then, connect the GND of SD card to the ground of Arduino.
  Connect CS to pin 14
  Connect SCK to pin 13
  MOSI connect to the pin 11
  Lastly, connect MISO to pin 12
*/
void led_blink(); // for trouble shooting
/*
   all servvo inputs should be on anolog pins
   all servo outputs MUST be on PMW pins
*/

int slave_chanel = 9;


//Chrono timer(Chrono::MICROS);
int rudder_in; // assign pin number for rudder
int rudder_out;


int throttle_in; // assign pin number for throttle
int throttle_out;

int auto_pilot_pin = 12; // assign pin nuber for auto pilot inturupt function - must be digital pin
int flip_pin = 7;
int barrel_roll_pin = 8;
int land_pin;
int takeoff_pin = 10;

//declare servo variables
Servo  rudder_servo,
       throttle_servo;

// declare initial possition of each servo in degrees

int rudder_start;
int throttle_start;

//declare maximum and minimum servo angles in degrees


int rudder_max;
int rudder_min;

int throttle_max;
int throttle_min;


// declare all nesicary functions
void manual(); //declare inturupt function

void gyro_update(); //returns usable gyro values
float gyro_roll = 0;
float gyro_pitch = 0;
float gyro_yaw = 0;

void velocity_update(); // updates current velocity
float velocity;

//declare auto pilot functions
void barrel_roll();
void land();
void takeoff();
void flip(); //backwards loop

float altitude(); // returns altitude with ultrasonc sensor. Range = 4.5 meters

void stabalize();

void transmit_code(int);
void request_acknowledge(int);

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  CurieIMU.begin();
  Serial.println("Serial Port Open");
  //Wire.begin();
  /*
    //detect sd card
    if (!SD.begin(4)) {
      Serial.println("initialization failed!");
      while (1);
    }
    Serial.println("initialization done.");

    flight_data = SD.open("flight_data.txt", FILE_WRITE);
    flight_data.close();
  */

  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  //assign pins and declare if they're input or output

  pinMode(rudder_in, INPUT);
  pinMode(rudder_out, OUTPUT);

  pinMode(throttle_in, INPUT);
  pinMode(throttle_out, OUTPUT);

  pinMode(auto_pilot_pin, INPUT);
  pinMode(flip_pin, INPUT);
  pinMode(barrel_roll_pin, INPUT);
  pinMode(takeoff_pin, INPUT);
  pinMode(land_pin, INPUT);

  //attach servos to pins

  throttle_servo.attach(throttle_out);

  //set servos to initial possitions
  rudder_servo.write(rudder_start);
  throttle_servo.write(throttle_start);

  // attachInterrupt(digitalPinToInterrupt(auto_pilot_pin), manual, HIGH);
}

byte y = 0;
bool command_finish = true;
unsigned long start_time = 0;
unsigned long finish_time = 1;


void loop() {

  /*  if (command_finish) {
      flight_data = SD.open("flight_data.txt", FILE_WRITE);
      if ( digitalRead(flip_pin) == HIGH) {
        transmit_code(1);
        start_time = millis();
        y = 1;

      }

      if ( digitalRead(barrel_roll_pin) == HIGH) {
        transmit_code(2);
        start_time = millis();
        y = 2;
      }

      if ( digitalRead(takeoff_pin) == HIGH) {
        transmit_code(3);
        start_time = millis();
        y = 3;
      }

    }
    else {
      request_acknowledge();
      finish_time = millis();
      unsigned long lag = finish_time - start_time;
      Serial.print("lag: ");
      Serial.println(lag);
      flight_data.print("command code: ");
      flight_data.println(y);
      flight_data.print("lag(ms): ");
      flight_data.println(lag);
      flight_data.close();
    }

  */



  //led_blink();
  gyro_update();
  velocity_update();


    Serial.print("gyro roll: "); Serial.println(gyro_roll);
    Serial.print("gyro pitch: "); Serial.println(gyro_pitch);
    Serial.print("gyro yaw: "); Serial.println(gyro_yaw);
    Serial.flush();


}
/*
   paramiters to transmit
    1 - data(1) or command(2)
    2 -
        if commmand
            aileron_l
            aileron_r
            elevator_l
            elevator_r
        if data
            gyro_pitch
            gyro_roll
            gyro_yaw
            accel_x
            accel_y
            accel_z
            ?

  r
    3 - send data/command peramiter
          command paramiter(degrees)
          data(float)
*/
void transmit_code(int x) {
  /*new paramiters
      string signal_type(command/data)
        string identifier
        float data
  */

  Wire.beginTransmission(slave_chanel);
  Wire.write(x);
  Wire.endTransmission();

  Serial.print("command sent: ");
  Serial.println(x);
  command_finish = false;
}

void request_acknowledge() {
  //for lag testing
  Wire.requestFrom(slave_chanel, 1);
  while (Wire.available() < 1) {}
  byte ack = Wire.read();
  Serial.print("ack: ");
  Serial.println(ack);
  command_finish = true;
}

void manual() {
  analogWrite(rudder_out, analogRead(rudder_in));
  analogWrite(throttle_out, analogRead(throttle_in));
}

/*
   uses ultrasonic sensor to determine altitude plane MUST be level for accurate reading
   returns altitude in cm range from 0 - 450cm
*/
float altitude() {

}

void gyro_update() {

  float gyro_roll_temp, gyro_pitch_temp, gyro_yaw_temp,
        delta_roll, delta_pitch, delta_yaw;

  delta_roll = gyro_roll;
  delta_pitch = gyro_pitch;
  delta_yaw = gyro_yaw;

  int roll, pitch, yaw;

  int accel_x_temp, accel_y_temp, accel_z_temp;

  //Serial.println("updating Gyro");
  CurieIMU.readGyro(roll, pitch, yaw); // reads and store raw data from gyro)

  gyro_roll_temp = (roll / 32768.9) * CurieIMU.getGyroRange(); // conver raw data into degrees/second for angular accel
  gyro_pitch_temp = (pitch / 32768.9) * CurieIMU.getGyroRange();
  gyro_yaw_temp = (yaw / 32768.9) * CurieIMU. getGyroRange();

  delta_roll += gyro_roll_temp / (CurieIMU.getGyroRate());// divide by gyro sample rate (Hz)
  delta_pitch += gyro_pitch_temp / (CurieIMU.getGyroRate());// find distance traveled during sample
  delta_yaw += gyro_yaw_temp / (CurieIMU.getGyroRate());


  CurieIMU.readAccelerometer(accel_x_temp, accel_y_temp, accel_z_temp);
  float accel_vec_tot = sqrt(accel_x_temp * accel_x_temp + accel_y_temp * accel_y_temp +  accel_z_temp * accel_z_temp);

  float angle_pitch_accel = asin((float)accel_y_temp / accel_vec_tot) * 57.296;
  float angle_roll_accel = asin((float)accel_x_temp / accel_vec_tot) * -57.296;

  delta_pitch = delta_pitch * 0.9996 + angle_pitch_accel * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  delta_roll = delta_roll * 0.9996 + angle_roll_accel * 0.0004;

  gyro_roll = gyro_roll * 0.9 + delta_roll * 0.1;
  gyro_pitch = gyro_pitch * 0.9 + delta_pitch * 0.1;

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

}

void led_blink() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}



void barrel_roll();
void land();
void takeoff();
void flip(); //backwards loop

float altitude(); // returns altitude with ultrasonc sensor. Range = 4.5 meters

void stabalize();

void transmit_code(int);
void request_acknowledge(int);

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  CurieIMU.begin();
  Serial.println("Serial Port Open");
  Wire.begin();

  //detect sd card
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  flight_data = SD.open("flight_data.txt", FILE_WRITE);
  flight_data.close();


  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  //assign pins and declare if they're input or output

  pinMode(rudder_in, INPUT);
  pinMode(rudder_out, OUTPUT);

  pinMode(throttle_in, INPUT);
  pinMode(throttle_out, OUTPUT);

  pinMode(auto_pilot_pin, INPUT);
  pinMode(flip_pin, INPUT);
  pinMode(barrel_roll_pin, INPUT);
  pinMode(takeoff_pin, INPUT);
  pinMode(land_pin, INPUT);

  //attach servos to pins

  throttle_servo.attach(throttle_out);

  //set servos to initial possitions
  rudder_servo.write(rudder_start);
  throttle_servo.write(throttle_start);

  // attachInterrupt(digitalPinToInterrupt(auto_pilot_pin), manual, HIGH);
}

byte y = 0;
bool command_finish = true;
unsigned long start_time, finish_time;

void loop() {

  if (command_finish) {
    flight_data = SD.open("flight_data.txt", FILE_WRITE);
    if ( digitalRead(flip_pin) == HIGH) {
      transmit_code(1);
      start_time = millis();
      y = 1;

    }

    if ( digitalRead(barrel_roll_pin) == HIGH) {
      transmit_code(2);
      start_time = millis();
      y = 2;
    }

    if ( digitalRead(takeoff_pin) == HIGH) {
      transmit_code(3);
      start_time = millis();
      y = 3;
    }

  }
  else {
    request_acknowledge();
    finish_time = millis();
    unsigned long lag = finish_time - start_time;
    Serial.print("lag: ");
    Serial.println(lag);
    flight_data.print("command code: ");
    flight_data.println(y);
    flight_data.print("lag(ms): ");
    flight_data.println(lag);
    flight_data.close();
  }
    




  // led_blink();
  // gyro_update();
  // velocity_update();
  /*
    Serial.print("gyro roll: "); Serial.println(gyro_roll);
    Serial.print("gyro pitch: "); Serial.println(gyro_pitch);
    //Serial.print("gyro yaw: "); Serial.println(gyro_yaw);
    Serial.println("");
  */


}
/*
   paramiters to transmit
    1 - data(1) or command(2)
    2 -
        if commmand
            aileron_l
            aileron_r
            elevator_l
            elevator_r
        if data
            gyro_pitch
            gyro_roll
            gyro_yaw
            accel_x
            accel_y
            accel_z
            ?

  r
    3 - send data/command peramiter
          command paramiter(degrees)
          data(float)
*/
void transmit_code(int x) {
  /*new paramiters
      string signal_type(command/data)
        string identifier
        float data
  */

  Wire.beginTransmission(slave_chanel);
  Wire.write(x);
  Wire.endTransmission();

  Serial.print("command sent: ");
  Serial.println(x);
  command_finish = false;
}

void request_acknowledge() {
  //for lag testing
  Wire.requestFrom(slave_chanel, 1);
  while (Wire.available() < 1) {}
  byte ack = Wire.read();
  Serial.print("ack: ");
  Serial.println(ack);
  command_finish = true;
}

void manual() {
  analogWrite(rudder_out, analogRead(rudder_in));
  analogWrite(throttle_out, analogRead(throttle_in));
}

/*
   uses ultrasonic sensor to determine altitude plane MUST be level for accurate reading
   returns altitude in cm range from 0 - 450cm
*/
float altitude() {

}

void gyro_update() {

  float gyro_roll_temp, gyro_pitch_temp, gyro_yaw_temp,
        gyro_roll_temp2, gyro_pitch_temp2, gyro_yaw_temp2;

  gyro_roll_temp2 = gyro_roll;
  gyro_pitch_temp2 = gyro_pitch;
  gyro_yaw_temp2 = gyro_yaw;

  int roll, pitch, yaw;

  int accel_x_temp, accel_y_temp, accel_z_temp;

  Serial.println("updating Gyro");
  CurieIMU.readGyro(roll, pitch, yaw); // reads and store raw data from gyro)

  gyro_roll_temp = (roll / 32768.9) * CurieIMU.getGyroRange(); // conver raw data into degrees/second for angular accel
  gyro_pitch_temp = (pitch / 32768.9) * CurieIMU.getGyroRange();
  gyro_yaw_temp = (yaw / 32768.9) * CurieIMU. getGyroRange();

  gyro_roll_temp2 += gyro_roll_temp / (CurieIMU.getGyroRate());// divide by gyro sample rate (Hz)
  gyro_pitch_temp2 += gyro_pitch_temp / (CurieIMU.getGyroRate());// find distance traveled during sample
  gyro_yaw_temp2 += gyro_yaw_temp / (CurieIMU.getGyroRate());


  CurieIMU.readAccelerometer(accel_x_temp, accel_y_temp, accel_z_temp);
  float accel_vec_tot = sqrt(accel_x_temp * accel_x_temp + accel_y_temp * accel_y_temp +  accel_z_temp * accel_z_temp);

  float angle_pitch_accel = asin((float)accel_y_temp / accel_vec_tot) * 57.296;
  float angle_roll_accel = asin((float)accel_x_temp / accel_vec_tot) * -57.296;

  gyro_pitch_temp2 = gyro_pitch_temp2 * 0.9996 + angle_pitch_accel * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  gyro_roll_temp2 = gyro_roll_temp2 * 0.9996 + angle_roll_accel * 0.0004;

  gyro_roll = gyro_roll * 0.9 + gyro_roll_temp2 * 0.1;
  gyro_pitch = gyro_pitch * 0.9 + gyro_pitch_temp2 * 0.1;

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

}

void led_blink() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}


