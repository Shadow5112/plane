#include<SD.h>
#include<SPI.h>
#include <Wire.h>
#include <Servo.h>

#include <LSM303.h>// for compass and accelerometer
#include <LPS.h>// for pressure sensor
#include <L3G.h>// for gyro

L3G gyro;
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32768, -32768, -32768};
LPS ps;

/*
   SD card pins
        mega 2560
          MOSI - 51
          MISO - 50
          SCK - 52
        uno
          MOSI - 11
          MISO - 12
          SCK - 13

          CS / chip select - digital pin
*/


///
///Variables
///
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ, gyro_cal_x, gyro_cal_y, gyro_cal_z;
float gyro_offset_x, gyro_offset_y, gyro_offset_z;
float rotX, rotY, rotZ;
float alt, alt_offset;

float vel;

float pitch_angle, roll_angle, yaw_angle, acc_vector;

unsigned long print_timer = 0;
unsigned long print_timer_2 = 0;

//declare servo variables
Servo  rudder_servo,
       throttle_servo,
       aileron_servo,
       elevator_servo;

int num_files = 3; // number of files that will be created

String file_1 = ("Accel.txt");
String file_2 = ("Gyro.txt");
String file_3 = ("Orien.txt");
String file_4 = ("Servo.txt");

File accel_data; // Variable for file data
File gyro_data;
File orientation_data;
File servo_data;

//first loop bools
bool first_print = true;
bool first_data = true;
///
///
///

///
///PINS
///

int chipSelect = 4; //Set ChipSelect for sd card reader to pin 4

int rudder_in; // assign pin number for rudder
int rudder_out = 9;

int throttle_in; // assign pin number for throttle
int throttle_out;

int aileron_in; // assign pin for ailerons
int aileron_out;

int elevator_in;// assign pin for elevators
int elevator_out;

int auto_pin; // assign pin nuber for auto pilot function
int flip_pin;       //inturupt must be on digital pin
int barrel_roll_pin;
int land_pin;
int takeoff_pin;

///
///
///

///
///INITIAL VAlUES
///

// declare initial possition of each servo in degrees

int rudder_start;
int throttle_start;
int aileron_start;
int elevator_start;
//declare maximum and minimum servo angles in degrees

int servo_max = 150; //max and min angle for all servos
int servo_min = 30;

///
///
///


///
///FUNCTIONS
///

// setup functions
void setupMPU();
void config_MPU();
void calibrateAccel();
void calilbrateAlt();

//data functions
void recordAccelData();
void convertAccelData();
void recordGyroData();
void convertGyroData();
void caculateAngle();
void displayGyroAccel();
void writeData();
void update_velocity();
float readAlt();
void displayData();

//auto pilot functions
void barrel_roll();
void land();
void takeoff();
void flip(); //backwards loop
void balance();
void manuel();

///
///
///
void setup()
{
  Serial.begin(9600);
  Wire.begin();

  pinMode(10, OUTPUT); // Pin 10 initialized to Output
  SD.begin(chipSelect); // Initialize the SD with ChipSelect

  //Setup AltIMU
  //
  if (!gyro.init()) {
    Serial.println("Failed  to autodetect gyro type");
    while (1);
  }

  if (!compass.init()) {
    Serial.println("failed to detect compass");
  }

  if (!ps.init()) {
    Serial.println("failed to detect presure sensor");
  }
  ps.enableDefault();
  gyro.enableDefault();
  gyro.writeReg(L3G::CTRL1, 0b010011111); //set data output rate to 400 Hz // i think
  compass.enableDefault();
  calibrateAlt();
  //////
  //////

  //
  //calibrate gyroscope
  for (int i = 0; i < 1000; i++)
  {
    recordGyroData();
    convertGyroData();

    gyro_cal_x += gyroX;
    gyro_cal_y += gyroY;
    gyro_cal_z += gyroZ;
    delay(3);
    Serial.print(gyro_cal_x);
    Serial.print(" ");
    Serial.print(gyro_cal_y);
    Serial.print(" ");
    Serial.println(gyro_cal_z);

  }

  gyro_offset_x = float(gyro_cal_x) / 1000.0;
  gyro_offset_y = float(gyro_cal_y) / 1000.0;
  gyro_offset_z = float(gyro_cal_z) / 1000.0;
  Serial.print(gyro_offset_x);
  Serial.print(" ");
  Serial.print(gyro_offset_y);
  Serial.print(" ");
  Serial.println(gyro_offset_z);
  /////
  /////

  //
  //initialise pins
  pinMode(rudder_in, INPUT); // inisialize  pinmodes for commands from the reciever
  pinMode(throttle_in, INPUT);
  pinMode(aileron_in, INPUT);
  pinMode(elevator_in, INPUT);
  pinMode(auto_pin, INPUT);
  pinMode(flip_pin, INPUT);
  pinMode(barrel_roll_pin, INPUT);
  pinMode(land_pin, INPUT);
  pinMode(takeoff_pin, INPUT);
  pinMode(land_pin, INPUT);
  /////
  /////

  //
  //setup servos
  rudder_servo.attach(rudder_out); //attach servo objects to pins
  aileron_servo.attach(aileron_out);
  elevator_servo.attach(elevator_out);
  throttle_servo.attach(throttle_out);

  rudder_servo.write(rudder_start); // set servos to start values
  aileron_servo.write(aileron_start);
  elevator_servo.write(elevator_start);
  throttle_servo.write(throttle_start);
  /////
  /////

  Serial.println("Setup Complete");

  
}



//read raw x, y, z vauees from accelerometer
void recordAccelData()
{
  compass.read();
  accelX = compass.a.x;
  accelY = compass.a.y;
  accelZ = compass.a.z;
}

//convert raw data into g
void convertAccelData()
{
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; // Converts raw Accel Data into G-Froce
  gForceZ = accelZ / 16384.0;
}

//read raw gyro data
void recordGyroData()
{
  gyro.read();
  gyroX = gyro.g.x;
  gyroY = gyro.g.y;
  gyroZ = gyro.g.z;

}

//convert gyro data into degrees/sec
void convertGyroData()
{
  float conv_factor = 8.75; // Conversion Factor given by data sheet for the 250deg/s setting
  rotX = gyroX / conv_factor;// Converts for Raw data to Deg/s
  rotY = gyroY / conv_factor;
  rotZ = gyroZ / conv_factor;
}

float acc_pitch, acc_roll, acc_vec;
unsigned long lag = 0;

//use gyro and acclerometer and commpass data to calculate orientation
void caculateAngle()
{
  float gyro_rate = 76.5;// sample rate of gyroscope - or at least some converstion factor

  acc_vector = sqrt ((gForceX * gForceX) + (gForceY * gForceY) + (gForceZ * gForceZ)); // Calc Accel vector
  acc_vec = acc_vector;
  float acc_roll_angle =  asin(gForceY / acc_vector) * (180.0 / 3.14); // Calc roll angle range +- 90 degrees
  float  acc_pitch_angle = asin(gForceX / acc_vector) * (180.0 / 3.14); // Calc pitch angle

  //unsigned long sample_time = millis() - gyro_timer; // calculate time since angle was last calculated in ms
  float delta_pitch = rotY / gyro_rate; // divite by gyro sample rate to find angle moved sincce last measument
  float delta_roll = rotX / gyro_rate;  // gyro_rate  declared as local variable
  float delta_yaw = rotZ / gyro_rate;

  roll_angle += delta_roll; // add change in angle to total angle
  pitch_angle += delta_pitch;
  yaw_angle += delta_yaw;

  ///
  ///restrict roll angle to 0-360
  ///
  if (roll_angle > 360) {
    roll_angle -= 360.0;
  }
  if (roll_angle < 0) {
    roll_angle += 360.0;
  }
  if (gForceY <= 0.01 and gForceY >= -0.01 and gForceX < 0.9 and gForceX > -0.9 and gForceZ > 0) {
    roll_angle = 0;
  }
  if (gForceY <= 0.01 and gForceY >= -0.01 and gForceX < 0.9 and gForceX > -0.9 and gForceZ < 0) {
    roll_angle = 180;
  }
  //
  //

  //pitch_angle += roll_angle * sin(yaw_angle * 0.000000533);
  //roll_angle -= pitch_angle * sin(yaw_angle * 0.000001066);

  ///
  ///correct acc_roll-angle to range of 0 - 360 degrees
  ///
  if (gForceY >= 0 and gForceZ <= 0) { // quadrant 1
    acc_roll_angle = 90.0 + (90.0 - acc_roll_angle);
  }
  if (gForceY < 0 and gForceZ < 0) { // quardant 4
    acc_roll_angle = 90 + (90.0 - acc_roll_angle);
  }
  if (gForceY < 0 and gForceZ > 0) { //quadrant 3
    acc_roll_angle = 270 + (90 + acc_roll_angle);
  }
  //
  //


  ///
  ///correct acc_pitch_angle to range of 0 - 360 degrees
  ///
  if (gForceX > 0 and gForceZ < 0) { //quadrant 2
    acc_pitch_angle = 90.0 + (90.0 - acc_pitch_angle);
  }

  if (gForceX < 0 and gForceZ < 0) {
    acc_pitch_angle = 90.0 + (90 - acc_pitch_angle);
  }
  if (gForceX < 0 and gForceZ > 0) {
    acc_pitch_angle = 270 + (90 + acc_pitch_angle);
  }
  //
  //

  if (first_data) {
    roll_angle = acc_roll_angle;
    pitch_angle = acc_pitch_angle;
    first_data = false;
  }
  else {
    if (acc_vector < 1.01 and acc_vector > 0.98)
    {
      roll_angle = roll_angle;// * 0.60 + acc_roll_angle * 0.4; // buffer gyroscope with acccerometer data
      pitch_angle = pitch_angle;// * 0.60 + acc_pitch_angle * 0.4;
    }
    else {
      roll_angle = roll_angle;// * 0.996 + acc_roll_angle * 0.004; // buffer gyroscope with acccerometer data
      pitch_angle = pitch_angle;// * 0.996 + acc_pitch_angle * 0.004;
    }
  }

  acc_roll = acc_roll_angle;
  acc_pitch = acc_pitch_angle;

  //when plane is horizontal yaw ~= compass heading
  if (roll_angle >= -5 and roll_angle <= 5) {
    yaw_angle = compass.heading();
  }


  if (!(roll_angle == roll_angle) or !(pitch_angle == pitch_angle))  //check for NaN
  {
    roll_angle = acc_roll_angle;
    pitch_angle = acc_pitch_angle;
  }

  lag = millis();
}


//display data on serial monitor
void displayData() {

  Serial.print("G ");
  Serial.print("roll: ");
  Serial.print(roll_angle);
  Serial.print(" pitch: ");
  Serial.print(pitch_angle);
  Serial.print(" yaw: ");
  Serial.print(yaw_angle);


  Serial.print(" heading: ");
  Serial.print(compass.heading());
  Serial.print(" mx: ");
  Serial.print(gForceX);
  Serial.print(" my: ");
  Serial.print(gForceY);
  Serial.print(" mz: ");
  Serial.print(gForceZ);
  Serial.print(" accle_pitch: ");
  Serial.print(acc_pitch);
  Serial.print(" accel_roll: ");
  Serial.print(acc_roll);
  Serial.print(" acc vect: ");
  Serial.print(acc_vec);
  Serial.print( " lag: ");
  Serial.println(millis() - lag);





}

//write data to SD card
//gyro, accel, orientation, and servo data writen to seperate files
void writeData() {
  accel_data = SD.open(file_1, FILE_WRITE);

  if (accel_data) {
    if (first_print) {
      accel_data.print("gForceX");
      accel_data.print(",");
      accel_data.print("gForceY");
      accel_data.print(",");
      accel_data.print("gForceZ");
      accel_data.print(",");
      accel_data.print("ms");
      accel_data.println(",");
    }
    accel_data.print(gForceX);
    accel_data.print(",");
    accel_data.print(gForceY);
    accel_data.print(",");
    accel_data.print(gForceZ);
    accel_data.print(",");
    accel_data.print(millis());
    accel_data.println(",");
  }
  accel_data.close();
  gyro_data = SD.open(file_2, FILE_WRITE);

  if (gyro_data) {
    if (first_print) {
      gyro_data.print("rotX");
      gyro_data.print(",");
      gyro_data.print("rotY");
      gyro_data.print(",");
      gyro_data.print("rotZ");
      gyro_data.print(",");
      gyro_data.print("millis");
      gyro_data.println(",");
    }
    gyro_data.print(rotX);
    gyro_data.print(",");
    gyro_data.print(rotY);
    gyro_data.print(",");
    gyro_data.print(rotZ);
    gyro_data.print(",");
    gyro_data.print(millis());
    gyro_data.println(",");
  }

  gyro_data.close();

  orientation_data = SD.open(file_3, FILE_WRITE);

  if (orientation_data) {
    if (first_print) {
      orientation_data.print("roll_angle");
      orientation_data.print(",");
      orientation_data.print("pitch_angle");
      orientation_data.print(",");
      orientation_data.print("yaw_angle");
      orientation_data.print(", ");
      orientation_data.print("millis");
      orientation_data.println(",");
    }
    orientation_data.print(roll_angle);
    orientation_data.print(",");
    orientation_data.print(pitch_angle);
    orientation_data.print(",");
    orientation_data.print("yaw_angle");
    orientation_data.print(",");
    orientation_data.print(millis());
    orientation_data.println(",");
  }
  orientation_data.close();

  servo_data = SD.open(file_4, FILE_WRITE);

  if (servo_data) {
    if (first_print) {
      servo_data.print("throttle_servo");
      servo_data.print(", ");
      servo_data.print("rudder_servo");
      servo_data.print(", ");
      servo_data.print("aileron_servo");
      servo_data.print(", ");
      servo_data.print("elevator_servo");
      servo_data.print(", ");
      servo_data.print("millis");
      servo_data.println(", ");
    }
    servo_data.print(throttle_servo.read());
    servo_data.print(", ");
    servo_data.print(rudder_servo.read());
    servo_data.print(", ");
    servo_data.print(aileron_servo.read());
    servo_data.print(", ");
    servo_data.print(elevator_servo.read());
    servo_data.print(", ");
    servo_data.print(millis());
    servo_data.println(", ");

  }

  servo_data.close();
  first_print = false;
}

//keeps the plane flying horizontal
void balance()
{
  int y;
  int x;
  if (roll_angle > 5)
  {
    y = 1 + aileron_sero.read();
    x = checkAngle(y);
    aileron_sero.write(x);
  }
  if (roll_angle < 5)
  {
    y = aileron_sero.read() - 1;
    x = checkAngle(y);
    aileron_sero.write(x);
  }
}

//checks if servo angle exceeds max or min angle
int checkAngle(int x)
{
  if (x < servo_min)
  {
    return servo_min;
  }

  else if (x > servo_max)
  {
    return servo_max;
  }

  else
  {
    return x;
  }
}

//passes signals from receiver directly to servos
void manuel() {
  analogWrite(aileron_out, analogRead(aileron_in));
  analogWrite(aileron_out, analogRead(aileron_in));
  analogWrite(elevator_out, analogRead(elevator_in));
  analogWrite(rudder_out, analogRead(rudder_in));
  analogWrite(throttle_out, analogRead(throttle_in));
}

//returns altitude +- 1m 
// 0 is altitude where board was initialised
float readAlt() {
  alt = ps.pressureToAltitudeMeters(ps.readPressureMillibars())- alt_offset ;
  return alt;
}

//adjusts altitude so that ground level = 0
void calibrateAlt() {
  float cal;

  for (int x = 0; x < 1000; x++) {
    cal += ps.pressureToAltitudeMeters(ps.readPressureMillibars());
  }
  cal /= 1000;

  alt_offset = cal;
}



