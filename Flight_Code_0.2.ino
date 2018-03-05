#include<SD.h>
#include<SPI.h>
#include <Wire.h>
#include <Servo.h>

#include <LSM303.h>// for compass and accelerometer
#include <LPS.h>// for pressure sensor
#include <L3G.h>// for gyro

L3G gyro;
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768}; 
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

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ, gyro_cal_x, gyro_cal_y, gyro_cal_z, accel_cal_x, accel_cal_y, accel_cal_z;
float rotX, rotY, rotZ;
float alt, alt_offset;

float vel;

float pitch_angle, roll_angle, yaw_angle, acc_vector;

unsigned long print_timer = 0;
unsigned long print_timer_2 = 0;

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
///PINS
///


//declare servo variables
Servo  rudder_servo,
       throttle_servo,
       aileron_servo,
       elevator_servo;

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
///INITIAL VALIUES
///

int num_files = 3; // number of files that will be created
String file_1 = ("Accel.txt");
String file_2 = ("Gyro.txt");
String file_3 = ("Orien.txt");

File accel_data; // Variable for file data
File gyro_data;
File orientation_data;

///
///FUNCTIONS
///

// declare all nesicary functions

//declare data functions
void setupMPU();
void config_MPU();
void recordAccelData();
void convertAccelData();
void recordGyroData();
void convertGyroData();
void caculateAngle();
void displayGyroAccel();
void writeData();
void update_velocity();

//declare auto pilot functions
void barrel_roll();
void land();
void takeoff();
void flip(); //backwards loop
void balance();
void manuel();

void displayData();
void calibrateAccel();
void calilbrateAlt();
float readAlt();

///
///FUNCTIONS
///

void setup()
{

  Serial.begin(9600);

  Wire.begin();

  pinMode(10, OUTPUT); // Pin 10 initialized to Output
  SD.begin(chipSelect); // Initialize the SD with ChipSelect
 
    if(!gyro.init()){
    Serial.println("Failed  to autodetect gyro type");
    while(1);  
  }

  if(!compass.init()){
    Serial.println("failed to detect compass");
  }

  if(!ps.init()){
    Serial.println("failed to detect presure sensor");
    while(1);
  }
  ps.enableDefault();
  gyro.enableDefault();
  compass.enableDefault();
   
   ///// Calibrate gyro/accel ////////////
   for (int i = 0; i < 1000; i++)
  {
    recordGyroData();
    convertGyroData();

    gyro_cal_x += gyroX;
    gyro_cal_y += gyroY;
    gyro_cal_z += gyroZ;
    
    accel_cal_x += accelX;
    accel_cal_y += accelY;
    accel_cal_z += accelZ;  
     
     delay(3);
    /*  Serial.print(gyro_cal_x);
      Serial.print(" ");
      Serial.print(gyro_cal_y);
      Serial.print(" ");
      Serial.println(gyro_cal_z);
    */
  }
  
  accel_cal_x /= 1000.0
  accel_cal_y /= 1000.0
  accel_cal_z /= 1000.0
    
  gyro_cal_x /= 1000.0;
  gyro_cal_y /= 1000.0;
  gyro_cal_z /= 1000.0;
  Serial.print(gyro_cal_x);
  Serial.print(" ");
  Serial.print(gyro_cal_y);
  Serial.print(" ");
  Serial.println(gyro_cal_z);

  accel_cal_x += accelX;
  accel_cal_y += accelY;
  accel_cal_z += accelZ;
  
////////////////////////////////////

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

  rudder_servo.attach(rudder_out); //attach servo objects to pins
  aileron_servo.attach(aileron_out);
  elevator_servo.attach(elevator_out);
  throttle_servo.attach(throttle_out);

  rudder_servo.write(rudder_start); // set servos to start values
  aileron_servo.write(aileron_start);
  elevator_servo.write(elevator_start);
  throttle_servo.write(throttle_start);
   
  calibrateAlt();

  Serial.println("Setup Complete");
  Serial.println(alt_offset);
  
}

void loop()
{

  recordGyroData();
  gyroX -= gyro_cal_x;
  gyroY -= gyro_cal_y;
  gyroZ -= gyro_cal_z;

  /*Serial.print(gyroX);
    Serial.print(" ");
    Serial.print(gyroY);
    Serial.print(" ");
    Serial.println(gyroZ);
  */
  convertGyroData();
  recordAccelData();
  caculateAngle();
  convertAccelData();
  displayData();
  balance();

  if (int(millis() - print_timer) > 100)
  {
    writeData();
    print_timer = millis();
  }


  if (auto_pin == LOW){
    
  }
  else{
    manuel();
  }
}




void recordAccelData()
{
  compass.read();
  accelX = compass.a.x;
  accelY = compass.a.y;
  accelZ = compass.a.z;
}

void convertAccelData()
{
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; // Converts raw Accel Data into G-Froce
  gForceZ = accelZ / 16384.0;
}

void recordGyroData()
{
  gyro.read();
  gyroX = gyro.g.x;
  gyroY = gyro.g.y;
  gyroZ = gyro.g.z;

}

void convertGyroData()
{
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; // Conversion Factor given by data sheet for the 250deg/s setting
  rotZ = gyroZ / 131.0; // Converts for Raw data to Deg/s
}

void caculateAngle()
{


  acc_vector = sqrt ((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ)); // Calc Accel vector

  float acc_roll_angle = asin(accelX / acc_vector) * (180.0 / 3.14); // Calc roll angle
  float  acc_pitch_angle = asin(accelY / acc_vector) * (180.0 / 3.14); // Calc pitch angle

  //unsigned long sample_time = millis() - gyro_timer; // calculate time since angle was last calculated in ms
  float delta_pitch = rotX / 250; // convert to ms to s and find angle moved sincce last measument
  float delta_roll = rotY / 250;
  float delta_yaw = rotZ / 250;

  roll_angle += delta_roll; // add change in angle to total angle
  pitch_angle += delta_pitch;
  yaw_angle += delta_yaw;

  //pitch_angle += roll_angle * sin(yaw_angle * 0.000000533);
  //roll_angle -= pitch_angle * sin(yaw_angle * 0.000001066);

  roll_angle = roll_angle * 0.92 + acc_roll_angle * 0.08; // buffer gyroscope with acccerometer data
  pitch_angle = pitch_angle * 0.96 + acc_pitch_angle * 0.04;
  if (!(roll_angle == roll_angle))
  {
    roll_angle = acc_roll_angle;
    pitch_angle = acc_pitch_angle;
  }
}

void displayData(){

  Serial.print("G ");
  Serial.print("X: ");
  Serial.print(roll_angle);
  Serial.print(" Y: ");
  Serial.print(pitch_angle);
  Serial.print(" Z: ");
  Serial.print(yaw_angle);
  Serial.print(" heading: ");
  Serial.print(compass.heading());
  Serial.print(" Alt: ");
  Serial.print(readAlt());
  Serial.print(" temp: ");
  Serial.println(ps.readTemperatureC());

}
void writeData() {
  accel_data = SD.open(file_1, FILE_WRITE);

  if (accel_data) {
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
    orientation_data.print(roll_angle);
    orientation_data.print(",");
    orientation_data.print(pitch_angle);
    orientation_data.print(",");
    orientation_data.print(millis());
    orientation_data.println(",");
  }
  orientation_data.close();
}
void balance()
{
  int y;
  int x = roll_angle;
  if (roll_angle > 5)
  {
    y = 1 + rudder_servo.read();
    x = checkAngle(y);
    rudder_servo.write(x);
  }
  if (roll_angle < 5)
  {
    y = rudder_servo.read() - 1;
    x = checkAngle(y);
    rudder_servo.write(x);
  }
}

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

void manuel() {
  analogWrite(aileron_out, analogRead(aileron_in));
  analogWrite(aileron_out, analogRead(aileron_in));
  analogWrite(elevator_out, analogRead(elevator_in));
  analogWrite(rudder_out, analogRead(rudder_in));
  analogWrite(throttle_out, analogRead(throttle_in));
}

float readAlt(){
  alt = ps.pressureToAltitudeMeters(ps.readPressureMillibars()) - alt_offset;
  return alt;
}

void calibrateAlt(){
  float cal;

  for(int x = 0; x < 1000; x++){
    cal += ps.pressureToAltitudeMeters(ps.readPressureMillibars());
  }
  cal /= 1000;

  alt_offset = cal;
}




