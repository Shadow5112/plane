#include <LSM303.h>// for compass and accelerometer
#include <LPS.h>// for pressure sensor
#include <L3G.h>// for gyro

#include<Wire.h>
#include<SD.h>
#include<SPI.h>
#include<Servo.h>

L3G gyro;
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768}; 
LPS ps;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ, gyro_cal_x, gyro_cal_y, gyro_cal_z, accel_cal_x, accel_cal_y, accel_cal_z;
float rotX, rotY, rotZ;

float alt, alt_offset;
float vel;

float pitch_angle, roll_angle, yaw_angle, acc_vector;
int chipSelect = 22;
unsigned long print_timer = 0;

void recordAccelData();
void convertAccelData();
void recordGyroData();
void convertGyroData();
void caculateAngle();
void displayGyroAccel();
void writeData();
void displayData();
void calibrateAccel();
void calilbrateAlt();
float readAlt();


void setup() {
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

  calibrateAlt();
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
  Serial.println("Setup Complete");
  Serial.println(alt_offset);
}

void loop() {

  recordGyroData();
  convertGyroData();
  recordAccelData();
  convertAccelData();
  caculateAngle();
  if (int(millis() - print_timer) > 500)
  {
    displayData();
    print_timer = millis();
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
