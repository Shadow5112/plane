#include<SD.h>
#include<SPI.h>
#include <Wire.h>
#include <Servo.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ, gyro_cal_x, gyro_cal_y, gyro_cal_z;
float rotX, rotY, rotZ;

float vel;

float pitch_angle, roll_angle, yaw_angle, acc_vector;
unsigned long print_timer = 0;
unsigned long print_timer_2 = 0;
unsigned long gyro_timer = 0;
unsigned long accel_timer = 0;

int rudder_in; // assign pin number for rudder
int rudder_out;


int throttle_in; // assign pin number for throttle
int throttle_out;

int auto_pilot_pin; // assign pin nuber for auto pilot inturupt function - must be digital pin
int flip_pin;
int barrel_roll_pin;
int land_pin;
int takeoff_pin = 10;

//declare servo variables
Servo  rudder_servo,
       throttle_servo,
       aileron_l_servo,
       aileron_r_servo,
       elevator_l_servo,
       elevator_r_servo;

// declare initial possition of each servo in degrees

int rudder_start;
int throttle_start;
int aileron_l_start;
int aileron_r_start;
int elevator_r_start;
int elevator_l_start;
//declare maximum and minimum servo angles in degrees


int rudder_max;
int rudder_min;

int throttle_max;
int throttle_min;


// declare all nesicary functions
void manual(); //declare inturupt function

//declare auto pilot functions
void barrel_roll();
void land();
void takeoff();
void flip(); //backwards loop

float altitude(); // returns altitude with ultrasonc sensor. Range = 4.5 meters

void stabalize();

int num_files = 3; // number of files that will be created
String file_1 = ("AccelData.txt");
String file_2 = ("GyroData.txt");
String file_3 = ("Orientaion.txt");

int chipSelect = 4; //Set ChipSelect to pin 4
File accel_data; // Variable for file data
File gyro_data;
File orientation_data;

void setupMPU();
void config_MPU();
void recordAccelData();
void convertAccelData();
void recordGyroData();
void convertGyroData();
void caculateAngle();
void displayGyroAccel();
void writeData(int);
void update_velocity();

void setup()
{
  Serial.begin(9600);
  /*
  Serial.print("Roll");
  Serial.print(",");
  Serial.print("Pitch");
  Serial.print(",");
  Serial.print("GyroX");
  Serial.print(",");
  Serial.print("GyroY");
  Serial.print(",");
  Serial.print("GyroZ");
  Serial.print(",");
  Serial.print("AccelX");
  Serial.print(",");
  Serial.print("AccelY");
  Serial.print(",");
  Serial.println("AccelZ");
  */
  Wire.begin();
  setupMPU();
  config_MPU();

  pinMode(10, OUTPUT); // Pin 10 initialized to Output
  SD.begin(chipSelect); // Initialize the SD with ChipSelect
  for (int i; i < 1000; i++)
  {
    recordGyroData();
    convertGyroData();

    gyro_cal_x += gyroX;
    gyro_cal_y += gyroY;
    gyro_cal_z += gyroZ;
    delay(10);

  }

  gyro_cal_x /= 1000;
  gyro_cal_y /= 1000;
  gyro_cal_z /= 1000;

}

void loop()
{

  recordGyroData();
  gyroX -= gyro_cal_x;
  gyroY -= gyro_cal_y;
  gyroZ -= gyro_cal_z;
  convertGyroData();
  recordAccelData();
  caculateAngle();
  convertAccelData();
  displayGyroAccel();
  update_velocity();

  if (int(millis() - print_timer) > 100)
  {
    for (int x = 1; x <= num_files; x++) {
      writeData(x);
    }

    print_timer = millis();
  }

}

void update_velocity() {


  //vel += ;
}

void setupMPU()
{
  Wire.beginTransmission(0b1101000); //Access the I2C address for the MPU
  Wire.write(0x6B); //Accessing Power Management Register
  Wire.write(0b00000000); // Turning off sleep setting
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000); // Access the I2C address of the MPU
  Wire.write(0x1B); //Accessing register 1B ... Gyro Config
  Wire.write(0x00000000); //Setting the gyro +/- 250deg/s
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000); //Access the I2C address of the MPU
  Wire.write(0x1C); //Accessing the 1C ...  Accel Config
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission();

}

void config_MPU()
{
  Wire.beginTransmission(0b1101000);// Access the I2C address for the MPU
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 12);
  while (Wire.available() < 12)
  {
    accelX = Wire.read() << 8 | Wire.read();
    accelY = Wire.read() << 8 | Wire.read();
    accelZ = Wire.read() << 8 | Wire.read();
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
  }
}

void recordAccelData()
{

  Wire.beginTransmission(0b1101000); // Access the I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Data Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request 6 Accel registers... 6 registers needed because the accel stores 2 bytes of data for each of the 3 direction measured

  while (Wire.available() < 6); // Allows 6 bytes of Data to be stored

  accelX = Wire.read() << 8 | Wire.read(); //Store data for accelX
  accelY = Wire.read() << 8 | Wire.read(); //Store data for accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store data for accelZ

}

void convertAccelData()
{
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; // Converts raw Accel Data into G-Froce
  gForceZ = accelZ / 16384.0;
}

void recordGyroData()
{

  Wire.beginTransmission(0b1101000); // Access the I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Data Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request 6 Gyro registers

  while (Wire.available() < 6);

  gyroX = Wire.read() << 8 | Wire.read(); //Store data for gyroX
  gyroY = Wire.read() << 8 | Wire.read(); //Store data for gyroY
  gyroZ = Wire.read() << 8 | Wire.read(); //Store data for gyroZ

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

  unsigned long sample_time = millis() - gyro_timer; // calculate time since angle was last calculated in ms
  float delta_pitch = rotX * sample_time / 1000; // convert to ms to s and find angle moved sincce last measument
  float delta_roll = rotY * sample_time / 1000;
  float delta_yaw = rotZ * sample_time / 1000;

  roll_angle += delta_roll; // add change in angle to total angle
  pitch_angle += delta_pitch;
  yaw_angle += delta_yaw;

  pitch_angle += roll_angle * sin(yaw_angle * 0.000001066);
  roll_angle -= pitch_angle * sin(yaw_angle * 0.000001066);
  
  roll_angle = roll_angle * 0.96 + acc_roll_angle * 0.04; // buffer gyroscope with acccerometer data
  pitch_angle = pitch_angle * 0.94 + acc_roll_angle * 0.04;


  gyro_timer = millis();
}

void displayGyroAccel()
{
  if (int(millis()) - print_timer_2 >= 500) {
    Serial.print(roll_angle);
    Serial.print(",");
    Serial.print(pitch_angle);
    Serial.print(",");
    Serial.print(rotX);
    Serial.print(",");
    Serial.print(rotY);
    Serial.print(",");
    Serial.print(rotZ);
    Serial.print(",");
    Serial.print(gForceX);
    Serial.print(",");
    Serial.print(gForceY);
    Serial.print(",");
    Serial.println(gForceZ);
    print_timer_2 = millis();
  }
}

void writeData(int file_name) {
  switch (file_name) {
    case 1:
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
        accel_data.close();
      }
      break;

    case 2:
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
        gyro_data.close();
      }
      break;

    case 3:
      orientation_data = SD.open(file_3, FILE_WRITE);
      if (orientation_data) {
        orientation_data.print(roll_angle);
        orientation_data.print(",");
        orientation_data.print(pitch_angle);
        orientation_data.print(",");
        orientation_data.print(millis());
        orientation_data.println(",");
        orientation_data.close();
      }
      break;

    default:
      break;

  }
}
