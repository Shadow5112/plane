#include<SD.h>; 
#include<SPI.h>;
#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
 
long gyroX, gyroY, gyroZ,gyro_cal_x,gyro_cal_y,gyro_cal_z;
float rotX, rotY, rotZ;

float pitch_angle,roll_angle,acc_vector;
int startTime = 0;

int chipSelect = 4; //Set ChipSelect to pin 4
File MPUdata; // Variable for file data

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
  Wire.requestFrom(0b1101000,12);
  while(Wire.available() < 12)
  {
  accelX = Wire.read()<<8|Wire.read();                                  
  accelY = Wire.read()<<8|Wire.read();                                  
  accelZ = Wire.read()<<8|Wire.read();                                                                     
  gyroX = Wire.read()<<8|Wire.read();                                 
  gyroY = Wire.read()<<8|Wire.read();                                 
  gyroZ = Wire.read()<<8|Wire.read();  #include<SD.h>;
#include<SPI.h>;
#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ, gyro_cal_x, gyro_cal_y, gyro_cal_z;
float rotX, rotY, rotZ;

float pitch_angle, roll_angle, acc_vector;
unsigned long startTime = 0;

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

void setup()
{
  Serial.begin(9600);
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
  Wire.begin();
  setupMPU();
  config_MPU();

  pinMode(10, OUTPUT); // Pin 10 initialized to Output
  SD.begin(chipSelect); // Initialize the SD with ChipSelect
  for (int i; i < 1000; i++)
  {
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

  if (int(millis() - startTime) > 100)
  {
    for (int x = 1; x <= 3; x++) {
      writeData(x);
    }

    startTime = millis();
  }

}

void writeData(int file_name) {
  switch (file_name) {
    case 1:
      accel_data = SD.open(file_1, FILE_WRITE);
      if(accel_data){
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
      if(gyro_data){
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
      if(orientation_data){
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

  gyroX = Wire.read() << 8 | Wire.read(); //Store data for accelX
  gyroY = Wire.read() << 8 | Wire.read(); //Store data for accelY
  gyroZ = Wire.read() << 8 | Wire.read(); //Store data for accelZ

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
  roll_angle = asin(accelX / acc_vector) * (180.0 / 3.14); // Calc roll angle
  pitch_angle = asin(accelY / acc_vector) * (180.0 / 3.14); // Calc pitch angle
}

void displayGyroAccel()
{
  if (int(millis()) - startTime >= 500) {
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
    startTime = micros();
  }
}

  }  
}

void recordAccelData() 
{

  Wire.beginTransmission(0b1101000); // Access the I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Data Readings 
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request 6 Accel registers... 6 registers needed because the accel stores 2 bytes of data for each of the 3 direction measured
  
  while(Wire.available() < 6); // Allows 6 bytes of Data to be stored 

  accelX = Wire.read()<<8|Wire.read(); //Store data for accelX
  accelY = Wire.read()<<8|Wire.read(); //Store data for accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store data for accelZ
 
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
  Wire.requestFrom(0b1101000,6); //Request 6 Gyro registers 

  while(Wire.available() < 6);

  gyroX = Wire.read()<<8|Wire.read(); //Store data for accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store data for accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store data for accelZ
  
}

void convertGyroData() 
{
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; // Conversion Factor given by data sheet for the 250deg/s setting
  rotZ = gyroZ / 131.0; // Converts for Raw data to Deg/s 
}

void caculateAngle()
{
  acc_vector = sqrt ((accelX*accelX) + (accelY*accelY) + (accelZ*accelZ)); // Calc Accel vector 
  roll_angle = asin(accelX/acc_vector)*(180.0/3.14);// Calc roll angle
  pitch_angle = asin(accelY/acc_vector)*(180.0/3.14);// Calc pitch angle
}

void displayGyroAccel() 
{
 if(int(millis()) -startTime >= 500){
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
  startTime = micros();
 }
}

void setup() 
{
  Serial.begin(9600);
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
  Wire.begin();
  setupMPU();
  config_MPU();
   
  pinMode(10,OUTPUT); // Pin 10 initialized to Output
  SD.begin(chipSelect); // Initialize the SD with ChipSelect
  for(int i; i<1000; i++)
  {
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
  
  MPUdata = SD.open("MPUdata.txt",FILE_WRITE);// Create a Write Only file on SD and open it.
  if(MPUdata) // Only do if data file opens sucessfully 
{
if (int(millis() - startTime) > 100)
    {
      Serial.println("File opened sucessfully");
      MPUdata.print(roll_angle);
      MPUdata.print(",");
      MPUdata.print(pitch_angle);
      MPUdata.print(",");
      MPUdata.print(gForceX);
      MPUdata.print(",");
      MPUdata.print(gForceY);
      MPUdata.print(",");
      MPUdata.print(gForceZ);
      MPUdata.print(",");
      MPUdata.print(rotX);
      MPUdata.print(",");
      MPUdata.print(rotY);
      MPUdata.print(",");
      MPUdata.print(rotZ);
      MPUdata.print(",");
      MPUdata.println(millis());
      MPUdata.close();
     startTime = millis();
    }
  } 
  
}
