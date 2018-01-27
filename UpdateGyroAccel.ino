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
  gyroZ = Wire.read()<<8|Wire.read();  
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
    Serial.println("File opened sucessfully");
    MPUdata.print(gForceX);
    MPUdata.print(",");
    MPUdata.print(gForceY);
    MPUdata.print(",");
    MPUdata.println(gForceZ);
    MPUdata.close();  
    delay(500);
  } 
  
}
