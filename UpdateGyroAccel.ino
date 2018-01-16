
#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

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
  convertAccelData();
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
  
  convertGyroData();

}

void convertGyroData() 
{
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; // Conversion Factor given by data sheet for the 250deg/s setting
  rotZ = gyroZ / 131.0; // Converts for Raw data to Deg/s 
}

void displayGyroAccel() 
{
  Serial.print("Gyro (deg/sec)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (gForce)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
}

void loop() 
{
  recordAccelData();
  recordGyroData();
  displayGyroAccel();
  delay(1000);

}
