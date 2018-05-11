#include <ArduinoSTL.h>

#include<SD.h>
#include<SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <vector>

#include <LSM303.h>// for compass and accelerometer
#include <LPS.h>// for pressure sensor
#include <L3G.h>// for gyro

L3G gyro;
LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32768, -32768, -32768};
LPS ps;

using namespace std;

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
/*******************************************************************
                          data Variables
******************************************************************
*/
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
std::vector<int16_t> g2 = {1, 1, 1};
vector<float> prev = {0, 0, 0};

long gyroX, gyroY, gyroZ;
unsigned long gyro_cal_x, gyro_cal_y, gyro_cal_z;
float gyro_offset_x, gyro_offset_y, gyro_offset_z;
float rotX, rotY, rotZ;
float alt, alt_offset;

float vel;

float pitch_angle, roll_angle, yaw_angle, acc_vector;
float acc_pitch, acc_roll, acc_vec;
unsigned long lag = 0;

unsigned long print_timer = 0;
unsigned long display_timer = 0;
unsigned long print_timer_2 = 0;
unsigned long gyro_timer = 0;

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

/************************************************************
                                PINS
*************************************************************
*/

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


int trig_pin =38;
int echo_pin = 36;

/****************************************************
                INITIAL VAlUES
*****************************************************/

// declare initial possition of each servo in degrees

int rudder_start;
int throttle_start;
int aileron_start;
int elevator_start;
//declare maximum and minimum servo angles in degrees

int aileron_servo_max = 150; //max and min angle for all servos
int aileron_servo_min = 30;
int elevator_servo_max = 150; //max and min angle for all servos
int elevator_servo_min = 30;
int throttle_servo_max = 150; //max and min angle for all servos
int throttle_servo_min = 30;



/*********************************************************
                FUNCTIONS
**********************************************************/

// setup functions
void setupMPU();
void config_MPU();
void calibrateAccel();
void calilbrateAlt();

//data functions
void caculateAngle();
void displayGyroAccel();
void writeData();
void update_velocity();
void displayData();
float readAlt_Br();
float readAlt_US();

//auto pilot functions
void barrel_roll();
void land();
void takeoff();
void flip(); //backwards loop
void balance();
void manuel();
void crash();


void setup()
{

  Serial.begin(9600);
  Wire.begin();

  pinMode(10, OUTPUT); // Pin 10 initialized to Output
  SD.begin(chipSelect); // Initialize the SD with ChipSelect

  /***************************
    Setup AltIMU
  ****************************/

  if (!gyro.init()) {
    Serial.println("Failed  to autodetect gyro type");
  }
  else{
      gyro.enableDefault();
  //gyro.writeReg(L3G::CTRL1, 0b010101111); //set data output rate to 400 Hz // i think
  // gyro.writeReg(L3G::CTRL2, 0b00100000);
  }

  if (!compass.init()) {
    Serial.println("failed to detect compass");
  }
  else{
      compass.enableDefault();
  }

  if (!ps.init()) {
    Serial.println("failed to detect presure sensor");
  }
  else{
      ps.enableDefault();
      calibrateAlt();
  }



  /*****************************
    calibrate gyroscope
  *****************************/
  if(gyro.init()){
  for (int i = 1; i <= 1000; i++)
  {

    gyro.read();
    gyro_cal_x += gyro.g.x;
    gyro_cal_y += gyro.g.y;
    gyro_cal_z += gyro.g.z;
    delay(10);

    Serial.print(i);
    Serial.print(":  ");
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
  }

  /*********************
    initialise pins
  *********************/
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
   
  pinMode(trig_pin, OUTPUT);// inisialize pins fro ultra sonic sensor
  pinMode(echo_pin, INPUT); 

  /*******************
    setup servos
  ********************/
  rudder_servo.attach(rudder_out); //attach servo objects to pins
  aileron_servo.attach(aileron_out);
  elevator_servo.attach(elevator_out);
  throttle_servo.attach(throttle_out);

  rudder_servo.write(rudder_start); // set servos to start values
  aileron_servo.write(aileron_start);
  elevator_servo.write(elevator_start);
  throttle_servo.write(throttle_start);


  Serial.println("Setup Complete");

}

void loop()
{

  caculateAngle();

  if ((millis() - display_timer)  > 150) {
    displayData();
    display_timer = millis();
  }


  if (int(millis() - print_timer) > 500)
  {
    writeData();
    print_timer = millis();
  }


  if (auto_pin == HIGH) {
    if (flip_pin == HIGH) {
      flip();
    }
    else if (takeoff_pin == HIGH) {
      takeoff();
    }
    else if (barrel_roll_pin == HIGH) {
      barrel_roll();
    }
    else if (land_pin == HIGH) {
      land();
    }
    else {
      balance();
    }
  }
  else {
    manuel();
  }
}




//use gyro and acclerometer and commpass data to calculate orientation
void caculateAngle()
{
  bool same = false;
  gyro.read();

  /**********************
    check for new data
   ***********************/
  int x = gyro.g.x; // read values from gyro
  int y = gyro.g.y;
  int z = gyro.g.z;
  //check if there is new data

  if ( x == g2[0] and y == g2[1] and z == g2[2]) { // compare with previous gyro values
    same = true;
  }
  else {
    same = false; // if there is new data then record new data
    g2[0] = x;
    g2[1] = y;
    g2[2] = z;
  }
  /******************
   ******************/


  if (!same) {

    /*****************
      orientation tracking with gyro
     ******************/
    float conv_factor_g = .00875; // Conversion Factor given by data sheet for the 250deg/s setting
    rotX = float(gyro.g.x - gyro_offset_x) * conv_factor_g;// Converts for Raw data to Deg/s
    rotY = float(gyro.g.y - gyro_offset_y) * conv_factor_g;
    rotZ = float(gyro.g.z - gyro_offset_z) * conv_factor_g;

    float gyro_rate = 189.4;// some convertion factor that is supposed to be based on the sample rate

    //int sample_time = micros() - gyro_timer; // calculate time since angle was last calculated in ms

    //gyro_rate =   1.0 /(float(sample_time) /1000000) ;
    float delta_pitch = rotY / gyro_rate; // divite by gyro sample rate to find angle moved sincce last measument
    float delta_roll = rotX / gyro_rate;  // gyro_rate  declared as local variable
    float delta_yaw = rotZ / gyro_rate;

    roll_angle += delta_roll; // add change in angle to total angle
    pitch_angle -= delta_pitch;
    yaw_angle -= delta_yaw;



    /******************************************
      orientation tracking with accelerometer
     ******************************************/
    compass.read();

    float conv_factor_a = 0.000061;
    gForceX = float(compass.a.x) * conv_factor_a ;
    gForceY = float(compass.a.y) * conv_factor_a ; // Converts raw Accel Data into G-Froce
    gForceZ = float(compass.a.z) * conv_factor_a ;

    acc_vector = sqrt ((gForceX * gForceX) + (gForceY * gForceY) + (gForceZ * gForceZ)); // Calc Accel vector
    acc_vec = acc_vector;

    float acc_roll_angle =  asin(gForceY / acc_vector) * (180.0 / 3.14); // Calc roll angle range +- 90 degrees
    float  acc_pitch_angle = asin(gForceX / acc_vector) * (180.0 / 3.14); // Calc pitch angle
    /****************************************************************
      restrict pitch, yaw, and roll angle as measured by gryo to 0-360
    ****************************************************************/
    if (roll_angle > 360) {
      roll_angle -= 360.0;
    }
    if (roll_angle < 0) {
      roll_angle += 360.0;
    }

    if (pitch_angle > 360) {
      pitch_angle -= 360.0;
    }
    if (pitch_angle < 0) {
      pitch_angle += 360.0;
    }

    if (yaw_angle > 360) {
      yaw_angle -= 360.0;
    }
    if (yaw_angle < 0) {
      yaw_angle += 360.0;
    }
    // reset roll as pllane passes 0 or 180 degrees if plane is not pointing strait up or down
    if (gForceY <= 0.015 and gForceY >= -0.015 and gForceX < 0.9 and gForceX > -0.9 and gForceZ > 0) {
      roll_angle = 0;
    }
    if (gForceY <= 0.015 and gForceY >= -0.015 and gForceX < 0.9 and gForceX > -0.9 and gForceZ < 0) {
      roll_angle = 180;
    }
    if (gForceX <= 0.015 and gForceX >= -0.015 and gForceZ > 0) {
      pitch_angle = 0;
    }
    /******************
    ******************/

    //pitch_angle += roll_angle * sin(yaw_angle * 0.000000533);
    //roll_angle -= pitch_angle * sin(yaw_angle * 0.000001066);

    /******************************************************
      correct acc_roll-angle to range of 0 - 360 degrees
    ******************************************************/
    if (gForceY >= 0 and gForceZ <= 0) { // quadrant 1
      acc_roll_angle = 90.0 + (90.0 - acc_roll_angle);
    }
    if (gForceY < 0 and gForceZ < 0) { // quardant 4
      acc_roll_angle = 90 + (90.0 - acc_roll_angle);
    }
    if (gForceY < 0 and gForceZ > 0) { //quadrant 3
      acc_roll_angle = 270 + (90 + acc_roll_angle);
    }


    /******************************************************
      correct acc_pitch_angle to range of 0 - 360 degrees
    *******************************************************/
    if (gForceX > 0 and gForceZ < 0) { //quadrant 2
      acc_pitch_angle = 90.0 + (90.0 - acc_pitch_angle);
    }

    if (gForceX < 0 and gForceZ < 0) {
      acc_pitch_angle = 90.0 + (90 - acc_pitch_angle);
    }
    if (gForceX < 0 and gForceZ > 0) {
      acc_pitch_angle = 270 + (90 + acc_pitch_angle);
    }


    /******************************
      final orientation calculation
     ********************************/
    //when plane is horizontal yaw ~= compass heading
    if (roll_angle >= -5 and roll_angle <= 5) {
      yaw_angle = compass.heading();
    }


    if (first_data) {
      roll_angle = acc_roll_angle;
      pitch_angle = acc_pitch_angle;
      first_data = false;
      prev = { roll_angle, pitch_angle, yaw_angle};
    }
    else {
      roll_angle = roll_angle * 0.9996 + acc_roll_angle * 0.0004; // buffer gyroscope with acccerometer data
      pitch_angle = pitch_angle * 0.9996 + acc_pitch_angle * 0.0004;
      prev = { roll_angle, pitch_angle, yaw_angle};
    }





    /*******************************
      check for NaN values in data
     ********************************/
    if (!(roll_angle == roll_angle) or !(pitch_angle == pitch_angle) or !(yaw_angle == yaw_angle))  //check for NaN
    {
      roll_angle = prev[0];
      pitch_angle = prev[1];
      yaw_angle = prev[2];
    }

    /**********************
       values for displayData()
     **********************/
    acc_roll = acc_roll_angle;
    acc_pitch = acc_pitch_angle;
    lag = micros();
  }


}


//display data on serial monitor
//when testing ensure that lag due to serial output does not exceed the output rate of the IMU
//for 250Hz lag must stay under ~4ms
void displayData() {

  /***************************
    servo values
   ***************************
    Serial.print("aileron servo: ");
    Serial.print(aileron_servo.read());
    Serial.print(" elevator servo: ");
    Serial.print(elevator_servo.read());
    Serial.print(" rudder servo: ");
    Serial.print(rudder_servo.read());
    Serial.print(" throttle_servo: ");
    Serial.print(throttle_servo.read());
  */

  /**************************
    final pitch roll and yaw
   **************************/
    Serial.print("roll: ");
    Serial.print(roll_angle);
    Serial.print(" pitch: ");
    Serial.print(pitch_angle);
    Serial.print(" yaw: ");
    Serial.print(yaw_angle);


  /************************
    heading and Gforce values
   *************************
    Serial.print(" heading: ");
    Serial.print(compass.heading());
    Serial.print(" gx: ");
    Serial.print(gForceX);
    Serial.print(" gy: ");
    Serial.print(gForceY);
    Serial.print(" gz: ");
    Serial.print(gForceZ);
  */


  /**************************
    accelerometer pitch and roll
   **************************
  Serial.print(" accel_roll: ");
  Serial.print(acc_roll);
  Serial.print(" accle_pitch: ");
  Serial.print(acc_pitch);
  Serial.print(" acc vect: ");
  Serial.print(acc_vec);
*/


  Serial.print( " lag: ");
  Serial.println(micros() - lag);
}

//write data to SD card
//gyro, accel, orientation, and servo data writen to seperate files
void writeData() {


  /************************************
    accelerometer Gforce data
   ************************************/

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

  /*****************************
    gyro deg/s data
   *****************************/
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

  /***************************************************
    orientation data: roll, pitch, yaw, time stamp(ms)
   *****************************************************/
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
    orientation_data.print(yaw_angle);
    orientation_data.print(",");
    orientation_data.print(millis());
    orientation_data.println(",");
  }
  orientation_data.close();
  /**************************************
    servo angles
   * *****************************************
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
  */
  first_print = false;
}

//keeps the plane flying horizontal
void balance()
{
  int y;
  int x;
  if (roll_angle > 5)
  {
    y = 1 + aileron_servo.read();
    x = checkAngle(y);
    aileron_servo.write(x);
  }
  else if (roll_angle < 5)
  {
    y = aileron_servo.read() - 1;
    x = checkAngle(y);
    aileron_servo.write(x);
  }
  else {
    aileron_servo.write(aileron_start);
  }
}

//checks if servo angle exceeds max or min angle
int checkAngle(int x)
{
  if (x < aileron_servo_min)
  {
    return aileron_servo_min;
  }

  else if (x > aileron_servo_max)
  {
    return aileron_servo_max;
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

//returns  altutude in cm with ~3mm precision
//range - 400cm
float readAlt_US(){
  long duration;
  float m, true_alt;
  float roll = roll_angle;
  float pitch = pitch_angle;

  // restric pitch and roll 
  
  if(roll > 180){
    roll -= 360;
  }
  if(pitch > 180){
    pitch -= 360;
  }
  
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);  

  pinMode(echo_pin, INPUT);
  duration = pulseIn(echo_pin, HIGH);

  m = duration*.0343/2;
  
  //calculate true altitude acounting for pitch and roll
  true_alt = m * sqrt(1 - cos(90-roll)*cos(90-roll) - cos(90-pitch)*cos(90-pitch)); 
  
  if( m < 400 ){
  return true_alt;
  }
  else{
    return -1;
  }
}
//returns altitude +- 1m
// 0 is altitude where board was initialised
float readAlt_Br() {
  alt = ps.pressureToAltitudeMeters(ps.readPressureMillibars()) - alt_offset ;
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

void takeoff() {
  if (alt  < 6) {
    if (throttle_servo.read() < throttle_servo_max) {
      throttle_servo.write(throttle_servo_max);
    }
    if (pitch_angle < 30) {
      elevator_servo.write(elevator_servo.read() + 1);
    }
    if (pitch_angle > 30) {
      elevator_servo.write(elevator_start);
    }
  }
}

void land() {
  crash();
}

void crash() {
  if (throttle_servo.read() < throttle_servo_max) {
    throttle_servo.write(throttle_servo_max);
  }
  if (pitch_angle < 250) {
    elevator_servo.write(elevator_servo_max);
  }
  else if (pitch_angle > 280) {
    elevator_servo.write(elevator_servo_min);
  }
  else
  {
    elevator_servo.write(elevator_start);
  }

}


