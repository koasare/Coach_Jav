#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM9DS0.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter



//set up serial port for GPS
SoftwareSerial mySerial(3,2);
SoftwareSerial bluetooth(5,4); //bluetooth tx to d5, rx to d4
//initialize GPS module
Adafruit_GPS GPS(&mySerial);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
     
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double totalAccel;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// Final output values
double flastAngle;
double flastSpeed;
int ilastAngle = 0;
int ilastSpeed = 0;

// TODO: Make calibration routine

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
  

   Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x03; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  
  Serial.println("");
//  Serial.println("Adafruit GPS library basic test!");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); // 5 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
uint32_t timer1 = millis();

void loop(void) 
{  
 /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  totalAccel = sqrt(accX*accX + accY*accY + accZ*accZ);
//  Serial.print(totalAccel); Serial.print(',');
//  if (totalAccel > 40000) {
//    Serial.print(totalAccel); Serial.print(',');
//    Serial.print(accX); Serial.print(',');
//    Serial.print(accY); Serial.print(',');
//    Serial.print(accZ); Serial.print(',');
//    Serial.println("");
//  }
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

 // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

 #ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


//Serial.print(kalAngleX); Serial.print(",");
//
//  Serial.print(roll); Serial.print(",");
//  Serial.print(gyroXangle); Serial.print(",");
//  Serial.print(compAngleX); Serial.print(",");
//  Serial.print(kalAngleX); Serial.print(",");
////
////  Serial.print("\t");
////
//  Serial.print(pitch); Serial.print(",");
//  Serial.print(gyroYangle); Serial.print(",");
//  Serial.print(compAngleY); Serial.print(",");

//#if 0 // Set to 1 to print the temperature
//  Serial.print("\t");
//
//  double temperature = (double)tempRaw / 340.0 + 36.53;
//  Serial.print(temperature); Serial.print(",");
//#endif

// read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
//  if (GPSECHO)
//    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
//    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer1 > millis()) timer1 = millis();

  // must wait 0.1 sec b/c gps spits out data at 10 Hz
  if (millis() - timer1 > 100) {
//    Serial.print(millis() - timer1); Serial.print(',');
    timer1 = millis(); // reset the timer
//    Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
//    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//      Serial.print(", ");
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
//    }
    
//    //Serial.print("Fix: ");
//    Serial.print((int)GPS.fix); Serial.print(',');
//    //Serial.print(" quality: ");
//    Serial.print((int)GPS.fixquality); Serial.print(',');
//    //Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.print(GPS.milliseconds); Serial.print(',');
//    //Serial.print("Date: ");
//    Serial.print(GPS.month, DEC); Serial.print('/');
//    Serial.print(GPS.day, DEC); Serial.print("/20");
//    Serial.print(GPS.year, DEC); Serial.print(",");
//    //if (GPS.fix) {
//      //Serial.print("Speed (knots): ");
//      Serial.print(GPS.speed); Serial.print(",");
//      double meterPerSecond = GPS.speed*0.514444; //convert knots to m/s
//      Serial.print(meterPerSecond); Serial.print(",");
//      double mph = GPS.speed*1.15078; //convert knots to mph
//      Serial.print(mph); Serial.print(",");
//      //Serial.print("Location: ");
////      Serial.print(GPS.latitude, 4);
////      Serial.print(GPS.lat);
////      Serial.print(","); 
////      Serial.print(GPS.longitude, 4);
////      Serial.print(GPS.lon); Serial.print(',');
//      //Serial.print("Location (in degrees, works with Google Maps): ");
//      Serial.print(GPS.latitudeDegrees, 4);
//      Serial.print(","); 
//      Serial.print(GPS.longitudeDegrees, 4); Serial.print(',');
//
//
//
//      //Serial.print("Angle: ");
//      Serial.print(GPS.angle); Serial.print(',');
//      //Serial.print("Altitude: ");
//      Serial.print(GPS.altitude); Serial.print(',');
//      //Serial.print("Satellites: ");
//      Serial.print((int)GPS.satellites); Serial.print(',');
//      Serial.print(millis() - timer1); Serial.print(',');
  }

  double meterPerSecond = GPS.speed*0.514444; //convert knots to m/s
  
  //multiply floats by 1000 to get all necessary numbers
  flastAngle = (float)kalAngleY;
  flastSpeed = (float)meterPerSecond;

  //convert to integers
  ilastAngle = (int)flastAngle;
  ilastSpeed = (int)flastSpeed;

  char lastAngleStr[5] = {0,0,0,0,0}; //char angle assigment
  char lastSpeedStr[5] = {0,0,0,0,0}; //char speed assignment

  // convert angle * 1000 and speed * 1000 to strings
  sprintf(lastAngleStr, "%d", ilastAngle);
  sprintf(lastSpeedStr, "%d", ilastSpeed);

  //convert to proper integer type string number
//  Serial.print('#');Serial.print(lastAngleStr[0]);Serial.print(lastAngleStr[1]);Serial.print(lastAngleStr[2]);Serial.print(",");
//  Serial.print(lastSpeedStr[0]);Serial.print(lastSpeedStr[1]);Serial.print(lastSpeedStr[2]);Serial.print(",");Serial.print('~');
//  Serial.println("");

//   bluetooth.print('#');bluetooth.print(lastAngleStr[0]);bluetooth.print(lastAngleStr[1]);bluetooth.print(lastAngleStr[2]);bluetooth.print(",");
//   bluetooth.print(lastSpeedStr[0]);bluetooth.print(lastSpeedStr[1]);bluetooth.print(lastSpeedStr[2]);bluetooth.print(",");bluetooth.print('~');
//   bluetooth.println("");

   delay(250);


  if (totalAccel > 20000) 
  {
    delay(10);
    Serial.print('#');Serial.print(lastAngleStr[0]);Serial.print(lastAngleStr[1]);Serial.print(lastAngleStr[2]);Serial.print(",");
    Serial.print(lastSpeedStr[0]);Serial.print(lastSpeedStr[1]);Serial.print(lastSpeedStr[2]);Serial.print(",");Serial.print('~');
    Serial.println("");
    
    bluetooth.print('#');bluetooth.print(lastAngleStr[0]);bluetooth.print(lastAngleStr[1]);bluetooth.print(lastAngleStr[2]);bluetooth.print(",");
    bluetooth.print(lastSpeedStr[0]);bluetooth.print(lastSpeedStr[1]);bluetooth.print(lastSpeedStr[2]);bluetooth.print(",");bluetooth.print('~');
    bluetooth.println("");
    delay(10000);

    while (Serial.available())
      Serial.read();

    while (bluetooth.available())
      bluetooth.read();
  }
  
  
  
  

//  Serial.print(kalAngleY);Serial.print(",");
//  double meterPerSecond = GPS.speed*0.514444; //convert knots to m/s
//  Serial.print(meterPerSecond);
//  Serial.println("");
//  bluetooth.print(kalAngleY); bluetooth.print(",");
//  bluetooth.print(meterPerSecond); bluetooth.print(",");
//  bluetooth.println("");


  
//  if (totalAccel > 40000) {
//    Serial.print(kalAngleY); Serial.print(",");
//    double meterPerSecond = GPS.speed*0.514444; //convert knots to m/s
//    Serial.print(meterPerSecond); Serial.print(",");
//    double mph = GPS.speed*1.15078; //convert knots to mph
//    Serial.print(mph); Serial.print(",");
//    Serial.println("");
//    delay(60000);
//  }

//  Serial.println("");
}
