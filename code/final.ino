// AUTHOR: Siyuan Shi
// Arduino sketch for the gesture detection unlock
// board: SAMD21 Xplained Pro, IMU: MPU6050.
// Part of the sketch is taken from an example made by Federico Terzi, Kristian Lauszus, TKJ Electronics

#include <Wire.h>
#include <stdlib.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

// Pins number used for I/O
int btnPin1 = 3;
int LEDPin = 2;

// count number of times the button is pressed
int cnt = 0;

// record sequence length
int SeqLen = 0;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// Variables that will store temp serial sensor data
int16_t S_AcX[100];
int16_t S_AcY[100];
int16_t S_AcZ[100];
int16_t S_GyX[100];
int16_t S_GyY[100];
int16_t S_GyZ[100];
int id = 0;

// Variables that will store record serial sensor data
int16_t Rcd_AcX[100];
int16_t Rcd_AcY[100];
int16_t Rcd_AcZ[100];
int16_t Rcd_GyX[100];
int16_t Rcd_GyY[100];
int16_t Rcd_GyZ[100];

// DTW matrix
int16_t cost[100][100];

// DTW matrix initial variables
int idx = 0;
int a = 0;
int b = 0;
int m = 0;
int PathLen = 0;

// reconstruction path
int16_t res[200];

// detection variables
int varAcX = 0;
int varAcY = 0;
int varAcZ = 0;
int varGyX = 0;
int varGyY = 0;
int varGyZ = 0;
int mean = 0;

int temvar = 0;

// threshold
int maxvarAcX = 0;
int maxvarAcY = 0;
int maxvarAcZ = 0;
int maxvarGyX = 0;
int maxvarGyY = 0;
int maxvarGyZ = 0;

// sequence used to calculate threshold
int16_t Com_AcX[100];
int16_t Com_AcY[100];
int16_t Com_AcZ[100];
int16_t Com_GyX[100];
int16_t Com_GyY[100];
int16_t Com_GyZ[100];

// declare functions
int16_t DTW(int16_t Rcd_xxx[], int16_t S_xxx[], int idx, int a, int b, int m, int PathLen, int var, int mean);
void resetdtw();
int Mean(int varxxxList[]);
int Var(int varxxxList[], int m);

// Status variables, used with buttons
int precBtn1 = HIGH;

void setup(){
  // Set the pin mode of the buttons using the internal pullup resistor
  pinMode(btnPin1, INPUT_PULLUP);
  pinMode(LEDPin, OUTPUT);

  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
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
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

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

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  // Start the serial communication
  Serial.begin(38400);
}
void loop(){
  // turn the LED off by making the voltage LOW
  digitalWrite(LEDPin, HIGH);
  
  // Read the values of the buttons
  int resultBtn1 = digitalRead(btnPin1);

  // ON btn1 pressed, start the batch and light up the yellow LED
  if (precBtn1 == HIGH && resultBtn1 == LOW)
  {
    Serial.println("STARTING BATCH");
    digitalWrite(LEDPin, LOW);    // turn the LED on by making the voltage HIGH
    cnt += 1;
  }

  // If the btn1 is hold, reads the data from the sensor and sends it through the communication channel
  if (resultBtn1==LOW)
  {
    digitalWrite(LEDPin, LOW);   // turn the LED on (HIGH is the voltage level)

    /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
  
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
  
    /* Print Data */
    #if 1 // Set to 1 to activate
      Serial.print(accX); Serial.print("  ");
      Serial.print(accY); Serial.print("  ");
      Serial.print(accZ); Serial.print("  ");
    
      Serial.print(gyroX); Serial.print(" ");
      Serial.print(gyroY); Serial.print(" ");
      Serial.print(gyroZ); Serial.print(" ");
    
      Serial.print("\t");
    #endif
    
    #if 0 // Set to 1 to print kalman
      Serial.print(roll); Serial.print("  ");
      Serial.print(gyroXangle); Serial.print("  ");
      Serial.print(compAngleX); Serial.print("  ");
      Serial.print(kalAngleX); Serial.print(" ");
    
      Serial.print("\t");
    
      Serial.print(pitch); Serial.print(" ");
      Serial.print(gyroYangle); Serial.print("  ");
      Serial.print(compAngleY); Serial.print("  ");
      Serial.print(kalAngleY); Serial.print(" ");
    #endif
    
    #if 0 // Set to 1 to print the temperature
      Serial.print("\t");
    
      double temperature = (double)tempRaw / 340.0 + 36.53;
      Serial.print(temperature); Serial.print(" ");
    #endif
  
    Serial.print("\r\n");
    delay(2);
    
    // store data
    if (id < 100) // sequence length is at most 50
    {
      S_AcX[id] = accX;
      S_AcY[id] = accY;
      S_AcZ[id] = accZ;
      S_GyX[id] = gyroX;
      S_GyY[id] = gyroY;
      S_GyZ[id] = gyroZ;
      //Serial.println(id);
      id++;
    }
    //Serial.println("SAVED TO S temp");
  }

  // Closes the batch when the button is released
  if (precBtn1 == LOW && resultBtn1 == HIGH)
  {
    SeqLen = id;
    Serial.println("Total length: "); 
    Serial.println(SeqLen);
    id = 0; // reset id to 0
    Serial.println("CLOSING BATCH");
    digitalWrite(LEDPin, HIGH);    // turn the LED off by making the voltage LOW
    if (cnt <= 5) // record mode, record 5 times
    {
      if (cnt == 1) // base sequence
      {
        for(int j = 0; j < 100; j++)
        {
          Rcd_AcX[j] = S_AcX[j];
          Rcd_AcY[j] = S_AcY[j];
          Rcd_AcZ[j] = S_AcZ[j];
          Rcd_GyX[j] = S_GyX[j];
          Rcd_GyY[j] = S_GyY[j];
          Rcd_GyZ[j] = S_GyZ[j];
        }
        Serial.println(" SAVED TO RCD BASE ");
      }
      else // compare record sequence to base sequence, compute vars and store them
      {
        for(int j = 0; j < 100; j++)
        {
          Com_AcX[j] = S_AcX[j];
          Com_AcY[j] = S_AcY[j];
          Com_AcZ[j] = S_AcZ[j];
          Com_GyX[j] = S_GyX[j];
          Com_GyY[j] = S_GyY[j];
          Com_GyZ[j] = S_GyZ[j];
        }
        Serial.println(" SAVED TO COM ");
        
        // For AcX
        resetdtw();
        temvar = DTW(Rcd_AcX, Com_AcX, idx, a, b, m, PathLen, temvar, mean);
        Serial.println("varAcX: ");
        Serial.println(temvar);
        if (maxvarAcX < temvar)
        {
          maxvarAcX = temvar;
        }
        Serial.println("maxvarAcX: ");
        Serial.println(maxvarAcX);
  
        // For AcY
        resetdtw();
        temvar = DTW(Rcd_AcY, Com_AcY, idx, a, b, m, PathLen, temvar, mean);
        Serial.println("varAcY: ");
        Serial.println(temvar);
        if (maxvarAcY < temvar)
        {
          maxvarAcY = temvar;
        }
        Serial.println("maxvarAcY: ");
        Serial.println(maxvarAcY);
  
        // For AcZ
        resetdtw();
        temvar = DTW(Rcd_AcZ, Com_AcZ, idx, a, b, m, PathLen, temvar, mean);
        Serial.println("varAcZ: ");
        Serial.println(temvar);
        if (maxvarAcZ < temvar)
        {
          maxvarAcZ = temvar;
        }
        Serial.println("maxvarAcZ: ");
        Serial.println(maxvarAcZ);
  
        // Gyro
        // For GyX
        resetdtw();
        temvar = DTW(Rcd_GyX, Com_GyX, idx, a, b, m, PathLen, temvar, mean);
        Serial.println("varGyX: ");
        Serial.println(temvar);
        if (maxvarGyX < temvar)
        {
          maxvarGyX = temvar;
        }
        Serial.println("maxvarGyX: ");
        Serial.println(maxvarGyX);
        
        // For GyY
        resetdtw();
        temvar = DTW(Rcd_GyY, Com_GyY, idx, a, b, m, PathLen, temvar, mean);
        Serial.println("varGyY: ");
        Serial.println(temvar);
        if (maxvarGyY < temvar)
        {
          maxvarGyY = temvar;
        }
        Serial.println("maxvarGyY: ");
        Serial.println(maxvarGyY);
        
        // For GyZ
        resetdtw();
        temvar = DTW(Rcd_GyZ, Com_GyZ, idx, a, b, m, PathLen, temvar, mean);
        Serial.println("varGyZ: ");
        Serial.println(temvar);
        if (maxvarGyZ < temvar)
        {
          maxvarGyZ = temvar;
        }
        Serial.println("maxvarGyZ: ");
        Serial.println(maxvarGyZ);
      }
    }
    else // detection mode, use DTW algorithm
    {
      Serial.println(" DETECTING...... ");
      // For AcX
      resetdtw();
      varAcX = DTW(Rcd_AcX, S_AcX, idx, a, b, m, PathLen, varAcX, mean);
      Serial.println("varAcX: ");
      Serial.println(varAcX);

      // For AcY
      resetdtw();
      varAcY = DTW(Rcd_AcY, S_AcY, idx, a, b, m, PathLen, varAcY, mean);
      Serial.println("varAcY: ");
      Serial.println(varAcY);

      // For AcZ
      resetdtw();
      varAcZ = DTW(Rcd_AcZ, S_AcZ, idx, a, b, m, PathLen, varAcZ, mean);
      Serial.println("varAcZ: ");
      Serial.println(varAcZ);

      // Gyro
      // For GyX
      resetdtw();
      varGyX = DTW(Rcd_GyX, S_GyX, idx, a, b, m, PathLen, varGyX, mean);
      Serial.println("varGyX: ");
      Serial.println(varGyX);

      // For GyY
      resetdtw();
      varGyY = DTW(Rcd_GyY, S_GyY, idx, a, b, m, PathLen, varGyY, mean);
      Serial.println("varGyY: ");
      Serial.println(varGyY);

      // For GyZ
      resetdtw();
      varGyZ = DTW(Rcd_GyZ, S_GyZ, idx, a, b, m, PathLen, varGyZ, mean);
      Serial.println("varGyZ: ");
      Serial.println(varGyZ);

      if (valid(varAcX, varAcY, varAcZ, varGyX, varGyY, varGyZ, maxvarAcX, maxvarAcY, maxvarAcZ, maxvarGyX, maxvarGyY, maxvarGyZ))
      {
        Serial.println("MATCHED");
        digitalWrite(LEDPin, LOW);    // turn the LED on by making the voltage HIGH
        delay(1000);
        digitalWrite(LEDPin, HIGH);
        delay(1000);
        digitalWrite(LEDPin, LOW);    // turn the LED on by making the voltage HIGH
        delay(1000);
        digitalWrite(LEDPin, HIGH);
        delay(1000);
        digitalWrite(LEDPin, LOW);    // turn the LED on by making the voltage HIGH
        delay(1000);
        digitalWrite(LEDPin, HIGH);
        delay(1000);
      }
      else
      {
        Serial.println("UNMATCHED SEQUENCE");
      }
      
    }
  }

  // Saves the button states
  precBtn1 = resultBtn1;
}

int16_t CalculateEuclideanDistance(int16_t x, int16_t y)
{
  if (x > y)
    return x - y;
  else
    return y - x;
}

int16_t min3(int16_t a, int16_t b, int16_t c)
{
  if (a <= b && a <= c)
    return a;
  else if (b <= a && b <= c)
    return b;
  else
    return c;
}

bool valid(int varAcX, int varAcY, int varAcZ, int varGyX, int varGyY, int varGyZ, int maxvarAcX, int maxvarAcY, int maxvarAcZ, int maxvarGyX, int maxvarGyY, int maxvarGyZ)
{
  if (varAcX < maxvarAcX*1.3 && varAcY < maxvarAcY*1.3 && varAcZ < maxvarAcZ*1.3 && varGyX < maxvarGyX*1.3 && varGyY < maxvarGyY*1.3 && varGyZ < maxvarGyZ*1.3)
    return true;
  else
    return false;
}

int16_t DTW(int16_t Rcd_xxx[], int16_t S_xxx[], int idx, int a, int b, int m, int PathLen, int var, int mean)
{
  // Calculate the first column:
  for (int i = 1; i < 100; i++) 
  {
    cost[i][0] = cost[i - 1][0] + CalculateEuclideanDistance(Rcd_xxx[i], S_xxx[0]);
  }

  // Calculate the first row:
  for (int j = 1; j < 100; j++) 
  {
    cost[0][j] = cost[0][j - 1] + CalculateEuclideanDistance(Rcd_xxx[0], S_xxx[j]);
  }

  // Fill the matrix:
  for (int i = 1; i < 100; i++)
  {
    for (int j = 1; j < 100; j++)
    {
      cost[i][j] = min3( cost[i - 1][j], cost[i][j - 1], cost[i - 1][j - 1] ) + CalculateEuclideanDistance(Rcd_xxx[i], S_xxx[j]);
    }
  }
//  Serial.println("costAcX: ");
//  Serial.println(cost[49][49]);
  return abs(cost[99][99]);
}

void resetdtw()
{
  idx = 0;
  a = 99;
  b = 99;
  m = 0;
  PathLen = 0;
  mean = 0;
}

// I2C
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
