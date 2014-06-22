//
//  Head Tracker Sketch
//

const char* PROGMEM infoString = "ED Tracker Calibration V2.4";

//
// Changelog:
// 2014-05-05 Initial Version
// 2014-05-20 Replace calibration loop - simple iterative method
// 2014-06-02 Mess around with stuff
// 2014-06-10 Add temps. Allow individial biasing of accel axis
// 2014-06-15 Add option to clear bias to factory defaults
// 2014-06-22 Fix LED blinking

/* ============================================
EDTracker device code is placed under the MIT License
Copyright (c) 2014 Rob James

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#define POLLMPUx

#define EMPL_TARGET_ATMEGA328

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)     Serial.print (x);
#define DEBUG_PRINTLN(x)  Serial.println (x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#include <Wire.h>
#include <I2Cdev.h>
#include <EEPROM.h>

#include <helper_3dmath.h>

extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

#define SDA_PIN 2
#define SCL_PIN 3
#define LED_PIN 17


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

unsigned char revision;

bool outputMode = false;

int   adjustAccel = 0; // 0 = all, 1 = x, 2 = y, 3 = z only

long gBias[3], aBias[3], fBias[3];

/* EEPROM Offsets for config and calibration stuff*/
#define EE_VERSION 0
// these are now longs (4 bytes)
#define EE_XGYRO 1
#define EE_YGYRO 5
#define EE_ZGYRO 9
#define EE_XACCEL 13
#define EE_YACCEL 17
#define EE_ZACCEL 21
//1 byte
#define EE_ORIENTATION 25
// 2 bytes
#define EE_XDRIFTCOMP 26

//Need some helper funct to read/write integers
void writeIntEE(int address, int value ) {
  EEPROM.write(address + 1, value >> 8); //upper byte
  EEPROM.write(address, value & 0xff); // write lower byte
}

int readIntEE(int address) {
  return (EEPROM.read(address + 1) << 8 | EEPROM.read(address));
}

void writeLongEE(int address,  long value) {
  for (int i = 0; i < 4; i++)
  {
    EEPROM.write(address++, value & 0xff); // write lower byte
    value = value >> 8;
  }
}

long readLongEE(int address) {
  return ((long)EEPROM.read(address + 3) << 24 |
          (long)EEPROM.read(address + 2) << 16 |
          (long)EEPROM.read(address + 1) << 8 |
          (long)EEPROM.read(address));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(LED_PIN, OUTPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  //
  //  // Disable internal I2C pull-ups
  //  cbi(PORTC, 4);
  //  cbi(PORTC, 5);

  // Gyro sensitivity:      2000 degrees/sec
  // Accel sensitivity:     2g
  // Gyro Low-pass filter:  42Hz
  // DMP Update rate:       100Hz
  initialize_mpu() ;
  //grab the factory bias values

  delay(100);
  mpu_set_dmp_state(1);
  delay(100);
  mpu_read_6050_accel_bias(fBias);
  delay(100);
  //loadBiases(); // on start up just have he factory bias in there
  delay(100);
  //mpu_get_biases
  //  enable_mpu();
}

/***************************************
* Invensense Hardware Abstracation Layer
***************************************/
unsigned char dmp_on;

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ    (100)


/****************************************
* Gyro/Accel/DMP Configuration
****************************************/

//unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
//unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
//unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000.  Default:  2000

long quat[4];
unsigned char more ;
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
boolean blinkState;
unsigned long tick = 0, tick2 = 0;
float lastX;
int samples = 0;
void loop()
{
  parseInput();

  if (millis() > tick)
  {
    tick = millis() + 200;
    digitalWrite(LED_PIN, blinkState);
    blinkState=!blinkState;
  }

  // libs chopped so timestamp not returned
  dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);


  Quaternion q( (float)(quat[0] >> 16) / 16384.0f,
                (float)(quat[1] >> 16) / 16384.0f,
                (float)(quat[2] >> 16) / 16384.0f,
                (float)(quat[3] >> 16) / 16384.0f);

  // Use some code to convert to R P Y
  float newZ =  atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
  float newY = -asin(-2.0 * (q.x * q.z - q.w * q.y));
  float newX = -atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);

  // scale to range -32767 to 32767
  newX = newX   * 10430.06;
  newY = newY   * 10430.06;
  newZ = newZ   * 10430.06;

  if (outputMode)
  {
    Serial.print(newX , 5 ); // Yaw
    Serial.print("\t");
    Serial.print(newY, 5 ); // Pitch
    Serial.print("\t");
    Serial.print(newZ, 5 ); // Roll
    Serial.print("\t");
    tripple(accel);
    tripple(gyro);
    Serial.println("");
    samples++;

    if (millis() > tick2)
    {
      tick2 = millis() + 2000;
      long t;
      mpu_get_temperature (&t, 0);
      Serial.print("T\t");
      Serial.println(t);

      Serial.print("D\t");
      Serial.print((newX - lastX) / (float)samples);
      Serial.print("\t");
      Serial.println(0);

      samples = 0;
      lastX = newX;

    }
  }
  return;
}


//add fine adjust / single axis adjust
//toggle Gyro
//toggle Accel
//toggle x y z
//
//cool!

void parseInput()
{
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    byte command = Serial.read();

    if (command == 'S')
    {
      outputMode = false;
      Serial.println("S"); //silent
    }
    else if (command == 'x')
    {
      adjustAccel = 1;
      Serial.println("x");
    }
    else if (command == 'y')
    {
      adjustAccel = 2;
      Serial.println("y");
    }
    else if (command == 'z')
    {
      adjustAccel = 3;
      Serial.println("z");
    }
    else if (command == 'a')
    {
      adjustAccel = 0;
      Serial.println("a");
    }
    else if (command == '0')
    {
      for (int i = 0; i < 3; i++)
        gBias[i] = aBias[i] = 0;
      saveBias();
    }
    else if (command == 'V')
    {
      Serial.println("V"); //verbose
      //Serial.print("I\t");
      //Serial.println(infoString);
      outputMode = true;
    }
    else if (command == 'H')
    {
      Serial.println("H"); // Hello
    }
    else if (command == 'I')
    {
      Serial.print("I\t");
      Serial.println(infoString);
      loadBiases();
      mess("M\tGyro Bias ", gBias);
      mess("M\tAccel Bias ", aBias);
      mess("M\tFact Bias ", fBias);

      Serial.print("M\tMPU Revision ");
      Serial.println(revision);
    }
    else if (command == 'B')
    {
      update_bias();
    }
    else if (command == 'F')
    {
      //flip where bias values are stored
      flipBias();
    }

    while (Serial.available() > 0)
      command = Serial.read();
  }
}

void tap_cb (unsigned char p1, unsigned char p2)
{
  Serial.print("M\tTap Detected ");
  Serial.print((int)p1);
  Serial.print(" ");
  Serial.println((int)p2);
}


void  initialize_mpu() {

  mpu_init(&revision);

  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  mpu_set_gyro_fsr (2000);//250
  mpu_set_accel_fsr(2);//4
  //mpu_set_lpf(98);

  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);

  dmp_load_motion_driver_firmware();

  dmp_set_orientation(B10001000);

  dmp_register_tap_cb(&tap_cb);

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
                                DMP_FEATURE_SEND_RAW_GYRO;

  dmp_features = dmp_features |  DMP_FEATURE_TAP ;

  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);

  return ;
}


//void enable_mpu() {
//#ifndef POLLMPU
//  EICRB |= (1 << ISC60) | (1 << ISC61); // sets the interrupt type for EICRB (INT6)
//  EIMSK |= (1 << INT6); // activates the interrupt. 6 for 6, etc
//#endif
//
// // mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
//  dmp_on = 1;
//}

// New bias function that peforms a simple adjust/test/adjust loop
// Gives better, near zero, biased raw values which hopefully
// gives better DMP results.

void update_bias()
{
  long gyrozero[3];
  int samples = 100;
  unsigned short i;

  //mpu_read_6050_accel_bias(aBias);
  //mpu_read_gyro_bias(gBias);

  for (i = 0; i < 3; i++)
  {
    gyrozero[i] = 0;
  }

  Serial.println("M\t Sampling..");

  // set gyro to zero and accel to factory bias
  mpu_set_gyro_bias_reg(gyrozero);
  mpu_set_accel_bias_6050_reg(fBias, 0);

  delay(100);

  //return;
  //Serial.println(samples);

  for (int s = 0; s < samples; s++)
  {
    //physical values in Q16.16 format
    //mpu_get_biases(gBias, aBias);

    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

    if (adjustAccel == 0  || adjustAccel == 1)
    {
      if (accel[0] >= 1) aBias[0]++;
      else if (accel[0] <= -1) aBias[0]--;
    }

    if (adjustAccel == 0  || adjustAccel == 2)
    {
      if (accel[1] >= 1) aBias[1]++;
      else if (accel[1] <= -1) aBias[1]--;
    }

    if (adjustAccel == 0  || adjustAccel == 3)
    {
      if (accel[2] > 16384) aBias[2]++;
      else if (accel[2] < 16384) aBias[2]--;
    }

    if (adjustAccel == 0)
    {
      if (gyro[0] > 1) gBias[0] = gBias[0] - 1;
      else if (gyro[0] < -1) gBias[0] = gBias[0] + 1;

      if (gyro[1] > 1) gBias[1]--;
      else if (gyro[1] < -1) gBias[1]++;

      if (gyro[2] > 1) gBias[2]--;
      else if (gyro[2] < -1) gBias[2]++;
    }

    mess("M\tGyro Bias ", gBias);
    mess("M\tAccell Bias ", aBias);


    //push the  factory bias back
    mpu_set_accel_bias_6050_reg(fBias, 0);
    mpu_set_gyro_bias_reg(gBias);
    mpu_set_accel_bias_6050_reg(aBias, 1);
    delay(17);
  }

  mess("M\tGyro Bias ", gBias);
  mess("M\tAccel Bias ", aBias);

  saveBias();
  loadBiases();
  return;
}

void saveBias()
{
  for (  int i = 0; i < 3; i++)
  {
    writeLongEE (EE_XGYRO + i * 4, gBias[i]);
    writeLongEE (EE_XACCEL + i * 4, aBias[i]);
  }
}

void tripple(short *v)
{
  for (int i = 0; i < 3; i++)
  {
    Serial.print(v[i] ); //
    Serial.print("\t");
  }
}

void mess(char *m, long*v)
{
  Serial.print(m);
  Serial.print(v[0]); Serial.print(" / ");
  Serial.print(v[1]); Serial.print(" / ");
  Serial.println(v[2]);
}

void loadBiases() {

  //reset back to factory settings
  mpu_set_accel_bias_6050_reg(fBias, 0);
  delay(100);

  for (int i = 0; i < 3; i++)
  {
    gBias[i] = readLongEE (EE_XGYRO  + i * 4);
    aBias[i] = readLongEE (EE_XACCEL + i * 4);
  }
  mpu_set_gyro_bias_reg(gBias);
  mpu_set_accel_bias_6050_reg(aBias, 1);
  return ;
}


void flipBias()
{
  long zeros[3] = {0, 0, 0};
  Serial.println("M\t Push Bias to DMP Regs.");
  // mpu_set_accel_bias_6050_reg(fBias,0);

  delay (100);
  mpu_set_gyro_bias_reg(zeros);


  unsigned short sens;

  mpu_get_accel_sens(&sens);
  Serial.print("M\t Accel Sens ");
  Serial.println(sens);

  long a[3];

  for (int i = 0; i < 3; i++)
    a[i] = (aBias[i] * (long)sens) / (long)4096; //+/- 8g to q16i

  //dmp_set_accel_bias(a);

  float fsens;

  mpu_get_gyro_sens(&fsens);
  Serial.print("M\t Gyro Sens ");
  Serial.println(fsens);

  for (int i = 0; i < 3; i++)
  {
    a[i] = (long)(gBias[i] * 32767); // in +/- 1000dps con to phys Q16

    Serial.print("M\t UGH ");
    Serial.println(a[i]);
  }
  dmp_set_gyro_bias(a);
}


