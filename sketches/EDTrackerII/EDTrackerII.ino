//TODO
//enable scale adjust ? via UI  + -
//add mouse emu


//
//  Head Tracker Sketch
//

const char* PROGMEM infoString = "EDTrackerII V2.10";

//
// Changelog:
// 2014-05-05 Migrate V1 Head Tracker to new port of Invensense libs
// 2014-05-13 Remove dodgy comment line. Move bias values away from user editable section
// 2014-05-16 Stuff
// 2014-05-20 Amend version number to keep in line with changes
// 2014-05-23 Set Gyro and Accel FSR to keep DMP happy (undocumented req?)
// 2014-05-28 Fix constraint
// 2014-05-28 Test implementation of basic sping-back to counter yaw drift
// 2014-05-28 Increase sample rate from 100 to 200 hz.
// 2014-06-02 Fix drift comp value stored in EEPROM
// 2014-06-02 Push bias to DMP rather than MPU
// 2014-06-03 Remove revision for now.
// 2014-06-11 Put revision back in plus temps. Toggle linear/exp via UI. Say Hi.
// 2014-06-15 Fix yaw lock at 180. Reduce recalibration delay
// 2014-06-20 Wrap drift comp value and also wrap DMP + drift comp to prevent yaw lock


/* ============================================
EDTracker device code is placed under the MIT License

Copyright (c) 2014 Rob James, Dan Howell

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

//Linear/exponential response now controled via UI
float xExpScale = 8.0;
float yExpScale = 8.0;
float zExpScale = 8.0;

float xScale = 4.0;
float yScale = 4.0;
float zScale = 4.0;

//Variables used continual auto yaw compensation
float dzX = 0.0;
float lX = 0.0;
unsigned int ticksInZone = 0;
unsigned int reports = 0;

#define POLLMPUx

#define EMPL_TARGET_ATMEGA328

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)    Serial.print (x);
#define DEBUG_PRINTLN(x)  Serial.println (x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#include <EEPROM.h>
#include <Wire.h>
#include <I2Cdev.h>

#include <helper_3dmath.h>
extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}


float xDriftComp = 0.0;

/* EEPROM Offsets for config and calibration stuff*/
#define EE_VERSION 0
// these are now longs (4 bytes)
#define EE_XGYRO 1
#define EE_YGYRO 5
#define EE_ZGYRO 9
#define EE_XACCEL 13
#define EE_YACCEL 17
#define EE_ZACCEL 21
// 1 byte
#define EE_ORIENTATION 25
// 2 bytes
#define EE_XDRIFTCOMP 26

//0 for linear, 1 for exponential
#define EE_EXPSCALEMODE 28

//2 bytes   Q8:8
#define EE_SCALEADJUST 29

#define SDA_PIN 2
#define SCL_PIN 3

#define LED_PIN 17 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define BUTTON_PIN 10

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif


enum outputModeType {
  OFF,
  DBG,
  UI
};

outputModeType outputMode = OFF;

float lastX, lastY, lastZ;
float dX, dY, dZ;
int driftSamples = 0;
boolean expScaleMode = 0;
float   scaleAdjust = 1.0;
unsigned char revision;

// packet structure for InvenSense teapot demo
unsigned long lastMillis;
unsigned long lastUpdate;

float cx, cy, cz = 0.0;

long gBias[3], aBias[3], fBias[3];;

//Running count of samples - used when recalibrating
int   sampleCount = 0;
boolean calibrated = false;

//Allows the MPU6050 to settle for 10 seconds.
//There should be no drift after this time
unsigned short  calibrateTime     = 10000;

//Number of samples to take when recalibrating
byte  recalibrateSamples =  200;

// Holds the time since sketch stared
unsigned long  nowMillis;
boolean blinkState;

TrackState_t joySt;

/* The mounting matrix below tells the MPL how to rotate the raw
 * data from the driver(s).
 */

static byte gyro_orients[4] =
{
  B10001000, // Z Up X Forward
  B10000101, // X right
  B10101100, // X Back
  B10100001 // X Left
}; //ZYX

byte orientation = 1;

//Need some helper funct to read/write integers
void writeIntEE(int address, int value) {
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

#ifdef DEBUG
  outputMode = UI;
#endif;

  //long l = readLongEE(0);
  //Serial.println(l);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  //  pinMode(SDA_PIN, INPUT);
  //  pinMode(SCL_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // send a I2C stop signal
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SDA_PIN, LOW);

  orientation = constrain(EEPROM.read(EE_ORIENTATION), 0, 3);

  expScaleMode = EEPROM.read(EE_EXPSCALEMODE);

  xDriftComp = (float)readIntEE(EE_XDRIFTCOMP) / 256.0;

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 12; // 24 400kHz I2C clock (200kHz if CPU is 8MHz)

  // Disable internal I2C pull-ups
  cbi(PORTD, 0);
  cbi(PORTD, 1);

  // Initialize the MPU:
  //
  // Gyro sensitivity:      2000 degrees/sec
  // Accel sensitivity:     2 g
  // Gyro Low-pass filter:  42Hz
  // DMP Update rate:       100Hz

  DEBUG_PRINTLN("M\tInit MPU...");

  if ( initialize_mpu() ) {
    delay(100);
    enable_mpu();
    delay(100);
    mpu_read_6050_accel_bias(fBias);
    delay(100);
    loadBiases();

  }
  else {
    DEBUG_PRINTLN("M\tInit Failed");
    while (1);
  }

  DEBUG_PRINTLN("M\t\Settling.");

}

/*****************************************
* Conversion Factors
*****************************************/

/****************************************
* Gyro/Accel/DMP State
****************************************/
unsigned long sensor_timestamp;

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ    (200)

/****************************************
* Gyro/Accel/DMP Configuration
****************************************/

void recenter()
{
  if (outputMode == UI)
  {
    Serial.println("M\tRecentering");
  }
  sampleCount = 0;
  cx = cy = cz = 0;
  calibrated = false;
}
//unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
//unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
//unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000.  Default:  2000
boolean new_gyro , dmp_on;
void loop()
{
  blink();
  nowMillis = millis();

  // If the MPU Interrupt occurred, read the fifo and process the data

#ifdef POLLMPU
  if (true)    //new_gyro && hal.dmp_on)
#else
  if (new_gyro && dmp_on)
#endif
  {
    short gyro[3], accel[3], sensors;
    unsigned char more = 0;
    long quat[4];
    sensor_timestamp = 1;
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

    //
    if (!more)
      new_gyro = 0;

    if (sensor_timestamp == 0)
    {
      Quaternion q( (float)(quat[0] >> 16) / 16384.0f,
                    (float)(quat[1] >> 16) / 16384.0f,
                    (float)(quat[2] >> 16) / 16384.0f,
                    (float)(quat[3] >> 16) / 16384.0f);

      // Calculate Yaw/Pitch/Roll
      // Update client with yaw/pitch/roll and tilt-compensated magnetometer data

      // Use some code to convert to R P Y
      float newZ =  atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
      float newY = -asin(-2.0 * (q.x * q.z - q.w * q.y));
      float newX = -atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);

      // if we're still in the initial 'settling' period do nothing else...
      if (nowMillis < calibrateTime)
      {
        return;
      }

      // scale to range -32767 to 32767
      newX = newX   * 10430.06;
      newY = newY   * 10430.06;
      newZ = newZ   * 10430.06;

      if (!calibrated)
      {
        if (sampleCount < recalibrateSamples)
        { // accumulate samples
          cx += newX;
          cy += newY;
          cz += newZ;
          sampleCount ++;
        }
        else
        {
          calibrated = true;
          cx = cx / (float)sampleCount;
          cy = cy / (float)sampleCount;
          cz = cz / (float)sampleCount;

          dX = dY = dZ = 0.0;
          driftSamples = -2;
          recalibrateSamples = 100;// reduce calibrate next time around
          if (outputMode == UI)
          {
            Serial.print("I\t");
            Serial.println(infoString);
            Serial.println("M\tRecentered");
          }
          //pushBias2DMP();
        }
        return;
      }

      // Have we been asked to recalibrate ?
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        recenter();
        return;
      }

      short mag[3];
      unsigned long timestamp;

      // mpu_get_compass_reg(mag, &timestamp);

      // apply calibration offsets
      newX = newX - cx;

      // this should take us back to zero BUT we may have wrapped so ..
      if (newX < -32768.0)
        newX += 65536.0;

      if (newX > 32768.0)
        newX -= 65536.0 ;

      newY = newY - cy;
      newZ = newZ - cz;

      //clamp at 90 degrees left and right
      newX = constrain(newX, -16383.0, 16383.0);
      newY = constrain(newY, -16383.0, 16383.0);
      newZ = constrain(newZ, -16383.0, 16383.0);

      long  iX ;
      long  iY ;
      long  iZ ;

      if (expScaleMode) {
        iX = (0.000122076 * newX * newX * xExpScale) * (newX / abs(newX)); //side mount = yaw
        iY = (0.000122076 * newY * newY * yExpScale) * (newY / abs(newY)); //side mount = pitch
        iZ = (0.000122076 * newZ * newZ * zExpScale) * (newZ / abs(newZ)); //side mount = roll
      }
      else
      {
        // and scale to out target range plus a 'sensitivity' factor;
        iX = (newX * xScale );
        iY = (newY * yScale );
        iZ = (newZ * zScale );
      }

      // clamp after scaling to keep values within 16 bit range
      iX = constrain(iX, -32767, 32767);
      iY = constrain(iY, -32767, 32767);
      iZ = constrain(iZ, -32767, 32767);

      // Do it to it.
      joySt.xAxis = iX ;
      joySt.yAxis = iY;
      joySt.zAxis = iZ;

      if (outputMode == UI)
      {
        Serial.print(iX ); // Yaw
        Serial.print("\t");
        Serial.print(iY); // Pitch
        Serial.print("\t");
        Serial.print(iZ); // Roll
        Serial.print("\t");

        tripple(accel);
        tripple(gyro);
        Serial.println("");
      }

      Tracker.setState(&joySt);
      reports++;


      //self centering
      // if we're looking ahead, give or take
      //  and not moving
      //  and pitch is levelish then start to count
      if (outputMode != UI )
      {
        if (fabs(iX) < 3000.0 && fabs(iX - lX) < 5.0 && fabs(iY) < 600)
        {
          ticksInZone++;
          dzX += iX;
        }
        else
        {
          ticksInZone = 0;
          dzX = 0.0;
        }
        lX = iX;

        // if we stayed looking ahead-ish long enough then adjust yaw offset
        if (ticksInZone >= 10)
        {
          // NB this currently causes a small but visible jump in the
          // view. Useful for debugging!
          dzX = dzX * 0.1;
          cx += dzX * 0.1;
          ticksInZone = 0;
          dzX = 0.0;
        }
      }

      parseInput();

      // Apply X axis drift compensation every 1 second
      if (nowMillis > lastUpdate)
      {
        //depending on your mounting
        cx = cx + xDriftComp;

        if (cx > 65536.0)
          cx = cx - 65536.0;
        else if (cx < -65536.0 )
          cx = cx + 65536.0;

        lastUpdate = nowMillis + 100;

        driftSamples++;

        if (driftSamples > 0)
        {
          dX += (newX - lastX);
        }
        lastX = newX;


        DEBUG_PRINT("X/Y/Z\t");
        DEBUG_PRINT(newX  );
        DEBUG_PRINT("\t\t");
        DEBUG_PRINT(newY );
        DEBUG_PRINT("\t\t");
        DEBUG_PRINT(newZ );
        DEBUG_PRINT("\t\t");

        DEBUG_PRINTLN(dX / (float)driftSamples  );
        if (outputMode == UI)
        {
          Serial.print("D\t");
          Serial.print(dX / (float)driftSamples);
          Serial.print("\t");
          Serial.println(xDriftComp);

          //          Serial.print("M\t Updates per second ");
          //          Serial.println(reports);
          reports = 0;

          long t;
          mpu_get_temperature (&t, 0);
          Serial.print("T\t");
          Serial.println(t);
        }

        //        DEBUG_PRINT("\t\t");
        //        DEBUG_PRINT(dY / (float)driftSamples );
        //        DEBUG_PRINT("\t\t");
        //        DEBUG_PRINTLN(dZ / (float)driftSamples );
      }
    }
  }
}


void parseInput()
{
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    byte command = Serial.read();

    if (command == 'S')
    {
      outputMode = OFF;
      Serial.println("S"); //silent
      dmp_set_fifo_rate(DEFAULT_MPU_HZ);

    }
    else if (command == 'H')
    {
      Serial.println("H"); // Hello
    }
    else if (command == 't')
    {
      //toggle linear/expoinential mode
      expScaleMode = !expScaleMode;
      EEPROM.write(EE_EXPSCALEMODE, expScaleMode);
      Serial.print("s\t");
      Serial.print(expScaleMode);
      Serial.print("\t");
      Serial.println(scaleAdjust);
    }
    else if (command == 'V')
    {
      Serial.println("V"); //verbose
      Serial.print("I\t");
      Serial.println(infoString);

      Serial.print("s\t");
      Serial.print(expScaleMode);
      Serial.print("\t");
      Serial.println(scaleAdjust);

      outputMode = UI;
      if (DEFAULT_MPU_HZ > 101)
        dmp_set_fifo_rate(DEFAULT_MPU_HZ / 2);

    }
    else if (command == 'I')
    {
      Serial.print("I\t");
      Serial.println(infoString);

      Serial.println("M\t----------------");
      Serial.print("M\tOrientation ");
      Serial.println(orientation);

      Serial.print("M\tDrift Comp");
      Serial.println(xDriftComp);

      Serial.print("M\tDrift Rate ");
      Serial.println((dX / (float)driftSamples));

      mess("M\tGyro Bias ", gBias);
      mess("M\tAccel Bias ", aBias);
      mess("M\tFact Bias ", fBias);

      Serial.print("M\tMPU Revision ");
      Serial.println(revision);

      Serial.print("s\t");
      Serial.print(expScaleMode);
      Serial.print("\t");
      Serial.println(scaleAdjust);

    }
    else if (command == 'P')
    {
      mpu_set_dmp_state(0);
      orientation = (orientation + 1) % 4; //0 to 3
      dmp_set_orientation(gyro_orients[orientation]);
      mpu_set_dmp_state(1);
      Serial.print("M\tOrienation ");
      Serial.println(orientation);
      EEPROM.write(EE_ORIENTATION, orientation);
    }
    else if (command == 'R')
    {
      //recalibrate offsets
      recenter();
    }
    else if (command == 'H')
    {
      Serial.println("H"); // Hello
    }
    else if (command == 'D')
    {
      //Save Drift offset
      xDriftComp = (dX / (float)driftSamples) + xDriftComp;
      writeIntEE(EE_XDRIFTCOMP, (int)(xDriftComp * 256.0));
      //Serial.println("M\tSaved Drift Comp ");
      //Serial.println(xDriftComp);
      Serial.print("R\t");
      Serial.println(xDriftComp);
    }
//    else if (command == 'F')
//    {
//      pushBias2DMP();
//    }

    while (Serial.available() > 0)
      command = Serial.read();
  }
}


/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void) {
  new_gyro = 1;
}
#ifndef POLLMPU
ISR(INT6_vect) {
  new_gyro = 1;
}
#endif

void tap_cb (unsigned char p1, unsigned char p2)
{
  return;
  //  if (outputMode == UI)
  //  {
  //    Serial.print("M\tTap Detected");
  ////    Serial.print((int)p1);
  ////    Serial.print(" ");
  ////    Serial.println((int)p2);
  //  }
}



boolean initialize_mpu() {
  int result;

  mpu_init(&revision);

  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  mpu_set_gyro_fsr (2000);//250
  mpu_set_accel_fsr(2);//4
  //mpu_set_lpf(98);

  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  //mpu_set_lpf(42);

  /* To initialize the DMP:
   * 1. Call dmp_load_motion_driver_firmware(). .
   * 2. Push the gyro and accel orientation matrix to the DMP.
   * 4. Call dmp_enable_feature(mask) to enable different features.
   * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
   */

  dmp_load_motion_driver_firmware();

  DEBUG_PRINTLN("Firmware Loaded ");

  dmp_set_orientation(gyro_orients[orientation]);

  dmp_register_tap_cb(&tap_cb);

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
                                DMP_FEATURE_SEND_CAL_GYRO;// | DMP_FEATURE_GYRO_CAL;

  //testing with auto gyrpo call off and having a larger manual gyro compensartion
  //next is to combine linear with log curve do give soft middle and linear outside.

  dmp_features = dmp_features |  DMP_FEATURE_TAP ;

  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);

  return true;
}

void disable_mpu() {
  mpu_set_dmp_state(0);
  //hal.dmp_on = 0;
  dmp_on = 0;

#ifndef POLLMPU
  EIMSK &= ~(1 << INT6);      //deactivate interupt
#endif
}

void enable_mpu() {
#ifndef POLLMPU
  EICRB |= (1 << ISC60) | (1 << ISC61); // sets the interrupt type for EICRB (INT6)
  EIMSK |= (1 << INT6); // activates the interrupt. 6 for 6, etc
#endif

  mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
  dmp_on = 1;
}


//
void loadBiases() {
  //  gBias[0] = readLongEE (EE_XGYRO);
  //  gBias[1] = readLongEE (EE_YGYRO);
  //  gBias[2] = readLongEE (EE_ZGYRO);
  //
  //  aBias[0] = readLongEE (EE_XACCEL);
  //  aBias[1] = readLongEE (EE_YACCEL);
  //  aBias[2] = readLongEE (EE_ZACCEL);

  for (int i = 0; i < 3; i++)
  {
    gBias[i] = readLongEE (EE_XGYRO  + i * 4);
    aBias[i] = readLongEE (EE_XACCEL + i * 4);
  }

  //dmp_set_gyro_bias(gBias); <- all sorts of undocumented shit
  //dmp_set_accel_bias(aBias);

  mpu_set_gyro_bias_reg(gBias);
  mpu_set_accel_bias_6050_reg(aBias, true);

  return ;
}


//void pushBias2DMP()
//{
//  //Serial.println("M\t Push Bias to DMP Regs.");
//
//  if (outputMode == UI)
//  {
//    Serial.println("M\tPush Bias to DMP");
//  }
//
//  mpu_set_accel_bias_6050_reg(fBias, 0);
//
//  unsigned short accel_sens;
//
//  mpu_get_accel_sens(&accel_sens);
//
//  //
//  //    Serial.print("M\t Accel Sens ");
//  //    Serial.println(accel_sens);
//  //
//  long a[3];
//
//  for (int i = 0; i < 3; i++)
//    a[i] = (aBias[i] * (long)accel_sens); //<<6;
//
//  dmp_set_accel_bias(a);
//
//}


void blink()
{
  unsigned short delta = 100;

  if (calibrated)
    delta = 300;

  if (nowMillis > lastMillis + delta)
  {
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    lastMillis = nowMillis;
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
