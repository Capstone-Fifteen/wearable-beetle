// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

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

// CircularBuffer Libraries
#include <CircularBuffer.h>

// MegunoLink Libraries for Exponential Filter
#include "MegunoLink.h"
#include "Filter.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define EMG_INPUT_PIN 0     // define the used EMG input channel
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//Acc + Gyro Global Vars for Current Reading
int AccX = 0;
int AccY = 0;
int AccZ = 0;

float AccVectorMag = 0.0;

int GyroYaw = 0;
int GyroPitch = 0;
int GyroRoll = 0;

// MegunoLink Exponential Filter Vars
long FilterWeight = 5;
ExponentialFilter<long> AccXFilter(FilterWeight, 0);
ExponentialFilter<long> AccYFilter(FilterWeight, 0);
ExponentialFilter<long> AccZFilter(FilterWeight, 0);

ExponentialFilter<long> AccVectorMagFilter(FilterWeight, 0);

ExponentialFilter<long> GyroYawFilter(FilterWeight, 0);
ExponentialFilter<long> GyroPitchFilter(FilterWeight, 0);
ExponentialFilter<long> GyroRollFilter(FilterWeight, 0);

//CircularBuffer Buffer Vars
CircularBuffer<int, 50> AccXBuffer; // size 50, 1 second of AccX Data
CircularBuffer<int, 50> AccYBuffer; // size 50, 1 second of AccY Data
CircularBuffer<int, 50> AccZBuffer; // size 50, 1 second of AccZ Data

CircularBuffer<float, 50> AccVectorMagBuffer; // size 50, 1 second of Acceleration vector maginitude Data

CircularBuffer<int, 50> GyroYawBuffer; // size 50, 1 second of Gyro Yaw Data
CircularBuffer<int, 50> GyroPitchBuffer; // size 50, 1 second of Gyro Pitch Data
CircularBuffer<int, 50> GyroRollBuffer; // size 50, 1 second of Gyro Roll Data

// Beetle IMU Sampling Rate
int IMU_SAMPLE_INTERVAL = 20; //20 millis between IMU read

// Beetle last action timestamp (millis)
long int imu_last_read_time = 0; //when the IMU was last read by the Beetle

// Features vars
int dancer_state = 0; // 0 is IDLE; 1 is Dancing

bool feature_arm_pointing_down = false;

int feature_left_right_null = 0; // 0 null; 1 left; 2 right

int feature_high_acc = 0; // percentage of window in high acceleration
int feature_low_acc = 0;// percentage of window in low acceleration

float emg_mean = 0; // mean absolute value of emg during entire session
long long emg_samples = 0; //number of emg samples taken during entire session
int emg_startup_count = 0; // number of readings for the emg to calibrate and initialise

int fatigue_flag = 0; // 0 not fatigued; 1 fatigued

//Feature thresholds

// Threshold: Arm Pointing Down
int gryo_roll_delta_threshold_max = 20; // difference btw current roll andgle and pointing down angle (90 degrees)e.g should be below 20 to be considered to be pointing downwards
int gryo_roll_delta_threshold_min = 0;

// Threshold: Magnitude of Acceleration Vector for Dancing
float acc_vector_mag_threshold = 0.20; 

// Threshold: Proportion of Acceleration Vector Magnitudes with High Acceleration
int prop_high_acc = 30;


// Threshold: Detect if moving LEFT or RIGHT when IDLE
int acc_z_move_threshold = 500;

// Threshold: Detect possible dancer fatigue
float fatigue_threshold = 0.25;

// ================================================================
// ===               ACTIVITY DETECTION ROUTINE                ===
// ================================================================

void detectActivity() {

  // Feature_1: Arm Pointing Down
  GyroRollBuffer.push(90 - GyroRoll);
  
  int arm_pointing_up_count = 0;
  int arm_pointing_down_count = 0;
  
  int GyroRollBufferSize = GyroRollBuffer.size();
  for (int i = 0; i < GyroRollBufferSize - 1; i++) {
    if (GyroRollBuffer[i] > gryo_roll_delta_threshold_min && GyroRollBuffer[i] < gryo_roll_delta_threshold_max) {
      arm_pointing_down_count++;
    } else {
      arm_pointing_up_count++;
    }
  }

  if (arm_pointing_down_count > arm_pointing_up_count) {
    feature_arm_pointing_down = true;
//    Serial.println(feature_arm_pointing_down);
  } else {
    feature_arm_pointing_down = false;
//    Serial.println(feature_arm_pointing_down);
  }
  

  // Feature_2: Detect window of High Acceleration
  AccVectorMagBuffer.push(AccVectorMag);
  int AccVectorMagBufferSize = AccVectorMagBuffer.size();
  int high_acc_count = 0;
  int low_acc_count = 0;
  for (int i = 0; i < AccVectorMagBufferSize; i++) {
    if (AccVectorMagBuffer[i] > acc_vector_mag_threshold) {
      high_acc_count++;
    } else {
      low_acc_count++;
    }
  }
  feature_high_acc = (int)(100.0 * high_acc_count / AccVectorMagBufferSize );
  feature_low_acc = (int)(100.0 * low_acc_count / AccVectorMagBufferSize);
//  Serial.println(feature_high_acc);
//  Serial.println(feature_low_acc);


  // Feature_3: Detect if moving LEFT or RIGHT when IDLE
  if (dancer_state == 0) {
    AccXBuffer.push(AccX);
    AccZBuffer.push(AccZ);
  } else {
    AccXBuffer.clear();
    AccZBuffer.clear() ;
  }

  //Feature_3a: Detection of Acc spike pattern
  //    - AccZ (+ve) AND AccX (+ve) BEFORE AccZ (-ve) AND AccX (-ve) -> LEFT
  //    - AccZ (-ve) AND AccX (-ve) BEFORE AccZ (+ve) AND AccX (+ve) -> RIGHT

  bool acc_pos_peak_detected = false;
  bool acc_neg_peak_detected = false;

  bool move_left_detected = false;
  bool move_right_detected = false;

  int AccXBufferSize = AccXBuffer.size();
  int AccZBufferSize = AccZBuffer.size();
  
  for (int i = 0; i< min(AccXBufferSize, AccZBufferSize); i++) {
    if ((AccZBuffer[i] > acc_z_move_threshold) && (AccZBuffer[i] > 0 && AccXBuffer[i] > 0)) {
      acc_pos_peak_detected = true;
      if (acc_neg_peak_detected == true) {
        move_left_detected = false;
        move_right_detected = true;
      }
    } else if ((AccZBuffer[i] < -acc_z_move_threshold) && (AccZBuffer[i] < 0 && AccXBuffer[i] < 0)) {
      acc_neg_peak_detected = true;
      if (acc_pos_peak_detected == true) {
        move_left_detected = true;
        move_right_detected = false;
      }
    }
  }

  if (move_left_detected == true && move_right_detected == false) {
    feature_left_right_null = 1;
    Serial.println(feature_left_right_null);
  } else if (move_left_detected == false && move_right_detected == true) {
    feature_left_right_null = 2;
    Serial.println(feature_left_right_null);
  }
  
  

  // State Transition: Dancer State Transition
  if (dancer_state == 0) {
    if (feature_arm_pointing_down == false) {
      dancer_state = 1; // IDLE -> DANCING
    }
  } else if (dancer_state == 1) {
    if (feature_arm_pointing_down == true) {
      dancer_state = 0; // DANCING -> IDLE
    }
  }
}

void detectFatigue() {
//  int emg_mean = 0; // mean absolute value of emg during entire session
//  int emg_samples = 0; //number of emg samples taken during entire session
//  int emg_startup_count = 0; // number of readings for the emg to calibrate and initialise
  TimePlot Plot;
  int EMGValue = analogRead(EMG_INPUT_PIN);
//  Plot.SendData("Raw-EMG", EMGValue/1000.0);
  if (emg_startup_count < 150) {
    emg_startup_count++;
  } else {
    emg_samples++;
    emg_mean = (emg_mean * (emg_samples - 1) + EMGValue/1000.0) / emg_samples;
//    Plot.SendData("EMG-Mean", emg_mean);
  }
  // Set flag: Dancer Fatigue Flag
  if (fatigue_flag == 0) {
    if (emg_mean >= fatigue_threshold) {
      fatigue_flag = 1; // not fatigued -> fatigued
//      Plot.SendData("fatigue_flag", fatigue_flag);
    }
  } else if (fatigue_flag == 1) {
    if (emg_mean < fatigue_threshold) {
      fatigue_flag = 0; // fatigued -> not fatigued
//      Plot.SendData("fatigue_flag", fatigue_flag);
    }
  }
  
}

void setIMUOffsets() {
  
  int beetle_id = 2; // CHANGE AND UPLOAD FOR SELECTED BEETLE ID

  if (beetle_id == 1) { //OG Beetle
    mpu.setXGyroOffset(71);
    mpu.setYGyroOffset(-19);
    mpu.setZGyroOffset(-259);

    mpu.setXAccelOffset(-5568);
    mpu.setYAccelOffset(-1729);
    mpu.setZAccelOffset(1318);
  
  } else if (beetle_id == 2) { // Beetle with EMG
    mpu.setXGyroOffset(120);
    mpu.setYGyroOffset(-26);
    mpu.setZGyroOffset(-12);

    mpu.setXAccelOffset(-5451);
    mpu.setYAccelOffset(-2764);
    mpu.setZAccelOffset(1660);
  } else if (beetle_id == 3) { // Beetle with no strap yet
    mpu.setXGyroOffset(42);
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(48);

    mpu.setXAccelOffset(-855);
    mpu.setYAccelOffset(2004);
    mpu.setZAccelOffset(1444);
  }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
/*    
    pinMode(INTERRUPT_PIN, INPUT);
*/
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
/*
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    setIMUOffsets();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
//        mpu.CalibrateAccel(6);
//        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
/*
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
*/
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      delay(1000);
    }

    if (millis() > imu_last_read_time + IMU_SAMPLE_INTERVAL) {
      imu_last_read_time = millis();
      
          // read a packet from FIFO
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);

//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);

        AccXFilter.Filter(aaReal.x);
        AccYFilter.Filter(aaReal.y);
        AccZFilter.Filter(aaReal.z);

//        AccVectorMagFilter.Filter();

        GyroYawFilter.Filter(ypr[0] * 180/M_PI);
        GyroPitchFilter.Filter(ypr[1] * 180/M_PI);
        GyroRollFilter.Filter(ypr[2] * 180/M_PI);

        AccX = AccXFilter.Current();
        AccY = AccYFilter.Current();
        AccZ = AccZFilter.Current();

        AccVectorMag = sqrt(sq(AccX/16384.0) + sq(AccY/16384.0) + sq(AccZ/16384.0)); // units in terms of g
        
        GyroYaw = GyroYawFilter.Current();
        GyroPitch = GyroPitchFilter.Current();
        GyroRoll = GyroRollFilter.Current();
             
        detectActivity();
        TimePlot Plot;
//        Plot.SendData("Filtered-AccX", AccXFilter.Current());
//        Plot.SendData("Filtered-AccY", AccYFilter.Current());
//        Plot.SendData("Filtered-AccZ", AccZFilter.Current());
//        Plot.SendData("Filtered-AccVectorMag", AccVectorMag);
//        Plot.SendData("Filtered-GyroYaw", GyroYawFilter.Current());
//        Plot.SendData("Filtered-GyroPitch", GyroPitchFilter.Current());
//        Plot.SendData("Filtered-GyroRoll", GyroRollFilter.Current());

//        Serial.print("areal\t");
//        Serial.print(AccX/16384.0);
//        Serial.print("\t");
//        Serial.print(aaReal.y);
//        Serial.print("\t");
//        Serial.println(aaReal.z);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
      }
      
      detectFatigue();
    }
    

}
