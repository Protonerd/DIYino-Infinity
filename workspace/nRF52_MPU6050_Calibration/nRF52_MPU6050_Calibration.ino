// Arduino sketch that returns calibration offsets for MPU6050 //   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors. 
// The effect of temperature has not been taken into account so I can't promise that it will work if you 
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.

// EEPROM emulation for nRF52 controller:
// Sources and inspirations:
// https://github.com/NordicPlayground/nRF5-flash-storage-examples
// example code (see end of the file):
// https://github.com/NordicPlayground/nRF5-flash-storage-examples/blob/master/fstorage_example_ascii/main.c
// source code (can only be found mysteriously here):
// https://github.com/lancaster-university/nrf51-sdk/tree/master/source/nordic_sdk/components/libraries/fstorage only for nRF51?
// https://github.com/kylemanna/nordic-sdk-nrf5/tree/master/components/libraries/fstorage
// https://os.mbed.com/teams/mbed-os-examples/code/mbed-os-example-ble-EddystoneService/docs/tip/nrfConfigParamsPersistence_8cpp_source.html

// Adafruit BLE/ARM solution Little File System:
// https://github.com/ARMmbed/littlefs
// https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide?view=all
// Example using Adafruit's own LFS library https://gist.github.com/stonehippo/ffbed2f32e1813f4019ecedd22062636
// related topic: https://forums.adafruit.com/viewtopic.php?f=22&t=132306
// The bootloader start address will be stored in NRF_UICR->NRFFW[0] (@0x100010014)

// Nordic SD
// D:\Arduino\nRF52_Infinity_DevEnv\arduino-1.8.9\portable\packages\adafruit\hardware\nrf52\0.11.1\cores\nRF5\nordic\softdevice\s132_nrf52_6.1.1_API
// https://github.com/sandeepmistry/arduino-nRF5

/* ==========  LICENSE  ==================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2011 Jeff Rowberg
 
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
 =========================================================
 */

// I2Cdev and MPU6050 must be installed as libraries
#include <I2Cdev.h>
#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

// I'm not sure what it is for?

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

//#define STREAM

#define DEBUGVERBOSE

#define MAIN_BUTTON     12
#define LOCKUP_BUTTON   4

#define MPU_INTERRUPT
///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int discardfirstmeas=100;  // Amount of initial measurements to be discarded
int acel_deadzone=10;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=10;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int accel_offset_divisor=8; //8;
int gyro_offset_divisor=4; //4;
// deadzone: amount of variation between 2 consecutive measurements

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
I2Cdev i2ccomm;
//MPU6050 mpu(0x69); // <-- use for AD0 high
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t mpuFifoCount;     // count of all bytes currently in FIFO
// calibrated acc/gyro values
int16_t ax_zero, ay_zero, az_zero;
int16_t gx_zero, gy_zero, gz_zero;
// orientation/motion vars
Quaternion quaternion;           // [w, x, y, z]         quaternion container
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
static Quaternion quaternion_last;  // [w, x, y, z]         quaternion container
static Quaternion quaternion_reading; // [w, x, y, z]         quaternion container
static VectorInt16 aaWorld_last; // [x, y, z]            world-frame accel sensor measurements
static VectorInt16 aaWorld_reading; // [x, y, z]            world-frame accel sensor measurements

int16_t ax, ay, az,gx, gy, gz;


int16_t mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int16_t ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
int16_t ax_initoffset,ay_initoffset,az_initoffset,gx_initoffset,gy_initoffset,gz_initoffset;
int16_t ax_offsetEEPROM,ay_offsetEEPROM,az_offsetEEPROM,gx_offsetEEPROM,gy_offsetEEPROM,gz_offsetEEPROM;
unsigned int calibratedOffsetAdress = 0;
bool forceCalibration = true;
bool CalibResult=false;
unsigned long sndRepeat = millis();

unsigned int loopcount=0;

/***************************************************************************************************
 * LED String variables
 */
uint8_t LEDbrightness;
int8_t signSoundFile, signVolume, signBrightness;

/**********************************************
 * Little File System Variables (based on Adafruit littleFS
 */


File file(InternalFS);
#define MPUCALIBFILE    "/MPUCalib.txt"

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {


  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
#endif

// Serial line for debug
Serial.begin(115200);


  

  //setup finished. Boot ready. We notify !


  // initialize device
  mpu.initialize();

  #ifdef MPU_INTERRUPT
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    // define Interrupt Service for aux. switch
    attachInterrupt(0, ISR_MPUInterrupt, FALLING); // int.0 is the pin2 on the Nano board
    // define D2 (interrupt0) as input
    pinMode(2, INPUT_PULLUP);
  #endif
  
  if (mpu.testConnection() ) {

    // reset offsets
    ax_initoffset=0;
    ay_initoffset=0;
    az_initoffset=0;
    gx_initoffset=0;
    gy_initoffset=0;
    gz_initoffset=0;
  
    mpu.setXAccelOffset(ax_initoffset);
    mpu.setYAccelOffset(ay_initoffset);
    mpu.setZAccelOffset(az_initoffset);
    mpu.setXGyroOffset(gx_initoffset);
    mpu.setYGyroOffset(gy_initoffset);
    mpu.setZGyroOffset(gz_initoffset);
    

  // set the fll scale range of the gyro- and accelerometer respectively
  mpu.setFullScaleGyroRange(0); //0: 250deg/s | 1: 500deg/s | 2: 1000deg/s | 3: 2000deg/s
  mpu.setFullScaleAccelRange(0); //0: 2g | 1: 4g | 2: 8g | 3: 16g
          
  }

    // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
#if defined LS_INFO
    Serial.println(F("Enabling DMP..."));
#endif
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
#if defined LS_INFO
    Serial.println(
      F(
        "Enabling interrupt detection (Arduino external interrupt 0)..."));
#endif
    //    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
#if defined LS_INFO
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
#if defined LS_INFO
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
#endif
  }

  // configure the motion interrupt for clash recognition
  // INT_PIN_CFG register
  // in the working code of MPU6050_DMP all bits of the INT_PIN_CFG are false (0)
  mpu.setDLPFMode(3);
  mpu.setDHPFMode(0);
  //mpu.setFullScaleAccelRange(3);
  mpu.setIntMotionEnabled(true); // INT_ENABLE register enable interrupt source  motion detection
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntFIFOBufferOverflowEnabled(false);
  mpu.setIntI2CMasterEnabled(false);
  mpu.setIntDataReadyEnabled(false);
  //  mpu.setMotionDetectionThreshold(10); // 1mg/LSB
  mpu.setMotionDetectionThreshold(10); // 1mg/LSB
  mpu.setMotionDetectionDuration(2); // number of consecutive samples above threshold to trigger int
  // configure Interrupt with:
  // int level active low
  // int driver open drain
  // interrupt latched until read out (not 50us pulse)
  i2ccomm.writeByte(MPU6050_DEFAULT_ADDRESS, 0x37, 0xF0);
  // enable only Motion Interrut
  i2ccomm.writeByte(MPU6050_DEFAULT_ADDRESS, 0x38, 0x40);
  mpuIntStatus = mpu.getIntStatus();


  /***** MP6050 MOTION DETECTOR INITIALISATION  *****/
  // start the file system lib and see if we have anything persisted
  InternalFS.begin();
  // format the Flash storage if the Calibration is forced
  if (forceCalibration) {
        int inByte=0; 
        Serial.println(F("If you really want to force calibration, the Flash Storage needs to be formatted. press \"y\" to proceed \n"));
        while (!Serial.available()){
          delay(100);
        }
        if (Serial.available() > 0) {
          inByte = Serial.read();
        }
        if (inByte==121) { // "y"
          Serial.print("Formating ... ");
          delay(100); // for message appear on monitor

          // Format 
          InternalFS.format();

          Serial.println("Done");          
        }
        else {
           Serial.println("Continuing without formatting the Flash storage, new results will be appended to existing Calibration file MPUCalib.txt");
           state=10;  // do nothing
        }

  }
  
  file.open(MPUCALIBFILE, FILE_O_READ); // this also creates the file if it does not exist
  if ( file ) {
    Serial.println(MPUCALIBFILE " file exists");
    uint32_t readlen;
    int16_t buffer[6] = { 0 };
    readlen = file.read(buffer, sizeof(buffer));

    buffer[readlen] = 0;
    for (uint8_t i =0; i<sizeof(buffer);i++) {
      Serial.println(buffer[i]);
    }
    // reading in calibration values and storing calibration offsets in the MPU
    Serial.print("X-Axis Acceleration Offset: ");Serial.println(buffer[0]);
    Serial.print("Y-Axis Acceleration Offset: ");Serial.println(buffer[1]);
    Serial.print("Z-Axis Acceleration Offset: ");Serial.println(buffer[2]);
    Serial.print("X-Axis Gyro Offset:         ");Serial.println(buffer[3]);
    Serial.print("Y-Axis Gyro Offset:         ");Serial.println(buffer[4]);
    Serial.print("Z-Axis Gyro Offset:         ");Serial.println(buffer[5]);
    mpu.setXAccelOffset(buffer[0]);
    mpu.setYAccelOffset(buffer[1]);
    mpu.setZAccelOffset(buffer[2]);
    mpu.setXGyroOffset(buffer[3]);
    mpu.setYGyroOffset(buffer[4]);
    mpu.setZGyroOffset(buffer[5]);
    file.close();
    delay(1000);
  }
  else {
    Serial.print("Open " MPUCALIBFILE " file to write ... ");
    if( file.open(MPUCALIBFILE, FILE_O_WRITE) )
    {
      Serial.println("Do nothing");
    }
    else {
      Serial.println("Failed!");
    }
  }

  
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop() {
  
// variable declaration
int16_t mpu_caliboffset_AccX, mpu_caliboffset_AccY, mpu_caliboffset_AccZ, mpu_caliboffset_GyroX, mpu_caliboffset_GyroY, mpu_caliboffset_GyroZ;
int inByte=0; 

  
  if (state==0){
      Serial.println("\nReading sensors for first time...");
      meansensors();
      //if (abs(mean_ax)>=32000){ax_initoffset=-mean_ax;Serial.println("\nRemove X-axis deadlock...");}
      //if (abs(mean_ay)>=32000){ay_initoffset=-mean_ay;Serial.println("\nRemove Y-axis deadlock...");}
      //if (mean_az<-32000){az_initoffset=-mean_az;Serial.println("\nRemove Z-axis deadlock...");}
      //if (abs(mean_gx)>=32000){gx_initoffset=-mean_gx;Serial.println("\nRemove Gyro X-axis deadlock...");}
      //if (abs(mean_gy)>=32000){gy_initoffset=-mean_gy;Serial.println("\nRemove Gyro Y-axis deadlock...");}
      //if (mean_gz<-32000){gz_initoffset=-mean_gz;Serial.println("\nRemove Gyro Z-axis deadlock...");}
      state++;
      delay(1000);
  }

  if (state==1) {
    Serial.println("\nCalculating offsets...");
    CalibResult=calibration();
    if (CalibResult) {
      Serial.println("\nCalibration successful!");
    }
    else {
      Serial.println("\nCalibration failed!");
    }
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    ax_offsetEEPROM=ax_offset;
    ay_offsetEEPROM=ay_offset;
    az_offsetEEPROM=az_offset;
    gx_offsetEEPROM=gx_offset;
    gy_offsetEEPROM=gy_offset;
    gz_offsetEEPROM=gz_offset;
    
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset+ax_initoffset); 
    Serial.print("\t");
    Serial.print(ay_offset+ay_initoffset); 
    Serial.print("\t");
    Serial.print(az_offset+az_initoffset); 
    Serial.print("\t");
    Serial.print(gx_offset+gx_initoffset); 
    Serial.print("\t");
    Serial.print(gy_offset+gy_initoffset); 
    Serial.print("\t");
    Serial.println(gz_offset+gz_initoffset); 
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    
        // wait for ready
      while (Serial.available() && Serial.read()); // empty buffer
      if (CalibResult==false) {
        while (!Serial.available()){
          Serial.println(F("If you want to store the calibrated offset values into EEPROM, press \"y\" \n"));
          delay(1500);
        }
        if (Serial.available() > 0) {
        //while (Serial.available() && Serial.read()); // empty buffer again
          inByte = Serial.read();
          #ifdef DEBUGVERBOSE
            Serial.print("Pressed key: ");Serial.println(inByte);
          #endif
        }
        if (inByte==121) { // "y"
          CalibResult=true;
        }
        else {
           Serial.println("Exiting without storing calibrated offset values into EEPROM");
           state=10;  // do nothing
        }
      }
    
      if (CalibResult==true) {
            char writeBufferCalib[2];
            writeBufferCalib[0]=(ax_offsetEEPROM+ax_initoffset)&(0xff);
            writeBufferCalib[1]=((ax_offsetEEPROM+ax_initoffset)>>8) & 0xff;
            file.write(writeBufferCalib, sizeof(writeBufferCalib) );

            writeBufferCalib[0]=(ay_offsetEEPROM+ay_initoffset)&(0xff);
            writeBufferCalib[1]=((ay_offsetEEPROM+ay_initoffset)>>8) & 0xff;
            file.write(writeBufferCalib, sizeof(writeBufferCalib) );
 
            writeBufferCalib[0]=(az_offsetEEPROM+az_initoffset)&(0xff);
            writeBufferCalib[1]=((az_offsetEEPROM+az_initoffset)>>8) & 0xff;
            file.write(writeBufferCalib, sizeof(writeBufferCalib) );

            writeBufferCalib[0]=(gx_offsetEEPROM+gx_initoffset)&(0xff);
            writeBufferCalib[1]=((gx_offsetEEPROM+gx_initoffset)>>8) & 0xff;
            file.write(writeBufferCalib, sizeof(writeBufferCalib) );

            writeBufferCalib[0]=(gy_offsetEEPROM+gy_initoffset)&(0xff);
            writeBufferCalib[1]=((gy_offsetEEPROM+gy_initoffset)>>8) & 0xff;
            file.write(writeBufferCalib, sizeof(writeBufferCalib) );
 
            writeBufferCalib[0]=(gz_offsetEEPROM+gz_initoffset)&(0xff);
            writeBufferCalib[1]=((gz_offsetEEPROM+gz_initoffset)>>8) & 0xff;
            file.write(writeBufferCalib, sizeof(writeBufferCalib) );

 

            file.close();
            Serial.println("Calibrated offset values stored in the internal Flash!");
            state++;
      }
  }
      
 if (state==3) {  //  start of the test routine
  state++;
 }
else{
  state=4;
}
 if (state==4) { // execute test routine
   if (millis() - sndRepeat > 3000) { // repeat first sound file every 3 secs
      sndRepeat=millis();
   }


  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
             // display tab-separated accel/gyro x/y/z values
            Serial.print("Acceleration- and Gyro values:\t");
            Serial.print(ax); Serial.print("\t");
            Serial.print(ay); Serial.print("\t");
            Serial.print(az); Serial.print("\t");
            Serial.print(gx); Serial.print("\t");
            Serial.print(gy); Serial.print("\t");
            Serial.println(gz);            
 
  mpuIntStatus = mpu.getIntStatus();
    if (mpuIntStatus > 60 and mpuIntStatus < 70) {
      /*
       * THIS IS A CLASH  !
       */
                  Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");
                  Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");
                  Serial.println("                       CLASH!                                    ");
                  Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");
                  Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");



                } 

 }
}
///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  
  while (i<(buffersize+discardfirstmeas+1)){
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>discardfirstmeas && i<=(buffersize+discardfirstmeas)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
  
  Serial.print("Results of measurements a/g:\t");
  Serial.print(mean_ax); Serial.print("\t");
  Serial.print(mean_ay); Serial.print("\t");
  Serial.print(mean_az); Serial.print("\t");
  Serial.print(mean_gx); Serial.print("\t");
  Serial.print(mean_gy); Serial.print("\t");
  Serial.println(mean_gz);
}

bool calibration(){
  ax_offset=-mean_ax/accel_offset_divisor;
  ay_offset=-mean_ay/accel_offset_divisor;
  //az_offset=-mean_az/accel_offset_divisor;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/gyro_offset_divisor;
  gy_offset=-mean_gy/gyro_offset_divisor;
  gz_offset=-mean_gz/gyro_offset_divisor;
  while (1){
    int ready=0;
    mpu.setXAccelOffset(ax_offset+ax_initoffset);
    mpu.setYAccelOffset(ay_offset+ay_initoffset);
    mpu.setZAccelOffset(az_offset+az_initoffset);

    mpu.setXGyroOffset(gx_offset+gx_initoffset);
    mpu.setYGyroOffset(gy_offset+gy_initoffset);
    mpu.setZGyroOffset(gz_offset+gz_initoffset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    //if (abs(mean_az)<=acel_deadzone) ready++;
    //else az_offset=az_offset-mean_az/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) {
     return true;   
     break;
    }

    Serial.print("Resulting offset calibration value a/g:\t");
  Serial.print(ax_offset+ax_initoffset); Serial.print("\t");
  Serial.print(ay_offset+ay_initoffset); Serial.print("\t");
  Serial.print(az_offset+az_initoffset); Serial.print("\t");
  Serial.print(gx_offset+gx_initoffset); Serial.print("\t");
  Serial.print(gy_offset+gy_initoffset); Serial.print("\t");
  Serial.println(gz_offset+gz_initoffset);
  loopcount=loopcount+1;
  Serial.print("Loop Cnt: ");Serial.println(loopcount);
  if (loopcount==10) {
     return false;   
     break; // exit the calibration routine if no stable results can be obtained after 10 calibration loops
    }
  }
}

void ISR_MPUInterrupt() {
    Serial.print(F("\nMPU6050 interrupt "));
}
