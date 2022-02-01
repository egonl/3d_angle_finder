/* 
  3D Angle Finder firmware for ESP8266 and MPU6050
  
  GY-521  NodeMCU
  MPU6050 devkit 1.0
  board   Lolin         Description
  ======= ==========    ====================================================
  VCC     VU (5V USB)   Not available on all boards so use 3.3V if needed.
  GND     G             Ground
  SCL     D1 (GPIO05)   I2C clock
  SDA     D2 (GPIO04)   I2C data
  XDA     not connected
  XCL     not connected
  AD0     not connected
  INT     D8 (GPIO15)   Interrupt pin
*/

#define INTERRUPT_PIN 15 // use pin 15 on ESP8266

// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID           "To be filled in..."
#define BLYNK_DEVICE_NAME           "3D Angle Finder"
#define BLYNK_AUTH_TOKEN            "To be filled in..."
#define BLYNK_SSID                  "To be filled in..."
#define BLYNK_PSWD                  "To be filled in..."

// Comment this out to disable prints and save space
//#define BLYNK_PRINT Serial

#include <BlynkSimpleEsp8266.h>

#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

typedef struct YPR
{
  float Y;
  float P;
  float R;
} YPR;

YPR ypr, yprZero = { 0.0f , 0.0f, 0.0f };

bool hold = false;

BlynkTimer timer;

BLYNK_WRITE(V0)
{
  // V0 is set when pushing zero
  yprZero.Y = ypr.Y;
  yprZero.P = ypr.P;
  yprZero.R = ypr.R;  
}

BLYNK_WRITE(V1)
{
  // V1 is 1 when hold is enabled
  hold = param.asInt() == 1;
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
}

void myTimerEvent()
{
  Blynk.virtualWrite(V2, millis() / 1000);
  // Define additional datastreams in Blynk to hold yaw, pitch and roll.
  Blynk.virtualWrite(V4, getAngleDifference(yprZero.Y, ypr.Y));
  Blynk.virtualWrite(V5, getAngleDifference(yprZero.P, ypr.P));
  Blynk.virtualWrite(V6, getAngleDifference(yprZero.R, ypr.R));  
}

float getAngleDifference(float a1, float a2)
{
    float d = a2 - a1;
    d += (d > 180) ? -360 : (d < -180) ? 360 : 0;
    return d;
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void ICACHE_RAM_ATTR dmpDataReady()
{
    mpuInterrupt = true;
}

void mpu_setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // egonl: run IMU_zero sample to those offsets.
  //   Output: X acceleration, Y acceleration, Z acceleration, X gyro, Y gyro, and Z gyro. E.g.:
  //   [-2627,-2626] --> [-12,3] [307,308] --> [-9,5] [1075,1075] --> [16375,16385] [136,137] --> [0,3] [47,48] --> [-2,2] [-63,-62] --> [-2,1]
  mpu.setZAccelOffset(1075);
  mpu.setXGyroOffset(136);
  mpu.setYGyroOffset(47);
  mpu.setZGyroOffset(-63);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

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
}

void setup(void)
{
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, BLYNK_SSID, BLYNK_PSWD);
  timer.setInterval(500L, myTimerEvent); 
  mpu_setup();
}

void mpu_loop()
{
  if (hold) return;
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }
  
  // check for DMP data ready interrupt (this should happen frequently)
  if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll((float*) &ypr, &q, &gravity);
    ypr.Y *= 180/M_PI;
    ypr.P *= 180/M_PI;
    ypr.R *= 180/M_PI;
    
    Serial.print("ypr\t");
    Serial.print(ypr.Y);
    Serial.print("\t");
    Serial.print(ypr.P);
    Serial.print("\t");
    Serial.println(ypr.R);
  }
}

void loop(void)
{
  Blynk.run();
  timer.run();
  mpu_loop();
}
