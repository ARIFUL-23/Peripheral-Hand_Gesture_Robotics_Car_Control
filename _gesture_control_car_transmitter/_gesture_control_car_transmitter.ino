#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // Connect BT RX PIN to Arduino 11 PIN | Connect BT TX PIN to Arduino 10 PIN

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  
#define LED_PIN 13 

MPU6050 mpu;
bool blinkState = false;
bool dmpReady = false;  
uint8_t mpuIntStatus;  
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           
VectorFloat gravity;    
float ypr[3];           
float pitch = 0;
float roll = 0;
float yaw = 0;
int x, y;
int prevX = 0; // To track previous X position
int prevY = 0; // To track previous Y position

volatile bool mpuInterrupt = false;    

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  
  Serial.begin(38400);
  BTSerial.begin(9600);  
  
  while (!Serial); 

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(126);
  mpu.setYGyroOffset(57);
  mpu.setZGyroOffset(-69);
  mpu.setZAccelOffset(1869); 

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (!dmpReady) return;
  
  while (!mpuInterrupt && fifoCount < packetSize) {}
  
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;

    if (roll > -100 && roll < 100)
      x = map (roll, -100, 100, 0, 100);

    if (pitch > -100 && pitch < 100)
      y = map (pitch, -100, 100, 0, 100);

    Serial.print(x);
    Serial.print("\t");
    Serial.println(y);
    Serial.print("\t");

    int speed = 0;
    int turn = 0;
    
    if (y >= 55) {
      speed = map(y, 55, 75, 0, 255); 
      Serial.print("Backward");
      Serial.print("\t");
      Serial.println(speed);
      // Increase speed from 145 to 172
    } else if (y<45) {
     speed = map(y, 25, 45, 0, 255); 
      Serial.print("Forward");
      Serial.print("\t");
      Serial.println(speed);
      // Maximum speed below 127
    } 
    
    // else {
    //   speed = 0;
    //   Serial.print("Stop");
    //   Serial.print("\t");
    //   Serial.print(speed);
       // Stop if not moving forward or backward
   // }

    if (y < prevY) {
      // If hand is moved backwards, reverse speed calculation
      speed *= -1;
    }

    if (x >= 45 && x <= 55) {
      turn = 0; // No turn
      Serial.print("Stop");
      Serial.print("\t");
     
    } else if (x > 55) {
      turn = map(x, 56, 100, 0, 255); 
      Serial.print("Right");
      Serial.print("\t");
      // Turn right
    } else if (x < 45) {
      turn = map(x, 44, 0, -255, 0); 
      Serial.print("Left");
      Serial.print("\t");
     
      // Turn left
    }

    BTSerial.write(speed); // Send speed value
    BTSerial.write(turn); // Send turn value

    prevX = x; // Update previous X position
    prevY = y; // Update previous Y position
#endif

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
