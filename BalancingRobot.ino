//by Graham Clifford

//Credit to Joop Brokking for my MPU6050 code. www.brokking.net/

//initializing libraries
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <SPI.h>
#include <I2Cdev.h>

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
MPU6050 mpu;
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

// pid variables
float pitch_setpoint = 0.0;
int prev_error = 0;
int int_error = 0;
int Kp = 20;
int Ki = 0;
int Kd = 0;
int loop_timer = 0;
int dt = 1000;

//receiver globals
RF24 radio(A0, 10); // radio(CE_pin,CSN_pin)
uint8_t address[][6] = {"1Node", "2Node"}; // Let these addresses be used for the pair
char data[33] = {}; //32 bytes is max transmission size, +1 bite for NULL byte at end of string.
char ack[33] = {};

//motor globals
int motor1 = 9;
int motor2 = 5;
int motor3 = 3;
int motor4 = 6;
int motorHigh[] = {0,0};
int motorLow[] = {0,0};
float motor_go = 0;

// ================================================================
// ===                   INTERRUPT DETECTION                    ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                Motor Driver Initialization               ===
// ================================================================

void motorDriverInit() {
  Serial.println("Initializing motor pins...");
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void gyroSetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

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
    mpu.setXGyroOffset(83);
    mpu.setYGyroOffset(-20);
    mpu.setZGyroOffset(-33);
    mpu.setZAccelOffset(1308); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void gyroRun(){
  if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            for(int i = 0; i < 3; i ++){
              ypr[i] = ypr[i] * 180/M_PI;
            }
        #endif
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void setup() {
    Serial.begin(38400);
    motorDriverInit();
    gyroSetup();
    gyroRun();
}

void loop() {
    // ================================================================
    // ===               Gyroscope/Accelerometer Loop               ===
    // ================================================================
    gyroRun();

    // switch the direction of the motor depending on the sign of the pitch
    if(ypr[1] < 0){
      motorHigh[0] = motor3;
      motorHigh[1] = motor1;
      motorLow[0] = motor4;
      motorLow[1] = motor2;
    } else {
      motorHigh[0] = motor4;
      motorHigh[1] = motor2;
      motorLow[0] = motor3;
      motorLow[1] = motor1;
    }

    // PID control
    int error = abs(ypr[1] - pitch_setpoint);
    motor_go = Kp * error + (Ki * int_error)*dt + Kd * (error - prev_error)/dt;

    int_error += error;
    prev_error = error;

    if (motor_go > 255) {
      motor_go = 255;
    } else if (motor_go < 0) {
      motor_go = 0;
    }

    for (int i = 0; i < 2; i ++){
      analogWrite(motorHigh[i],motor_go);
      digitalWrite(motorLow[i],LOW);
    }
    Serial.print("\tmotor go: ");Serial.println(motor_go);
    while (micros() - loop_timer < dt){;}    //Wait until the loop_timer reaches 1000us (1000Hz) before starting the next loop
    loop_timer = micros();  //Reset the loop timer
}
