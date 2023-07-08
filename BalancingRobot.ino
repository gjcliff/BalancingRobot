//by Graham Clifford

//Credit to Joop Brokking for my MPU6050 code. www.brokking.net/

//initializing libraries
//#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include "RF24.h"
#include "ArduPID.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
// #include "I2Cdev.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h

//PID globals
ArduPID pid_roll;
ArduPID pid_pitch;
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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

float roll_setpoint = 0.0;
float pitch_setpoint = 0.0;
float p = 11;
float i = 0;
float d = 10;
int check;

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

// join I2C bus (I2Cdev library doesn't do this automatically

// ================================================================
// ===                  Radio Initialization                 ===
// ================================================================
void radioInit() {
  while (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
  }
  Serial.println(F("radio initialized"));
  radio.enableAckPayload(); //Allow ack(nowledgement) payloads. This will let us send data back to transmitter without manually changing the radio modes on both Arduinos.
  radio.enableDynamicPayloads(); //Need for sending ack payloads on pipes other than 0 and 1. This enables it on all pipes.
  radio.setRetries(5, 15);
  radio.openWritingPipe(address[1]);     // always uses pipe 0
  radio.openReadingPipe(1, address[0]); // using pipe 1

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the number of bytes we need to transmit a float

  // additional setup specific to the node's role
  radio.startListening(); // put radio in RX mode
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

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

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

void transmitString(String s) {
  uint8_t len = s.length() + 1;
  char data[len];
  s.toCharArray(data, len);
  if (radio.write(&data, sizeof(data))) {
  }
}

void setup() {
    Serial.begin(115200);
    //radioInit();
    motorDriverInit();
    //Serial.println("Waiting for transmission from controller to begin MPU6050 intitialization");  //wait to receive a tranmission from the controller before initializing the gyroscope accelerometer
    gyroSetup();
    gyroRun();

    //   pid_p.begin(&angle_pitch_output, &thrust_mod, &pitch_set, p, i, d);
    //   pid_r.begin(&angle_roll_output, &thrust_mod, &roll_set, p, i, d);
    //   pid_az.begin(&acc_z, &thrust_mod, &acc_z_set, p, i, d);

    pid_roll.begin(&ypr[1], &motor_go, &roll_setpoint, p, i, d);
    pid_roll.setOutputLimits(-255, 255);
    pid_roll.setBias(0);
    pid_roll.setWindUpLimits(-5, 5);
    pid_roll.setSampleTime(10);
    pid_roll.start();
    }

void loop() {
    // ================================================================
    // ===               Gyroscope/Accelerometer Loop               ===
    // ================================================================
    gyroRun();

    //Serial.print("pitch: "); Serial.print(ypr[1]); Serial.print("\troll: "); Serial.print(ypr[2]);

    //   if (radio.available()) {
    //     int bytes = radio.getPayloadSize();
    //     radio.read(&data, bytes);
    //     radio.writeAckPayload(0, &ack, bytes);
    //   }

    // ================================================================
    // ===                        Flight Loop                       ===
    // ================================================================
    //Leveling Logic

    //this if loop switches the direction of the motor depending on if the
    //roll angle is positive or negative. The 0 degree angle is pointing straight up here.
    pid_roll.compute();

    if(ypr[1] < 0){
      motorHigh[0] = motor3;
      motorHigh[1] = motor1;
      motorLow[0] = motor4;
      motorLow[1] = motor2;
    } else {
      motor_go = -motor_go;
      motorHigh[0] = motor4;
      motorHigh[1] = motor2;
      motorLow[0] = motor3;
      motorLow[1] = motor1;
    }

    for (int i = 0; i < 2; i ++){
      // if (check == 1){
      //   motor_go += 40;
      // }
      analogWrite(motorHigh[i],motor_go);
      digitalWrite(motorLow[i],LOW);
    }
    // motor_go[1] += motor_go_mod;

    // pid_pitch.compute();
    // motor_go[0] += motor_go_mod;
    // motor_go[1] -= motor_go_mod;

    //Serial.println("motor_go 0: " + motor_go[0] + "motor_go 1: " + motor_go[1] + "motor_go_mod 0: " + motor_go_mod[0] + "motor_go_mod 1: " + motor_go_mod[1]);
    // for (int i = 0; i < 2; i ++){
    //analogWrite(motor1, motor_go);
    Serial.print("\tmotor go: ");Serial.println(motor_go);
      // Serial.print("\tmotor go mod: ");Serial.println(motor_go_mod);
    // }
    //Serial.print("thrust mod: "); Serial.println(thrust_mod);
    // for (int i = 0; i < 2; i++) {
    //     analogWrite(motor[i], thrust[i]);
    //     //Serial.print("thrust "); Serial.print(i); Serial.print(": "); Serial.println(thrust[0]);
    // }
    // int i = 0;
    // boolean yes = false;
    // while (micros() - loop_timer < 4000){    //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
    //     if(i > 0) {
    //         Serial.println("i is greater than 0");
    //     }
    // }
    // loop_timer = micros();  //Reset the loop timer
    delay(10);
}
