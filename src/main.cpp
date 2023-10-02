
#include "Wire.h"
#include "Arduino.h"
#include "math.h"
#include <FreeRTOS.h>
#include "QuickPID.h"
#include "BluetoothSerial.h"

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps612.h"
#include "MPU6050.h" 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define motor1pin1 14
#define motor1pin2 27
#define PWMpin1 12

#define motor2pin1 32
#define motor2pin2 33
#define PWMpin2 25

void readAngles();
void angleMes();


BluetoothSerial SerialBT;

//MPU SECTION

MPU6050 mpu;

float pitch;
long velocity;

float trimPot;
float trimAngle;

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;

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
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float gyroVector[3]; 
////


// PID
float Setpoint, Input, Output;

QuickPID myPID(&Input, &Output, &Setpoint);
// PID

int16_t gyroX, gyroRate;
int16_t accY, accZ;

float accAngle;
float angleY, angleZ, gyroY, gyroZ,gg,gg1;
float pid_out;
volatile float kat_ac;
float aktualny_kat=angleY;

char Temp[5]; 
byte pos=0; //bluetooth
char  Direction;

static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;
static TaskHandle_t task_3 = NULL;



// ROZNE WARIANTY NASTAW
//float Kp = 18, Ki = 0.4, Kd = 0.8;
//float Kp = 40.2, Ki = 5, Kd =4.1;
//float Kp = 24.5, Ki = 0, Kd =0;
float Kp = 24.6, Ki = 0.1 , Kd =0.17; 

void angleMes(){
    readAngles();
  

  pitch = (ypr[1] * 180/PI); // adjust to degrees

 
}


void OdczytZyro(void *parameter){
  while(1){

  vTaskDelay(5/portTICK_PERIOD_MS);  
  }
}


void PID_algo(void *parameter){
  while(1){

  angleY=pitch; 
  Input = angleY;
  myPID.Compute();
  pid_out=Output;
  if(pid_out < 5 && pid_out > -5) pid_out = 0; 


//Serial.print(angleY);
//Serial.print("  ");        Dane na port szeregowy
//Serial.println(pid_out);
vTaskDelay(5/portTICK_PERIOD_MS); 

  }
 
}


void moveForward(int pwm){
ledcWrite(2, pwm);
ledcWrite(3, 0);
ledcWrite(0, 0);
ledcWrite(1, pwm);
}

void moveBack(int pwm){
ledcWrite(2, 0);
ledcWrite(3, pwm);
ledcWrite(0, pwm);
ledcWrite(1, 0);
}



void Bluetooth(void *parameters){
  while(1){
  if (SerialBT.available()){  
    Temp[pos] =SerialBT.read();
    if(Temp[pos]=='#'){
      Temp[pos]=0;
      pos=0;
    }
    else{
      if (pos<4) 
      pos++;
    }
    Direction=Temp[0];
  Serial.println(Direction);
  SerialBT.println(Setpoint);
  }
  vTaskDelay(50/portTICK_PERIOD_MS);
  }
}    

//MPU READ

void dmpDataReady() {
     IMUdataReady = 1;
}       

void readAngles()  {

    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
     } 
     
     else if (mpuIntStatus & 0x02) {
       
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        IMUdataReady = 0;
    }
}

void Euler()  {

    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
     
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
     } 
     
     else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180/PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/PI);

        IMUdataReady = 0;
    }
}

void Drag(){
  Serial.println(mpu.getAccelerationY());
}


/// MPU READ END





void setup(void) {
  Serial.begin(115200);
  SerialBT.begin("ESP32_ROBO");

  pinMode(motor1pin1,OUTPUT);// define pin as output
  pinMode(motor1pin2,OUTPUT);
  pinMode(motor2pin1,OUTPUT);
  pinMode(motor2pin2,OUTPUT);  
  pinMode(PWMpin2,OUTPUT); 

  ledcSetup(0,22000,8);
  ledcSetup(1,22000,8);
  ledcSetup(2,22000,8);
  ledcSetup(3,22000,8);
  ledcAttachPin(motor1pin1,0); 
  ledcAttachPin(motor1pin2,1);
  ledcAttachPin(motor2pin1,2);
  ledcAttachPin(motor2pin2,3);
  

///MPU SETUP
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  //kalibracja
  mpu.setXGyroOffset(220); //220
  mpu.setYGyroOffset(76); //76
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688); //1788

if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

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
/// MPU SETUP END
  

  Input = pitch;
  Setpoint = 0.1;

  //PID
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(-255,255);
  myPID.SetSampleTimeUs(10000);
  

  //PID on
  myPID.SetMode(myPID.Control::automatic);
  //delay(2000);



  xTaskCreate(
    OdczytZyro,    // Function that should be called
    "UltraSonicC",   // Name of the task (for debugging)
    2048,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    &task_1            // Task handle
  );

  xTaskCreate(
    PID_algo,    
    "PID",   
    4096,            
    NULL,            
    1,               
    &task_2           
  );

  xTaskCreate(
    Bluetooth,     
    "Bluetooth",    
    2048,             
    NULL,            
    1,               
    &task_3           
  );

//delay(1000);

}



void loop() {


angleMes();


 if(pid_out<-0.05){
moveBack(-pid_out);
}
if(pid_out>0.05){
moveForward(pid_out);
}

if (Direction=='U')
{
  Setpoint = 2;
}

if (Direction=='B')
{
  Setpoint = -2;
}

if (Direction=='S')
{
  Setpoint = 0.1;
}


}
