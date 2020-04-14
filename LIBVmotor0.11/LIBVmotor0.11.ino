/*
 * Vatilator Motor Controller
 * 
 * >DO NOT put any thing make the loop slow or interrupted,
 *  the PiD functin need real time to calculate
 *  - Don't use delay() in 'Loop'
 *  - Try not to print alot of data with serial in 'Loop'
 *  - Don't use 'while()'
 */
 
#include <EEPROM.h>
#include <Wire.h>
#include <Encoder.h>
#include <simplePID.h>
#include <BTS7960.h>
#include <L298N.h>

#define STATION 8 //I2C slave station Address
#define HALL 4

//Motor Drive Variable
#define chA 2   //Motor's encoder channel A
#define chB 3   //Motor's encoder channel B
#define ENA 6    //L298N control pin: ENA
#define IN1 7    //L298N control pin: IN1
#define IN2 9    //L298N control pin: IN2

//Motor Spec Variable
#define mGear 78.0  //Motor's gear ratio
#define mPpr 11.0   //Number of tick per one revolution capture by the encoder
#define mPulse 4.0  //Always 4, the encoder library return 4 in 1 tick from the encoder

//PID Control Variable
#define Kp 150.0    //Gain Kp of PID
#define Ki 120.0    //Gain Ki of PID
#define ZERO 0.0    //Zero value
long t, xt, dt;                   //timer
long En, xEn;                     //Store the encoder value
float goPos=0.0, cPos=0.0, dPos;  //Store the Position
float goSpeed=0, cSpeed=0;        //Target Speed, current Speed
int cmd;                          //digital command from PID, the PWM value

//Setting Variable
#define RPB 1.3           //Number of round to rotate to one breath inhale
int TV, TI, IE;
float TV_MIN=0, TV_MAX=680;
float TV2RPB=0.0;
bool pushIn=false;                  //define action of push IN or release the bag
int timeIn, timeOut;                //time to push in and push out in millisecond
float speedIn, speedOut, speedMov;  //speed to push in and push out of the motor, in round per second (rps)
unsigned long timer=0;              //to store the timer of running, millis(), millisecond

// Initialize all Library
simplePID PiD(Kp, Ki);            //Kp, Ki, use only PI, name it to "PiD"
ramp Ramp(3.0, 2.0, 0.005); //Acceleration, Max Speed, Tolerance, name it to "Ramp"
Encoder Enc(chA, chB);      //Encoder pins, name it to "Enc"
L298N motor(IN1, IN2, ENA);

void setup() {
  Serial.begin(9600);
  pinMode(HALL, INPUT);

  //I2C Setup
  Wire.begin(STATION);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent);       // I2C receive data event


  //Get Setting from EEPROM
  TV = EEPROM.read(10)*10;    // read TV from address 10, already divided by 10 when put to store, multiply 10 back to value
  TI = EEPROM.read(11)*100;   // read TI from address 11, already divided by 100 when put to store, multiply 100 back to value
  IE = EEPROM.read(12);       //read IE from address 12
  BPM_Timing();

  //Motor Setup
  motor.init();
  while(true){
    motor.Drive(-200);
    if(digitalRead(HALL)) break;;
  }
  cPos=0.0;
  motor.Drive(0);
  
  //Wait a while
  delay(2000);
  xEn = Enc.read(); //TODO move arm to init position
  timer = millis();
  xt=millis();
  
}


void loop() {
  readSerial();
  MotorControl();
}

void MotorControl(){
  t = millis();
  dt = t-xt;
  
  /*-- Check speed and position every 10ms --*/
  if(dt >= 10){
    xt=t;                               //Store time to last timer
    En = Enc.read();                    //Get encoder value
    dPos =(En-xEn)/(mGear*mPpr*mPulse); //Calculate round for 10ms
    cPos += dPos;                       //Calculate current Position
    cSpeed = 1000.0*dPos/dt;            //Calculate speed in Round per second (RPS)
    xEn=En;                             //Store current encoder to last value
  }

  
  /*-- Correction Calculate --*/
  if(pushIn){                                             //Motor Push in to TV2RPB position
    goPos = TV2RPB;
    speedMov = speedIn;
    if(millis()-timer>=timeIn || abs(goPos-cPos)<=0.04){
      timer = millis();
      pushIn = false;
    }
  }else{                                                  //Motor push out to 0.0 position
    goPos = ZERO;
    speedMov = -speedIn;
    if(millis()-timer>=timeOut){
      timer = millis();
      pushIn = true;
      //if(digitalRead(HALL))cPos = 0.0;
    }
  }
  
  if(abs(goPos-cPos)<1.1 && abs(goPos-cPos)>0.1) goSpeed = speedMov;
  else goSpeed = Ramp.cmd(goPos, cPos, dt); 
  //goSpeed = Ramp.cmd(goPos, cPos, dt);
  goSpeed *= 1.3;         //Mantain the 30% speed, because of PiD drop to correct the error or do the better PiD tuning
  cmd = PiD.cmd(goSpeed, cSpeed, dt);    //PiD generate digital control to motor
  motor.Drive(cmd);
}

void readSerial() {
  if(Serial.available()) {
    String inString = Serial.readString();
    goPos = inString.toFloat();
    Serial.println("dspeed: "+(String)goPos);
  }
}

//Calculate the Breath timing from Setting
void BPM_Timing(){
  timeIn = TI;
  timeOut = TI*(IE*0.5); 
  TV2RPB = (TV/TV_MAX)*RPB;
  speedIn = TV2RPB*1000.0/(float)timeIn;
}

//I2C Receive Data and update to EEPROM
void receiveEvent(int howMany) {

  //Read each byte from master
  TV = Wire.read()*10;
  TI = Wire.read()*100;
  IE = Wire.read();

  //Each EEPROM address can store (0-255), make sure the data in range or it will lose the correct value
  EEPROM.update(10, TV/10);
  EEPROM.update(11, TI/100);
  EEPROM.update(12, IE);
  BPM_Timing();
  Serial.print("TV:");
  Serial.print(TV);
  Serial.print(" | TI:");
  Serial.print(TI);
  Serial.print(" | IE:");
  Serial.println(IE*0.5);
}
