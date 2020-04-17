/* LIBV Project
 * Website: https://bvmvent.org/
 * 
 * #VENTILATOR MOTOR CONTROL#
 * 
 * NOTE:
 * >DO NOT put any thing make the loop slow or interrupted,
 *  the PiD functin need real time to calculate
 *  - Don't use delay() in 'Loop'
 *  - Try not to print alot of data with serial in 'Loop'
 *  - Don't use 'while()'
 *
 * LIBRARY:
 * - Encoder: https://github.com/PaulStoffregen/Encoder
 * - simplePID: https://github.com/eTRONICSKH/SimplePID-Arduino-Library
 * - BTS7960 Driver: https://github.com/eTRONICSKH/BTS7960-Driver-Arduino-Library
 * - button: https://github.com/eTRONICSKH/SimpleButton-Arduino-Library
 */
 
#include <EEPROM.h>
#include <Wire.h>
#include <Encoder.h>
#include <simplePID.h>
#include <BTS7960.h>
#include <button.h>

#define STATION_I2C_ADD 8 //I2C slave station Address

//VAL: EPPROM Address
const int TV_ADD= 10, TI_ADD=11, IE_ADD=12;

//P-IN: Hall switch
#define HALL1_PIN 11
#define HALL2_PIN 10

//P-IN: Button
#define BREATH_PIN 12

//P-IN: Encoder
#define chA 2   //Motor's encoder channel A
#define chB 3   //Motor's encoder channel B

//P-IN: Analog sensor
#define PRES1_PIN A2
#define PRES2_PIN A3

//P-OUT: Motor driver, BTS7960
#define LPWM 5
#define RPWM 6

//VAL: Motor's spec
#define mGear 51.0  //Motor's gear ratio
#define mPpr 11.0   //Number of tick per one revolution capture by the encoder
#define mPulse 4.0  //Always 4, the encoder library return 4 in 1 tick from the encoder

//VAL: PID Parameters
#define Kp 140.0    //Proportional gain
#define Ki 120.0    //Integral gain
#define ZERO 0.0    //Zero value

//VAR: PID Variable 
long t, xt, dt;                   //timer
long En, xEn;                     //Store the encoder value
float goPos=0.0, aPos=0.0, dPos;  //Store the Position
float goSpeed=0.0, aSpeed=0.0;    //Target Speed, current Speed
int cmd;                          //digital command from PID, the PWM value

//Setting Variable
#define RPB 0.62           //Number of round to rotate to one breath inhale
int TV, TI, IE;
float TV_MIN=0, TV_MAX=680;
float TV2RPB=0.0;
//bool pushIn=false;                  //define action of push IN or release the bag
int timeIn, timeOut, timeRest;                //time to push in and push out in millisecond
float speedIn, speedOut, speedMov;  //speed to push in and push out of the motor, in round per second (rps)
unsigned long timer=0;              //to store the timer of running, millis(), millisecond

//VAR: Ven stat
int venStat=0;
const int pushIN=0;
const int pushOUT=1;
const int pushRest=2;

//VAR: Breath Button
bool breathing = false;
bool brth_hold_stat = false;
const int brth_hold_time = 3000;

simplePID PiD(Kp, Ki);      //Kp, Ki, use only PI, name it to "PiD"
ramp Ramp(3.0, 2.0, 0.005); //Acceleration, Max Speed, Tolerance, name it to "Ramp"
Encoder Enc(chA, chB);      //Encoder pins, name it to "Enc"
BTS7960 motor(LPWM, RPWM);

button BREATH_BUT(BREATH_PIN);
button HALL1(HALL1_PIN);
button HALL2(HALL2_PIN);

void setup() {
  Serial.begin(9600);

  //Initialize the digital input
  BREATH_BUT.begin(true);
  HALL1.begin(false);
  HALL2.begin(false);

  //I2C Setup
  Wire.begin(STATION_I2C_ADD);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent);       // I2C receive data event

  
  EEPROM_read(); //Get Setting from EEPROM
  BPM_Timing();  //Calc breath timing

  
  motor.init();  //Motor Setup

  //Initialize the Arms
  while(true){
    motor.setPWM(-200);
    if(HALL2.press()) break;;
  }
  motor.setPWM(0);
  delay(500);

  //Reset Encoder & Timer
  Enc.write(0);
  xEn = Enc.read();
  timer = millis();
  xt=millis();
}


void loop() {
  //readSerial();  //Testing cmd from Serial

  if(!breathing){
    goPos = ZERO; //go to HOME position
    if(BREATH_BUT.push()) breathing=true; //Run breathing
  }else{
    if(BREATH_BUT.holdTime() >= brth_hold_time) brth_hold_stat=true;
    if(brth_hold_stat && BREATH_BUT.push()){
      brth_hold_stat=false;
      breathing=false;
    }

    switch(venStat){
      case pushIN:
        goPos = TV2RPB;
        speedMov = speedIn;
        if(millis()-timer>=timeIn || abs(goPos-aPos)<=0.04){
          timer = millis();
          venStat = pushOUT;
        }
        break;
      case pushOUT:
        goPos = ZERO;
        speedMov = speedOut;
        if(millis()-timer>=timeOut){
          timer = millis();
          venStat = pushRest;
        }
        break;
      case pushRest:
        if(millis()-timer>=timeRest){
          timer = millis();
          venStat = pushIN;
        }
        break;
    }

  }

  

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
    aPos += dPos;                       //Actual Position
    aSpeed = 1000.0*dPos/dt;            //Actual speed in Round per second (RPS)
    xEn=En;                             //Passing last encoder value
  }
  
  if(abs(goPos-aPos)<1.1 && abs(goPos-aPos)>0.05) goSpeed = speedMov;
  else goSpeed = Ramp.cmd(goPos, aPos, dt); 
  //goSpeed = Ramp.cmd(goPos, aPos, dt);
  goSpeed *= 1.3;         //Mantain the 30% speed, PiD drop to correct the error (bad tuning) or do the better PiD tuning
  cmd = PiD.cmd(goSpeed, aSpeed, dt);    //PiD generate digital control to motor
  motor.setPWM(cmd);
}

void readSerial() {
  if(Serial.available()) {
    String inString = Serial.readString();
    goPos = inString.toFloat();
    Serial.println("dspeed: "+(String)goPos);
  }
}

//Alarm sound and LED, silent button to silence the alarm for period of time
void Alarm(bool _stat, int type){
  //TODO: define alarm type
  //TODO: trigger the alarm with _stat
  //TODO: silent button
}

//Calculate the Breath timing from Setting
void BPM_Timing(){
  timeIn = TI;
  timeOut = (TI*IE*0.5)/2.0;
  timeRest = timeOut; 
  TV2RPB = (TV/TV_MAX)*RPB;
  speedIn = TV2RPB*1000.0/(float)timeIn;
  speedOut = -(TV2RPB*1000.0/(float)timeOut);
}

//I2C Receive Data and update to EEPROM
void receiveEvent(int howMany) {

  //Read each byte from master
  TV = Wire.read()*10;
  TI = Wire.read()*100;
  IE = Wire.read();

  EEPROM_update();
  BPM_Timing();
  Serial.print("TV:");
  Serial.print(TV);
  Serial.print(" | TI:");
  Serial.print(TI);
  Serial.print(" | IE:");
  Serial.println(IE*0.5);
}

void EEPROM_update(){
  EEPROM.update(TV_ADD, TV/10);
  EEPROM.update(TI_ADD, TI/100);
  EEPROM.update(IE_ADD, IE);
}

void EEPROM_read(){
  TV = EEPROM.read(TV_ADD)*10;
  TI = EEPROM.read(TI_ADD)*100;
  IE = EEPROM.read(IE_ADD);
}
