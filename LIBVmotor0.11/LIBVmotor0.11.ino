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

struct EEPROM_ADDR {
  //Running Mode
  const int MODE = 10;

  //VCV Mode Data Address
  const int VCV_IP = 11;
  const int VCV_TV = 12;
  const int VCV_RR = 13;
  const int VCV_IE = 14;
  const int VCV_PEEP = 15;

  //BPAP Mode Data Address
  const int BPAP_IP = 16;
  const int BPAP_PEEP = 17;
  const int BPAP_TP = 18;
  const int BPAP_FST = 19;
};

/*************************/
/*    CONSTANT NUMBER    */
/*************************/

//Constant to make the setting value in single-byte range (0-225)
struct SINGLE_BYTE_CONST {
  const float TV = 0.1;
  const float IE = 10.0;
  const float FST= 10.0;
};

/*************************/
/*     SETTING VALUE     */
/*************************/

struct VCV_MODE {
  int IP;
  int TV;
  int RR;
  float IE;
  int PEEP;
};

struct BPAP_MODE {
  int IP;
  int PEEP;
  int TP;
  float FST;
};

bool modeVCV=true;

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
#define mGear 139.0  //Motor's gear ratio
#define mPpr 7.0   //Number of tick per one revolution capture by the encoder
#define mPulse 4.0  //Always 4, the encoder library return 4 in 1 tick from the encoder

//VAL: PID Parameters
#define Kp 800.0    //Proportional gain
#define Ki 10.0    //Integral gain
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
float TV_MIN=0, TV_MAX=800;
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
const int brth_hold_time = 2000;

EEPROM_ADDR EEPROMaddress;
VCV_MODE VCVsetting;
BPAP_MODE BPAPsetting;
SINGLE_BYTE_CONST singleByteConst;

simplePID PiD(Kp, Ki);      //Kp, Ki, use only PI, name it to "PiD"
ramp Ramp(1.0, 2.0, 0.005); //Acceleration, Max Speed, Tolerance, name it to "Ramp"
Encoder Enc(chA, chB);      //Encoder pins, name it to "Enc"
BTS7960 motor(LPWM, RPWM);

button BREATH_BUT(BREATH_PIN, HIGH);
//button HALL1(HALL1_PIN);
//button HALL2(HALL2_PIN);

void setup() {
  Serial.begin(115200);

  //Initialize the digital input
  BREATH_BUT.init();
  pinMode(HALL1_PIN, INPUT);
  //HALL1.begin(false);
  //HALL2.begin(false);

  //I2C Setup
  Wire.begin(STATION_I2C_ADD);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent);       // I2C receive data event

  EEPROM_read(); //Get Setting from EEPROM
  BPM_Timing();  //Calc breath timing
  
  motor.init();  //Motor Setup

  //Initialize the Arms
  while(true){
    motor.setPWM(-100);
    if(!digitalRead(HALL1_PIN)) break;
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
    if(BREATH_BUT.push()) {
    	breathing=true; //Run breathing
    	BREATH_BUT.resetHold();
	}
  }else{
    if(BREATH_BUT.onHold() >= brth_hold_time) {
    	breathing=false;
    	BREATH_BUT.resetHold();
    }
    switch(venStat){
      case pushIN:
        goPos = TV2RPB;
        speedMov = speedIn;
        
//        if(millis()-timer>=timeIn || abs(goPos-aPos)<=0.04){
//          timer = millis();
//          venStat = pushOUT;
//        }
        if(abs(goPos-aPos)<=0.01) venStat = pushOUT; //reached desire position, move on
        
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
  goSpeed *= 1.4;         //Mantain the 40% speed, PiD drop to correct the error (bad tuning) or do the better PiD tuning
  cmd = PiD.cmd(goSpeed, aSpeed, dt);    //PiD generate digital control to motor
  if(abs(cmd)<=10) cmd=0;
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
  if (modeVCV)
  {
    /*New variable to OLD system for demo*/
    TV = VCVsetting.TV;
    IE = VCVsetting.IE;
    TI = round(60000.0 / (VCVsetting.RR*(1+IE)));
    /*New variable to OLD system for demo*/

    timeIn = TI;
    timeOut = (TI*IE)/2.0;
    timeRest = timeOut; 
    TV2RPB = (TV/TV_MAX)*RPB;
    speedIn = TV2RPB*1000.0/(float)timeIn;
    speedOut = -(TV2RPB*1000.0/(float)timeOut);
    Serial.print("TV%: ");
    Serial.println(TV/TV_MAX);
  }

}

//I2C Receive Data and update to EEPROM
void receiveEvent(int howMany) {

  //Read each byte from master
  modeVCV = (bool)Wire.read();
  VCVsetting.IP = Wire.read();
  VCVsetting.TV = Wire.read()/singleByteConst.TV;
  VCVsetting.RR = Wire.read();
  VCVsetting.IE = Wire.read()/singleByteConst.IE;
  VCVsetting.PEEP = Wire.read();

  BPAPsetting.IP = Wire.read();
  BPAPsetting.PEEP = Wire.read();
  BPAPsetting.TP = Wire.read();
  BPAPsetting.FST = Wire.read()/singleByteConst.FST;

  Serial.println("-------------------");
  Serial.print("Mode: ");Serial.println(modeVCV);
  Serial.print("IP: ");Serial.println(VCVsetting.IP);
  Serial.print("TV: ");Serial.println(VCVsetting.TV);
  Serial.print("RR: ");Serial.println(VCVsetting.RR);
  Serial.print("IE: ");Serial.println(VCVsetting.IE);
  Serial.print("PEEP: ");Serial.println(VCVsetting.PEEP);
  Serial.println("-------------------");

  EEPROM_update();
  BPM_Timing();
}

void EEPROM_update(){
  EEPROM.update(EEPROMaddress.MODE, (int)modeVCV);

  EEPROM.update(EEPROMaddress.VCV_IP, VCVsetting.IP);
  EEPROM.update(EEPROMaddress.VCV_TV, int(VCVsetting.TV*singleByteConst.TV));
  EEPROM.update(EEPROMaddress.VCV_RR, VCVsetting.RR);
  EEPROM.update(EEPROMaddress.VCV_IE, int(VCVsetting.IE*singleByteConst.IE));
  EEPROM.update(EEPROMaddress.VCV_PEEP, VCVsetting.PEEP);

  EEPROM.update(EEPROMaddress.BPAP_IP, BPAPsetting.IP);
  EEPROM.update(EEPROMaddress.BPAP_PEEP, BPAPsetting.PEEP);
  EEPROM.update(EEPROMaddress.BPAP_TP, BPAPsetting.TP);
  EEPROM.update(EEPROMaddress.BPAP_FST, int(BPAPsetting.FST*singleByteConst.FST));
}

void EEPROM_read(){
  modeVCV = (bool)EEPROM.read(EEPROMaddress.MODE);

  VCVsetting.IP   = EEPROM.read(EEPROMaddress.VCV_IP);
  VCVsetting.TV   = EEPROM.read(EEPROMaddress.VCV_TV)/singleByteConst.TV;
  VCVsetting.RR  = EEPROM.read(EEPROMaddress.VCV_RR);
  VCVsetting.IE   = EEPROM.read(EEPROMaddress.VCV_IE)/singleByteConst.IE;
  VCVsetting.PEEP = EEPROM.read(EEPROMaddress.VCV_PEEP);
  
  BPAPsetting.IP   = EEPROM.read(EEPROMaddress.BPAP_IP);
  BPAPsetting.PEEP = EEPROM.read(EEPROMaddress.BPAP_PEEP);
  BPAPsetting.TP   = EEPROM.read(EEPROMaddress.BPAP_TP);
  BPAPsetting.FST  = EEPROM.read(EEPROMaddress.BPAP_FST)/singleByteConst.FST;
}
