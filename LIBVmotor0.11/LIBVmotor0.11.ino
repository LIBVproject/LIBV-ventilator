/* LIBV Project
 * Website: https://bvmvent.org/
 * GitHub: https://github.com/LIBVproject/LIBV-ventilator
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

#define DEBUG false

#define STATION_I2C_ADD 8 //I2C slave station Address

//VAL: EPPROM Address
const int TV_ADD= 10, TI_ADD=11, IE_ADD=12;

enum I2C_Func{
  I2C_CMD_MOTOR,
  I2C_DATA_SETTING,
  I2C_DATA_PRESSURE,
  I2C_REQUEST_PLATEAU,
  I2C_REQUEST_POSITION,
  I2C_REQUEST_PLATEAU_AND_POSITION
};

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
#define LED_PIN 13

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
#define RPB 0.4           //Number of round to rotate to one breath inhale
float TV2RPB=0.0;
//bool pushIn=false;                  //define action of push IN or release the bag
double speedSet = 0.0;  //speed to push in and push out of the motor, in round per second (rps)

unsigned long timer=0;              //to store the timer of running, millis(), millisecond
bool isBreathing=false;     //Machine is running or not, true: help breathing & false: standby
uint8_t state = 0;
enum States{
  INHALE,       //0
  PLATEAU,      //1
  EXHALE,       //2
  REST          //3
};

struct STATE_TIMER{
  uint16_t inhale;
  uint16_t plateau = 100; //100 from MIT
  uint16_t exhale;
  uint16_t rest;
};

struct STATE_SPEED{
  double inhale;
  double exhale;
};

struct ACTUAL_RECORD{
  uint8_t Pressure;
  uint8_t Plateau;      // Platual pressure
  uint8_t Position;
};

uint8_t i2c_request_func = I2C_REQUEST_PLATEAU;


EEPROM_ADDR EEPROMaddress;
VCV_MODE VCVsetting;
BPAP_MODE BPAPsetting;
SINGLE_BYTE_CONST singleByteConst;
STATE_TIMER timing;
ACTUAL_RECORD actual;
STATE_SPEED speed;

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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
  //HALL1.begin(false);
  //HALL2.begin(false);

  //I2C Setup
  Wire.begin(STATION_I2C_ADD);        // join i2c bus with address #8
  Wire.onReceive(receiveEvent);       // I2C receive data event
  Wire.onRequest(requestEvent);       // register response event

  EEPROM_read(); //Get Setting from EEPROM
  BPM_Timing();  //Calc breath timing
  
  motor.init();  //Motor Setup

  //Initialize the Arms
  timer = millis();
  while(true){
    motor.setPWM(-200);
    if(!digitalRead(HALL1_PIN) || millis()-timer>=10000) break;
  }
  motor.setPWM(0);
  delay(300);

  motor.setPWM(100);
  delay(500);

  timer = millis();
  while(true){
    motor.setPWM(-80);
    if(!digitalRead(HALL1_PIN) || millis()-timer>=5000) break;
  }
  motor.setPWM(0);
  delay(500);
  digitalWrite(LED_PIN, 1);

  //Reset Encoder & Timer
  Enc.write(0);
  xEn = Enc.read();
  timer = millis();
  xt=millis();
}


void loop() {
  //readSerial();  //Testing cmd from Serial

  if(!isBreathing){
  	speedSet = speed.exhale;
    goPos = ZERO; //go to HOME position
  }else{
    switch(state){
      /* Inhale state*/
      case INHALE:
        goPos = TV2RPB;
        speedSet = speed.inhale;
        
        /*
        if(millis()-timer>=timeIn || abs(goPos-aPos)<=0.04){
          timer = millis();
          venStat = pushOUT;
        }
		    */

        if(abs(goPos-aPos)<=0.01) {
        	timer = millis();
        	state = PLATEAU; //reached desire position, move on
        }
        break;

      /* Meassuring Plateau Pressure */
      case PLATEAU:
        speedSet = ZERO;
        actual.Plateau = actual.Pressure;
        if (millis()-timer >= timing.plateau){
          timer = millis();
          state = EXHALE;
        }
        break;

      /* Exhal state */
      case EXHALE:
        goPos = ZERO;
        speedSet = speed.exhale;
        if(millis()-timer >= timing.exhale){
          timer = millis();
          state = REST;
        }
        break;

      /* Motor rest */
      case REST:
        if(millis()-timer>= timing.rest){
          timer = millis();
          state = INHALE;
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
  
  if(abs(goPos-aPos)<1.1 && abs(goPos-aPos)>0.05) goSpeed = speedSet;
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
    TV2RPB = inString.toFloat();
    Serial.println("Target: "+(String)TV2RPB);
  }
}

//Calculate the Breath timing from Setting
void BPM_Timing(){
  if (modeVCV){
    timing.inhale = round(60000.0 / (VCVsetting.RR*(1+VCVsetting.IE)));
    timing.exhale = (timing.inhale*VCVsetting.IE)/2.0; //exhale time is haft of it and the rest is motor's resting 
    timing.rest = timing.exhale - timing.plateau; 

    TV2RPB = min(camPosition(VCVsetting.TV), RPB);

    speed.inhale = TV2RPB*1000.0/(float)timing.inhale;
    speed.exhale = -(TV2RPB*1000.0/(float)timing.exhale);
  }

}

float camPosition(int _tv){
	switch (_tv) {
      case 100:
        return 0.165;//
        break;
        
      case 150:
        return 0.18;//
        break;
        
      case 200:
        return 0.195;//
        break;
        
	    case 250:
	      return 0.205;//
	      break;

	    case 300:
	      return 0.223;//
	    break;

	    case 350:
	      return 0.231;//
	      break;

	    case 400:
	      return 0.245;//
	      break;

	    case 450:
	      return 0.253;//
	      break;

	    case 500:
	      return 0.27;//
	      break;

	    case 550:
	      return 0.28;//
	      break;

	    case 600:
	      return 0.29;//
	      break;

	    case 650:
	      return 0.30;//
	      break;

	    case 700:
	      return 0.31;//
	      break;

	    case 750:
	      return 0.32;//
	      break;

	    case 800:
	      return 0.335;//0.335
	      break;

	    default:
	      return 0.23;
	}

}

//I2C Receive Data and update to EEPROM
void receiveEvent(int howMany) {
  uint8_t _func = Wire.read();
  if (DEBUG) Serial.println("I2C Func: "+(String)_func);
  switch (_func) {
      case I2C_CMD_MOTOR:    //Read as motor command
        isBreathing = (bool)Wire.read();
        break;

      case I2C_DATA_SETTING: // Read as setting data
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
        break;

      case I2C_DATA_PRESSURE:
        actual.Pressure = Wire.read();
        break;

      // Request function
      default:
        i2c_request_func = Wire.read();
        break;
  }
  //Read each byte from master
  if (DEBUG) {
    Serial.println("-------------------");
    Serial.print("Mode: ");Serial.println(modeVCV);
    Serial.print("IP: ");Serial.println(VCVsetting.IP);
    Serial.print("TV: ");Serial.println(VCVsetting.TV);
    Serial.print("RR: ");Serial.println(VCVsetting.RR);
    Serial.print("IE: ");Serial.println(VCVsetting.IE);
    Serial.print("PEEP: ");Serial.println(VCVsetting.PEEP);
    Serial.println("-------------------");
  }
  

  EEPROM_update();
  BPM_Timing();
}

//I2C response to master request
void requestEvent(){
  Wire.write(actual.Plateau);
  Wire.write(actual.Position);
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
  VCVsetting.RR   = EEPROM.read(EEPROMaddress.VCV_RR);
  VCVsetting.IE   = EEPROM.read(EEPROMaddress.VCV_IE)/singleByteConst.IE;
  VCVsetting.PEEP = EEPROM.read(EEPROMaddress.VCV_PEEP);
  
  BPAPsetting.IP   = EEPROM.read(EEPROMaddress.BPAP_IP);
  BPAPsetting.PEEP = EEPROM.read(EEPROMaddress.BPAP_PEEP);
  BPAPsetting.TP   = EEPROM.read(EEPROMaddress.BPAP_TP);
  BPAPsetting.FST  = EEPROM.read(EEPROMaddress.BPAP_FST)/singleByteConst.FST;
}
