/* LIBV Project
 * Website: https://bvmvent.org/
 * 
 * #VENTILATOR SYSTEM MONITORING#
 *
 * LIBRARY:
 * - Encoder: https://github.com/PaulStoffregen/Encoder
 * - simplePID: https://github.com/eTRONICSKH/SimplePID-Arduino-Library
 * - BTS7960 Driver: https://github.com/eTRONICSKH/BTS7960-Driver-Arduino-Library
 * - button: https://github.com/eTRONICSKH/SimpleButton-Arduino-Library
 */

/*
 * General Setting Parameters                           Unit
 * - CMV: Continuous Mandatory Ventilation Mode
 * - VCV: Volume Controlled Ventilation Mode
 * - PRVC: Pressure Regulated Volume Control Mode
 * - BPAP: Belevel Pos. Airway Pressure
 * - IP: Inspiratory Pressure                           cm.H2O
 * - TV: Tidal Volume                                   ml
 * - BPM: Breaths Per Minute
 * - IE: Inspiratory/Expiratory ratio
 * - PEEP: Positive End-Expiratory Pressure             cm.H20
 * - PIP: Peak Inspiratory Pressure 
 * - TP: Trigger Pressure                               cm.H20
 * - FST: Failsafe Time ratio
 * - RR: Resporatory Rate 								Breathe Per minutes
 *
 */

#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <button.h>
#include <SparkFun_MS5803_I2C.h>
#include <SFE_BMP180.h>

/*------------------------------- PARAMETERS ------------------------------*/

/*************************/
/*      I2C ADDRESS      */
/*************************/
#define I2C_ADDR_MOTOR 8
#define I2C_ADDR_LCD 0x27

enum I2C_Func{
  I2C_CMD_MOTOR,
  I2C_DATA_SETTING
};

/*************************/
/*        SETTING        */
/*************************/

struct DEFAULT_SETTING {
  const int IP = 15;
  const int TV = 400;
  const int RR = 10;
  const int IE = 1;
  const int PEEP = 5;
  const int TP = 2;
  const int FST = 1;
} defaultSetting;

struct MAX_SETTING {
  const int IP = 60;
  const int TV = 800;
  const int RR = 30;
  const int IE = 3;
  const int PEEP = 15;
  const int TP = 14;
  const int FST = 3;
} maxSetting;

struct MIN_SETTING {
  const int IP = 0;
  const int TV = 100;
  const int RR = 10;
  const int IE = 1;
  const int PEEP = 5;
  const int TP = 2;
  const int FST = 1;
} minSetting;


/*************************/
/*    EEPROM ADDRESS     */
/*************************/

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
} EEPROMaddress;

/*************************/
/*    CONSTANT NUMBER    */
/*************************/
//Constant to make the setting value in single-byte range (0-225)
struct SINGLE_BYTE_CONST {
  const float TV = 0.1;
  const float IE = 10.0;
  const float FST= 10.0;
} singleByteConst;

//Each step changing of rotary encoders
struct ROTARY_STEP_CHANGE {
  const int IP = 5;
  const int TV = 50;
  const int RR = 2;
  const float IE = 0.5;
  const int PEEP = 1;
  const int TP = 1;
  const float FST = 0.5;
} stepChange;


/*************************/
/*        I/O PIN        */
/*************************/
//P-IN: Rotary encoder
struct ROTARY_PIN {
  const int CLK = 2;
  const int DT = 3;
  const int SW = 4;
} rotaryPin;

//P-IN: Button on control panel
struct BUTTON_PIN {
  const int SET = 5;    //Save adjustment setting button pin
  const int CANCEL = 6;  //Cancel adjustment saving button pin
  const int SELECT = 7;  //TV adjustment button pin
  const int TI = 8;      //TI adjustment button pin
  const int IE = 9;      //IE adjustment button pin
  const int SILENT = A0; //Silence the alarm button pin
  const int BREATHE = A1;
} buttonPin;

//P-IN: Hall switch sensor
#define HALL1_PIN 11
#define HALL2_PIN 10

//P-OUT: Alarm
#define BUZZER_PIN 12      //Buzzer pin
#define LED_PIN 13         //LED Alarm flash pin

/*************************/
/*   LCD SPECIFICATION   */
/*************************/
#define LCD_DISP_ROW 4
#define LCD_DISP_COLUMN 20


/*--------------------------------- VARIABLE ------------------------------*/

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

struct ROTARY_VALUE {
  long Value = 0;
  long lastValue = 0;
  int Change = 0;
};

/*************************/
/*    DISPLAY ON LCD     */
/*************************/

struct ACTUAL_DISPLAY{
  float pressureRead;
  float capturePIP;
  float capturePEEP;
  float PIP;
  float PEEP;
};

enum  screenID {
  scr_PowerON,
  scr_Setting_VCV,
  scr_Setting_BPAP,
  scr_VCVSetting_IP,
  scr_VCVSetting_TV,
  scr_VCVSetting_RR,
  scr_VCVSetting_IE,
  scr_VCVSetting_PEEP,
  scr_VCVDisplay_Actual,
  scr_VCVDisplay_Setting,
  scr_VCVAlarm,
  scr_BPAPSetting_IP,
  scr_BPAPSetting_PEEP,
  scr_BPAPSetting_TP,
  scr_BPAPSetting_FST,
  scr_BPAPDisplay_Actual,
  scr_BPAPDisplay_Setting,
  scr_BPAPlarm
};

struct TIMING{
  const long main_scr_timout = 500;        // back to main screen timout, after no change for 30s
  unsigned long main_scr_timer = 0;        // Reset at Beep function
} timing;

uint8_t Screen=scr_PowerON;

bool modeVCV=true;
bool inSetting=false;       // true: display setting adjusment, false: save setting and display current setting.
bool disp_modeSet=true; 	  //Screen display when start setup mode, true: start with chosing VCV or BPAP, false: go to element setup
bool disp_modeVCV=true; 	  //Screen display mode for setup the elements, true: VCV and false: BPAP
bool disp_select=false;		  //Select the element to adjust.
bool disp_change=false;     //rotary change status.
bool isAlarm=false;

bool isBreathing=false;     //Machine is running or not, true: help breathing & false: standby
unsigned long breathBTtimer = 0;

bool pressureSensor = false;

/*-------------------------- STRUCTURE VARIABLES ----------------------------*/

/*************************/
/*  Variable Structure   */
/*************************/
VCV_MODE VCVsetting, VCVdisplay;
BPAP_MODE BPAPsetting, BPAPdisplay;
ROTARY_VALUE rotary;
ACTUAL_DISPLAY actual;

/*------------------------------ New Obj ------------------------------*/

LiquidCrystal_I2C lcd(I2C_ADDR_LCD, LCD_DISP_COLUMN, LCD_DISP_ROW);
Encoder rotaryEnc(rotaryPin.DT, rotaryPin.CLK);
//MS5803 pressure(ADDRESS_LOW);//  ADDRESS_HIGH = 0x76  or  ADDRESS_LOW  = 0x77
SFE_BMP180 BMP180;

button setBT(buttonPin.SET, HIGH);
button cancelBT(buttonPin.CANCEL, HIGH);
button selectBT(buttonPin.SELECT, HIGH);
button silentBT(buttonPin.SILENT, HIGH);
button breatheBT(buttonPin.BREATHE, HIGH);

button switch1(HALL1_PIN, HIGH);
button swithc2(HALL2_PIN, HIGH);

void setup() {
  Serial.begin (115200);

  /* Initialize button */
  setBT.init();
  cancelBT.init();
  selectBT.init();
  silentBT.init();
  breatheBT.init();

  /*Initialize limit switch */
  switch1.init();
  swithc2.init();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, 0);
  digitalWrite(LED_PIN, 0);

  /*  Initialize LCD   */
  lcd.begin();
  lcd.backlight();

  EEPROM_read();
  SETTING2DISPLAY();
  wireData(I2C_ADDR_MOTOR, I2C_DATA_SETTING);

  startUpScreen();

  /* Pressure sensor */
  pressureSensor = pressureBEGIN();
  if (!pressureSensor){
    //pressure sensor fail, Alarm
    while(1){
      Beep(3, 400);
      if (cancelBT.push()) break;  //Skip pressure sensor
    }
  }
}

void loop() {
  /*  Breathe button action
   * - Push to run (start to move with current setting data)
   * - Hold for 2 seconds to stop (return arms to home position)
   */
  if (!isBreathing && breatheBT.push()){
    breatheBT.resetHold();
    Beep(3, 100);
    isBreathing = true; // Run the motor
    wireData(I2C_ADDR_MOTOR, I2C_CMD_MOTOR);
    Screen = modeVCV? scr_VCVDisplay_Actual:scr_BPAPDisplay_Actual;
  }else if (isBreathing && breatheBT.onHold()>=2000){
    breatheBT.resetHold();
    Beep(3, 100);
    isBreathing = false; // Run the motor
    wireData(I2C_ADDR_MOTOR, I2C_CMD_MOTOR);
  }

  /* Read Pressure
   * - Capture PIP and PEEP for actual displays
   */
  bool pressurStatus= pressureCMH2O(actual.pressureRead); //pressure in cmH2O
  
  //record PIP and PEEP pressure
  if (actual.pressureRead>actual.capturePIP) actual.capturePIP = actual.pressureRead;
  if (actual.pressureRead<actual.capturePEEP) actual.capturePEEP = actual.pressureRead;

  //Capture PIP when Arms arrived at Home position
  if (switch1.push() || swithc2.push()){
    actual.PIP = actual.capturePIP;
    actual.capturePIP = 0.0;
  }

  //Capture PEEP when Arms left home position
  if (switch1.release() || swithc2.release()){
    actual.PEEP = actual.capturePEEP;
    actual.capturePEEP = 40.0;
  }

  /* Back to main screen after timout */
  if (millis()-timing.main_scr_timer >= timing.main_scr_timout) {
    if (isBreathing) {
      Screen = modeVCV ? scr_VCVDisplay_Actual : scr_BPAPDisplay_Actual;
    }else{
      Screen = modeVCV ? scr_VCVDisplay_Setting : scr_BPAPDisplay_Setting;
    }
  }

  /*  Control panel action
   *
   */

  rotary.Value = rotaryEnc.read()/2;

  //Setting Display
  if(inSetting){
  	if(disp_modeSet){	//Select mode for setting
  		Screen = disp_modeVCV ? scr_Setting_VCV : scr_Setting_BPAP; //Set display Screen
  		if (selectBT.push()){	//Switch selected mode
  			Beep();
  			disp_modeVCV = !disp_modeVCV;	//Switching select mode between VCV & BPAP
  		}
  		
  		if(setBT.push()){ //Chose mode, move to elements			
  			Beep();
  			disp_modeSet = false;	//Jump to element adjusting
  			Screen = disp_modeVCV ? scr_VCVSetting_IP : scr_BPAPSetting_IP; //Start screen at first element
  			delay(300);
  		}

  		else if(cancelBT.push()) { //Back to current current setting display
  			Beep();
        Screen = disp_modeVCV ? scr_VCVDisplay_Actual : scr_BPAPDisplay_Actual; 
  			inSetting = false;	//Cancel current adjusting, jump to last setting display
  		}

  	}else{	//After mode is selected, elements adjusting
  		if(disp_modeVCV){	//Adjust VCV elements
  			if (selectBT.push()){
  				Beep();
  				SETTING2DISPLAY(); //Transfer setting data to display, incase data is not SET before moving to the next element
  				Screen++;	//Arrow move to next element
  				if (Screen > scr_VCVSetting_PEEP) Screen = scr_VCVSetting_IP; //After last element, move arrow to first element
  			}

  			switch (Screen) {
  			    case scr_VCVSetting_IP:
  			      if (rotary.Value != rotary.lastValue){ //Rotary changing Event
  						rotary.Change = rotary.Value - rotary.lastValue; //Record rotoray change
  						rotary.lastValue = rotary.Value; //Reset rotary change
  						VCVdisplay.IP += rotary.Change*stepChange.IP; //Changing value with rotary move, each step change by stepChange value
  						if(inRange(VCVdisplay.IP, minSetting.IP, maxSetting.IP)) Beep(); //Beep only when data in limit range.
  						disp_change = true; //sth has changed.
  				  }
  			      VCVdisplay.IP = constrain(VCVdisplay.IP, minSetting.IP, maxSetting.IP); //Keep value in range (Minimum - Maximum)
  			      break;

  			    case scr_VCVSetting_TV:
  			      if (rotary.Value != rotary.lastValue){ //Rotary changing Event
  						rotary.Change = rotary.Value - rotary.lastValue; //Record rotoray change
  						rotary.lastValue = rotary.Value; //Reset rotary change
  						VCVdisplay.TV += rotary.Change*stepChange.TV; //Changing value with rotary move, each step change by stepChange value
  						if(inRange(VCVdisplay.TV, minSetting.TV, maxSetting.TV)) Beep(); //Beep only when data in limit range.
  						disp_change = true; //sth has changed.
  				  }
  			      VCVdisplay.TV = constrain(VCVdisplay.TV, minSetting.TV, maxSetting.TV); //Keep value in range (Minimum - Maximum)
  			      break;

  			    case scr_VCVSetting_RR:
  			      if (rotary.Value != rotary.lastValue){ //Rotary changing Event
  						rotary.Change = rotary.Value - rotary.lastValue; //Record rotoray change
  						rotary.lastValue = rotary.Value; //Reset rotary change
  						VCVdisplay.RR += rotary.Change*stepChange.RR; //Changing value with rotary move, each step change by stepChange value
  						if(inRange(VCVdisplay.RR, minSetting.RR, maxSetting.RR)) Beep(); //Beep only when data in limit range.
  						disp_change = true; //sth has changed.
  				  }
  			      VCVdisplay.RR = constrain(VCVdisplay.RR, minSetting.RR, maxSetting.RR); //Keep value in range (Minimum - Maximum)
  			      break;

  			    case scr_VCVSetting_IE:
  			      if (rotary.Value != rotary.lastValue){ //Rotary changing Event
  						rotary.Change = rotary.Value - rotary.lastValue; //Record rotoray change
  						rotary.lastValue = rotary.Value; //Reset rotary change
  						VCVdisplay.IE += rotary.Change*stepChange.IE; //Changing value with rotary move, each step change by stepChange value
  						if(inRange(VCVdisplay.IE, minSetting.IE, maxSetting.IE)) Beep(); //Beep only when data in limit range.
  						disp_change = true; //sth has changed.
  				  }
  			      VCVdisplay.IE = constrain(VCVdisplay.IE, minSetting.IE, maxSetting.IE); //Keep value in range (Minimum - Maximum)
  			      break;

  			    case scr_VCVSetting_PEEP:
  			      if (rotary.Value != rotary.lastValue){ //Rotary changing Event
  						rotary.Change = rotary.Value - rotary.lastValue; //Record rotoray change
  						rotary.lastValue = rotary.Value; //Reset rotary change
  						VCVdisplay.PEEP += rotary.Change*stepChange.PEEP; //Changing value with rotary move, each step change by stepChange value
  						if(inRange(VCVdisplay.PEEP, minSetting.PEEP, maxSetting.PEEP)) Beep(); //Beep only when data in limit range.
  						disp_change = true; //sth has changed.
  				  }
  			      VCVdisplay.PEEP = constrain(VCVdisplay.PEEP, minSetting.PEEP, maxSetting.PEEP); //Keep value in range (Minimum - Maximum)
  			      break;

  			    default:
  			      Screen = scr_VCVSetting_IP;
  			   	  break;
  			}
  		}else{	//Adjust BPAP elements
  			if (selectBT.push()){
  				Beep();
  				SETTING2DISPLAY(); //Transfer setting data to display, incase data is not SET before moving to the next element
  				Screen++;	//Arrow move to next element
  				if (Screen > scr_BPAPSetting_FST) Screen = scr_BPAPSetting_IP; //After last element, move arrow to first element
  			}

  			switch (Screen) {
  			    case scr_BPAPSetting_IP:
  			      	if (rotary.Value != rotary.lastValue){ //Rotary changing Event
  						rotary.Change = rotary.Value - rotary.lastValue; //Record rotoray change
  						rotary.lastValue = rotary.Value; //Reset rotary change
  						BPAPdisplay.IP += rotary.Change*stepChange.IP; //Changing value with rotary move, each step change by stepChange value
  						if(inRange(BPAPdisplay.IP, minSetting.IP, maxSetting.IP)) Beep(); //Beep only when data in limit range.
  						disp_change = true; //sth has changed.
  				  	}
  			      	BPAPdisplay.IP = constrain(BPAPdisplay.IP, minSetting.IP, maxSetting.IP); //Keep value in range (Minimum - Maximum)
  			      	break;

  			    case scr_BPAPSetting_PEEP:
	  				if (rotary.Value != rotary.lastValue){ //Rotary changing Event
  						rotary.Change = rotary.Value - rotary.lastValue; //Record rotoray change
  						rotary.lastValue = rotary.Value; //Reset rotary change
  						BPAPdisplay.PEEP += rotary.Change*stepChange.PEEP; //Changing value with rotary move, each step change by stepChange value
  						if(inRange(BPAPdisplay.PEEP, minSetting.PEEP, maxSetting.PEEP)) Beep(); //Beep only when data in limit range.
  						disp_change = true; //sth has changed.
  				  	}
  			      	BPAPdisplay.PEEP = constrain(BPAPdisplay.PEEP, minSetting.PEEP, maxSetting.PEEP); //Keep value in range (Minimum - Maximum)
  			      	break;

  			    case scr_BPAPSetting_TP:
	  			    if (rotary.Value != rotary.lastValue){ //Rotary changing Event
  						rotary.Change = rotary.Value - rotary.lastValue; //Record rotoray change
  						rotary.lastValue = rotary.Value; //Reset rotary change
  						BPAPdisplay.TP += rotary.Change*stepChange.TP; //Changing value with rotary move, each step change by stepChange value
  						if(inRange(BPAPdisplay.TP, minSetting.TP, maxSetting.TP)) Beep(); //Beep only when data in limit range.
  						disp_change = true; //sth has changed.
  				  	}
  			      	BPAPdisplay.TP = constrain(BPAPdisplay.TP, minSetting.TP, maxSetting.TP); //Keep value in range (Minimum - Maximum)
  			      	break;

  			    case scr_BPAPSetting_FST:
	  			    if (rotary.Value != rotary.lastValue){ //Rotary changing Event
  						rotary.Change = rotary.Value - rotary.lastValue; //Record rotoray change
  						rotary.lastValue = rotary.Value; //Reset rotary change
  						BPAPdisplay.FST += rotary.Change*stepChange.FST; //Changing value with rotary move, each step change by stepChange value
  						if(inRange(BPAPdisplay.FST, minSetting.FST, maxSetting.FST)) Beep(); //Beep only when data in limit range.
  						disp_change = true; //sth has changed.
  				  	}
  			      	BPAPdisplay.FST = constrain(BPAPdisplay.FST, minSetting.FST, maxSetting.FST); //Keep value in range (Minimum - Maximum)
  			      	break;

  			    default:
  			      Screen = scr_BPAPSetting_IP;
  			      break;
  			}
  		}

  		//Save selected element's data
  		if (setBT.push()){
        if(disp_change){
          setBT.resetHold();
          Beep();
          setBT.resetHold();
          DISPLAY2SETTING();  //Transfer display data to setting data
          EEPROM_update();  //Update data to memory
          disp_change = false; //Change has been saved, nth changed.
        }
  		}else if(cancelBT.push()){	//Back to Selecting mode
  			Beep();
  			if (disp_change){
  				SETTING2DISPLAY();		//Cancel unsaved adjusting, Display saved setting
  				disp_change = false;    //Reset changing state
  			}else disp_modeSet = true;	//Jump back to Select Mode
  		}

  		//Confirm to use the setting, exit setting screen to display
  		if (setBT.onHold()>=1500){
        setBT.resetHold();
  			Beep(2,200);
  			wireData(I2C_ADDR_MOTOR, I2C_DATA_SETTING);
        Screen = modeVCV? scr_VCVDisplay_Actual:scr_BPAPDisplay_Actual;
  			inSetting = false; //Jump to display current setting
  		}

  	}
  	
  }else{

  	/*Working space*/

  	rotary.lastValue = rotary.Value; //Reset rotary change accidently
  	//TODO : display current setting
  	if (modeVCV){
  		if (isBreathing){

        //TODO: check the alarm

        /* Alarm off */
  			if (isAlarm){
  				//TODO: Trigger Alarm function
  				//TODO: if select is pressed, show current setting
          if (selectBT.push()){
            Beep();
            Screen++;
            if (Screen > scr_VCVAlarm) Screen = scr_VCVDisplay_Actual;
          }

  			}else{
          /* Switch screen */
          if (selectBT.push()){
            Beep();
            Screen++;
            if (Screen > scr_VCVDisplay_Setting) Screen = scr_VCVDisplay_Actual;
          }
        }

  		}else{
  			Screen = scr_VCVDisplay_Setting;
  		}

  	}else{

  		if (isBreathing){
  			//TODO: Display Actual data
  			//TODO: Checking Alarm 
  			if (isAlarm){
  				//TODO: Trigger Alarm function
  				//TODO: if select is pressed, show current setting
  			}
  		}else{
  			Screen = scr_BPAPDisplay_Setting;
  		}

  	}

  	//Go to setting: press Cancel
  	if (cancelBT.push()){
  		Beep(2, 200);
  		inSetting = true;		//Jump to Setting
  		disp_modeSet = false;	//Jump to change current setting
  	}
  }

  lcdDiplay(Screen);
}

bool pressureBEGIN(){
  return BMP180.begin();  //return sensor initialize status
}

bool pressureCMH2O(float &_pressure){
  bool _status = false;
  if (pressureSensor){
    const float atmospheric_pressure = 1013.25; //mbar
    char status;
    double T,P;
    status = BMP180.startTemperature();
    if (status != 0){
      delay(status); // Wait for the measurement to complete
      status = BMP180.getTemperature(T);
      if (status != 0){
        status = BMP180.startPressure(3); //3 highest precision
        if (status != 0){
          delay(status); // Wait for the measurement to complete
          status = BMP180.getPressure(P,T);
          if (status != 0){
            _pressure = (P - atmospheric_pressure)*1.02; //cmH2O = mbar * 1.02
            _status = true; //no error
          }
        }
      }
    }
  }
  
  return _status;
}

void startUpScreen(){
  Screen=scr_PowerON;
  lcdDiplay(Screen);
  EEPROM_read();
  Beep(2, 200);
  delay(2000);
  Beep();
}

void lcdDiplay(uint8_t _screen){
  
  struct LCD_ROW {
  String R1;
  String R2;
  String R3;
  String R4;
  } dispRow;

  //Make IE display on LCD like, 1 or 1.5 or 2
  String str_vcv_ie = (String)VCVdisplay.IE;
  str_vcv_ie[3]=' ';
  if((int)(VCVdisplay.IE*10)%10 == 0){
  	str_vcv_ie[1]=' ';
  	str_vcv_ie[2]=' ';
  }

  //Make FST display on LCD like, 1 or 1.5 or 2
  String str_bpap_fst = (String)BPAPdisplay.FST;
  str_bpap_fst[3]='s';
  if((int)(BPAPdisplay.FST*10)%10 == 0){
  	str_bpap_fst[1]='s';
  	str_bpap_fst[2]=' ';
  	str_bpap_fst[3]=' ';
  }

  switch (_screen) {
      case scr_PowerON:
        //           -----------------------
        dispRow.R1 = "  LIBV OPEN SOURCE  ";
        dispRow.R2 = " VENTILATOR PROJECT ";
        dispRow.R3 = "                    ";
        dispRow.R4 = "   www.bvmvent.org  ";
        //           -----------------------
        break;

      case scr_Setting_VCV:
        //           -----------------------
        dispRow.R1 = "    Select Mode:    ";
        dispRow.R2 = "                    ";
        dispRow.R3 = "   >VCV     BPAP    ";
        dispRow.R4 = "                    ";
        //           -----------------------
        break;

      case scr_Setting_BPAP:
        //           -----------------------
        dispRow.R1 = "    Select Mode:    ";
        dispRow.R2 = "                    ";
        dispRow.R3 = "    VCV    >BPAP    ";
        dispRow.R4 = "                    ";
        //           -----------------------
        break;

      case scr_VCVSetting_IP:
        //           -----------------------
        dispRow.R1 = "VCV Setting:        ";
        dispRow.R2 = ">IP "+(String)VCVdisplay.IP+"cm            ";
        dispRow.R3 = " TV " + (String)VCVdisplay.TV + "ml  RR   "+(String)VCVdisplay.RR+"  ";
        dispRow.R4 = " IE 1:"+str_vcv_ie+" PEEP "+ (String)VCVdisplay.PEEP +"cm";
        //           -----------------------
        break;

      case scr_VCVSetting_TV:
        //           -----------------------
        dispRow.R1 = "VCV Setting:        ";
        dispRow.R2 = " IP "+(String)VCVdisplay.IP+"cm            ";
        dispRow.R3 = ">TV " + (String)VCVdisplay.TV + "ml  RR   "+(String)VCVdisplay.RR+"  ";
        dispRow.R4 = " IE 1:"+str_vcv_ie+" PEEP "+ (String)VCVdisplay.PEEP +"cm";
        //           -----------------------
        break;

      case scr_VCVSetting_RR:
        //           -----------------------
        dispRow.R1 = "VCV Setting:        ";
        dispRow.R2 = " IP "+(String)VCVdisplay.IP+"cm            ";
        dispRow.R3 = " TV " + (String)VCVdisplay.TV + "ml >RR   "+(String)VCVdisplay.RR+"  ";
        dispRow.R4 = " IE 1:"+str_vcv_ie+" PEEP "+ (String)VCVdisplay.PEEP +"cm";
        //           -----------------------
        break;

      case scr_VCVSetting_IE:
        //           -----------------------
        dispRow.R1 = "VCV Setting:        ";
        dispRow.R2 = " IP "+(String)VCVdisplay.IP+"cm            ";
        dispRow.R3 = " TV " + (String)VCVdisplay.TV + "ml  RR   "+(String)VCVdisplay.RR+"  ";
        dispRow.R4 = ">IE 1:"+str_vcv_ie+" PEEP "+ (String)VCVdisplay.PEEP +"cm";
        //           -----------------------
        break;

      case scr_VCVSetting_PEEP:
        //           -----------------------
        dispRow.R1 = "VCV Setting:        ";
        dispRow.R2 = " IP "+(String)VCVdisplay.IP+"cm            ";
        dispRow.R3 = " TV " + (String)VCVdisplay.TV + "ml  RR   "+(String)VCVdisplay.RR+"  ";
        dispRow.R4 = " IE 1:"+str_vcv_ie+">PEEP "+ (String)VCVdisplay.PEEP +"cm";
        //           -----------------------
        break;

      case scr_VCVDisplay_Actual:
        //           -----------------------
        dispRow.R1 = "     VCV ACTUAL     ";
        dispRow.R2 = " PIP  "+(String)actual.PIP+"cm    ";
        dispRow.R3 = " PEEP "+(String)actual.PEEP+"cm    ";
        dispRow.R4 = "   <SEE SETTING>    ";
        //           -----------------------
        break;

      case scr_VCVDisplay_Setting:
        //           -----------------------
        dispRow.R1 = "      VCV MODE      ";
        dispRow.R2 = " PEEP "+ (String)VCVdisplay.PEEP +"cm           ";
        dispRow.R3 = " RR "+(String)VCVdisplay.RR+"     IE 1:"+str_vcv_ie;
        dispRow.R4 = " IP "+(String)VCVdisplay.IP+"cm   TV "+ (String)VCVdisplay.TV + "ml ";
        //           -----------------------
        break;

      case scr_VCVAlarm: 
        //           -----------------------
        dispRow.R1 = "   >>> ALARM <<<    ";
        dispRow.R2 = "                    ";
        dispRow.R3 = " IP SET:  400ml     ";
        dispRow.R4 = " IP ACTL: 350ml     ";
        //           -----------------------
        break;

      case scr_BPAPSetting_IP:
        //           -----------------------
        dispRow.R1 = "   BPAP Setting:    ";
        dispRow.R2 = ">IP "+(String)BPAPdisplay.IP+"cm  PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        if (BPAPdisplay.IP<10) dispRow.R2 = ">IP "+(String)BPAPdisplay.IP+"cm   PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm  FST  "+str_bpap_fst+" ";
        if(BPAPdisplay.TP<10) dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm   FST  "+str_bpap_fst+" ";
        dispRow.R4 = "                    ";
        //           -----------------------
        break;

      case scr_BPAPSetting_PEEP:
        //           -----------------------
        dispRow.R1 = "   BPAP Setting:    ";
        dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm >PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        if (BPAPdisplay.IP<10) dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm  >PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm  FST  "+str_bpap_fst+" ";
        if(BPAPdisplay.TP<10) dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm   FST  "+str_bpap_fst+" ";
        dispRow.R4 = "                    ";
        //           -----------------------
        break;

      case scr_BPAPSetting_TP:
        //           -----------------------
        dispRow.R1 = "   BPAP Setting:    ";
        dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm  PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        if (BPAPdisplay.IP<10) dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm   PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        dispRow.R3 = ">TP "+(String)BPAPdisplay.TP+"cm  FST  "+str_bpap_fst+" ";
        if(BPAPdisplay.TP<10) dispRow.R3 = ">TP "+(String)BPAPdisplay.TP+"cm   FST  "+str_bpap_fst+" ";
        dispRow.R4 = "                    ";
        //           -----------------------
        break;

      case scr_BPAPSetting_FST:
        //           -----------------------
        dispRow.R1 = "   BPAP Setting:    ";
        dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm  PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        if (BPAPdisplay.IP<10) dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm   PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm >FST  "+str_bpap_fst+" ";
        if(BPAPdisplay.TP<10) dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm  >FST  "+str_bpap_fst+" ";
        dispRow.R4 = "                    ";
        //           -----------------------
        break;

      case scr_BPAPDisplay_Actual:
        //           -----------------------
        dispRow.R1 = "    BPAP ACTUAL     ";
        dispRow.R2 = " PA 40cm            ";
        dispRow.R3 = " TV 200ml   PIP 40cm";
        dispRow.R4 = "   <SEE SETTING>    ";
        //           -----------------------
        break;

      case scr_BPAPDisplay_Setting:
        //           -----------------------
        dispRow.R1 = "     BPAP MODE      ";
        dispRow.R2 = "                    ";
        dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm   FST  "+str_bpap_fst;
        dispRow.R4 = " IP "+(String)BPAPdisplay.IP+"cm   PEEP "+(String)BPAPdisplay.PEEP;
        //           -----------------------
        break;

      case scr_BPAPlarm:	
        //           -----------------------
        dispRow.R1 = " >> BPAP  FIALED << ";
        dispRow.R2 = " >> VCV FAILSAFE << ";
        dispRow.R3 = " ACTUAL:  PA  40cm  ";
        dispRow.R4 = " TV 200ml PIP 400ml ";
        //           -----------------------
        break;

      default:
        // do something
        break;
  }

  //Print new data
  lcd.setCursor(0,0);
  lcd.print(dispRow.R1);
  lcd.setCursor(0,1);
  lcd.print(dispRow.R2);
  lcd.setCursor(0,2);
  lcd.print(dispRow.R3);
  lcd.setCursor(0,3);
  lcd.print(dispRow.R4);
}

void Beep(){
    timing.main_scr_timer = millis(); // Reset back to main screen timer
    digitalWrite(BUZZER_PIN, 1);
    delay(5);
    digitalWrite(BUZZER_PIN, 0);
}

void Beep(int _num, long _t){
  timing.main_scr_timer = millis(); // Reset back to main screen timer
  for(int i=1; i<=_num; i++){
    delay(_t);
    digitalWrite(BUZZER_PIN, 1);
    delay(40);
    digitalWrite(BUZZER_PIN, 0);
  }
}

void SETTING2DISPLAY(){
	//disp_modeVCV = modeVCV;
	VCVdisplay.IP = VCVsetting.IP;
	VCVdisplay.TV = VCVsetting.TV;
	VCVdisplay.RR = VCVsetting.RR;
	VCVdisplay.IE = VCVsetting.IE;
	VCVdisplay.PEEP = VCVsetting.PEEP;

	BPAPdisplay.IP = BPAPsetting.IP;
	BPAPdisplay.PEEP = BPAPsetting.PEEP;
	BPAPdisplay.TP = BPAPsetting.TP;
	BPAPdisplay.FST = BPAPsetting.FST;
}

void DISPLAY2SETTING(){
	modeVCV = disp_modeVCV;
	VCVsetting.IP = VCVdisplay.IP;
	VCVsetting.TV = VCVdisplay.TV;
	VCVsetting.RR = VCVdisplay.RR;
	VCVsetting.IE = VCVdisplay.IE;
	VCVsetting.PEEP = VCVdisplay.PEEP;

	BPAPsetting.IP = BPAPdisplay.IP;
	BPAPsetting.PEEP = BPAPdisplay.PEEP;
	BPAPsetting.TP = BPAPdisplay.TP;
	BPAPsetting.FST = BPAPdisplay.FST;
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


void wireData(uint8_t _addr, I2C_Func _func){
  Wire.beginTransmission(_addr);
  Wire.write(_func);

  switch (_func) {
      case I2C_CMD_MOTOR:     // Send motor cmd to slave
        Wire.write((int)isBreathing);
        break;

      case I2C_DATA_SETTING:  // Send setting data to slave
        Wire.write(modeVCV);
        Wire.write(VCVsetting.IP);    
        Wire.write(int(VCVsetting.TV*singleByteConst.TV));
        Wire.write(VCVsetting.RR); 
        Wire.write(int(VCVsetting.IE*singleByteConst.IE)); 
        Wire.write(VCVsetting.PEEP); 
        Wire.write(BPAPsetting.IP);
        Wire.write(BPAPsetting.PEEP); 
        Wire.write(BPAPsetting.TP); 
        Wire.write(int(BPAPsetting.FST*singleByteConst.FST));
        break;
  }

  Wire.endTransmission();
}

bool inRange(int _val, int _min, int _max){
	return (_val >= _min) && (_val <= _max);
}

//birth to 6 weeks: 30–40 breaths per minute
//6 months: 25–40 breaths per minute
//3 years: 20–30 breaths per minute
//6 years: 18–25 breaths per minute
//10 years: 17–23 breaths per minute
//Adults: 12-18 breaths per minute[9]
//Elderly ≥ 65 years old: 12-28 breaths per minute.
//Elderly ≥ 80 years old: 10-30 breaths per minute
