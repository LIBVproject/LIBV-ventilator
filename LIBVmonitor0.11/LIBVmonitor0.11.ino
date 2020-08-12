/* LIBV Project
 * Website: https://bvmvent.org/
 * GitHub: https://github.com/LIBVproject/LIBV-ventilator
 * 
 * #VENTILATOR SYSTEM MONITORING#
 *
 * LIBRARY:
 * - Encoder: https://github.com/PaulStoffregen/Encoder
 * - simplePID: https://github.com/eTRONICSKH/SimplePID-Arduino-Library-V01
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
 * - RR: Resporatory Rate                 Breathe Per minutes
 */

#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <button.h>
#include <SFE_BMP180.h>
#include <Adafruit_SleepyDog.h>

#define DEBUG false

/*------------------------------- PARAMETERS ------------------------------*/

/*************************/
/*      I2C ADDRESS      */
/*************************/
#define I2C_ADDR_MOTOR 8
#define I2C_ADDR_LCD 0x27

enum I2C_Func{
  I2C_CMD_MOTOR,
  I2C_DATA_SETTING,
  I2C_DATA_PRESSURE,
  I2C_REQUEST_PLATEAU,
  I2C_REQUEST_POSITION,
  I2C_REQUEST_PLATEAU_AND_POSITION
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
  const int POWER = 8;      //TI adjustment button pin
  const int BREATHE = 9;      //IE adjustment button pin
  const int SILENT = 10; //Silence the alarm button pin
} buttonPin;

//P-IN: Hall switch sensor
const int HALL1_PIN = A0;
const int HALL2_PIN = A1;

//P-OUT: Alarm
const int RELAY_PIN  = 11;
const int BUZZER_PIN = 12;      //Buzzer pin
const int LED_PIN    = 13;         //LED Alarm flash pin

/*************************/
/*   LCD SPECIFICATION   */
/*************************/
const int LCD_DISP_ROW    = 4;
const int LCD_DISP_COLUMN = 20;

const int WATCH_DOG_TIMER = 5000;

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
  int8_t pressureRead;
  int8_t captureIP;
  int8_t capturePEEP;
  int8_t IP;
  int8_t PEEP;
  uint8_t Plateau;
  uint8_t Position;
};

enum  screenID {
  scr_PowerON,
  scr_PressureFail,
  scr_Setting_VCV,
  scr_Setting_BPAP,
  scr_VCVSetting_IP,
  scr_VCVSetting_TV,
  scr_VCVSetting_RR,
  scr_VCVSetting_IE,
  scr_VCVSetting_PEEP,
  scr_VCVDisplay_Actual,
  scr_VCVDisplay_Setting,
  scr_VCVAlarm_PEEP,
  scr_VCVAlarm_IP,
  scr_BPAPSetting_IP,
  scr_BPAPSetting_PEEP,
  scr_BPAPSetting_TP,
  scr_BPAPSetting_FST,
  scr_BPAPDisplay_Actual,
  scr_BPAPDisplay_Setting,
  scr_BPAPlarm
};

enum Alarm_Type {
  primary_alarm,    //0
  secondary_alarm,  //1
};

struct TIMING{
  const long main_scr_timout = 20000;        // back to main screen timout, after no change for 30s
  long main_scr_timer = 0;                   // Reset at Beep function
  const long pressure_send_time = 50;
  long pressure_send_timer = 0;
} timing;

struct ALARMING{
  const long silent_timout = 600000;          // silent buzzer for 10mn after silent button is pusheed
  const float pressure_ip_offset = 10.0;       // tolorance of IP alarm (cm)
  const float pressure_peep_offset = 5.0;     // tolorance of PEEP alarm (cm)
  long off_timer = 0;                         // reset alarm loop
  long silent_timer = 0;                      // reset silent buzzer
  bool buzzer_is_silenced = false;            // buzzer sound state
  bool pressure_sensor_fail = false;          // Pressure sensor status
  bool pressure_ip_fail = false;              // IP pressure out of range
  bool pressure_peep_fail = false;            // PEEP pressure out of range
  bool i2c_communication_fail = false;        // I2C communication check fail
} alarming;

uint8_t Screen=scr_PowerON;           //Screen display ID, start with Power On screen

bool modeVCV=true;
bool inSetting    = false;       // true: display setting adjusment, false: save setting and display current setting.
bool disp_modeSet = true;     //Screen display when start setup mode, true: start with chosing VCV or BPAP, false: go to element setup
bool disp_modeVCV = true;     //Screen display mode for setup the elements, true: VCV and false: BPAP
bool disp_select  = false;     //Select the element to adjust.
bool disp_change  = false;     //rotary change status.
bool isAlarm      = false;

bool isPowered  = false;
bool isBreathing= false;     //Machine is running or not, true: help breathing & false: standby




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

button powerBT(buttonPin.POWER, HIGH);
button setBT(buttonPin.SET, HIGH);
button cancelBT(buttonPin.CANCEL, HIGH);
button selectBT(buttonPin.SELECT, HIGH);
button silentBT(buttonPin.SILENT, HIGH);
button breatheBT(buttonPin.BREATHE, HIGH);

button switch1(HALL1_PIN, HIGH);
button swithc2(HALL2_PIN, HIGH);

void setup() {
  Serial.begin (115200);
  int countdownMS = Watchdog.enable(WATCH_DOG_TIMER);
  
  /* Initialize button */
  powerBT.init();
  setBT.init();
  cancelBT.init();
  selectBT.init();
  silentBT.init();
  breatheBT.init();

  /*Initialize limit switch */
  switch1.init();
  swithc2.init();

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, 1);
  digitalWrite(BUZZER_PIN, 0);
  digitalWrite(LED_PIN, 0);
  delay(200);

  /*  Initialize LCD   */
  lcd.begin();
  lcd.clear();
  lcd.noBacklight();

  EEPROM_read();
  SETTING2DISPLAY();
  alarming.i2c_communication_fail=!wireData(I2C_ADDR_MOTOR, I2C_DATA_SETTING);

  /* Pressure sensor */
  //pressureBEGIN();
}

void loop() {
  Watchdog.reset();       // Reset watchdog timer
  Serial.println(isPowered);
  /*  Breathe button action
   * - Push to run (start to move with current setting data)
   * - Hold for 2 seconds to stop (return arms to home position)
   */
  if(!isPowered && powerBT.push()){
    powerBT.resetHold();
    lcd.backlight();
    startUpScreen();
    isPowered = true;
  }else if (isPowered && powerBT.onHold()>=3000) {
    powerBT.resetHold();
    isBreathing = false; // stop the motor
    alarming.i2c_communication_fail=!wireData(I2C_ADDR_MOTOR, I2C_CMD_MOTOR);
    delay(500);
    lcd.clear();
    lcd.noBacklight();
    isPowered = false;
  }
  
  if (isPowered && !isBreathing && breatheBT.push()){
    breatheBT.resetHold();
    Beep(3, 100);
    isBreathing = true; // Run the motor
    alarming.i2c_communication_fail=!wireData(I2C_ADDR_MOTOR, I2C_CMD_MOTOR);
    Screen = modeVCV? scr_VCVDisplay_Actual:scr_BPAPDisplay_Actual;
  }else if (isPowered && isBreathing && breatheBT.onHold()>=2000){
    breatheBT.resetHold();
    Beep(3, 100);
    isAlarm = false;     // Reset alarm for new operation
    isBreathing = false; // Run the motor
    alarming.i2c_communication_fail=!wireData(I2C_ADDR_MOTOR, I2C_CMD_MOTOR);
  }

  /* Back to main screen after timout
   * Status: tested, best
   */
  if (millis()-timing.main_scr_timer >= timing.main_scr_timout) {
    inSetting = false;
    if (isBreathing) {
      Screen = modeVCV ? scr_VCVDisplay_Actual : scr_BPAPDisplay_Actual;
    }else{
      Screen = modeVCV ? scr_VCVDisplay_Setting : scr_BPAPDisplay_Setting;
    }
  }

  /* Read Pressure
  * - Check pressure sensor status for Alarm
  * - Capture PIP and PEEP for actual displays
  */
  //alarming.pressure_sensor_fail = !pressureCMH2O(actual.pressureRead); //pressure in cmH2O
  isAlarm = alarming.pressure_sensor_fail;
  //Serial.println(actual.pressureRead);

  // Pressure operating only when the sensor not fail
  if (!alarming.pressure_sensor_fail){
    /* Send pressure data to slave
     * send every pressure_send_timer
     */
    if (millis()-timing.pressure_send_timer>=timing.pressure_send_time) {
      timing.pressure_send_timer = millis();
      //alarming.i2c_communication_fail=!wireData(I2C_ADDR_MOTOR, I2C_DATA_PRESSURE);
    }
  
    //record PIP and PEEP pressure
    if (actual.pressureRead>actual.captureIP) actual.captureIP = actual.pressureRead;
    if (actual.pressureRead<actual.capturePEEP) actual.capturePEEP = actual.pressureRead;
    //Capture PIP when Arms arrived at Home position
    if (swithc2.push()){
      actual.IP = actual.captureIP;
      actual.captureIP = 0.0; // reset IP value
      alarming.i2c_communication_fail=!wireRequest(I2C_ADDR_MOTOR);
    }

    //Capture PEEP when Arms left home position
    if (swithc2.release()){
      actual.PEEP = actual.capturePEEP;
      actual.capturePEEP = 40.0;
    }
  }

  /*  Control panel action
   *  
   */
  rotary.Value = rotaryEnc.read()/2;

  //Setting Display
  if(inSetting){
    if(disp_modeSet){ //Select mode for setting
      Screen = disp_modeVCV ? scr_Setting_VCV : scr_Setting_BPAP; //Set display Screen
      if (selectBT.push()){ //Switch selected mode
        Beep();
        disp_modeVCV = !disp_modeVCV; //Switching select mode between VCV & BPAP
      }
      
      if(setBT.push()){ //Chose mode, move to elements      
        Beep();
        disp_modeSet = false; //Jump to element adjusting
        Screen = disp_modeVCV ? scr_VCVSetting_IP : scr_BPAPSetting_IP; //Start screen at first element
        delay(300);
      }

      else if(cancelBT.push()) { //Back to current current setting display
        Beep();
        Screen = disp_modeVCV ? scr_VCVDisplay_Actual : scr_BPAPDisplay_Actual; 
        inSetting = false;  //Cancel current adjusting, jump to last setting display
      }

    }else{  //After mode is selected, elements adjusting
      if(disp_modeVCV){ //Adjust VCV elements
        if (selectBT.push()){
          Beep();
          SETTING2DISPLAY(); //Transfer setting data to display, incase data is not SET before moving to the next element
          Screen++; //Arrow move to next element
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
      }else{  //Adjust BPAP elements
        if (selectBT.push()){
          Beep();
          SETTING2DISPLAY(); //Transfer setting data to display, incase data is not SET before moving to the next element
          Screen++; //Arrow move to next element
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
      }else if(cancelBT.push()){  //Back to Selecting mode
        Beep();
        if (disp_change){
          SETTING2DISPLAY();    //Cancel unsaved adjusting, Display saved setting
          disp_change = false;    //Reset changing state
        }else disp_modeSet = true;  //Jump back to Select Mode
      }

      //Confirm to use the setting, exit setting screen to display
      if (setBT.onHold()>=1500){
        setBT.resetHold();
        Beep(2,200);
        alarming.i2c_communication_fail=!wireData(I2C_ADDR_MOTOR, I2C_DATA_SETTING);
        Screen = modeVCV? scr_VCVDisplay_Actual:scr_BPAPDisplay_Actual;
        inSetting = false; //Jump to display current setting
      }
    }
    
  }else{
    rotary.lastValue = rotary.Value; //Reset rotary change accidently

    if (modeVCV){
      if (isBreathing){
        // Alarm trigger
        if (!alarming.pressure_sensor_fail){
          alarming.pressure_ip_fail = outOfRange(actual.IP, VCVsetting.IP, alarming.pressure_ip_offset);
          alarming.pressure_peep_fail = outOfRange(actual.PEEP, VCVsetting.PEEP, alarming.pressure_peep_offset);
          isAlarm = (alarming.pressure_ip_fail || alarming.pressure_peep_fail);
        }
        //Display Alarm
        if (isAlarm){
          if (alarming.pressure_peep_fail) Screen = scr_VCVAlarm_PEEP;
          if (alarming.pressure_ip_fail) Screen = scr_VCVAlarm_IP;
          if (selectBT.press()) Screen = scr_VCVDisplay_Actual;
        }else{
          Screen = scr_VCVDisplay_Actual;
        }
      }else{
        Screen = scr_VCVDisplay_Setting;
      }

    }else{
      //BPAP Operation mode
      //TODO: Update same to VCV mode

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
      inSetting = true;   //Jump to Setting
      disp_modeSet = false; //Jump to change current setting
    }
  }

  if (alarming.pressure_sensor_fail) Screen = scr_PressureFail;

  /* Alarm Sound Event
   * silent button
   */
  if (isAlarm) {
    if (silentBT.press()){
      silentBT.resetHold();
      alarming.buzzer_is_silenced = true;
      alarming.silent_timer = millis();
    }
    //Reset the buzzer alarm state to not silent
    if (alarming.buzzer_is_silenced){
      alarming.buzzer_is_silenced = (millis()-alarming.silent_timer<=alarming.silent_timout);
    }
    Alarm(secondary_alarm, alarming.off_timer, alarming.buzzer_is_silenced);
  }else{
    alarming.silent_timer = millis();
    alarming.off_timer = millis();
  }

  if(isPowered) lcdDiplay(Screen);
}

void pressureBEGIN(){
  while(!BMP180.begin()){
    lcdDiplay(scr_PressureFail);  //Display Alarm sensor fail.
    Beep(2, 200);
    if (cancelBT.push()) break;
  }
}

bool pressureCMH2O(int8_t &_pressure){
  bool _status = false;
  const float atmospheric_pressure = 1013; // 1013 mbar
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
        lcd_print(F("  LIBV OPEN SOURCE  "),
                  F(" VENTILATOR PROJECT "),
                  F("                    "),
                  F("   www.bvmvent.org  "));
        break;

      case scr_PressureFail:
        lcd_print(F("   >>> ALARM <<<    "),
                  F("                    "),
                  F("Pressure Sensor Fail"),
                  F("  (Check & Reboot)  "));
        break;

      case scr_Setting_VCV:
        lcd_print(F("    Select Mode:    "),
                  F("                    "),
                  F("   >VCV     BPAP    "),
                  F("                    "));
        break;

      case scr_Setting_BPAP:
        lcd_print(F("    Select Mode:    "),
                  F("                    "),
                  F("   VCV     >BPAP    "),
                  F("                    "));
        break;

      case scr_VCVSetting_IP:
        lcd_print(F("VCV Setting:        "),
                  ">IP "+(String)VCVdisplay.IP+"cm            ",
                  " TV " + (String)VCVdisplay.TV + "ml  RR   "+(String)VCVdisplay.RR+"  ",
                  " IE 1:"+str_vcv_ie+" PEEP "+ (String)VCVdisplay.PEEP +"cm");
        break;

      case scr_VCVSetting_TV:
        lcd_print(F("VCV Setting:        "),
                  " IP "+(String)VCVdisplay.IP+"cm            ",
                  ">TV " + (String)VCVdisplay.TV + "ml  RR   "+(String)VCVdisplay.RR+"  ",
                  " IE 1:"+str_vcv_ie+" PEEP "+ (String)VCVdisplay.PEEP +"cm");
        break;

      case scr_VCVSetting_RR:
        lcd_print(F("VCV Setting:        "),
                  " IP "+(String)VCVdisplay.IP+"cm            ",
                  " TV " + (String)VCVdisplay.TV + "ml >RR   "+(String)VCVdisplay.RR+"  ",
                  " IE 1:"+str_vcv_ie+" PEEP "+ (String)VCVdisplay.PEEP +"cm");
        break;

      case scr_VCVSetting_IE:
        lcd_print(F("VCV Setting:        "),
                  " IP "+(String)VCVdisplay.IP+"cm            ",
                  " TV " + (String)VCVdisplay.TV + "ml  RR   "+(String)VCVdisplay.RR+"  ",
                  ">IE 1:"+str_vcv_ie+" PEEP "+ (String)VCVdisplay.PEEP +"cm");
        break;

      case scr_VCVSetting_PEEP:
        lcd_print(F("VCV Setting:        "),
                  " IP "+(String)VCVdisplay.IP+"cm            ",
                  " TV " + (String)VCVdisplay.TV + "ml  RR   "+(String)VCVdisplay.RR+"  ",
                  " IE 1:"+str_vcv_ie+">PEEP "+ (String)VCVdisplay.PEEP +"cm");
        break;

      case scr_VCVDisplay_Actual:
        lcd_print(F("     VCV ACTUAL     "),
                  " IP      "+(String)actual.IP+"cm       ",
                  " Plateau "+(String)actual.Plateau+"cm       ",
                  " PEEP    "+(String)actual.PEEP+"cm       ");
        break;

      case scr_VCVDisplay_Setting:
        lcd_print(F("      VCV MODE      "),
                  " PEEP "+ (String)VCVdisplay.PEEP +"cm           ",
                  " RR "+(String)VCVdisplay.RR+"     IE 1:"+str_vcv_ie,
                  " IP "+(String)VCVdisplay.IP+"cm   TV "+ (String)VCVdisplay.TV + "ml ");
        break;

      case scr_VCVAlarm_PEEP:
        lcd_print(F("   >>> ALARM <<<    "),
                  F("                    "),
                  " PEEP SET:  "+(String)VCVdisplay.PEEP+"cm    ",
                  " PEEP ACTL: "+(String)actual.PEEP+"cm    ");
        break;

      case scr_VCVAlarm_IP:
        lcd_print(F("   >>> ALARM <<<    "),
                  F("                    "),
                  " IP SET:  "+(String)VCVdisplay.IP+"cm     ",
                  " IP ACTL: "+(String)actual.IP+"cm     ");
        break;

      case scr_BPAPSetting_IP:
        //           -----------------------
        dispRow.R2 = ">IP "+(String)BPAPdisplay.IP+"cm  PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        if (BPAPdisplay.IP<10) dispRow.R2 = ">IP "+(String)BPAPdisplay.IP+"cm   PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm  FST  "+str_bpap_fst+" ";
        if(BPAPdisplay.TP<10) dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm   FST  "+str_bpap_fst+" ";
        //           -----------------------
        lcd_print(F("   BPAP Setting:    "),
                  dispRow.R2,
                  dispRow.R3,
                  F("                    "));
        break;

      case scr_BPAPSetting_PEEP:
        //           -----------------------
        dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm >PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        if (BPAPdisplay.IP<10) dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm  >PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm  FST  "+str_bpap_fst+" ";
        if(BPAPdisplay.TP<10) dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm   FST  "+str_bpap_fst+" ";
        //           -----------------------
        lcd_print(F("   BPAP Setting:    "),
                  dispRow.R2,
                  dispRow.R3,
                  F("                    "));
        break;

      case scr_BPAPSetting_TP:
        //           -----------------------
        dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm  PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        if (BPAPdisplay.IP<10) dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm   PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        dispRow.R3 = ">TP "+(String)BPAPdisplay.TP+"cm  FST  "+str_bpap_fst+" ";
        if(BPAPdisplay.TP<10) dispRow.R3 = ">TP "+(String)BPAPdisplay.TP+"cm   FST  "+str_bpap_fst+" ";
        //           -----------------------
        lcd_print(F("   BPAP Setting:    "),
                  dispRow.R2,
                  dispRow.R3,
                  F("                    "));
        break;

      case scr_BPAPSetting_FST:
        //           -----------------------
        dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm  PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        if (BPAPdisplay.IP<10) dispRow.R2 = " IP "+(String)BPAPdisplay.IP+"cm   PEEP "+(String)BPAPdisplay.PEEP+"cm ";
        dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm >FST  "+str_bpap_fst+" ";
        if(BPAPdisplay.TP<10) dispRow.R3 = " TP "+(String)BPAPdisplay.TP+"cm  >FST  "+str_bpap_fst+" ";
        //           -----------------------
        lcd_print(F("   BPAP Setting:    "),
                  dispRow.R2,
                  dispRow.R3,
                  F("                    "));
        break;

      case scr_BPAPDisplay_Actual:
        lcd_print(F("    BPAP ACTUAL     "),
                  F(" PA 40cm            "),
                  F(" TV 200ml    IP 40cm"),
                  F("   <SEE SETTING>    "));
        break;

      case scr_BPAPDisplay_Setting:
        lcd_print(F("     BPAP MODE      "),
                  F("                    "),
                  " TP "+(String)BPAPdisplay.TP+"cm   FST  "+str_bpap_fst,
                  " IP "+(String)BPAPdisplay.IP+"cm   PEEP "+(String)BPAPdisplay.PEEP);
        break;

      case scr_BPAPlarm:  
        lcd_print(F(" >> BPAP  FIALED << "),
                  F(" >> VCV FAILSAFE << "),
                  F(" ACTUAL:  PA  40cm  "),
                  F(" TV 200ml IP  40cm  "));
        break;

      default:
        // do something
        break;
  }
}

void lcd_print(String _r0, String _r1, String _r2, String _r3){
  delay(1);
  //Print new data
  lcd.setCursor(0,0);
  lcd.print(_r0);
  lcd.setCursor(0,1);
  lcd.print(_r1);
  lcd.setCursor(0,2);
  lcd.print(_r2);
  lcd.setCursor(0,3);
  lcd.print(_r3);
}

void Alarm(Alarm_Type _num, long &_alarm_timer, bool _silent_buzzer){
  uint16_t _delay, _sound_num, _loop_timer;

  // TODO: define the alarm type (_num)

  switch (_num) {
      case primary_alarm:
        _delay = 200;
        _sound_num = 1;
        _loop_timer = 500;
        break;
      case secondary_alarm:
        _delay = 200;
        _sound_num = 2;
        _loop_timer = 1500;
        break;
  }  

  if (millis()-_alarm_timer >= _loop_timer) {
    for (int i = 0; i < _sound_num; ++i){
      delay(_delay);
      digitalWrite(BUZZER_PIN, !_silent_buzzer);
      digitalWrite(LED_PIN, 1);
      delay(20);
      digitalWrite(LED_PIN, 0);
      digitalWrite(BUZZER_PIN, 0);
    }
    _alarm_timer = millis();
  }
  
}


void Beep(){
    timing.main_scr_timer = millis(); // Reset back to main screen timer
    digitalWrite(BUZZER_PIN, 1);
    delay(10);
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


bool wireData(uint8_t _addr, I2C_Func _func){
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

      case I2C_DATA_PRESSURE: //Send pressure data
        Wire.write(int(actual.pressureRead));
        break;
  }
  uint8_t _slave_status = Wire.endTransmission();
  return _slave_status == 0; // Transmission success
}

bool wireRequest(uint8_t _addr){
  bool _bus_status = false;
  if (Wire.requestFrom(_addr, 2) == 2) {  // correct slave response
    while(Wire.available()){
      actual.Plateau = Wire.read();
      actual.Position = Wire.read();
    }
    if (DEBUG) Serial.println("[I2C] Collecting data from slave.");
    _bus_status = true;
  }
    
  return _bus_status;
}

bool inRange(int _val, int _min, int _max){
  return (_val >= _min) && (_val <= _max);
}

bool outOfRange(float _input, float _set, float _offset){
  return abs(_set - _input) >= _offset;
}
