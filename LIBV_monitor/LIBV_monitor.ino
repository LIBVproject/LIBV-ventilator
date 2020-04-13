//Vatilator system Monitoring
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <button.h>

#define ROTARY_CLK 3
#define ROTARY_DT 2

#define SAVE_PIN  8   //Save adjustment setting button pin
#define CANCEL_PIN  9 //Cancel adjustment saving button pin
#define TV_PIN A2     //TV adjustment button pin
#define TI_PIN A1     //TI adjustment button pin
#define IE_PIN A0     //IE adjustment button pin
#define BUZZER 5
#define HALL 6

long ROTARY_COUNT = 0, ROTARY_LAST=0;
bool TV_SAVE=false, TI_SAVE=false, IE_SAVE=false;

int TV=100, TI=1000, IE=2;
const int TV_MIN=0, TV_MAX=680, TI_MIN=800, TI_MAX=3000, IE_MIN=2, IE_MAX=8;

//Adjustment Variable (Setting Change)
bool ad_Stat=false;        //Adjustment status
int ad_TV, ad_TI, ad_IE;   //
long ad_t, ad_tout=20000;        //Adjustment timeout timer, set timeout for 20s

//Display Variable 
int ds_TV, ds_TI, ds_IE;

long t=0;

Encoder rotary(ROTARY_DT, ROTARY_CLK);
LiquidCrystal_I2C lcd(0x27, 16, 2);
button SAVE_BUT(SAVE_PIN);
button CANCEL_BUT(CANCEL_PIN);
button TV_BUT(TV_PIN);
button TI_BUT(TI_PIN);
button IE_BUT(IE_PIN);

void setup() {
  Serial.begin (9600);
  lcd.begin();
  lcd.backlight();

  pinMode(BUZZER, OUTPUT);
  pinMode(HALL, INPUT);

  SAVE_BUT.begin(true);
  CANCEL_BUT.begin(true);
  TV_BUT.begin(true);
  TI_BUT.begin(true);
  IE_BUT.begin(true);
  
  BEEP();
  
  lcd.setCursor(1,0);
  lcd.print("  STARTING...   ");
  
  TV = EEPROM.read(10)*10;
  TI = EEPROM.read(11)*100;
  IE = EEPROM.read(12);
  ad_TV = TV;
  ad_TI = TI;
  ad_IE = IE;
  delay(2000);

  wireData();
  BEEP();
  lcd.setCursor(1,0);
  lcd.print("     READY     ");
  lcd.setCursor(1,1);
  lcd.print("CHECK SETTINGS ");
  delay(3000);
}

void loop() {
  ROTARY_COUNT = rotary.read()/2;

  if(TV_BUT.press() && !TI_BUT.press() && !IE_BUT.press()){       //when only TV button is pressed
    ad_t = millis();
    ad_Stat = true;
    if(ROTARY_COUNT != ROTARY_LAST){
      ad_TV += (ROTARY_COUNT - ROTARY_LAST)*10;
      ROTARY_LAST = ROTARY_COUNT;
      ad_TV = constrain(ad_TV, TV_MIN, TV_MAX);
    }
  }else if(!TV_BUT.press() && TI_BUT.press() && !IE_BUT.press()){ //when only TI button is pressed
    ad_t = millis();
    ad_Stat = true;
    if(ROTARY_COUNT != ROTARY_LAST){
      ad_TI += (ROTARY_COUNT - ROTARY_LAST)*100;
      ROTARY_LAST = ROTARY_COUNT;
      ad_TI = constrain(ad_TI, TI_MIN, TI_MAX);
    }
  }else if(!TV_BUT.press() && !TI_BUT.press() && IE_BUT.press()){ //when only IE button is pressed
    ad_t = millis();
    ad_Stat = true;
    if(ROTARY_COUNT != ROTARY_LAST){
      ad_IE += (ROTARY_COUNT - ROTARY_LAST);
      ROTARY_LAST = ROTARY_COUNT;
      ad_IE = constrain(ad_IE, IE_MIN, IE_MAX);
    }
  }else{  //No button is pressed or more than one are pressed, capture moving encoder to prevent the unwanted adjustment jump at button press.
    ROTARY_LAST = ROTARY_COUNT;
  }

  //Save and Send data
  if(ad_Stat){
    if(SAVE_BUT.push()){
      TV = ad_TV;
      TI = ad_TI;
      IE = ad_IE;
      EEPROM.update(10, TV/10);
      EEPROM.update(11, TI/100);
      EEPROM.update(12, IE);
      wireData();
      ad_Stat = false;
    }else if(CANCEL_BUT.push() || (millis()- ad_t >= ad_tout)){
      ad_TV = TV;
      ad_TI = TI;
      ad_IE = IE;
      ad_t = millis();
      ad_Stat = false;
    }
    ds_TV = ad_TV;
    ds_TI = ad_TI;
    ds_IE = ad_IE;
  }else{
    ds_TV = TV;
    ds_TI = TI;
    ds_IE = IE;
  }
  
  //Line the data for Display
  String str_TV, str_TI, str_IE, str_BPM;

  str_TV = (String)ds_TV;
  str_TI = (String)((float)ds_TI/1000.0);
  str_TI[3] = 's';

  if(ds_IE%2 == 0){
    str_IE = "IE=1:"+(String)(ds_IE/2)+"   ";
  }else{
    str_IE = "IE=1:"+(String)(((ds_IE-1)/2))+".5"+" ";
  }
  
  str_BPM ="BPM=" + (String)(round(60.0 / (((float)ds_TI/1000.0) + ((float)ds_TI/1000.0)*((float)ds_IE/2.0))));

  //Display Data on LCD
  lcd.setCursor(0,0);
  lcd.print("TV=" + str_TV + "ml TI=" + str_TI);
  lcd.setCursor(0,1);
  lcd.print(str_IE + str_BPM + "  ");
}

void BEEP(){
  digitalWrite(BUZZER, 1);
  delay(200);
  digitalWrite(BUZZER, 0);
}

void wireData(){
  Wire.beginTransmission(8);
  Wire.write(TV/10);
  Wire.write(TI/100);    
  Wire.write(IE); 
  Wire.endTransmission();
}

//birth to 6 weeks: 30–40 breaths per minute
//6 months: 25–40 breaths per minute
//3 years: 20–30 breaths per minute
//6 years: 18–25 breaths per minute
//10 years: 17–23 breaths per minute
//Adults: 12-18 breaths per minute[9]
//Elderly ≥ 65 years old: 12-28 breaths per minute.
//Elderly ≥ 80 years old: 10-30 breaths per minute
