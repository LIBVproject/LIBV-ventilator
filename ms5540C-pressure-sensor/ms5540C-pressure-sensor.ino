/*
MS5541 Pressure Sensor calwords readout
This program will read your MS5441 or compatible pressure sensor every 5 seconds and show you the calibration words, the calibration factors, 
the raw values and the compensated values of temperature and pressure.
Once you read out the calibration factors you can define them in the header of any sketch you write for the sensor.
Pins:
MS5541 sensor attached to pins 10 - 13:
MOSI: pin 11
MISO: pin 12
SCK: pin 13
MCLK: pin 9
CS is not in use, but might be pin 10
created August 2011
by SMStrauch based on application note AN510 from www.intersema.ch (http://www.meas-spec.com/downloads/Using_SPI_Protocol_with_Pressure_Sensor_Modules.pdf), 
and with help of robtillaart and ulrichard. Thanks!
*/

// include library:
#include <SPI.h>

// generate a MCKL signal pin
const int clock = 9;

void resetsensor() //this function keeps the sketch a little shorter
{
 SPI.setDataMode(SPI_MODE0); 
 SPI.transfer(0x15);
 SPI.transfer(0x55);
 SPI.transfer(0x40);
}

void setup() {
 Serial.begin(9600);
 SPI.begin(); //see SPI library details on arduino.cc for details
 SPI.setBitOrder(MSBFIRST);
 SPI.setClockDivider(SPI_CLOCK_DIV32); //divide 16 MHz to communicate on 500 kHz
 pinMode(clock, OUTPUT);
 delay(100);
}

void loop() 
{
 TCCR1B = (TCCR1B & 0xF8) | 1 ; //generates the MCKL signal
 analogWrite (clock, 128) ; 

 resetsensor();//resets the sensor - caution: afterwards mode = SPI_MODE0!

 //Calibration word 1
 unsigned int result1 = 0;
 unsigned int inbyte1 = 0;
 SPI.transfer(0x1D); //send first byte of command to get calibration word 1
 SPI.transfer(0x50); //send second byte of command to get calibration word 1
 SPI.setDataMode(SPI_MODE1); //change mode in order to listen
 result1 = SPI.transfer(0x00); //send dummy byte to read first byte of word
 result1 = result1 << 8; //shift returned byte 
 inbyte1 = SPI.transfer(0x00); //send dummy byte to read second byte of word
 result1 = result1 | inbyte1; //combine first and second byte of word
 Serial.print("Calibration word 1 =");
 Serial.println(result1);

 resetsensor();//resets the sensor

 //Calibration word 2; see comments on calibration word 1
 unsigned int result2 = 0;
 byte inbyte2 = 0; 
 SPI.transfer(0x1D);
 SPI.transfer(0x60);
 SPI.setDataMode(SPI_MODE1); 
 result2 = SPI.transfer(0x00);
 result2 = result2 <<8;
 inbyte2 = SPI.transfer(0x00);
 result2 = result2 | inbyte2;
 Serial.print("Calibration word 2 =");
 Serial.println(result2);  

 resetsensor();//resets the sensor

 //Calibration word 3; see comments on calibration word 1
 unsigned int result3 = 0;
 byte inbyte3 = 0;
 SPI.transfer(0x1D);
 SPI.transfer(0x90); 
 SPI.setDataMode(SPI_MODE1); 
 result3 = SPI.transfer(0x00);
 result3 = result3 <<8;
 inbyte3 = SPI.transfer(0x00);
 result3 = result3 | inbyte3;
 Serial.print("Calibration word 3 =");
 Serial.println(result3);  

 resetsensor();//resets the sensor

 //Calibration word 4; see comments on calibration word 1
 unsigned int result4 = 0;
 byte inbyte4 = 0;
 SPI.transfer(0x1D);
 SPI.transfer(0xA0);
 SPI.setDataMode(SPI_MODE1); 
 result4 = SPI.transfer(0x00);
 result4 = result4 <<8;
 inbyte4 = SPI.transfer(0x00);
 result4 = result4 | inbyte4;
 Serial.print("Calibration word 4 =");
 Serial.println(result4);

 //now we do some bitshifting to extract the calibration factors 
 //out of the calibration words; read datasheet AN510 for better understanding
 long c1 = result1 >> 3 & 0x1FFF;
 long c2 = ((result1 & 0x07) << 10) | ((result2 >> 6) & 0x03FF);
 long c3 = (result3 >> 6) & 0x03FF;
 long c4 = (result4 >> 7) & 0x07FF;
 long c5 = ((result2 & 0x003F) << 6) | (result3 & 0x003F);
 long c6 = result4 & 0x007F;

 Serial.println(c1);
 Serial.println(c2);
 Serial.println(c3);
 Serial.println(c4);
 Serial.println(c5);
 Serial.println(c6);

 resetsensor();//resets the sensor

 //Temperature:
 unsigned int tempMSB = 0; //first byte of value
 unsigned int tempLSB = 0; //last byte of value
 unsigned int D2 = 0;
 SPI.transfer(0x0F); //send first byte of command to get temperature value
 SPI.transfer(0x20); //send second byte of command to get temperature value
 delay(35); //wait for conversion end
 SPI.setDataMode(SPI_MODE1); //change mode in order to listen
 tempMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
 tempMSB = tempMSB << 8; //shift first byte
 tempLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
 D2 = tempMSB | tempLSB; //combine first and second byte of value
 Serial.print("Temperature raw =");
 Serial.println(D2); //voilá!

 resetsensor();//resets the sensor

 //Pressure:
 unsigned int presMSB = 0; //first byte of value
 unsigned int presLSB =0; //last byte of value
 unsigned int D1 = 0;
 SPI.transfer(0x0F); //send first byte of command to get pressure value
 SPI.transfer(0x40); //send second byte of command to get pressure value
 delay(35); //wait for conversion end
 SPI.setDataMode(SPI_MODE1); //change mode in order to listen
 presMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
 presMSB = presMSB << 8; //shift first byte
 presLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
 D1 = presMSB | presLSB; //combine first and second byte of value
 Serial.print("Pressure raw =");
 Serial.println(D1);

 //calculation of the real values by means of the calibration factors and the maths
 //in the datasheet. const MUST be long
 const long UT1 = (c5 << 3) + 10000;
 const long dT = D2 - UT1;
 const long TEMP = 200 + ((dT * (c6 + 100)) >> 11);
 const long OFF  = c2 + (((c4 - 250) * dT) >> 12) + 10000;
 const long SENS = (c1/2) + (((c3 + 200) * dT) >> 13) + 3000;
 long PCOMP = (SENS * (D1 - OFF) >> 12) + 1000;
 float TEMPREAL = TEMP/10;

 Serial.print("Real Temperature in °C=");
 Serial.println(TEMPREAL);

 Serial.print("Compensated pressure in mbar =");
 Serial.println(PCOMP);

 //2nd order compensation only for T > 0°C
 const long dT2 = dT - ((dT >> 7 * dT >> 7) >> 3);
 const float TEMPCOMP = (200 + (dT2*(c6+100) >>11))/10;
 Serial.print("2nd order compensated temperature in °C =");
 Serial.println(TEMPCOMP);  

 delay(5000);
}
