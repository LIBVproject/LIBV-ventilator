/* Pressure Sensors
 * part number: 160MDAA5
 * Specification:
 * - Supply : 5V
 * - Output : Analog
 * - Pressure range: ±160 mbar
 */

 #define read_pin A0
 #define DEBUG false

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println(pressureCMH2O(read_pin));
  delay(50);
}

int pressureCMH2O(byte _pin){
  const float _Vs = 5.0;        // Supply voltage (volt)
  const float _Pmin = -160.0;   // Minimum Pressure (mbar)
  const float _Pmax = 160.0;    // Maximum Pressure (mbar)
  int _analog = analogRead(_pin);
  float _Vin = _Vs*(_analog/1024.0);
  float _pressure = ((_Pmax - _Pmin)*(_Vin-(0.1*_Vs))/(0.8*_Vs))+_Pmin; //mbar
  if(DEBUG){
    Serial.println(">> pressureCMH2O DEBUG <<");
    Serial.println("Analog read: "+(String)_analog);
    Serial.println("Voltage read: "+(String)_Vin+" V");
    Serial.println("Pressure Cal: "+(String)_pressure+" mbar");
    Serial.println("-------------------------");
  }
  return _pressure*1.02; //cmH2O
}
