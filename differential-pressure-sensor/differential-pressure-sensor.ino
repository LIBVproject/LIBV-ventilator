#define Vs 5.0

int value[10];
int i=0;
int smoth=0;
float Vout, Pkpa;

unsigned long print_timer=0, sample_timer=0;

void setup() {
  Serial.begin(115200);
  
}

void loop() {
  /*Read every 10ms*/
  if(millis()-sample_timer>=10){
     value[i] = analogRead(A0);
    i++;
    if(i>9) i=0;
    smoth = 0;
    for(int k=0;k<10;k++){
      smoth += value[k];
    }
    smoth /= 10.0;
    Vout = smoth * 5.0 / 1023.0;
    Pkpa = (Vout/Vs - 0.5)/0.2; //following MPXV7002 datasheet page 6
  }

  /* Print every 100ms*/
  if(millis()-print_timer>=100){
    print_timer = millis();
    //  Serial.print(Vout);
    //  Serial.print(" | P(kpa): ");
    //  Serial.print(Pkpa);
    //  Serial.print(" | P(cmH2O): ");
    Serial.println(Pkpa*10.1972);
  }
}
