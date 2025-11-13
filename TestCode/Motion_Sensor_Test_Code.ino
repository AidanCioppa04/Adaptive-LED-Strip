#include <hp_BH1750.h>

#define MOTION_PIN 4
int reading;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  pinMode(MOTION_PIN, INPUT);
  Serial.print("setup\n");
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(MOTION_PIN)){
    reading=1;
  }
  
  if((millis()%1000)==0){
    delay(1);
    Serial.print(reading);
    reading=0;
    
  }
}
