#include <hp_BH1750.h>

//sla to sla
//scl to scl
//vin to 5v
//grn to grn
hp_BH1750 BH1750; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //initialize sensor, print if failed
  bool avail = BH1750.begin(BH1750_TO_GROUND);
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {};                                        
  }

  int lux;
}

void loop() {
  // put your main code here, to run repeatedly:
  BH1750.start();   //starts a measurement
  float lux=BH1750.getLux();  //  waits until a conversion finished
  Serial.println(lux);
  delay(250);
}


// sample readings
//85.00
//83.75
//136.25
//136.67
//1617.08
//3429.58




