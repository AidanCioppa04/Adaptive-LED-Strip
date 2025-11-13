#include <hp_BH1750.h>
#include <Arduino_FreeRTOS.h>
#include <FastLED.h>

//define pins

//define constants
#define NUM_LEDS 5

//global variables
volatile int brightness;
volatile int state = 0;

//FastLED
CRGB leds[NUM_LEDS];



void toggleLED(){
  //turn LED on/off based on state
  for(int dot = 0; dot < NUM_LEDS; dot++)
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  // try everything in one loop

  //check motion sensor
  if(digitLRead(MOTION_PIN)){
    state=(state+1)%2
  }

  //turn LEDs on/odd
  toggleLED();

  //if on
  if(state){
    

  }
}
