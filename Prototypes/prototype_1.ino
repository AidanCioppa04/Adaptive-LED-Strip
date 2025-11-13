#include <Arduino_FreeRTOS.h>
#include <hp_BH1750.h>
#include <FastLED.h>
#include <Arduino.h>

//PINS
#define LED_PIN 4
#define MOTION_PIN 2

//Mutexes
SemaphoreHandle_t xStateMutex;
SemaphoreHandle_t xBrightnessMutex;

//Tasks
TaskHandle_t xHandle = NULL;
TaskHandle_t xHandle = NULL;
TaskHandle_t xHandle = NULL;

//Light Sensor setup
hp_BH1750 BH1750;

//FastLED setup
#define NUM_LEDS 10
CRGB leds[NUM_LEDS];


volatile int BRIGHTNESS;
volatile int STATE = 0;

inline void takeBrightnessMutex(){
  xSemaphoreTake(xBrightnessMutex,0);
}
inline void returnBrightnessMutex(){
  xSemaphoreGive(xBrightnessMutex);
}

void adjustBrightness(float lux){
  takeBrightnessMutex();
  BRIGHTNESS=255 - lux
  returnBrightnessMutex();
}



void ambientSensor(){
  //this will monitor ambient light sensor
  for(;;){
    BH1750.start();   //starts a measurement
    float lux=BH1750.getLux();  //  waits until a conversion finished
    Serial.println(lux);
    adjustBrightness(lux);
    Serial.print("adjusting brightness");
    vTaskDelay(10);
    //change to using signal if brightness change reaches a delta??
  }
}

inline void takeStateMutex(){
  xSemaphoreTake(xStateMutex,0);
}
inline void returnStateMutex(){
  xSemaphoreGive(xStateMutex);
}

void togglePower(){
  takeStateMutex();
  STATE=(STATE+1)%2;
  returnStateMutex();
}

void motionSensor(){
  //this will monitor the motion sensor for on/off
  for(;;){
    if(digitalRead(MOTION_PIN)){
      togglePower();
    }
    Serial.print(STATE);
  }
}

void LEDController(){
  //this will adjust brightness based on ambient light sensor
  for(;;){
    takeBrightnessMutex();
    FastLED.setBrightness(BRIGHTNESS);
    returnBrightnessMutex();
    //try and scale brightness gradually
  }
}

//designate functions for different light modes
//use signals to start/stop different ones from running
void LEDCascade(){}






void setup() {
  //set pinMode
  pinMode(MOTION_PIN, INPUT);

  //mutex creation
  xStateMutex = xSemaphoreCreateMutex();
  xBrightnessMutex = xSemaphoreCreateMutex();

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(25); // 0-255

  //initialize sensor and print if error
  bool avail = BH1750.begin(BH1750_TO_GROUND);
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {};                                        
  }

  xTaskCreate(ambientSensor, "Ambient Light Task", 128, NULL, 2, NULL, NULL);
  xTaskCreate(motionSensor, "Motion Detection Task", 128, NULL, 3, NULL, NULL);
  xTaskCreate(LEDController, "Brightness Adjust Task", 128, 1, NULL, NULL);
  

}

void loop() {
  //empty loop, going to use tasks
}
