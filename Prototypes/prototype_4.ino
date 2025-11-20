// --- Includes and Definitions ---
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <hp_BH1750.h>
#include <FastLED.h>

#define MAX_LUX 3000          // Sets upper limit for Lux
#define NUM_LEDS 1           // Number of LEDs used in strip
#define INIT_BRIGHTNESS 50    // Sets initial brightness for strip

#define MOTION_PIN 2          // PIR sensor input
#define LED_PIN 9             // PWM-capable pin for LED strip

hp_BH1750 BH1750;             // Ambient Sensor
CRGB leds[NUM_LEDS];

// --- Shared Variables ---
volatile bool motionDetected = false;
volatile int targetBrightness = INIT_BRIGHTNESS;
volatile int currentBrightness = INIT_BRIGHTNESS;
volatile int isOn = 0;


void setup() {
  pinMode(MOTION_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.print("setup\n");

  // Initialize sensor, print if failed
  bool avail = BH1750.begin(BH1750_TO_GROUND);
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {};                                        
  }

  // Setup LED strip
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(INIT_BRIGHTNESS); // 0-255

}

void loop() {
  // --- Motion Sensor --- //
  if (digitalRead(MOTION_PIN) == HIGH) {
      motionDetected = true;
      Serial.println("motion detected");
    } else {
      motionDetected = false;
    }
  // --- Light Sensor --- //
  BH1750.start();   //starts a measurement
  float lux = BH1750.getLux();  //  waits until a conversion finished
  if(lux>5000){lux=MAX_LUX;} //caps lux input to 3000
  int mapped = map(lux, 0, MAX_LUX, 0, 255);  // Normalize to PWM range
  targetBrightness = mapped;  

  if (motionDetected && !isOn) {        
    isOn=1;;
    currentBrightness = (currentBrightness + targetBrightness)/2;    // Set brightness to average of target and current        
    FastLED.setBrightness(currentBrightness);
    stripRedSolid(); 
  }
  else if(motionDetected && isOn) {
    isOn=0;
    FastLED.clear();  // Turn off LED
    FastLED.show();
  }
  delay(5000);
}

// Helper Functions
void stripRedSolid(){
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB::Red;
  }
  FastLED.show();
}
