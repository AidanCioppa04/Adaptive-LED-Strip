// --- Includes and Definitions ---
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <hp_BH1750.h>
#include <FastLED.h>

#define MAX_LUX 255          // Sets upper limit for Lux
#define NUM_LEDS 4           // Number of LEDs used in strip
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

// --- Task Prototypes ---
void MotionTask(void *pvParameters);
void LightSensorTask(void *pvParameters);
void LEDControlTask(void *pvParameters);

// --- Setup ---
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
  FastLED.clear();
  FastLED.show();

  // Create tasks
  BaseType_t result;

  result = xTaskCreate(LightSensorTask, "LightSensor", 128, NULL, 1, NULL);
  Serial.println(result == pdPASS ? "LightSensorTask created" : "LightSensorTask creation failed");
  result = xTaskCreate(LEDControlTask, "LEDControl", 128, NULL, 1, NULL);
  Serial.println(result == pdPASS ? "LEDControlTask created" : "LEDControlTask creation failed");
}
void loop() {
}

// --- Task Definitions ---

// Ambient Light Sensor Task
void LightSensorTask(void *pvParameters) {
  Serial.println("in light task");
  while (true) {
    BH1750.start();   //starts a measurement
    int lux = (int)BH1750.getLux();  //  waits until a conversion finished
    if(lux>255){lux=MAX_LUX;} //caps lux input to 3000
    targetBrightness = lux;  
    vTaskDelay(pdMS_TO_TICKS(300));  // Sample every 300ms
  }
}

// LED Control Task
void LEDControlTask(void *pvParameters) {
  stripRedSolid();
  while (true) {
    currentBrightness = (currentBrightness + targetBrightness)/2;    // Set brightness to average of target and current
    FastLED.setBrightness(currentBrightness);
    FastLED.show();
    Serial.print("set brightness:");
    Serial.println(currentBrightness);

    UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
    Serial.print("MotionTask stack watermark: ");
    Serial.println(watermark);

    vTaskDelay(pdMS_TO_TICKS(500)); //poll every half second
  }
}

// Helper Functions
void stripRedSolid(){
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB::Red;
  }
  FastLED.show();
}
