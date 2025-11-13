
// --- Includes and Definitions ---
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <hp_BH1750.h>
#include <FastLED.h>

#define MAX_LUX 3000          // Sets upper limit for Lux
#define NUM_LEDS 10           // Number of LEDs used in strip
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

// --- Synchronization Primitives ---
SemaphoreHandle_t motionSemaphore;
SemaphoreHandle_t brightnessMutex;

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
  // Create synchronization primitives
  motionSemaphore = xSemaphoreCreateBinary();
  brightnessMutex = xSemaphoreCreateMutex();

  // Initialize sensor, print if failed
  bool avail = BH1750.begin(BH1750_TO_GROUND);
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {};                                        
  }

  // Setup LED strip
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(INIT_BRIGHTNESS); // 0-255

  // Create tasks
  xTaskCreate(MotionTask, "Motion", 128, NULL, 2, NULL);
  xTaskCreate(LightSensorTask, "LightSensor", 128, NULL, 1, NULL);
  xTaskCreate(LEDControlTask, "LEDControl", 128, NULL, 1, NULL);
}

// --- Loop (unused in FreeRTOS) ---
void loop() {
  // Empty — FreeRTOS handles scheduling
}

// --- Task Definitions ---

// Motion Detection Task
void MotionTask(void *pvParameters) {
  while (true) {
    if (digitalRead(MOTION_PIN) == HIGH) {
      motionDetected = true;
      xSemaphoreGive(motionSemaphore);  // Notify LED task
    } else {
      motionDetected = false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Poll every 100ms
  }
}

// Ambient Light Sensor Task
void LightSensorTask(void *pvParameters) {
  while (true) {
    BH1750.start();   //starts a measurement
    float lux=BH1750.getLux();  //  waits until a conversion finished
Serial.print("lux: ");   // Debug print lux
Serial.println(lux, 2);  // Prints with 2 decimal places
    if(lux>5000){lux=MAX_LUX;} //caps lux input to 3000
    int mapped = map(lux, 0, MAX_LUX, 0, 255);  // Normalize to PWM range

    if (xSemaphoreTake(brightnessMutex, portMAX_DELAY)) {
      targetBrightness = mapped;  
      xSemaphoreGive(brightnessMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(500));  // Sample every 500ms
  }
}

// LED Control Task
void LEDControlTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(motionSemaphore, portMAX_DELAY)) {
      if (motionDetected && !isOn) {
        if (xSemaphoreTake(brightnessMutex, portMAX_DELAY)) {
          currentBrightness = (currentBrightness + targetBrightness)/2;    // Set brightness to average of target and current
          FastLED.setBrightness(currentBrightness);
          xSemaphoreGive(brightnessMutex);
          stripRedSolid();
        }
      }
      if(motionDetected && isOn) {
        stripOff();  // Turn off LED
      }
    }
  }
}

// Helper Functions
void stripOff(){
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}
void stripRedSolid(){
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB::Red;
  }
  FastLED.show();
}