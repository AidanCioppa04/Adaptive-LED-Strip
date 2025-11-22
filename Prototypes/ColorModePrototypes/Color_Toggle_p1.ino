/*
This implimentation is an adaped inplementation in which the strip is always on
The motion sensor is used to cycle between color patterns instead of determining on/off
Due to task function, LEDControlTask will handle this new behavior, as MotionTask only determines
if motion is detected or not

 -- TO ADD COLOR MODES --
update NUM_COLOR_MODES
inlude function signature as prototype, add to array, create function
*/
// --- Includes and Definitions ---
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <hp_BH1750.h>
#include <FastLED.h>

#define MAX_LUX 3000          // Sets upper limit for Lux
#define NUM_LEDS 5           // Number of LEDs used in strip
#define INIT_BRIGHTNESS 50    // Sets initial brightness for strip
#define FLOOR_BRIGHTNESS 5    // Brightness cannot drop below this
#define NUM_COLOR_MODES 3      // Number of different color modes

#define MOTION_PIN 2          // PIR sensor input
#define LED_PIN 9             // PWM-capable pin for LED strip

hp_BH1750 BH1750;             // Ambient Sensor
CRGB leds[NUM_LEDS];

// --- Shared Variables ---
volatile bool motionDetected = false;
volatile int targetBrightness = INIT_BRIGHTNESS;
volatile int currentBrightness = INIT_BRIGHTNESS;

// --- Task Prototypes ---
void MotionTask(void *pvParameters);
void LightSensorTask(void *pvParameters);
void LEDControlTask(void *pvParameters);

// --- Function Prototypes ---
void stripRedSolid();
void stripBlueSolid();
void stripAmethystSolid();

//Queue for modes
void (*colorModes[NUM_COLOR_MODES])() = {stripRedSolid, stripBlueSolid, stripAmethystSolid};

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

  // Create tasks
  BaseType_t result;

  result = xTaskCreate(MotionTask, "Motion", 512, NULL, 2, NULL); //96
  Serial.println(result == pdPASS ? "MotionTask created" : "MotionTask creation failed"); 
  
  result = xTaskCreate(LightSensorTask, "LightSensor", 512, NULL, 1, NULL); //128
  Serial.println(result == pdPASS ? "LightSensorTask created" : "LightSensorTask creation failed");
  
  result = xTaskCreate(LEDControlTask, "LEDControl", 512, NULL, 1, NULL); //128
  Serial.println(result == pdPASS ? "LEDControlTask created" : "LEDControlTask creation failed");

  vTaskStartScheduler();
}
void loop() {
}

// --- Task Definitions ---

// Motion Detection Task
void MotionTask(void *pvParameters) {
  while (true) {
    if (digitalRead(MOTION_PIN) == HIGH) {
      Serial.println("motion detected");
      motionDetected = true;  // Notify LED task
      vTaskDelay(pdMS_TO_TICKS(3000)); //delay task until lockout period ends
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
    int lux = (int)BH1750.getLux();  //  waits until a conversion finished
    if(lux>255){lux=MAX_LUX;} //caps lux input to 3000
    lux = map(lux, 0, MAX_LUX, 0, 255 - FLOOR_BRIGHTNESS);  // Normalize to PWM range
    targetBrightness = lux + FLOOR_BRIGHTNESS;
    vTaskDelay(pdMS_TO_TICKS(300));  // Sample every 300ms
  }
}

// LED Control Task
void LEDControlTask(void *pvParameters) {
  cycleColor();
  while (true) {
    if (motionDetected) {
      Serial.println("cycling");
      motionDetected = false;
      cycleColor();
    }
    currentBrightness = (currentBrightness + targetBrightness)/2;    // Set brightness to average of target and current
    FastLED.setBrightness(currentBrightness);
    FastLED.show();
    Serial.print("set brightness:");
    Serial.println(currentBrightness);
    vTaskDelay(pdMS_TO_TICKS(500)); //poll every half second
  }
}

// Helper Functions

//Color modes
void stripRedSolid(){
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB::Red;
  }
  FastLED.show();
}

void stripBlueSolid(){
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB::Blue;
  }
  FastLED.show();
}

void stripAmethystSolid(){
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB::Amethyst;
  }
  FastLED.show();
}

//Cycle Function
int cur=0;
void cycleColor(){
  colorModes[cur]();
  cur=(cur+1)%NUM_COLOR_MODES;
}
