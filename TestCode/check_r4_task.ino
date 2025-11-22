#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>


void PrintTask(void *pvParameters) {
  while (true) {
    Serial.println("Hello from FreeRTOS task");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}
  xTaskCreate(PrintTask, "Print", 256, NULL, 1, NULL);
  vTaskStartScheduler();
}
void loop() {}