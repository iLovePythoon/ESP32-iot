#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <freertos/semphr.h>
#define LM35_PIN 34 // Example pin for the LM35 temperature sensor
#define HUMIDITY_SENSOR_PIN 35 // Example pin for the humidity sensor

// Wi-Fi and Blynk credentials
char ssid[] = "MYSSID";
char pass[] = "MYPASSWORD";
char auth[] = "MYAPIKEY";

// Create a semaphore handle for accessing shared resources
SemaphoreHandle_t sensorDataMutex;

// Global variables for sensor data and warning messages
String WarnTemp;
String WarnHum;
float temperature;
float humidity;

void setup() {
  Serial.begin(9600); // Begin serial communication
  Blynk.begin(auth, ssid, pass); // Begin Blynk connection
  sensorDataMutex = xSemaphoreCreateMutex(); // Create a mutex for shared resource protection

  // Create tasks with designated priorities
  xTaskCreate(readSensorData, "Read Sensor Data", 10000, NULL, 3, NULL);
  xTaskCreate(processSensorData, "Process Sensor Data", 10000, NULL, 2, NULL);
  xTaskCreate(blynkTask, "Blynk Task", 10000, NULL, 1, NULL);
}

void loop() {
  // Empty loop - tasks are managed by FreeRTOS
}

// Task to read sensor data
void readSensorData(void *parameter) {
  for (;;) {
    // Read from analog pins
    int sensorValue_T = analogRead(LM35_PIN);
    int sensorValue_H = analogRead(HUMIDITY_SENSOR_PIN);

    // Protect shared resources with semaphore
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY)){
      // Convert sensor readings to meaningful values
      float voltage = sensorValue_T * 3.3 / 4095.0;
      temperature = voltage / 0.008;
      humidity = (sensorValue_H / 4095.0) * 100.0;

      // Print readings to Serial Monitor
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      // Release semaphore after accessing shared resources
      xSemaphoreGive(sensorDataMutex);
    }

    // Wait for 1 second before next reading
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task to process sensor data
void processSensorData(void *parameter) {
  for (;;) {
    // Protect shared resources with semaphore
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY)){
      // Check temperature and humidity conditions and set warnings
      if (temperature > 30.0) {
        WarnTemp="Warning High Temperature";
      }
      else{
        WarnTemp="Normal Temperature";
      }

      if (humidity > 80.0) {
        WarnHum="High Humidity Detected!";
      } 
      else if (humidity < 30.0) {
        WarnHum="Warning: Low humidity detected!";
      }
      else{
        WarnHum="Normal Humidity";
      }
      Serial.println(WarnHum); // Print humidity warning
      Serial.println(WarnTemp); // Print temperature warning

      // Release semaphore after accessing shared resources
      xSemaphoreGive(sensorDataMutex);
    }
    
    // Wait for 3 seconds before next processing
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// Task to send data to Blynk
void blynkTask(void *parameter) {
  for (;;) {
    Blynk.run(); // Necessary to run Blynk functions
    // Protect shared resources with semaphore
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY)){
      // Send data and warnings to Blynk app
      Blynk.virtualWrite(V5, temperature);
      Blynk.virtualWrite(V6, humidity);
      Blynk.virtualWrite(V4, WarnTemp);   
      Blynk.virtualWrite(V3, WarnHum);    

      // Release semaphore after accessing shared resources
      xSemaphoreGive(sensorDataMutex);
    }
    
    // Wait for 6 seconds before next update to Blynk
    vTaskDelay(pdMS_TO_TICKS(6000));
  }
}
