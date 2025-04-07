#include "sensor_max_30102.h"
void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  max30102_setup();

}

void loop()
{
  double* health = getHealthData();
  Serial.print("Heart Rate: ");
  Serial.print(health[0]);  // BPM is at index 0
  Serial.print("|| SpO2: ");
  Serial.println(health[1]);  // SpO2 is at index 1

}
