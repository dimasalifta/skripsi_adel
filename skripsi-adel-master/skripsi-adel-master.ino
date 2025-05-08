#include "sensor_max_30102.h"

#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  max30102_setup();
  
  Serial.println("=========Adafruit MLX90614 test========");
  Serial.println("");

  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

  Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());
  Serial.println("=======================================");

  
}

void loop()
{
  double* health = getHealthData();
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  
  Serial.print("Heart Rate: ");
  Serial.print(health[0]);  // BPM is at index 0
  Serial.print("|| SpO2: ");
  Serial.println(health[1]);  // SpO2 is at index 1

}
