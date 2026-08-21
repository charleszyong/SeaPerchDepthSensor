#include <SeaPerchDepthSensor.h>

SeaPerchDepthSensor sensor;

void setup() {
  Serial.begin(115200);

  // No motor is used in this example, so no safety callback is needed.
  sensor.begin();
}

void loop() {
  if (!sensor.update()) {
    // The sensor is unavailable or recovering. Try again shortly.
    delay(5);
    return;
  }

  Serial.print("Depth: ");
  Serial.print(sensor.depthCm(), 1);
  Serial.print(" cm, temperature: ");
  Serial.print(sensor.temperatureC(), 1);
  Serial.println(" C");

  delay(200);
}
