#include <SeaPerchDepthSensor.h>

SeaPerchDepthSensor sensor;

const int enableMotor = 9;
const int motorIn1 = 8;
const int motorIn2 = 7;

void motorOff() {
  analogWrite(enableMotor, 0);
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
}

void setup() {
  analogWriteResolution(8);
  pinMode(enableMotor, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  motorOff();

  Serial.begin(115200);

  // The library invokes motorOff before every sensor access and recovery.
  sensor.begin(motorOff);
}

void loop() {
  if (!sensor.update()) {
    // Never run a motor from an old reading after update() returns false.
    motorOff();
    delay(5);
    return;
  }

  Serial.print("Safe depth reading: ");
  Serial.print(sensor.depthCm(), 1);
  Serial.println(" cm");

  // Put your motor-control decision here. The next update() call will invoke
  // motorOff before it starts any I2C operation.
  delay(200);
}
