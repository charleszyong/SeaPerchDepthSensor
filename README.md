# SeaPerchDepthSensor

SeaPerchDepthSensor is a classroom-friendly Arduino library for reading depth
from an LPS35HW pressure sensor on an Arduino UNO R4 Minima. It hides the I2C
register protocol, zero-depth calibration, data validation, timeouts, and
automatic bus recovery behind a small API.

```cpp
#include <SeaPerchDepthSensor.h>

SeaPerchDepthSensor sensor;

void setup() {
  Serial.begin(115200);
  sensor.begin();
}

void loop() {
  if (!sensor.update()) {
    return;  // No safe new reading yet; recovery happens automatically.
  }

  Serial.println(sensor.depthCm());
}
```

## Supported hardware

- Arduino UNO R4 Minima (primary, hardware-tested target)
- Arduino UNO R4 WiFi and Arduino Nano R4 (same core, compile-tested)
- A 5 V-safe, level-shifting LPS35HW breakout at I2C address `0x5D`, such as
  the Adafruit LPS35HW breakout used by the original project
- Freshwater by default; saltwater conversion is selectable

This first release intentionally targets boards in Arduino's `renesas_uno`
platform because its recovery code depends on that core's bounded `Wire`
timeout behavior. Selecting another architecture produces a compile-time
message instead of silently removing the safety guarantees.

Connect the breakout to the UNO R4 Minima's `SDA`, `SCL`, power, and ground
pins. Follow the breakout manufacturer's voltage instructions. The Adafruit
breakout includes a regulator and level shifting, so its `Vin` can use the
UNO's 5 V supply. Use external I2C pull-ups when they are not already present
on the sensor board. Keep the bus wiring short and address motor electrical
noise with suitable suppression, decoupling, and grounding.

Do **not** connect a bare LPS35HW chip directly to 5 V power or logic. The bare
device is a 1.7-3.6 V part and requires suitable regulation and bidirectional
level shifting. The LPS35HW sensor package is water resistant; a breakout PCB,
wires, solder joints, and connectors are not automatically waterproof.

On UNO R4 WiFi, use the primary `SDA`/`SCL` pins at A4/A5. This release does
not manage the separate Qwiic connector's `Wire1` bus.

Hardware references:

- [Adafruit LPS35HW breakout pinout](https://learn.adafruit.com/lps35hw-water-resistant-pressure-sensor/pinouts)
- [ST LPS35HW datasheet](https://www.st.com/resource/en/datasheet/lps35hw.pdf)

## Install

After the library is accepted into Arduino Library Manager:

1. Open **Tools > Manage Libraries...** in Arduino IDE.
2. Search for **SeaPerchDepthSensor**.
3. Select the latest version and click **Install**.

Before registry acceptance, download a release ZIP from GitHub and use
**Sketch > Include Library > Add .ZIP Library...**.

## Basic API

```cpp
SeaPerchDepthSensor sensor;

sensor.begin();                 // Calibrate the current pressure as 0 cm.
sensor.update();                // True only for a new validated reading.
sensor.ready();                 // True when a safe reading is published.
sensor.depthCm();               // Depth relative to startup, positive down.
sensor.pressureHpa();           // Absolute pressure in hPa.
sensor.temperatureC();          // Temperature in degrees Celsius.
sensor.temperatureF();          // Temperature in degrees Fahrenheit.
sensor.faultCount();            // Number of detected runtime sample faults.
```

Keep the sensor still at the intended zero-depth reference while `begin()` is
running. Startup calibration averages 16 fresh samples. Automatic runtime
recovery preserves this software baseline instead of redefining the current
depth as zero.

For saltwater:

```cpp
sensor.begin(nullptr, SeaPerchDepthSensor::Water::Saltwater);
```

## Motorized projects

Define a function that immediately disables the motor, configure the motor pins
to a safe state first, then pass that function to `begin()`:

```cpp
void motorOff() {
  analogWrite(enableMotor, 0);
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
}

void setup() {
  // Configure motor pins, then make them safe before starting the sensor.
  motorOff();
  sensor.begin(motorOff);
}
```

The library calls this callback before every top-level sensor operation,
including fault recovery. When `update()` returns `false`, skip all motor
control for that loop. Do not use old getter values to drive a vehicle.

The microcontroller pins are high impedance during reset, before any sketch can
run. Use a physical pull-down on the motor driver's enable input, appropriate
travel limits, and a separate emergency stop for the final vehicle.

## Recovery behavior

The library uses:

- a 100 kHz I2C clock and 5 ms transaction timeout;
- checked byte counts and finite/range validation;
- bounded sensor reset and fresh-data waits;
- an open-drain, nine-clock I2C bus-clear sequence;
- fast resume before a full sensor reset;
- a software surface-pressure baseline retained through runtime recovery; and
- no dynamic allocation or unbounded polling loops.

`update()` performs timed recovery itself. A disconnected or stuck sensor never
authorizes motor control, but software cannot repair a physically held clock
line, missing power, or severe motor EMI.

The recovery process restarts the global `Wire` bus. Projects sharing that bus
with other I2C devices must revalidate those devices after a recovery.

## Examples

In Arduino IDE, open **File > Examples > SeaPerchDepthSensor**:

- **ReadDepth** prints validated depth and temperature readings.
- **MotorSafetyCallback** demonstrates the motor-off safety callback.

## Credits and license

The simple student-facing API grew from SeaPerch depth-controller work by
Talia Seshasai. The bounded I2C driver and recovery implementation were
developed for the Arduino UNO R4 Minima version of that project.

Released under the [MIT License](LICENSE). This is a community project and is
not an official Arduino, STMicroelectronics, Adafruit, RoboNation, or SeaPerch
product.
