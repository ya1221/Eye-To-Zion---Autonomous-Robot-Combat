# Wiring Guide: Eye-To-Zion

| Component               | Protocol | Pi 5 Pin (Physical) | Pi 5 BCM / GPIO | Wire Color / Notes                     | Voltage / Logic |
|-------------------------|----------|---------------------|-----------------|----------------------------------------|-----------------|
| **L298N Motor Driver**  |          |                     |                 |                                        |                 |
| - Left Motor ENA (PWM)  | PWM      | Pin 32              | GPIO 12 (PWM0)  | Green (Suggestion)                     | 3.3V Logic      |
| - Left Motor IN1        | Digital  | Pin 11              | GPIO 17         | Blue (Suggestion)                      | 3.3V Logic      |
| - Left Motor IN2        | Digital  | Pin 13              | GPIO 27         | Purple (Suggestion)                    | 3.3V Logic      |
| - Right Motor ENB (PWM) | PWM      | Pin 33              | GPIO 13 (PWM1)  | Yellow (Suggestion)                    | 3.3V Logic      |
| - Right Motor IN3       | Digital  | Pin 15              | GPIO 22         | Orange (Suggestion)                    | 3.3V Logic      |
| - Right Motor IN4       | Digital  | Pin 16              | GPIO 23         | Brown (Suggestion)                     | 3.3V Logic      |
| - Motor Ground          | Power    | Pin 6 or 14         | GND             | Black (Must share common GND with Pi!) | GND             |
| **ICM20948 IMU**        |          |                     |                 |                                        |                 |
| - VCC                   | Power    | Pin 1               | 3.3V Power      | Red                                    | 3.3V            |
| - GND                   | Power    | Pin 9               | GND             | Black                                  | GND             |
| - SDA                   | I2C1     | Pin 3               | GPIO 2 (SDA)    | Blue                                   | 3.3V            |
| - SCL                   | I2C1     | Pin 5               | GPIO 3 (SCL)    | Yellow                                 | 3.3V            |
| **Arduino Uno/Nano**    |          |                     |                 |                                        |                 |
| - Steering / Flag Comms | USB      | USB Port            | `/dev/ttyACM0`  | USB Cable                              | 5V via USB      |
| **YDLidar**             |          |                     |                 |                                        |                 |
| - Lidar Comms & Power   | USB      | USB Port            | `/dev/ttyUSB0`  | USB Cable                              | 5V via USB      |
| **Pi Camera**           |          |                     |                 |                                        |                 |
| - Camera Module         | MIPI CSI | CSI Port            | CSI             | Ribbon Cable                           | N/A             |
| **ICS-43434 I2S Mic**   |          |                     |                 |                                        |                 |
| - 3V3 (VCC)             | Power    | Pin 1 or 17         | 3.3V Power      | Red                                    | 3.3V            |
| - GND                   | Power    | Pin 9 or 39         | GND             | Black                                  | GND             |
| - SCK (BCLK)            | I2S      | Pin 12              | GPIO 18 (PCM_C) | Green (Suggestion)                     | 3.3V            |
| - WS (LRCLK)            | I2S      | Pin 35              | GPIO 19 (PCM_FS)| Yellow (Suggestion)                    | 3.3V            |
| - SD (Data Out)         | I2S      | Pin 38              | GPIO 20 (PCM_DI)| Blue (Suggestion)                      | 3.3V            |
| - L/R (Channel Select)  | Power    | Pin 9 or 39 (GND)   | GND (Left Ch)   | Tie to Ground for Left channel         | GND             |

## Power Supply Notes
*   **Raspberry Pi 5**: Requires a 5V/5A USB-C power supply for full performance.
*   **L298N Motor Driver**: Do **NOT** power the motors from the Raspberry Pi's 5V pins! Connect a dedicated battery (e.g., 7.4V LiPo or 12V battery pack) to the `12V` and `GND` terminals of the L298N. Make sure to connect the L298N's `GND` to one of the Raspberry Pi's `GND` pins to share a common ground logic reference.

## Component Block Diagram

```text
       [ Battery ]
        |       |
      (VCC)   (GND)
        |       |
  +-----v-------v-----+               +-------------------+
  |   L298N Driver    |               |  Raspberry Pi 5   |
  |                   |               |                   |
  | ENA   IN1   IN2   |<-- PWM/GPIO --| GPIO12, 17, 27    |
  | ENB   IN3   IN4   |<-- PWM/GPIO --| GPIO13, 22, 23    |
  |                   |               |                   |
  | GND (Common Ground)---------------| GND               |
  +-------------------+               +-------------------+
                                         |      |      |
                   I2C (SDA/SCL) --------+      |      |
                                 |              |      |
                          +------v-------+      |      |
                          | ICM20948 IMU |      |      |
                          +--------------+      |      |
                                                |      |
                    I2S (BCLK/WS/SD) -----------+      |
                                  |              |      |
                           +------v-------+      |      |
                           |  ICS-43434   |      |      |
                           +--------------+      |      |
                                                 |      |
                                  USB -----------+      |
                                  |                     |
                           +------v-------+             |
                           |  Arduino Uno |             |
                           | (Steering)   |             |
                           +--------------+             |
                                                       |
                                 USB ------------------+
                                 |
                          +------v-------+
                          |   YDLidar    |
                          +--------------+
```
