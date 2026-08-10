# Arduino — Steering & Shooting Controller

This directory contains the Arduino firmware for the robot's low-level
actuator control: front-wheel steering (via a single servo) and the
shooting mechanism (via a digital flag pin). The firmware runs on an
Arduino connected over USB-serial to the Raspberry Pi, which is the
higher-level compute running ROS2.

## File

- [servo_and_shooting.ino](servo_and_shooting.ino) — main (and only) sketch.

## Purpose

The Raspberry Pi computes steering angles for the robot's two front
wheels using an Ackermann steering model (as part of the ROS2 hardware
interface), and decides when to fire the shooting mechanism. The
Arduino has no knowledge of ROS2 or Ackermann geometry beyond what it
needs to convert two incoming wheel angles into a single physical servo
position, and to drive the shooter's output pin. It exists purely as a
thin, real-time serial-to-hardware bridge — all higher-level decisions
(path planning, targeting, when to shoot) are made upstream on the Pi.

## Data flow

```
ROS2 hardware interface (Raspberry Pi)
        │  USB-serial, 115200 baud, newline-terminated ASCII commands
        ▼
Arduino: processCommand()
        │
        ├── 'S' commands → ackermann_center_angle() → steeringServo.write()
        └── 'F' commands → handleFlag() → FLAG_PIN (digitalWrite)
```

1. The Pi sends single-line ASCII commands terminated with `\n` over
   USB-serial.
2. `loop()` reads incoming bytes into `rxBuffer` until a newline is
   seen, then hands the completed line to `processCommand()`.
3. Depending on the first character (`S` or `F`), the command is routed
   to the steering logic or the shooting logic.
4. The servo / flag pin state is updated immediately (no blocking
   delays in the main path, other than a short LED blink on `F`
   commands — see [Known quirks](#known-quirks)).

## Serial protocol

| Command | Meaning |
|---|---|
| `S<left_rad>,<right_rad>\n` | Two Ackermann wheel angles (radians) — left and right front wheel. |
| `S<rad>\n` | Single-angle fallback — both wheels assumed equal. |
| `F\n` | Fire once — pulses `FLAG_PIN` HIGH for `SHOOT_DURATION_MS`, then LOW. |
| `F1\n` | Shooting ON — `FLAG_PIN` held HIGH until told otherwise. |
| `F0\n` | Shooting OFF — `FLAG_PIN` held LOW. |

Angles are in **radians**; the Arduino converts to degrees internally
before writing to the servo.

## Steering: Ackermann inverse

The Pi provides independent left/right wheel angles (the correct
Ackermann geometry for each wheel), but the robot only has **one**
physical steering servo controlling both wheels via a shared linkage.
`ackermann_center_angle()` computes the single "center" bicycle-model
angle that best represents both wheel angles, using the cotangent
averaging relation:

```
cot(δ_center) = (cot(δ_left) + cot(δ_right)) / 2
```

This is rewritten in terms of tangent to avoid a division-by-zero when
driving straight ahead (where `tan(0) = 0`):

```
tan(δ_center) = 2·tan(δ_L)·tan(δ_R) / (tan(δ_L) + tan(δ_R))
```

If `tan(δ_L) + tan(δ_R)` is ~0 (both wheels near straight/opposite),
the function returns `0.0f` directly rather than dividing by a
near-zero denominator.

The resulting center angle (radians) is converted to degrees, scaled by
`STEERING_SIGN` (flip to `-1.0` if the servo linkage is mirrored),
offset from `SERVO_CENTER_DEG`, and clamped to
`[SERVO_MIN_DEG, SERVO_MAX_DEG]` before being written to the servo.

## Shooting

`FLAG_PIN` (pin 8) is a digital output, presumably driving a
relay/MOSFET for the shooting mechanism. Two modes are supported:

- **Pulse (`F`)**: sets the pin HIGH and records `shootStartTime`;
  `loop()` polls `isShooting` and clears the pin after
  `SHOOT_DURATION_MS` (100 ms) has elapsed. This is non-blocking — the
  main loop keeps servicing serial input and the steering timeout while
  the pulse is active.
- **Latched (`F1` / `F0`)**: directly sets the pin HIGH/LOW and leaves
  it there, for continuous-fire style control from the Pi.

## Safety behavior

- **Command timeout**: if no `S` command is received for
  `CMD_TIMEOUT_MS` (500 ms), the servo is forced back to
  `SERVO_CENTER_DEG` every loop iteration. This prevents the robot from
  holding a stale turn angle if the Pi hangs, disconnects, or the
  serial link drops.
- **Buffer overflow**: if an incoming line exceeds `rxBuffer` (64
  bytes) without a newline, `rxIndex` resets and the partial line is
  discarded.
- There is no equivalent timeout for the shooting flag — once latched
  ON via `F1`, it stays ON until an explicit `F0` (or `F`) is received.
  Upstream code is responsible for not leaving the shooter armed.

## Known quirks

- `handleFlag()` blocks for 50 ms (`delay(50)`) on every `F` command to
  blink the built-in LED as a debug indicator. This is a genuine
  blocking delay and will briefly stall processing of subsequent serial
  bytes/steering timeout checks while it runs.
- Malformed `F` commands (e.g. `F2`) are logged over serial
  (`LOG: Unknown F command received: ...`) but otherwise ignored.
- `S` commands with an unrecognized prefix (anything not starting with
  `S` or `F`) are silently dropped.

## Configuration constants

| Constant | Value | Meaning |
|---|---|---|
| `SERVO_PIN` | 7 | PWM pin driving the steering servo. |
| `SERVO_CENTER_DEG` | 90 | Servo angle for "wheels straight ahead" — tune to your linkage. |
| `SERVO_MIN_DEG` / `SERVO_MAX_DEG` | 45 / 135 | Mechanical steering limits. |
| `STEERING_SIGN` | 1.0 | Set to `-1.0` if the servo turns opposite to the expected direction. |
| `CMD_TIMEOUT_MS` | 500 | Time since last `S` command before forcing the servo to center. |
| `FLAG_PIN` | 8 | Digital output driving the shooting mechanism. |
| `SHOOT_DURATION_MS` | 100 | Pulse width (ms) for a single-shot `F` command. |

## Wiring summary

- Servo signal → pin 7 (PWM).
- Shooting mechanism control → pin 8 (digital out, active HIGH).
- Serial (USB) → Raspberry Pi, 115200 baud.
