# shooting

## Purpose
Control the physical shooting mechanism (flag/servo).

## Logic
Listens for shooting commands and actuates the Arduino-connected servo to indicate a 'shot' fired.

## Data Flow
- **Input:** `/shooting_cmd` (Boolean/Trigger).
- **Output:** Serial/PWM signal to the servo mechanism.
