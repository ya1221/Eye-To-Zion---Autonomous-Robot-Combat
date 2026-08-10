// ============================================================
// Ackermann Steering Servo Controller
// ============================================================
// Receives two Ackermann wheel angles (radians) from a
// Raspberry Pi over USB-serial, computes the exact bicycle-
// model center angle via the cotangent relation, and drives
// one servo on pin 7.
//
// Protocol from ROS2 hardware interface:
//   S<left_rad>,<right_rad>\n      (two wheels)
//   S<rad>\n                       (single value fallback)
//   F\n                            (shoot once - pulse HIGH then LOW)
//   F1\n                           (shooting ON  - flag pin HIGH)
//   F0\n                           (shooting OFF - flag pin LOW)
//
// Ackermann inverse (cotangent average):
//   cot(δ_center) = ( cot(δ_left) + cot(δ_right) ) / 2
//
// Rewritten to avoid division-by-zero at straight ahead:
//   tan(δ_center) = 2·tan(δ_L)·tan(δ_R) / (tan(δ_L)+tan(δ_R))
// ============================================================

#include <Servo.h>

// ---------------------- Configuration -----------------------
const int    SERVO_PIN         = 7;
const int    SERVO_CENTER_DEG  = 90;    // Servo neutral - adjust to match your linkage
const int    SERVO_MIN_DEG     = 45;    // Mechanical limit
const int    SERVO_MAX_DEG     = 135;   // Mechanical limit
const float  STEERING_SIGN     = 1.0;   // Flip to -1.0 if servo turns the wrong way
const unsigned long CMD_TIMEOUT_MS = 500;

const int    FLAG_PIN          = 8;     // Digital output for shooting mechanism

// --- NEW: Shooting Configuration ---
const unsigned long SHOOT_DURATION_MS = 100; // How long the pin stays HIGH for a single shot

// ---------------------- Globals -----------------------------
Servo steeringServo;

char    rxBuffer[64];
int     rxIndex = 0;
unsigned long lastCmdTime = 0;

// --- NEW: Shooting Timer Globals ---
unsigned long shootStartTime = 0;
bool          isShooting     = false;

// ---------------------- Setup -------------------------------
void setup() {
    Serial.begin(115200);

    steeringServo.attach(SERVO_PIN);
    steeringServo.write(SERVO_CENTER_DEG);

    pinMode(FLAG_PIN, OUTPUT);
    digitalWrite(FLAG_PIN, LOW);

    lastCmdTime = millis();
}

// ---------------------- Main Loop ---------------------------
void loop() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
            rxBuffer[rxIndex] = '\0';
            processCommand(rxBuffer);
            rxIndex = 0;
        } else if (c == '\r') {
            // Ignore carriage returns
        } else if (rxIndex < (int)sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = c;
        } else {
            rxIndex = 0;  // Buffer overflow - discard
        }
    }

    // Safety: center the servo if commands stop arriving
    if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
        steeringServo.write(SERVO_CENTER_DEG);
    }

    // --- NEW: Non-blocking shooting pulse check ---
    if (isShooting && (millis() - shootStartTime >= SHOOT_DURATION_MS)) {
        digitalWrite(FLAG_PIN, LOW);
        isShooting = false;
    }
}

// ---------------------- Ackermann Inverse --------------------
float ackermann_center_angle(float leftRad, float rightRad) {
    float tanL = tanf(leftRad);
    float tanR = tanf(rightRad);
    float sumTan = tanL + tanR;

    if (fabsf(sumTan) < 1e-6f) {
        return 0.0f;
    }

    return atanf(2.0f * tanL * tanR / sumTan);
}

// ---------------------- Command Parser ----------------------
void processCommand(const char* cmd) {
    if (cmd[0] == 'F') {
        handleFlag(cmd);
        return;
    }

    if (cmd[0] != 'S') return;

    float leftRad  = 0.0f;
    float rightRad = 0.0f;

    const char* comma = strchr(cmd + 1, ',');

    if (comma) {
        leftRad  = atof(cmd + 1);
        rightRad = atof(comma + 1);
    } else {
        leftRad  = atof(cmd + 1);
        rightRad = leftRad;
    }

    // Exact Ackermann inverse
    float centerRad = ackermann_center_angle(leftRad, rightRad);

    // Convert radians -> servo degrees
    float offsetDeg = centerRad * RAD_TO_DEG * STEERING_SIGN;
    int servoAngle  = SERVO_CENTER_DEG + (int)(offsetDeg + 0.5f);

    servoAngle = constrain(servoAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);

    steeringServo.write(servoAngle);
    lastCmdTime = millis();
}

// ---------------------- Flag Output Handler ------------------
// F\n  -> shoot once (pulse pin HIGH then LOW)
// F1\n -> shooting ON (pin HIGH)
// F0\n -> shooting OFF (pin LOW)
void handleFlag(const char* cmd) {
    // DEBUG: Blink built-in LED to confirm command received
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);

    if (cmd[1] == '\0') {
        Serial.println("LOG: Single shot triggered (Pulse)");
        digitalWrite(FLAG_PIN, HIGH);
        shootStartTime = millis();
        isShooting = true;
        return;
    }
    
    if (cmd[1] == '0') {
        Serial.println("LOG: Continuous shooting OFF");
        digitalWrite(FLAG_PIN, LOW);
        isShooting = false;
    } else if (cmd[1] == '1') {
        Serial.println("LOG: Continuous shooting ON");
        digitalWrite(FLAG_PIN, HIGH);
        isShooting = false;
    } else {
        // Optional: Catch any malformed commands (e.g., if you accidentally sent "F2")
        Serial.print("LOG: Unknown F command received: ");
        Serial.println(cmd);
    }
}
