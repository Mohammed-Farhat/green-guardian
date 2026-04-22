// ================================================================
// GREEN GUARDIAN — Arduino #1
// Motor Control + Encoder Odometry
// Final Version
//
// Serial protocol:
//   Receive from Pi: M:LEFT,RIGHT\n   (-249 to 249)
//   Send to Pi:      O:LEFT,RIGHT\n   (encoder ticks, 20Hz)
//                    OK\n             (command acknowledged)
//                    E:msg\n          (error)
//                    GG:ready\n       (boot confirmation)
//
// Watchdog: motors stop if no command received for 500ms
//
// Pin Map:
//   2  → Right encoder A (interrupt)
//   3  → Left  encoder A (interrupt)
//   4  → Left  encoder B (digital)
//   5  → IN1B  LEFT  forward PWM
//   6  → IN2A  RIGHT reverse PWM
//   7  → ENA   RIGHT enable
//   8  → ENB   LEFT  enable
//   9  → IN2B  LEFT  reverse PWM
//   10 → IN1A  RIGHT forward PWM
//   12 → Right encoder B (digital)
// ================================================================

// --- Motor driver pins ---
#define ENA   7
#define ENB   8
#define IN1A  10   // RIGHT forward PWM
#define IN2A  6    // RIGHT reverse PWM
#define IN1B  5    // LEFT  forward PWM
#define IN2B  9    // LEFT  reverse PWM

// --- Encoder pins ---
#define ENC_RIGHT_A  2    // interrupt 0
#define ENC_LEFT_A   3    // interrupt 1
#define ENC_RIGHT_B  12
#define ENC_LEFT_B   4

// --- Safety ---
#define MAX_PWM       249   // 98% of 255
#define WATCHDOG_MS   500   // stop if no command for 500ms
#define ODOM_INTERVAL 50    // send odometry every 50ms = 20Hz

// --- Encoder counters ---
volatile long enc_right = 0;
volatile long enc_left  = 0;

// --- Timing ---
unsigned long last_cmd_time  = 0;
unsigned long last_odom_time = 0;

// --- Serial buffer (fixed array avoids heap fragmentation) ---
char    serial_buffer[32];
uint8_t serial_buf_len = 0;

// --- Watchdog state ---
bool motors_stopped = false;

// ================================================================
// INTERRUPT SERVICE ROUTINES
// RIGHT is flipped because motor is physically mirrored on chassis
// ================================================================
void isr_right() {
  if (digitalRead(ENC_RIGHT_B) == HIGH) enc_right--;  // flipped
  else                                   enc_right++;  // flipped
}

void isr_left() {
  if (digitalRead(ENC_LEFT_B) == HIGH) enc_left++;
  else                                  enc_left--;
}

// ================================================================
// MOTOR CONTROL
// ================================================================
int clampPWM(int val) {
  if (val > MAX_PWM) return MAX_PWM;
  if (val < 0)       return 0;
  return val;
}

// speed: -249 to 249
// positive = forward, negative = reverse
void setRight(int speed) {
  if (speed >= 0) {
    analogWrite(IN1A, clampPWM(speed));
    analogWrite(IN2A, 0);
  } else {
    analogWrite(IN1A, 0);
    analogWrite(IN2A, clampPWM(-speed));
  }
}

void setLeft(int speed) {
  if (speed >= 0) {
    analogWrite(IN1B, clampPWM(speed));
    analogWrite(IN2B, 0);
  } else {
    analogWrite(IN1B, 0);
    analogWrite(IN2B, clampPWM(-speed));
  }
}

void stopMotors() {
  analogWrite(IN1A, 0); analogWrite(IN2A, 0);
  analogWrite(IN1B, 0); analogWrite(IN2B, 0);
}

// ================================================================
// SERIAL COMMAND PARSING
// Format: M:LEFT,RIGHT\n
// Example: M:200,-150\n
// ================================================================
void parseCommand(char* cmd) {
  // Strip trailing carriage return / spaces
  uint8_t len = strlen(cmd);
  while (len > 0 && (cmd[len - 1] == '\r' || cmd[len - 1] == ' ')) {
    cmd[--len] = '\0';
  }

  if (strncmp(cmd, "M:", 2) == 0) {
    char* comma = strchr(cmd + 2, ',');
    if (comma == NULL) {
      Serial.println("E:bad format, expected M:LEFT,RIGHT");
      return;
    }

    *comma = '\0';
    int left_speed  = atoi(cmd + 2);
    int right_speed = atoi(comma + 1);

    // Clamp to safe range
    left_speed  = constrain(left_speed,  -249, 249);
    right_speed = constrain(right_speed, -249, 249);

    setLeft(left_speed);
    setRight(right_speed);

    last_cmd_time  = millis();
    motors_stopped = false;
    Serial.println("OK");

  } else {
    Serial.print("E:unknown command: ");
    Serial.println(cmd);
  }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  // Motor driver
  pinMode(ENA,  OUTPUT); digitalWrite(ENA, HIGH);
  pinMode(ENB,  OUTPUT); digitalWrite(ENB, HIGH);
  pinMode(IN1A, OUTPUT); pinMode(IN2A, OUTPUT);
  pinMode(IN1B, OUTPUT); pinMode(IN2B, OUTPUT);
  stopMotors();

  // Encoders
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_A,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  pinMode(ENC_LEFT_B,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isr_right, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  isr_left,  RISING);

  last_cmd_time  = millis();
  last_odom_time = millis();

  Serial.println("GG:ready");
}

// ================================================================
// MAIN LOOP
// ================================================================
void loop() {

  // --- Read serial input ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serial_buffer[serial_buf_len] = '\0';
      parseCommand(serial_buffer);
      serial_buf_len = 0;
    } else if (serial_buf_len < sizeof(serial_buffer) - 1) {
      serial_buffer[serial_buf_len++] = c;
    } else {
      serial_buf_len = 0;  // buffer overflow — discard and reset
    }
  }

  // --- Watchdog ---
  if (millis() - last_cmd_time > WATCHDOG_MS) {
    if (!motors_stopped) {
      stopMotors();
      motors_stopped = true;
    }
  }

  // --- Send odometry at 20Hz ---
  if (millis() - last_odom_time >= ODOM_INTERVAL) {
    last_odom_time += ODOM_INTERVAL;

    // Atomically read and reset tick counters
    noInterrupts();
    long r = enc_right;
    long l = enc_left;
    enc_right = 0;
    enc_left  = 0;
    interrupts();

    Serial.print("O:");
    Serial.print(l);
    Serial.print(",");
    Serial.println(r);
  }
}