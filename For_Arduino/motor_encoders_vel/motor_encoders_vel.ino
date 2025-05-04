// Motor Driver Pins
#define LEFT_PWM_PIN 10   // ENA (Left Motor PWM)
#define LEFT_IN1 6        // Left Motor Direction 1
#define LEFT_IN2 8        // Left Motor Direction 2
#define RIGHT_PWM_PIN 11  // ENB (Right Motor PWM)
#define RIGHT_IN1 4       // Right Motor Direction 1
#define RIGHT_IN2 5       // Right Motor Direction 2

// Encoder Pins
#define ENCODER_LEFT_A 2  // Left Encoder A (Interrupt)
#define ENCODER_LEFT_B 9  // Left Encoder B
#define ENCODER_RIGHT_A 3 // Right Encoder A (Interrupt)
#define ENCODER_RIGHT_B 7 // Right Encoder B

// Robot Parameters
#define WHEEL_RADIUS 0.03   // 3 cm (meters)
#define WHEEL_BASE 0.20     // 20 cm between wheels
#define MAX_SPEED 1.23      // Max linear speed (m/s)
#define LOOPTIME 50 // 20Hz
// Encoder Variables
volatile int left_ticks = 0, right_ticks = 0;
unsigned long prev_time = 0;

// Interrupt Service Routines (ISRs) for Encoders
void leftEncoderISR() { left_ticks++; }
void rightEncoderISR() { right_ticks++; }
float Wz,Vx;
// Function to Convert Velocity to PWM
int velocityToPWM(float velocity) {
    return (int)constrain(round((velocity / MAX_SPEED) * 255), -255, 255);
}

// Function to Compute and Apply Wheel Speeds
void processVelocity(float Vx, float Wz) {
    float left_speed = Vx - (Wz * WHEEL_BASE / 2);
    float right_speed = Vx + (Wz * WHEEL_BASE / 2);

    int left_pwm = velocityToPWM(left_speed);
    int right_pwm = velocityToPWM(right_speed);

    Serial.print("Left PWM: "); Serial.print(left_pwm);
    Serial.print(" | Right PWM: "); Serial.println(right_pwm);

    // Apply PWM to Motors
    analogWrite(LEFT_PWM_PIN, abs(left_pwm));
    analogWrite(RIGHT_PWM_PIN, abs(right_pwm));

    // Control Motor Directions
    digitalWrite(LEFT_IN1, left_pwm >= 0 ? HIGH : LOW);
    digitalWrite(LEFT_IN2, left_pwm >= 0 ? LOW : HIGH);
    digitalWrite(RIGHT_IN1, right_pwm >= 0 ? HIGH : LOW);
    digitalWrite(RIGHT_IN2, right_pwm >= 0 ? LOW : HIGH);
}

// Function to Compute Wheel Speed from Encoders
void computeWheelSpeed() {
    unsigned long current_time = millis();
    float dt = (current_time - prev_time) / 1000.0; // Convert ms to seconds

    if (dt > 0) {
        float left_wheel_speed = (left_ticks / 240.0) * (2 * PI * WHEEL_RADIUS) / dt;
        float right_wheel_speed = (right_ticks / 240.0) * (2 * PI * WHEEL_RADIUS) / dt;
          Serial.print("speed ");
          Serial.print(left_wheel_speed);
          Serial.print(" ");
          Serial.print(right_wheel_speed );
          Serial.print(" ");
          Serial.println(LOOPTIME / 1000.0);

    }

    left_ticks = 0;
    right_ticks = 0;
    prev_time = current_time;
}

void setup() {
    Serial.begin(115200);

    // Motor Pins
    pinMode(LEFT_PWM_PIN, OUTPUT);
    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);
    pinMode(RIGHT_PWM_PIN, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);

    // Encoder Pins
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

    // Attach Interrupts for Encoder A Signals
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);

    prev_time = millis();
}

void loop() {
    if (Serial.available()) {

      String command = Serial.readStringUntil('\n'); // Read the full line
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex != -1) {
      String linear_str = command.substring(0, spaceIndex);
      String angular_str = command.substring(spaceIndex + 1);

       Vx = linear_str.toFloat();
       Wz = angular_str.toFloat();}

        processVelocity(Vx, Wz);

    }

    computeWheelSpeed();
    delay(100);
}
