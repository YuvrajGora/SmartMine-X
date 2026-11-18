/*
  SMARTMINE-X — Arduino UNO integrated prototype firmware
  Hardware assumed:
    - Arduino UNO / ATmega328P
    - 1 x HC-SR04 (front obstacle sensor)
    - L298N dual H-bridge
    - 4 DC motors arranged as left pair + right pair
    - MQ-2 analog gas/smoke module
    - DHT11 temperature/humidity module (3-pin breakout assumed)
    - 16x2 LCD with I2C backpack
    - Buzzer module
    - One warning LED with 220-330 ohm series resistor

  Libraries to install through Arduino IDE Library Manager:
    1. "DHT sensor library" by Adafruit
    2. "Adafruit Unified Sensor" by Adafruit (dependency, if requested)
    3. "LiquidCrystal I2C" (a common compatible library exposing
       LiquidCrystal_I2C(address, cols, rows), init(), backlight())

  IMPORTANT:
    - This is an educational prototype, NOT a certified mine-safety device.
    - MQ-2 value is raw ADC only, NOT calibrated ppm.
    - MQ-2 threshold must be calibrated in a safe, clean-air test.
    - One front ultrasonic sensor cannot see left/right obstacles.
    - Power wiring/current ratings must be verified separately.
    - Do not power motors through the Arduino or breadboard.
    - Do not connect a battery/external 5V rail to the UNO 5V pin while
      USB is connected unless you have a verified power isolation setup.
*/

// -------------------- LIBRARIES --------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// -------------------- LCD / SENSOR CONFIG --------------------
#define LCD_ADDRESS 0x27       // Try 0x3F if your LCD stays blank
#define LCD_COLUMNS 16
#define LCD_ROWS 2

#define DHT_TYPE DHT11

// -------------------- PIN ASSIGNMENTS --------------------
// HC-SR04
const byte TRIG_PIN = 2;
const byte ECHO_PIN = 3;

// L298N (remove ENA/ENB jumpers when using D5/D6 PWM)
const byte ENA_PIN = 5;         // PWM
const byte ENB_PIN = 6;         // PWM
const byte IN1_PIN = 7;
const byte IN2_PIN = 8;
const byte IN3_PIN = 9;
const byte IN4_PIN = 10;

// Analog sensors
const byte MQ2_PIN = A0;
const byte DHT_PIN = A1;

// Alerts
const byte BUZZER_PIN = A2;
const byte LED_PIN = A3;

// I2C LCD uses A4 (SDA) and A5 (SCL) on UNO.

// -------------------- TUNABLE SETTINGS --------------------
const int OBSTACLE_DISTANCE_CM = 25;

// Motor speed values: 0 to 255. Start low and increase only after
// verifying motor voltage/current and safe mechanical operation.
const int FORWARD_SPEED = 135;
const int REVERSE_SPEED = 120;
const int TURN_SPEED = 135;

// Timed blind turn because a single front sensor cannot choose a clear side.
const unsigned long REVERSE_TIME_MS = 350;
const unsigned long TURN_TIME_MS = 500;

// MQ-2 raw ADC threshold (0-1023). This is only a starting placeholder.
// Observe clean-air readings in Serial Monitor and calibrate appropriately.
const int MQ2_ALARM_THRESHOLD = 450;
const unsigned long MQ2_WARMUP_MS = 60000UL;

// Timing
const unsigned long DISTANCE_INTERVAL_MS = 100;
const unsigned long GAS_INTERVAL_MS = 200;
const unsigned long DHT_INTERVAL_MS = 2500;
const unsigned long LCD_INTERVAL_MS = 500;
const unsigned long SERIAL_INTERVAL_MS = 1000;

// HC-SR04 no-echo timeout (microseconds)
const unsigned long ECHO_TIMEOUT_US = 25000UL;

// -------------------- OBJECTS --------------------
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
DHT dht(DHT_PIN, DHT_TYPE);

// -------------------- ROVER STATES --------------------
enum RoverState {
  STATE_FORWARD,
  STATE_REVERSING,
  STATE_TURNING,
  STATE_GAS_ALERT,
  STATE_SENSOR_FAULT
};

RoverState roverState = STATE_SENSOR_FAULT;

unsigned long stateStartedAt = 0;
unsigned long lastDistanceAt = 0;
unsigned long lastGasAt = 0;
unsigned long lastDhtAt = 0;
unsigned long lastLcdAt = 0;
unsigned long lastSerialAt = 0;
unsigned long bootAt = 0;

int distanceCm = -1;            // -1 means no echo / invalid
int gasRaw = 0;
float temperatureC = NAN;
float humidityPercent = NAN;

byte invalidDistanceCount = 0;
byte validDistanceCount = 0;
bool turnRightNext = true;
bool gasAlarmActive = false;
bool mq2WarmedUp = false;

// -------------------- MOTOR CONTROL --------------------
// Positive speed = forward, negative = reverse, zero = stop.
// If one side rotates in the wrong direction, swap that side's motor
// output wires with power disconnected, or invert that side in code.
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left motor channel: OUT1/OUT2
  if (leftSpeed > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  // Right motor channel: OUT3/OUT4
  if (rightSpeed > 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  }

  analogWrite(ENA_PIN, abs(leftSpeed));
  analogWrite(ENB_PIN, abs(rightSpeed));
}

void stopMotors() {
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void beginReverse() {
  stopMotors();
  roverState = STATE_REVERSING;
  stateStartedAt = millis();
  setMotors(-REVERSE_SPEED, -REVERSE_SPEED);
  Serial.println(F("Obstacle: reversing"));
}

void beginTurn() {
  stopMotors();
  roverState = STATE_TURNING;
  stateStartedAt = millis();

  // Alternate pivot direction; this is blind because only one sensor exists.
  if (turnRightNext) {
    setMotors(TURN_SPEED, -TURN_SPEED);
    Serial.println(F("Turning right (timed, blind turn)"));
  } else {
    setMotors(-TURN_SPEED, TURN_SPEED);
    Serial.println(F("Turning left (timed, blind turn)"));
  }
  turnRightNext = !turnRightNext;
}

// -------------------- SENSOR FUNCTIONS --------------------
int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    return -1;
  }

  int cm = (int)(duration * 0.0343f / 2.0f);
  if (cm < 2 || cm > 400) {
    return -1;
  }
  return cm;
}

int readMq2Average() {
  long total = 0;
  const byte samples = 5;
  for (byte i = 0; i < samples; i++) {
    total += analogRead(MQ2_PIN);
    delay(3);
  }
  return (int)(total / samples);
}

void updateDhtReadings() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (!isnan(h) && !isnan(t)) {
    humidityPercent = h;
    temperatureC = t;
  } else {
    Serial.println(F("DHT11 read failed; check power, data pin and sensor type."));
  }
}

// -------------------- DISPLAY / ALERTS --------------------
void printLcdLine(byte row, const char *message) {
  char line[17];
  byte i = 0;

  while (i < 16 && message[i] != '\0') {
    line[i] = message[i];
    i++;
  }
  while (i < 16) {
    line[i] = ' ';
    i++;
  }
  line[16] = '\0';

  lcd.setCursor(0, row);
  lcd.print(line);
}

const char* stateText() {
  switch (roverState) {
    case STATE_FORWARD:      return "MOVING FORWARD";
    case STATE_REVERSING:    return "REVERSING";
    case STATE_TURNING:      return "TURNING";
    case STATE_GAS_ALERT:    return "GAS ALERT STOP";
    case STATE_SENSOR_FAULT: return "SENSOR CHECK";
    default:                 return "UNKNOWN";
  }
}

void updateAlerts() {
  if (roverState == STATE_GAS_ALERT) {
    digitalWrite(LED_PIN, HIGH);
    // Intermittent audible warning
    if ((millis() / 250UL) % 2 == 0) {
      tone(BUZZER_PIN, 2200);
    } else {
      noTone(BUZZER_PIN);
    }
  } else if (roverState == STATE_SENSOR_FAULT) {
    digitalWrite(LED_PIN, HIGH);
    noTone(BUZZER_PIN);
  } else if (roverState == STATE_REVERSING || roverState == STATE_TURNING) {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
  } else {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
  }
}

void updateLcd() {
  char line1[17];

  if (distanceCm >= 0) {
    snprintf(line1, sizeof(line1), "D:%3dcm G:%4d", distanceCm, gasRaw);
  } else {
    snprintf(line1, sizeof(line1), "D:---cm G:%4d", gasRaw);
  }

  printLcdLine(0, line1);
  printLcdLine(1, stateText());
}

void printSerialStatus() {
  Serial.print(F("Distance(cm): "));
  if (distanceCm >= 0) Serial.print(distanceCm);
  else Serial.print(F("NO ECHO"));

  Serial.print(F(" | MQ2 raw: "));
  Serial.print(gasRaw);

  Serial.print(F(" | Temp(C): "));
  if (!isnan(temperatureC)) Serial.print(temperatureC, 1);
  else Serial.print(F("N/A"));

  Serial.print(F(" | Humidity(%): "));
  if (!isnan(humidityPercent)) Serial.print(humidityPercent, 1);
  else Serial.print(F("N/A"));

  Serial.print(F(" | State: "));
  Serial.println(stateText());
}

// -------------------- SETUP --------------------
void setup() {
  // Keep motor driver outputs disabled during startup.
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  stopMotors();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MQ2_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);

  Serial.begin(9600);
  dht.begin();

  Wire.begin();
  lcd.init();
  lcd.backlight();

  printLcdLine(0, "SMARTMINE-X UNO");
  printLcdLine(1, "Starting...");
  Serial.println(F("\nSMARTMINE-X Arduino UNO firmware starting"));
  Serial.println(F("Motors remain stopped until valid front sensor readings."));
  Serial.println(F("MQ-2 output is raw ADC only; threshold needs calibration."));

  bootAt = millis();
  stateStartedAt = millis();
}

// -------------------- MAIN LOOP --------------------
void loop() {
  unsigned long now = millis();

  // 1. Read front distance periodically.
  if (now - lastDistanceAt >= DISTANCE_INTERVAL_MS) {
    lastDistanceAt = now;
    distanceCm = readDistanceCm();

    if (distanceCm >= 0) {
      if (validDistanceCount < 10) validDistanceCount++;
      invalidDistanceCount = 0;
    } else {
      if (invalidDistanceCount < 10) invalidDistanceCount++;
      validDistanceCount = 0;
    }
  }

  // 2. Read MQ-2 periodically.
  if (now - lastGasAt >= GAS_INTERVAL_MS) {
    lastGasAt = now;
    gasRaw = readMq2Average();
    mq2WarmedUp = (now - bootAt >= MQ2_WARMUP_MS);

    if (mq2WarmedUp && gasRaw >= MQ2_ALARM_THRESHOLD) {
      gasAlarmActive = true;
    } else {
      gasAlarmActive = false;
    }
  }

  // 3. Read DHT11 at a safe interval.
  if (now - lastDhtAt >= DHT_INTERVAL_MS) {
    lastDhtAt = now;
    updateDhtReadings();
  }

  // 4. Gas alarm has highest priority and stops the rover.
  if (gasAlarmActive) {
    if (roverState != STATE_GAS_ALERT) {
      Serial.println(F("MQ-2 threshold crossed: motors stopped."));
    }
    roverState = STATE_GAS_ALERT;
    stopMotors();
  } else if (roverState == STATE_GAS_ALERT) {
    // Require fresh valid ultrasonic readings before moving again.
    roverState = STATE_SENSOR_FAULT;
    stopMotors();
    validDistanceCount = 0;
  }

  // 5. Obstacle and navigation state machine.
  if (!gasAlarmActive) {
    switch (roverState) {
      case STATE_SENSOR_FAULT:
        stopMotors();
        if (validDistanceCount >= 3) {
          roverState = STATE_FORWARD;
          stateStartedAt = now;
          Serial.println(F("Front sensor valid; rover enabled."));
        }
        break;

      case STATE_FORWARD:
        if (invalidDistanceCount >= 3) {
          stopMotors();
          roverState = STATE_SENSOR_FAULT;
          Serial.println(F("Ultrasonic no-echo fault: motors stopped."));
        } else if (distanceCm >= 0 && distanceCm <= OBSTACLE_DISTANCE_CM) {
          beginReverse();
        } else {
          setMotors(FORWARD_SPEED, FORWARD_SPEED);
        }
        break;

      case STATE_REVERSING:
        if (now - stateStartedAt >= REVERSE_TIME_MS) {
          beginTurn();
        }
        break;

      case STATE_TURNING:
        if (now - stateStartedAt >= TURN_TIME_MS) {
          stopMotors();
          roverState = STATE_FORWARD;
          stateStartedAt = now;
          Serial.println(F("Turn complete; checking front again."));
        }
        break;

      case STATE_GAS_ALERT:
        stopMotors();
        break;
    }
  }

  // 6. Update warning outputs, LCD and Serial Monitor.
  updateAlerts();

  if (now - lastLcdAt >= LCD_INTERVAL_MS) {
    lastLcdAt = now;
    updateLcd();
  }

  if (now - lastSerialAt >= SERIAL_INTERVAL_MS) {
    lastSerialAt = now;
    printSerialStatus();
  }
}
