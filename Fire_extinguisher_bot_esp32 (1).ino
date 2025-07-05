#define BLYNK_TEMPLATE_ID "TMPL37BS2Imsu"
#define BLYNK_TEMPLATE_NAME "Fire Extinguisher Robo"
#define BLYNK_AUTH_TOKEN "SGOyTZH8hdVWsuyuaE8aBaKlO2GOl7Nu"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// === BLYNK AUTHENTICATION ===
char ssid[] = "ESP32_FIRE";
char pass[] = "IOT_FIRE";

// === FLAME SENSOR PINS ===
int flamePins[5] = {35, 32, 33, 25, 34}; 

struct FlameData {
  int direction;
  int intensity;
};

// === MOTOR PINS ===
#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23
#define ENA 16
#define ENB 17

// === PUMP ===
#define PUMP_PIN 5

// === ULTRASONIC SENSOR PINS ===
  #define TRIG_PIN 14
  #define ECHO_PIN 27
  #define OE 4

// === STATES ===
enum BotState { IDLE, MOVING, ADJUSTING_DISTANCE, SPRAYING, MANUAL };
BotState currentState = IDLE;

unsigned long stateStartTime = 0;
int targetDirection = -1;

// === BLYNK INPUT ===
int manualMove = 0;
bool manualPump = false;
bool showDistanceRequest = false;
bool manualMode = false;

int rotationDuration = 100; 
int minRotationDuration = 20;
bool rotatingLeft = false;
bool rotatingRight = false;

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  for (int i = 0; i < 5; i++) pinMode(flamePins[i], INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(OE, OUTPUT);
  digitalWrite(OE, HIGH);

  ledcAttach(IN1, 30000, 8);
  ledcAttach(IN2, 30000, 8);
  ledcAttach(IN3, 30000, 8);
  ledcAttach(IN4, 30000, 8);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

void loop() {
  Blynk.run();

  if (showDistanceRequest) {
    float distance = measureDistance();
    Serial.print("📏 Distance: ");
    Serial.println(distance);
    Blynk.virtualWrite(V0, distance); 
    showDistanceRequest = false;
  }

  if (manualMode) {
    manualControlManager();
  } else {
    flameBotManager();
  }
}

// === MANUAL MOVEMENT CONTROLS ===
BLYNK_WRITE(V3) { manualMove = 1; }
BLYNK_WRITE(V4) { manualMove = 2; }
BLYNK_WRITE(V5) { manualMove = 3; }
BLYNK_WRITE(V6) { manualMove = 4; }
BLYNK_WRITE(V7) { manualMove = 5; }
BLYNK_WRITE(V8) { manualPump = param.asInt(); }
BLYNK_WRITE(V9) {
  if (param.asInt() == 1) { 
    showDistanceRequest = true;
  }
}

// Manual/Auto mode switch
BLYNK_WRITE(V2) { manualMode = param.asInt(); }

void manualControlManager() {
  switch (manualMove) {
    case 1: moveForward(); break;
    case 2: rotateLeft(); break;
    case 3: rotateRight(); break;
    case 4: stopMotors(); break;
    case 5: moveBackward(); break;
    default: stopMotors(); break;
  }

  digitalWrite(PUMP_PIN, manualPump ? HIGH : LOW);
  Blynk.virtualWrite(V1, manualPump);
  Blynk.virtualWrite(V2, manualMove);
}

void rotateWithFlameTracking(bool rotateLeft) {
  unsigned long rotateStart = millis();
  int rotationStep = 10;
  bool flameLost = false;  // Flag to track if flame is lost

  while (millis() - rotateStart < rotationDuration) {
    if (rotateLeft) rotateLeftFunc();
    else rotateRightFunc();

    FlameData flameNow = getStrongestFlameDirection();

    if (flameNow.direction == 2) {
      // Flame is centered in front
      stopMotors();
      rotationDuration = 100;  // Reset rotation duration
      return;
    }

    if (flameNow.direction == -1) {
      // Flame is lost
      if (!flameLost) {
        stopMotors();
        delay(100);  // Wait briefly before attempting to rotate back
        flameLost = true;
        
        // Rotate in the opposite direction (backtrack) for a brief period
        if (rotateLeft) {
          rotateRightFunc();  // Rotate to the right if previously rotating left
        } else {
          rotateLeftFunc();  // Rotate to the left if previously rotating right
        }

        delay(50);  // Rotate back for 50ms

        // Check again after backtracking
        flameNow = getStrongestFlameDirection();
        if (flameNow.direction != -1) {
          // Flame reappeared, continue searching
          flameLost = false;
          continue;
        }
      }
    }

    delay(rotationStep);  // Rotate with small delay
  }

  stopMotors();
  rotationDuration = max(rotationDuration / 2, minRotationDuration);
}



void flameBotManager() {
  static unsigned long lastStateChangeTime = 0;
  static unsigned long lastFlameReadTime = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastFlameReadTime >= 20) {
    lastFlameReadTime = currentMillis;

    FlameData flameData = getStrongestFlameDirection();

    switch (currentState) {
      case IDLE:
        stopMotors();
        digitalWrite(PUMP_PIN, LOW);
        Blynk.virtualWrite(V1, 0);
        if (flameData.direction != -1) {
          currentState = MOVING;
        }
        break;

      case MOVING:
        if (flameData.direction == 2) {
          // Fire is already in front
          currentState = ADJUSTING_DISTANCE;
        } else if (flameData.direction != -1) {
          if (flameData.direction > 2) {
            rotateWithFlameTracking(true);  // rotate left
          } else if (flameData.direction < 2) {
            rotateWithFlameTracking(false); // rotate right
          }

          flameData = getStrongestFlameDirection();
          if (flameData.direction == 2) {
            currentState = ADJUSTING_DISTANCE;
          }
        } else {
          currentState = IDLE; // Flame lost
        }
        break;



      case ADJUSTING_DISTANCE:
        {
          float distance = measureDistance();
          if (distance > 19.5) {
            moveForward();
            delay(50);  
          } else if (distance < 17.5) {
            moveBackward();
            delay(50);  
          } else {
            stopMotors();
            currentState = SPRAYING;  
          }
        }
        break;

      case SPRAYING:
        digitalWrite(PUMP_PIN, HIGH);
        Blynk.virtualWrite(V1, 1);
        Serial.println("🚿 Spraying...");

        FlameData flameDataPostSpray = getStrongestFlameDirection();
        if (flameDataPostSpray.direction == -1 || flameDataPostSpray.intensity < 100) {
          digitalWrite(PUMP_PIN, LOW);
          Serial.println("✅ Fire extinguished.");
          currentState = IDLE;
        }
        break;
    }
  }
}

// === MOVEMENT FUNCTIONS ===
void moveToward(int dir) {
  switch (dir) {
    case 0: rotateRight(); break;
    case 1: slightRight(); break;
    case 2: moveForward(); break;
    case 3: slightLeft(); break;
    case 4: rotateLeft(); break;
    default: stopMotors(); break;
  }
}

void moveBackward() {
  ledcWrite(IN1, 128); ledcWrite(IN2, 0);
  ledcWrite(IN3, 128); ledcWrite(IN4, 0);
}

void moveForward() {
  ledcWrite(IN1, 0); ledcWrite(IN2, 128);
  ledcWrite(IN3, 0); ledcWrite(IN4, 128);
}

void slightRight() {
  ledcWrite(IN1, 128); ledcWrite(IN2, 0);
  ledcWrite(IN3, 0); ledcWrite(IN4, 128);
}

void rotateLeftFunc() {
  ledcWrite(IN1, 0); ledcWrite(IN2, 128);
  ledcWrite(IN3, 128); ledcWrite(IN4, 0);
}

void rotateRightFunc() {
  ledcWrite(IN1, 128); ledcWrite(IN2, 0);
  ledcWrite(IN3, 0); ledcWrite(IN4, 128);
}

void slightLeft() {
  ledcWrite(IN1, 0); ledcWrite(IN2, 128);
  ledcWrite(IN3, 128); ledcWrite(IN4, 0);
}

void rotateRight() {
  ledcWrite(IN1, 128); ledcWrite(IN2, 0);
  ledcWrite(IN3, 0); ledcWrite(IN4, 128);
}

void rotateLeft() {
  ledcWrite(IN1, 0); ledcWrite(IN2, 128);
  ledcWrite(IN3, 128); ledcWrite(IN4, 0);
}

void stopMotors() {
  ledcWrite(IN1, 0); ledcWrite(IN2, 0);
  ledcWrite(IN3, 0); ledcWrite(IN4, 0);
}

// === FLAME DIRECTION DETECTION ===
FlameData getStrongestFlameDirection() {
  int flameValues[5];
  int maxFlame = 0;
  int maxDirection = -1;
  int flameThreshold = 100;

  for (int i = 0; i < 5; i++) {
    flameValues[i] = analogRead(flamePins[i]);
    Serial.println(flameValues[i]);
  }

  for (int i = 0; i < 5; i++) {
    if (flameValues[i] > maxFlame) {
      maxFlame = flameValues[i];
      maxDirection = i;
    }
  }

  if (maxFlame < flameThreshold) {
    maxDirection = -1;
  }

  FlameData result = {maxDirection, maxFlame};
  return result;
}

// === DISTANCE MEASUREMENT FUNCTION ===
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  noInterrupts();
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  interrupts();
  float distance = duration * 0.034 / 2;
  return distance;
}