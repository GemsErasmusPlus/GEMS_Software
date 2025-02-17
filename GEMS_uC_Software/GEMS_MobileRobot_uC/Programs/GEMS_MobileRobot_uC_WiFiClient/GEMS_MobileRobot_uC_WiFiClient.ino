#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define BTN 0
#define A_1 1
#define A_2 2
#define A_3 10
#define CAN_RX 4
#define CAN_TX 5
#define FAULT_A 6
#define FAULT_B 7
#define MOT_I 8
#define PWM_A 11
#define PWM_B 12
#define E_A 41
#define E_B 40
#define E_C 13
#define LED_R 35
#define LED_Y 36
#define LED_G 37

#define ADC_R 12
#define PWM_R 10
#define PWM_F 10000

enum action { forward,
              backward,
              brake,
              coast };
enum state { off,
             red,
             yellow,
             green };

bool fault_A = false;
bool fault_B = false;
bool emergency_stop = false;

unsigned long time_0 = 0;
unsigned long time_1 = 0;

float frequency_limit_min = 0.1;    // Hz
float frequency_limit_max = 300.0;  // Hz
int motor_direction = 1;

const char* ssid = "GEMS_Robot_n1";
const char* password = "equalequal";

bool command_received = false;
int command_value = 0;


void setup() {
  Serial.begin(115200);

  pinMode(BTN, INPUT_PULLUP);
  pinMode(A_1, INPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(MOT_I, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);

  pinMode(E_A, INPUT);
  pinMode(E_B, INPUT);
  pinMode(E_C, INPUT);

  motor_state_LEDs(off);

  pinMode(FAULT_A, INPUT_PULLUP);
  pinMode(FAULT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FAULT_A), handle_fault_A, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAULT_B), handle_fault_B, FALLING);

  attachInterrupt(digitalPinToInterrupt(E_A), handle_encoder, FALLING);

  motor_action(brake, 0);

  analogWriteFrequency(PWM_A, PWM_F);
  analogWriteFrequency(PWM_B, PWM_F);

  analogWriteResolution(PWM_A, PWM_R);
  analogWriteResolution(PWM_B, PWM_R);

  motor_action(brake, 0);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  delay(300);
  Serial.println("Hello Robot n1 - motor driver - Left!");
}

void loop() {
  if (!fault_A && !fault_B) {
    command_request("L_value");
    if (command_received) {
      Serial.println(command_value);
      motor_state_LEDs(green);
      int pwm_value = 1023 - abs(command_value);
      if (command_value > 100) {
        motor_action(forward, pwm_value);
      } else if (command_value < -100) {
        motor_action(backward, pwm_value);
      } else {
        motor_action(brake, 0);
      }
      delay(100);
    } else {
      motor_action(brake, 0);

      motor_state_LEDs(yellow);
    }
  } else {
    motor_action(brake, 0);

    if (fault_A) {
      delay(1);
      if (digitalRead(FAULT_A) == LOW) {
        emergency_stop = true;
      }
      Serial.println("Motor driver FAULT!");
    }
    if (fault_B) {
      delay(1);
      if (digitalRead(FAULT_B) == LOW) {
        emergency_stop = true;
      }
      Serial.println("Current sensor FAULT");
    }
    if (emergency_stop) {
      motor_state_LEDs(red);
      Serial.println("Emergency STOP!");
      while (1) {}
    }
    fault_A = false;
    fault_B = false;
  }
}

float get_motor_frequency() {
  float f = 0.0;

  f = 1000000.0 / (float)(time_1 - time_0);  // Hz
  if ((f < frequency_limit_min) || (f > frequency_limit_max)) {
    f = 0.0;
  }

  return motor_direction * f;
}

void handle_encoder() {
  time_0 = time_1;
  time_1 = micros();
  motor_direction = -1 + 2 * digitalRead(E_B);
}

void handle_fault_A() {
  fault_A = true;
}

void handle_fault_B() {
  fault_B = true;
}

void motor_action(enum action x, int y) {
  int PWM_L = 0;
  int PWM_H = (2 << PWM_R) - 1;

  analogWrite(PWM_A, PWM_H);
  analogWrite(PWM_B, PWM_H);

  switch (x) {
    case forward:
      analogWrite(PWM_A, PWM_H);
      analogWrite(PWM_B, y);
      break;
    case backward:
      analogWrite(PWM_A, y);
      analogWrite(PWM_B, PWM_H);
      break;
    case brake:
      analogWrite(PWM_A, PWM_H);
      analogWrite(PWM_B, PWM_H);
      break;
    case coast://Don't use - Motor driver FAULT
      analogWrite(PWM_A, PWM_L);
      analogWrite(PWM_B, PWM_L);
      break;
  }
}

void motor_state_LEDs(enum state x) {
  int r = LOW;
  int y = LOW;
  int g = LOW;

  switch (x) {
    case red:
      r = HIGH;
      break;
    case yellow:
      y = HIGH;
      break;
    case green:
      g = HIGH;
      break;
  }
  digitalWrite(LED_R, r);
  digitalWrite(LED_Y, y);
  digitalWrite(LED_G, g);
}

void command_request(String s) {
  command_received = false;
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String url = "http://192.168.3.1/api?Question=" + s;
    http.begin(url);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      String payload = http.getString();
      StaticJsonDocument<128> doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        command_value = String(doc["Answer"]).toInt();
        command_received = true;
      }
    }
    http.end();
  } else {
    Serial.println("Wi-Fi not connected");
  }
}
