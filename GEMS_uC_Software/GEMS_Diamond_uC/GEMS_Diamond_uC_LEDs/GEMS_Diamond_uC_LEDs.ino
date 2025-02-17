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


int8_t state = 2;
bool key = true;

void setup() {
  Serial.begin(115200);

  pinMode(BTN, INPUT_PULLUP);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_Y, LOW);
  digitalWrite(LED_G, LOW);

  delay(300);
  Serial.println("Hello Diamond LEDs!");
}

void loop() {
  if (digitalRead(BTN) == LOW) {
    setLEDs(state);

    if (key) {
      state = state << 1;
    }
    else{
      state = state >> 1;
    }

    if (state == 8) {
      key = false;
    } else if (state == 2) {
      key = true;
    }

    delay(200);
  }
}

void setLEDs(uint8_t x) {
  digitalWrite(LED_R, x & (1<<1));
  digitalWrite(LED_Y, x & (1<<2));
  digitalWrite(LED_G, x & (1<<3));
}
