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
  pinMode(A_1, INPUT);
  pinMode(A_2, INPUT);
  pinMode(A_3, INPUT);

  delay(300);
  Serial.println("Hello Diamond ADCs!");
}

void loop() {
  if (digitalRead(BTN) == LOW) {
    int a_1 = analogRead(A_1);
    int a_2 = analogRead(A_2);
    int a_3 = analogRead(A_3);

    Serial.print(a_1);
    Serial.print(",");
    Serial.print(a_2);
    Serial.print(",");
    Serial.print(a_3);
    Serial.println("");

    delay(200);
  }
}
