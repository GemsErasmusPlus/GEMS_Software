#define BTN 0
#define USR 15  // error on PCB: no connection
#define CAN_RX 4
#define CAN_TX 5
#define SDA 11
#define SCL 12
#define UART_TX 43
#define UART_RX 44
#define BZZR_P 36
#define BZZR_N 35

#define FREQUENCY 500                 // Hz
#define HALF_PERIOD 1000000.0 / (2 * FREQUENCY)  // us

void setup() {
  Serial.begin(115200);

  pinMode(BTN, INPUT_PULLUP);
  pinMode(BZZR_P, OUTPUT);
  pinMode(BZZR_N, OUTPUT);

  digitalWrite(BZZR_P, LOW);
  digitalWrite(BZZR_N, LOW);

  delay(300);
  Serial.println("Hello Sapphire Buzzer!");
}

void loop() {
  if (digitalRead(BTN) == LOW) {
    digitalWrite(BZZR_P, HIGH);
    digitalWrite(BZZR_N, LOW);
    delayMicroseconds(HALF_PERIOD);
    digitalWrite(BZZR_P, LOW);
    digitalWrite(BZZR_N, HIGH);
    delayMicroseconds(HALF_PERIOD);
  }
}
