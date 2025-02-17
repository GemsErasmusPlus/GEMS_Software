#define MIC_L 0
#define MIC_R 1
#define LED_A 3
#define LED_B 7

#define LED_C 10
#define CAN_RX 4
#define CAN_TX 5
#define BTN 9
#define US_TRIG 20
#define US_ECHO 21

#define MIC_GAIN 123

#define SOUND_SPEED 343.0  // m/s
#define K SOUND_SPEED/2000000.0

bool toggle = false;
unsigned long start = 0;
unsigned long end = 0;

float distance = 0.0;

volatile bool ping = false;

void setup() {
  Serial.begin(115200);

  pinMode(MIC_L, INPUT);
  pinMode(MIC_R, INPUT);
  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_C, OUTPUT);
  pinMode(BTN, INPUT_PULLUP);

  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  digitalWrite(LED_A, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(LED_C, LOW);

  delay(300);
  Serial.println("Ruby Ultrasound!");
  delay(900);
  attachInterrupt(digitalPinToInterrupt(US_ECHO), echo, CHANGE);
}

void loop() {
  if (digitalRead(BTN) == LOW && toggle) {
    toggle = false;

    digitalWrite(US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG, LOW);
    
    ping = false;
    while(!ping){}
    start = micros();
    ping = false;
    while(!ping){}
    end = micros();

    distance = (float)(end-start) * K;
    Serial.print(distance);
    Serial.println(" m");
  }
  else if(digitalRead(BTN) == HIGH){
    toggle = true;
  }
}

void echo() {
  ping = !ping;
}
