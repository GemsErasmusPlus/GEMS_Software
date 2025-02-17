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

#define H 1
#define L 0
#define Z -1

int N = 0;
int inc = 1;
bool key = false;

void setup() {
  Serial.begin(115200);

  pinMode(MIC_L, INPUT);
  pinMode(MIC_R, INPUT);
  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_C, OUTPUT);
  pinMode(BTN, INPUT_PULLUP);

  digitalWrite(LED_A, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(LED_C, LOW);

  delay(300);
  Serial.println("Ruby LEDs!");
}

void loop() {
  if(!digitalRead(BTN) && key){
    N = N + inc;
    if(N == 1){
      inc = 1;
    }
    else if(N == 6){
      inc = -1;
    }
    single_LED(N);
    key = false;
  }
  else{
    key = true;
  }
  delay(100);
}

void single_LED(int n){
  switch (n){
    case 0:
      setCharlieplex(Z, Z, Z);
    break;
    case 1:
      setCharlieplex(L, H, Z);
    break;
    case 2:
      setCharlieplex(L, Z, H);
    break;
    case 3:
      setCharlieplex(Z, L, H);
    break;
    case 4:
      setCharlieplex(H, L, Z);
    break;
    case 5:
      setCharlieplex(H, Z, L);
    break;
    case 6:
      setCharlieplex(Z, H, L);
    break;
  }
}

void setCharlieplex(int A, int B, int C){
      setCharlieplexPin(LED_A, A);
      setCharlieplexPin(LED_B, B);
      setCharlieplexPin(LED_C, C);
}

void setCharlieplexPin(int X, int S){
  switch (S){
    case L:
      pinMode(X, OUTPUT);
      digitalWrite(X, LOW);
    break;
    case H:
      pinMode(X, OUTPUT);
      digitalWrite(X, HIGH);
    break;
    case Z:
      pinMode(X, INPUT);
    break;
  }
}
