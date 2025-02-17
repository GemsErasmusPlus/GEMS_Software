#define MIC_L 0
#define MIC_R 1
#define LED_A 3
#define LED_B 7
#define LED_C 10
#define CAN_RX 4
#define CAN_TX 5
#define BTN 9
#define US_TRIG 11
#define US_ECHO 12

#define MIC_GAIN 123

#define CONVERSIONS_PER_PIN 1
uint8_t adc_pins[] = { 0, 1 };
uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);
volatile bool adc_coversion_done = false;
adc_continuous_data_t *result = NULL;

void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}

static const int N = 10000;
float t[N];
float R[N];
float L[N];
int i = 0;

unsigned long start;

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

  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, 40000, &adcComplete);

  delay(300);
  Serial.println("Ruby MIC!");
  delay(900);
  analogContinuousStart();

  start = micros();
}

void loop() {
  if (adc_coversion_done == true) {
    adc_coversion_done = false;
    if (analogContinuousRead(&result, 0)) {
      t[i] = ((float)(micros() - start)) / 1000000.0;
      R[i] = ((float)result[0].avg_read_raw) / 4095.0;
      L[i] = ((float)result[1].avg_read_raw) / 4095.0;
      i = i + 1;
    }
  }
  if (i == N) {
    i = 0;
    for (int j = 0; j < N; j++) {
      Serial.print(t[j], 6);
      Serial.print(";");
      Serial.print(R[j],3);
      Serial.print(";");
      Serial.println(L[j],3);
    }
    start = micros();
  }
}
