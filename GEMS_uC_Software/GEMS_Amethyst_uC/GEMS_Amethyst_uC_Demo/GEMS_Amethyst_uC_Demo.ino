//-----Hardware-------------
//User button
#define BTN 9
//RGB LED
#define LED_R 7
#define LED_G 10
#define LED_B 4
//Ambient Light Sensor
#define LX_A 1
#define LX_B 3
#define LX_OUT 0
//I2C
#define SDA 5
#define SCL 6
//UART
#define RX 20
#define TX 21
//--------------------------

//-----Global variables-----
const int led_R_channel = 0;
const int led_G_channel = 1;
const int led_B_channel = 2;
const int pwm_frequency = 5000;  //Hz
const int pwm_resolution = 12;   //bit

//--------------------------

void setup() {
  pinMode(BTN, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  analogWrite(LED_R, 0);
  analogWriteFrequency(LED_R, pwm_frequency);
  analogWriteResolution(LED_R, pwm_resolution);

  analogWrite(LED_G, 0);
  analogWriteFrequency(LED_G, pwm_frequency);
  analogWriteResolution(LED_G, pwm_resolution);

  analogWrite(LED_B, 0);
  analogWriteFrequency(LED_B, pwm_frequency);
  analogWriteResolution(LED_B, pwm_resolution);

  pinMode(LX_A, INPUT);
  pinMode(LX_B, INPUT);
  pinMode(LX_OUT, INPUT);

  Serial.begin(115200);
  delay(300);
  Serial.println("Amethyst Demo!");
}

void loop() {
  delay(10);
  if (btn()) {
    rgb(200, 300, 200);
  } else {
    rgb(0, 0, 0);
    Serial.print(0);
    Serial.print(",");
    Serial.print(2000);
    Serial.print(",");
    Serial.println(lx(1));
  }
}

int btn() {
  return !digitalRead(BTN);
}

void rgb(int R, int G, int B) {
  analogWrite(LED_R, R);
  analogWrite(LED_G, G);
  analogWrite(LED_B, B);
}

int lx(int mode) {
  switch (mode) {
    case 0:
      pinMode(LX_A, INPUT);
      pinMode(LX_B, INPUT);
      break;
    case 1:
      pinMode(LX_A, INPUT);
      pinMode(LX_B, OUTPUT);
      digitalWrite(LX_B, LOW);
      break;
    case 2:
      pinMode(LX_A, OUTPUT);
      digitalWrite(LX_A, LOW);
      pinMode(LX_B, INPUT);
      break;
    default:
      pinMode(LX_A, INPUT);
      pinMode(LX_B, INPUT);
  }

  return analogRead(LX_OUT);
}
