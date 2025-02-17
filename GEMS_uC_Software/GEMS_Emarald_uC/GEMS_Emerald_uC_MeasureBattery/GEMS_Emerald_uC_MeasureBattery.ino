#define V_BATT 0
#define I_BATT 1
#define CAN_RX 4
#define CAN_TX 5
#define SHDN_UC 7
#define BTN 9

#define U_GAIN 0.565
#define I_GAIN 67.0
#define I_RESISTOR 0.005

void setup() {
  Serial.begin(115200);

  pinMode(V_BATT, INPUT);
  pinMode(I_BATT, INPUT);
  pinMode(SHDN_UC, OUTPUT);
  pinMode(BTN, INPUT_PULLUP);

  digitalWrite(SHDN_UC, LOW);

  delay(300);
  Serial.println("Emerald Demo!");
}

void loop() {
  if(!digitalRead(BTN)){
    digitalWrite(SHDN_UC, HIGH);
    delay(1000);
    digitalWrite(SHDN_UC, LOW);
  }
  else{
    float U = (float)analogRead(V_BATT) * 3000.0 / 4095.0 / U_GAIN;
    float I = (float)analogRead(I_BATT) * 3000.0 / 4095.0 / I_GAIN / I_RESISTOR;

    Serial.print("U = ");
    Serial.print(U);
    Serial.print(" mV");
    Serial.print(" ;  ");
    Serial.print("I = ");
    Serial.print(I);
    Serial.print(" mA");
    Serial.println("");
    delay(1000);
  }
}
