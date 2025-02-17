#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

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

#define WIDTH 128
#define HEIGHT 32

Adafruit_SSD1306 display(WIDTH, HEIGHT, &Wire, -1);

bool key = false;
int N = 0;

void setup() {
  Wire.begin(SDA, SCL);
  Serial.begin(115200);

  pinMode(BTN, INPUT_PULLUP);

   if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("GEMS");
  display.println("Sapphire");

  delay(300);
  display.display();
  Serial.println("Hello Sapphire OLED!");
}

void loop() {
  if (digitalRead(BTN) == LOW && key) {
    key = false;
    N = N + 1;
    display.clearDisplay();
    display.setTextSize(4);
    display.setCursor(0, 0);
    display.print(N);
    display.display();
  } else if (digitalRead(BTN) == HIGH && !key) {
    key = true;
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("GEMS");
    display.println("Sapphire");
    display.display();
  }
}
