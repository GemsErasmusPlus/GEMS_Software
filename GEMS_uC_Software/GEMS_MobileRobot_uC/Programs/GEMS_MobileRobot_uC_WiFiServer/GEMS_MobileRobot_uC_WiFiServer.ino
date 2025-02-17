#include <WiFi.h>
#include <PsychicHttp.h>
#include <LittleFS.h>

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

// Network settings
const char* ssid = "GEMS_Robot_n1";
const char* password = "equalequal";
const uint16_t serverPort = 80;
IPAddress local_ip(192, 168, 3, 1);
IPAddress gateway(192, 168, 3, 1);
IPAddress subnet(255, 255, 255, 0);

PsychicHttpServer server;

int L_value = 0;
int R_value = 0;

void setup() {
  Wire.begin(SDA, SCL);
  Serial.begin(115200);

  pinMode(BTN, INPUT_PULLUP);
  delay(1000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    while (1) {}
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ssid: GEMS_Robot_n1");
  display.println("pass: equalequal");
  display.println("");
  display.println("http://192.168.3.1");
  delay(300);
  display.display();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.config.max_uri_handlers = 100;
  server.listen(serverPort);

  server.serveStatic("/", LittleFS, "/www-ap/")->setFilter(ON_AP_FILTER);

  server.on("/api", HTTP_POST, [](PsychicRequest* request) {
    //load our JSON request
    StaticJsonDocument<1024> json;
    String body = request->body();
    DeserializationError err = deserializeJson(json, body);

    //create our response json
    StaticJsonDocument<64> output;

    //work with some params
    if (json.containsKey("L_value")) {
      L_value = String(json["L_value"]).toInt();
    }
    if (json.containsKey("R_value")) {
      R_value = String(json["R_value"]).toInt();
    }

    //serialize and return
    String jsonBuffer;
    serializeJson(output, jsonBuffer);
    return request->reply(200, "application/json", jsonBuffer.c_str());
  });

  server.on("/api", HTTP_GET, [](PsychicRequest* request) {
    //create a response object
    StaticJsonDocument<64> output;
    //work with some params
    if (request->hasParam("Question")) {
      int value = 0;
      String s = request->getParam("Question")->value();
      if (s == "L_value") {
        value = L_value;
      } else if (s == "R_value") {
        value = R_value;
      }
      output["Answer"] = value;
    }

    //serialize and return
    String jsonBuffer;
    serializeJson(output, jsonBuffer);
    return request->reply(200, "application/json", jsonBuffer.c_str());
  });

  delay(300);
  Serial.println("Hello Robot n1 - server!");
}

void loop() {
}
