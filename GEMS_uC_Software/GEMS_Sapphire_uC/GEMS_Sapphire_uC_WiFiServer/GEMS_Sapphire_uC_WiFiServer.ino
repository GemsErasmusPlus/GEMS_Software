#include <WiFi.h>

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

// Network settings
const char* ssid = "GEMS_Sapphire";
const char* password = "equalequal";
const uint16_t serverPort = 80;
IPAddress local_ip(192, 168, 3, 1);
IPAddress gateway(192, 168, 3, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(serverPort);

bool key = false;
int N = 0;

void setup() {
  Serial.begin(115200);

  pinMode(BTN, INPUT_PULLUP);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  server.begin();

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  delay(300);
  Serial.println("Hello Sapphire WiFi Server!");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        String message = client.readStringUntil('\n');
        if (message.length() == 1) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println("Connection: close");
          client.println();

          client.println("<!DOCTYPE HTML>");
          client.println("</html>");
          client.println("<h1>ERASMUS+ GEMS</h1>");
          client.println("<h2>Sapphire</h2>");
          client.println("</html>");

          client.println("");
          client.stop();
        }
      }
    }
  }
}
