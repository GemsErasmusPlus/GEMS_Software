// Board: ESP32C3 Dev Module
// Upload: settings(USB CDC on boot: Enable, plugin board, select board) then UPLOAD
// CPU Frequency: 80Mhz
// if (UPLOAD error) then (1.hold BOOT, 2.press and release RESET, 3.release BOOT, 4.UPLOAD)
void setup() {
 Serial.begin(115200);
}

void loop() {
  delay(3000);
  Serial.println("Hello Amethyst!");
}