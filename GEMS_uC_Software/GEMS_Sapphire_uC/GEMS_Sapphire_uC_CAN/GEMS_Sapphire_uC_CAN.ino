#include <ESP32-TWAI-CAN.hpp>

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

#define FREQUENCY 500                            // Hz
#define HALF_PERIOD 1000000.0 / (2 * FREQUENCY)  // us

void setup() {
  Serial.begin(115200);

  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

  if (!ESP32Can.begin()) {
    Serial.println("CAN ERROR!");
    while (1) {}
  }

  delay(300);
  Serial.println("Hello Sapphire CAN!");
}

void loop() {
  delay(3000);
  sendFrame();
  delay(1000);
  receiveFrame();
}

void sendFrame() {
  CanFrame Frame = { 0 };
  Frame.identifier = 0x653;
  Frame.extd = 0;
  //Frame.ss = 1;
  //Frame.self = 1;
  Frame.data_length_code = 8;
  Frame.data[0] = 2;
  Frame.data[1] = 1;
  Frame.data[2] = 5;
  Frame.data[3] = 0xAA;  // Best to use 0xAA (0b10101010) instead of 0
  Frame.data[4] = 0xAA;  // CAN works better this way as it needs
  Frame.data[5] = 0xAA;  // to avoid bit-stuffing
  Frame.data[6] = 0xAA;
  Frame.data[7] = 0xAA;
  // Accepts both pointers and references
  ESP32Can.writeFrame(Frame);
}

CanFrame receiveFrame() {
  CanFrame Frame;
  if (ESP32Can.readFrame(Frame, 1000)) {
    // Comment out if too many requests
    Serial.printf("Frame: %03X \r\n", Frame.identifier);
    if (Frame.identifier == 0x653) {
      Serial.printf("Data: %d", Frame.data[3]);
    }
  }
  return Frame;
}
