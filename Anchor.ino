#include <SPI.h>
#include <DW1000Ranging.h>  //https://github.com/thotro/arduino-dw1000
#include <WiFi.h>

#define ANCHOR_ADD "3"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin

String AddressString = "";
unsigned long WatchdogTimer = 0;
const long NullLimit = 5000;

void setup() {
  pinMode(2, OUTPUT);
  Serial.begin(115200);

  byte mac[6];

  Serial.print("MAC:");
  Serial.println(WiFi.macAddress());

  WiFi.macAddress(mac);
  AddressString = mac[0] + mac[1] + mac[2] + mac[3] + mac[4] + mac[5];

  Serial.print("ID:");
  Serial.println(AddressString);

  int Address_len = AddressString.length() + 1;
  char AddressArray[Address_len];
  AddressString.toCharArray(AddressArray, Address_len);

  for (int i = 0; i < Address_len; i++) {
    Serial.print(AddressArray[i]);
  }

  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);  //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachBlinkDevice(newBlink);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);

  //we start the module as an anchor
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY);

  //DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_LOWPOWER);
   DW1000Ranging.startAsAnchor(AddressArray, DW1000.MODE_SHORTDATA_FAST_ACCURACY , 0);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_ACCURACY);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY, 0);
  // DW1000Ranging.startAsAnchor(AddressArray, DW1000.MODE_LONGDATA_RANGE_ACCURACY, 0);
}

void loop() {
  DW1000Ranging.loop();

  if (millis() > WatchdogTimer + NullLimit) {
    Serial.println("Reboot !!!!!!!!!!!!!!!!!!!!");
    delay(5);
    ESP.restart();
  }
}

void newRange() {
  WatchdogTimer = millis();
  digitalWrite(2, HIGH);
  Serial.print("from: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRange());
  Serial.print(" m");
  Serial.print("\t RX power: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  Serial.println(" dBm");
  digitalWrite(2, LOW);
}

void newBlink(DW1000Device *device) {
  Serial.print("blink; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
