#include <SPI.h>
#include <DW1000Ranging.h>
#include <WiFi.h>

WiFiUDP Ludp;

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin


String AddressString = "";
String ID = "";

unsigned long WiFiRetryTimeTrigger = 5000;

const char *ssid = "wifi";
const char *pwd = "password";

//uint16_t Adelay = 17000;

void setup() {
  Serial.begin(115200);
  Serial.println("TAG WiFI");
  pinMode(2, OUTPUT);

  AddressString = WiFi.macAddress();
  ID = AddressString;

  //Serial.println(AddressString);

  AddressString = AddressString;


  int Address_len = AddressString.length() + 1;
  char AddressArray[Address_len];
  AddressString.toCharArray(AddressArray, Address_len);

  for (int i = 0; i < Address_len; i++) {
    Serial.print(AddressArray[i]);
  }


  delay(1000);
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);  //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module

  //DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true); //Do not use. It will make the ESP32 slow and only let big delta point pass. (bad point)

  //we start the module as a tag
  //DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  //DW1000Ranging.startAsTag(AddressArray, DW1000.MODE_LONGDATA_RANGE_ACCURACY, 0);
  DW1000Ranging.startAsTag(AddressArray, DW1000.MODE_SHORTDATA_FAST_ACCURACY , 0);
}

void loop() {
  DW1000Ranging.loop();

  if (WiFi.status() != WL_CONNECTED && millis() > WiFiRetryTimeTrigger) {  //wifi stoped
    digitalWrite(2, LOW);
    Serial.println("Reconnecting...");
    WiFi.begin(ssid, pwd);
    WiFiRetryTimeTrigger = millis() + 10000;
  }
}



void newRange() {
  //Serial.print("from: ");
  //Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);

  //Serial.print(" RX power: ");
  //Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  //Serial.print(" dBm");

  String Address = String(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);

  float Distance = DW1000Ranging.getDistantDevice()->getRange();

  String text = ID + "," + "P=" + Address + "&D=" + Distance + "\n";

  //Serial.println(text + String(WiFi.status() == WL_CONNECTED));

  if (Distance > -1) {
    //Serial.print(" Range: ");
    //Serial.print(Distance);
    //Serial.print(" m");

    if (WiFi.status() == WL_CONNECTED) {
      //Serial.println("Sending...");
      digitalWrite(2, LOW);

      long text_len = text.length() + 1;
      char TX[text_len];
      text.toCharArray(TX, text_len);

      Ludp.beginPacket("192.168.0.247", 1111);  // port to number reply
      Ludp.write((uint8_t *)TX, text_len);
      Ludp.endPacket();
      digitalWrite(2, HIGH);
      //Serial.println("Sended " + text);
      //Serial.println(" ");
    }
  }
}

void newDevice(DW1000Device *device) {
  Serial.print("New device :");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
  digitalWrite(2, LOW);
}
