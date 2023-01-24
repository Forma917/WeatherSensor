#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

// Sensor Sample Interval (minutes)
#define BME_SAMPLE_INTERVAL 1

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

// Radio pin assignment
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           5

// Radio TX freq. Must match RX's freq!
#define RF69_FREQ 915.0

// Addressing
#define DEST_ADDRESS   255
#define MY_ADDRESS     0

// What modem config to use
#define MODEM_CONFIG     RH_RF69::FSK_Rb2Fd5

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


// Adafruit_BME680 bme; // I2C
// Adafruit_BME680 bme(BME_CS); // hardware SPI
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// Initialize the buffer
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
int8_t from;

// func to handle transmitting and recieving acknowledgement
int8_t transmit(char *radiopacket,bool see_response){
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf); 
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      if (see_response){
        buf[len] = 0; // zero out remaining string
        Serial.print("Got reply from #"); Serial.print(from);
        Serial.print(" [RSSI :");
        Serial.print(rf69.lastRssi());
        Serial.print("] : ");
        Serial.println((char*)buf);
      }
      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
      
      return from;

    }
  } else {
    return -1;
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Setup params courtesy of the example code for the BME680
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual radio reset just in case
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);


  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");

  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed!");
  }

  rf69.setTxPower(20, true);  // range from 14-20 for power
  // Encryption key, not really relevant for this demo
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  rf69.setModemConfig(MODEM_CONFIG);

   // Attempt to find reciever
  char finddest[24] = "Searching for RX...";
  while (transmit((char*)finddest,1) == -1){
    Serial.println("No reciever found...");
    delay(3000);
  }

  Serial.print("Found valid RX at addr: ");
  Serial.println(from);
  // Set new dest addr to the reciever
  #define DEST_ADDRESS = from;
  
}

void loop() {
  // Initializing the data to send to RX
  char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
  char temp[8];
  char press[8];
  char humid[8];
  char gas[8];
  
  // In case of err send alert to RX.
  if (! bme.performReading()) {
    Serial.println("failed to read sensor!!!");
    sprintf(radiopacket,RH_RF69_MAX_MESSAGE_LEN,"Sensor failure on node %d!",MY_ADDRESS);
    transmit((char*)radiopacket,1);
    return;
  }
  
  //Data is in units *C, hPa, % Rel. Humidity, Kohm (respectively)
  sprintf(radiopacket,"%s,%s,%s,%s",
    dtostrf(bme.temperature,8,2,temp),
    dtostrf(bme.pressure / 100.0,8,2,press),
    dtostrf(bme.humidity,8,2,humid),
    dtostrf(bme.gas_resistance/1000.0,8,2,gas));

  transmit((char*)radiopacket,0);

  // Convert the sampling interval (mins) to ms
  delay(BME_SAMPLE_INTERVAL*60.0*1000.0);
}

// LED blink function for visual confirmation of sent packets.
void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}