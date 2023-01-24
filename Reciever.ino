#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define RF69_FREQ 915.0

// server address (0-254), TX is 0 right now
#define MY_ADDRESS     1



// What modem config to use. Very low bitrate for now
#define MODEM_CONFIG     RH_RF69::FSK_Rb2Fd5


// pin setup
#define RFM69_CS      10
#define RFM69_INT     3
#define RFM69_RST     9
#define LED           8

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 

void setup() 
{
  Serial.begin(115200);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not connected to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Reciever On!");
  Serial.println();

  // manual reset
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
    Serial.println("setFrequency failed");
  }


  rf69.setTxPower(20, true);  // range from 14-20 for power
  // Encryption key, not really relevant for this demo
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  rf69.setModemConfig(MODEM_CONFIG);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio recieving @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

// Message buffer and message to send back to Tx
uint8_t data[] = "logged";
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop() {
  if (rf69_manager.available())
  {
    // This code is based loosely off the RFM69 reciever demo
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      buf[len] = 0; // zero out remaining string
      
      // print relevant info as comma separated values.
      // Py script will grab it from the serial port and save to csv.
      Serial.print(from);
      Serial.print(",");
      Serial.print(rf69.lastRssi());
      Serial.print(",");
      Serial.println((char*)buf);
      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks

      // Send a reply back to the sensor
      if (!rf69_manager.sendtoWait(data, sizeof(data), from))
        Serial.println("Sending failed (no ack)");
    }
  }
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
