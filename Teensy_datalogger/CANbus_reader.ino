#include <FlexCAN_T4.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;

void canSniff(const CAN_message_t &msg);
bool inCanSniff = true;

void setup(void) {
  can1.begin();
  can1.setBaudRate(500000);


  can1.setMB(MB0,RX,EXT); //Configure the mailbox
  can1.setMB(MB1,RX,EXT); //Configure the mailbox
  can1.setMB(MB2,RX,EXT); //Configure the mailbox
  can1.setMB(MB3,RX,EXT); //Configure the mailbox
  can1.setMB(MB4,RX,EXT); //Configure the mailbox
  can1.setMB(MB5,RX,EXT); //Configure the mailbox

  can1.setMBFilter(REJECT_ALL); //reject all inputs prior to filtering
  can1.enableMBInterrupts();
  can1.setMBFilter(MB0, 0x680); 
  can1.setMBFilter(MB1, 0x681); 
  can1.setMBFilter(MB2, 0x682); 
  can1.setMBFilter(MB3, 0x051); 
  can1.setMBFilter(MB4, 0x151); 
  can1.setMBFilter(MB5, 0x251); 
  
  can1.onReceive(MB0,canSniff);
  can1.onReceive(MB1,canSniff);
  can1.onReceive(MB2,canSniff);
  can1.onReceive(MB3,canSniff);
  can1.onReceive(MB4,canSniff);
  can1.onReceive(MB5,canSniff);
}

void loop() {
  
  can1.events();

}

void canSniff(const CAN_message_t &msg) {
    Serial.print("MB "); Serial.print(msg.mb);
    Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" EXT: "); Serial.print(msg.flags.extended);
    Serial.print(" TS: "); Serial.print(msg.timestamp);
    Serial.print(" ID: "); Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.print(msg.buf[i], HEX); Serial.print(" ");
    } Serial.println();
  }



