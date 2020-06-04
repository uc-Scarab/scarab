#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL soft_serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

#define UPPER_BYTE(b) (b >> 8) //defines byte structure 
#define LOWER_BYTE(b) (b & 0xff)

const uint16_t DXL_ID[4] = { 2, 3, 4, 5 }; // current leg ID range is 2-5 , convert this to array 2-5 DH
const float DXL_PROTOCOL_VERSION = 1.0; //changed from 2.0 to 1.0 

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Turn off torque when configuring items in EEPROM area
  //Need to turn the torque off for all dynamixels DH
  for(int i = 2; i <= 5; i++){
  dxl.torqueOff(i);
  }

 for(int j = 2; j <= 5; j++){
  dxl.setOperatingMode(j, OP_POSITION);
 }
  //dxl.torqueOn(DXL_ID);
}

void transferData(){
 uint8_t ID[4] = { 2, 3, 4, 5 }; //ID for 8 bit transfer
 // uint16_t valueOfDyna = dxl.getPresentPosition(DXL_ID[0]);
uint16_t valueOfDyna;

byte outBuffer[12];

 valueOfDyna = dxl.getPresentPosition(ID[0]); 
 outBuffer[0] = ID[0]; 
 outBuffer[1] = LOWER_BYTE(valueOfDyna); 
 outBuffer[2] = UPPER_BYTE(valueOfDyna); 

 valueOfDyna = dxl.getPresentPosition(ID[1]); 
 outBuffer[3] = ID[1]; 
 outBuffer[4] = LOWER_BYTE(valueOfDyna); 
 outBuffer[5] = UPPER_BYTE(valueOfDyna);

 valueOfDyna = dxl.getPresentPosition(ID[2]); 
 outBuffer[6] = ID[2]; 
 outBuffer[7] = LOWER_BYTE(valueOfDyna); 
 outBuffer[8] = UPPER_BYTE(valueOfDyna);

 valueOfDyna = dxl.getPresentPosition(ID[3]); 
 outBuffer[9] = ID[3]; 
 outBuffer[10] = LOWER_BYTE(valueOfDyna); 
 outBuffer[11] = UPPER_BYTE(valueOfDyna);

DEBUG_SERIAL.write(outBuffer,12); 
DEBUG_SERIAL.flush(); 


/*
 byte outBuffer[3]; 
 outBuffer[0] = ID[0]; 
 outBuffer[1] = LOWER_BYTE(valueOfDyna); 
 outBuffer[2] = UPPER_BYTE(valueOfDyna); 

DEBUG_SERIAL.write(outBuffer,3); 
*/ 
}




void loop() {
  // put your main code here, to run repeatedly:
  //DH gets position
  //uint16_t position=dxl.getPresentPosition(DXL_ID);
 
/*
  for(int k = 2; k <= 5; k++){
    uint16_t valueOfDyna = dxl.getPresentPosition(k);
    DEBUG_SERIAL.print(String(valueOfDyna));
    DEBUG_SERIAL.print(",");
 } 
 */ 
transferData(); 
 delay(333);
}
