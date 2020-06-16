#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL soft_serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#define INT_JOIN_BYTE(u, l) (u << 8) | l

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
  uint8_t id = 5;
  uint16_t valueOfDyna = dxl.readControlTableItem(PRESENT_POSITION, id);
  //uint16_t valueOfDyna = 1000;
  uint8_t outBuffer[7];   

  outBuffer[0] = lowByte(60000);
  outBuffer[1] = highByte(60000);
  outBuffer[2] = 4;
  outBuffer[3] = 5;
  outBuffer[4] = lowByte(valueOfDyna);
  outBuffer[5] = highByte(valueOfDyna);
  outBuffer[6] = 244;

  //DEBUG_SERIAL.println("test");
 
  DEBUG_SERIAL.write(outBuffer, 7); 


/*
 byte outBuffer[3]; 
 outBuffer[0] = ID[0]; 
 outBuffer[1] = LOWER_BYTE(valueOfDyna); 
 outBuffer[2] = UPPER_BYTE(valueOfDyna); 

DEBUG_SERIAL.write(outBuffer,3); 
*/ 
}

//void recieveData(){
    //if(DEBUG_SERIAL.available() >= 3){
        //uint8_t check_buffer[3];
        //DEBUG_SERIAL.readBytes(check_buffer, 3);
        //uint16_t check = INT_JOIN_BYTE(check_buffer[1], check_buffer[0]);
        //if(int(check) != 60000){
            //DEBUG_SERIAL.flush();
            //digitalWrite(LED_BUILTIN, LOW);
        //} else {
            //digitalWrite(LED_BUILTIN, HIGH);
            //int payload = int(check_buffer[2]);
            //uint8_t message_buffer[payload];
            //DEBUG_SERIAL.readBytes(message_buffer, payload);

            //for(int i=0;i<payload -3;i+=3){
                //int id = int(message_buffer[i]);
                //uint16_t full_byte = INT_JOIN_BYTE(message_buffer[i + 2], message_buffer[i + 1]);
                //dxl.writeControlTableItem(GOAL_POSITION, id ,int(full_byte));


                //}

                    ////if (message_buffer[payload - 1] != 244){
                    ////DEBUG_SERIAL.flush();
                    ////}
 
        //}
    //}
//}





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
//recieveData();
 delay(100);
}
