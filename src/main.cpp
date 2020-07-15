#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>
#ifndef skipSoftSerial
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#endif
// #if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
//   #include <SoftwareSerial.h>
//   SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
//   #define DXL_SERIAL   Serial
//   #define DEBUG_SERIAL soft_serial
//   const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
// #endif
//#define DXL_SERIAL   Serial
//#define DEBUG_SERIAL Serial1
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#define INT_JOIN_BYTE(u, l) (u << 8) | l

#define UPPER_BYTE(b) (b >> 8) //defines byte structure 
#define LOWER_BYTE(b) (b & 0xff)

//const uint16_t DXL_ID[4] = { 2, 3, 4, 5 }; // current leg ID range is 2-5 , convert this to array 2-5 DH
const float DXL_PROTOCOL_VERSION = 1.0; //changed from 2.0 to 1.0 
const int DXL_ID = 1;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(9600);
  DEBUG_SERIAL.flush();
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  pinMode(LED_BUILTIN, OUTPUT);
  // Turn off torque when configuring items in EEPROM area
  //Need to turn the torque off for all dynamixels DH
  for(int i = 1; i <= 4; i++){
  dxl.torqueOff(i);
  }

 for(int j = 1; j <= 4; j++){
  dxl.setOperatingMode(j, OP_POSITION);
  // dxl.writeControlTableItem(TORQUE_ENABLE, j, 1);
  dxl.torqueOn(j);
  // DEBUG_SERIAL.println("test");
 }
  //dxl.torqueOn(DXL_ID);
}

void transferData(){
  for(int id=1; id<5;id++){
  uint16_t valueOfDyna = dxl.readControlTableItem(PRESENT_POSITION, id);
  //uint16_t valueOfDyna = 1000;
  // DEBUG_SERIAL.print("value");
  // DEBUG_SERIAL.println(valueOfDyna);
  // DEBUG_SERIAL.print("error:");
  // DEBUG_SERIAL.println(dxl.getLastLibErrCode());

  uint8_t outBuffer[7];   

  outBuffer[0] = lowByte(60000);
  outBuffer[1] = highByte(60000);
  outBuffer[2] = 4;
  outBuffer[3] = id;
  outBuffer[4] = lowByte(valueOfDyna);
  outBuffer[5] = highByte(valueOfDyna);
  outBuffer[6] = 244;

  DEBUG_SERIAL.write(outBuffer, 7); 
  }

}

void blink(){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

void recieveData(){
    if(DEBUG_SERIAL.available() >= 3){
      //  DEBUG_SERIAL.println("test");
      // blink();
        char check_buffer[3];
        DEBUG_SERIAL.readBytes(check_buffer, 3);
        uint16_t check = INT_JOIN_BYTE(check_buffer[1], check_buffer[0]);
        if(check != 60000){
            DEBUG_SERIAL.flush();
            blink();
        } else {
          DEBUG_SERIAL.println(check);
            digitalWrite(LED_BUILTIN, HIGH);
            uint8_t payload = check_buffer[2];
            uint8_t message_buffer[payload];
            DEBUG_SERIAL.readBytes(message_buffer, payload);

            for(int i=0;i<payload -4;i+=4){
                uint8_t id = message_buffer[i];
                
                uint8_t command = message_buffer[i + 1];
                uint16_t full_byte = INT_JOIN_BYTE(message_buffer[i + 3], message_buffer[i + 2]);
                dxl.writeControlTableItem(command, id ,full_byte);
                delay(1000);


                }

                    if (message_buffer[payload - 1] != 244){
                    DEBUG_SERIAL.flush();
                    }
 
        }
    
}
}


unsigned long last_serial = 0;
unsigned long time_now = 0;


void loop() {
    //dxl.writeControlTableItem(GOAL_POSITION, 5, 0);
    //delay(1000);
    //dxl.writeControlTableItem(GOAL_POSITION, 5, 1000);
    //delay(1000);
  ////uint16_t position=dxl.getPresentPosition(DXL_ID);
 

  //for(int k = 2; k <= 5; k++){
    //uint16_t valueOfDyna = dxl.getPresentPosition(k);
    //DEBUG_SERIAL.print(String(valueOfDyna));
    //DEBUG_SERIAL.print(",");
 //} 
 time_now = millis();
 if((time_now  - last_serial) >= 100){
 transferData(); 
 last_serial = time_now;
 }
// for(int i = 1; i < 5; i++){
//   dxl.writeControlTableItem(GOAL_POSITION, i, 2048);
//   delay(1000);
// }
 recieveData();
// DEBUG_SERIAL.printl("test");
// blink();
// dxl.torqueOn(1);
//  dxl.writeControlTableItem(GOAL_POSITION, 4, 2048);
//  delay(2000);
//  dxl.writeControlTableItem(GOAL_POSITION, 4, 1000);
//  delay(2000);
// dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 1);
//  for(int j = 1; j <= 4; j++){
//   // dxl.setOperatingMode(j, OP_POSITION);
//   dxl.writeControlTableItem(24, j, 1);
//  }

  // uint16_t test = dxl.readControlTableItem(PRESENT_POSITION, 1);
  // DEBUG_SERIAL.print("value:");
  // DEBUG_SERIAL.println(test);

  // DEBUG_SERIAL.print("error:");
  // DEBUG_SERIAL.println(dxl.getLastLibErrCode());

//DEBUG_SERIAL.println("test");
}

