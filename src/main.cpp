#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>
#ifndef skipSoftSerial
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#endif
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL Serial1
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

# a few functions for manipulating bytes
#define INT_JOIN_BYTE(u, l) (u << 8) | l

#define UPPER_BYTE(b) (b >> 8) //defines byte structure 
#define LOWER_BYTE(b) (b & 0xff)

const float DXL_PROTOCOL_VERSION = 1.0; //changed from 2.0 to 1.0 

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(9600);
  DEBUG_SERIAL.flush();
  
  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  pinMode(LED_BUILTIN, OUTPUT);
  // Turn off torque when configuring items in EEPROM area
  //Need to turn the torque off for all dynamixels DH
  for(int id = 1; id <= 4; id++){
  dxl.torqueOff(id);
  }

 for(int id = 1; id <= 4; id++){
  // set operating mode to position     
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
 }
}

void sendPositions(){
  // send positions of the dynamixels over serial
  for(int id=1; id<5;id++){
  uint16_t valueOfDyna = dxl.readControlTableItem(PRESENT_POSITION, id);
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

void recieveCommands(){
    // read dynamixel control table commands and pass them on to the Dynamixels
    if(DEBUG_SERIAL.available() >= 3){
        // reads two bytes of 
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

//sends positions every 100 milliseconds
time_now = millis();
 if((time_now  - last_serial) >= 100){
 sendPositions(); 
 last_serial = time_now;
 }

recieveCommands();

}

