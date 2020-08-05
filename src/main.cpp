#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>
#ifndef skipSoftSerial
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#endif
//#define DXL_SERIAL   Serial // serial connection to Dynamixels
//#define COMPUTER_SERIAL Serial1 // serial connection to computer
// a few functions for manipulating bytes
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
// a few functions for manipulating bytes
#define INT_JOIN_BYTE(u, l) (u << 8) | l
#define UPPER_BYTE(b) (b >> 8) //defines byte structure
#define LOWER_BYTE(b) (b & 0xff)
#define baudrate 115200
#define dxlTimeout 60

const float DXL_PROTOCOL_VERSION = 1.0; //changed from 2.0 to 1.0
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
void setup()
{
  // put your setup code here, to run once:
  COMPUTER_SERIAL.begin(baudrate);
  COMPUTER_SERIAL.flush();
  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Turn off torque when configuring items in EEPROM area
  for (int id = 1; id <= 25; id++)
  {
    dxl.torqueOff(id);
  }
  for (int id = 1; id <= 25; id++)
  {
    // set operating mode to position
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.writeControlTableItem(id, MOVING_SPEED, 500);
    dxl.torqueOn(id);
    dxl.writeControlTableItem(MAX_TORQUE, id, 200);
  }
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}  

void dynamixelWrite(uint8_t motor_id, uint8_t command_id, uint16_t value){
    for(int i = 0; i < 3; i++){  
          dxl.writeControlTableItem(command_id, motor_id, value, dxlTimeout);
          int errorCode = dxl.getLastLibErrCode();
          if(errorCode == 0){
              break;
          }
         
}
}

void sendPositions()
{
  // sends positions of the dynamixels over serial
  for (int id = 1; id < 25; id++)
  {
    uint16_t valueOfDyna = dxl.readControlTableItem(PRESENT_POSITION, id);
    uint8_t outBuffer[7];
    //starts with header of first two bytes which is always 60000
    outBuffer[0] = lowByte(60000);
    outBuffer[1] = highByte(60000);
    // third byte payload which is the number of bytes in the message
    outBuffer[2] = 4;
    // the message reports the position of any number of motors with an id, and two bytes for the position
    outBuffer[3] = id;
    outBuffer[4] = lowByte(valueOfDyna);
    outBuffer[5] = highByte(valueOfDyna);
    //then the message ends with the footer which is always 244
    outBuffer[6] = 244;
    COMPUTER_SERIAL.write(outBuffer, 7);
  }
}
void blink()
{
  //blinks the inbuilt led for debugging
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
}
void setCurrentPositionAndVelocity(uint16_t goal_position, uint8_t dynid)
{
  //Takes goal and current servo position and calculates velocity required to reach goal within 100ms.
  //
  if (goal_position > 4095)
  {
    goal_position = 4095;
  }
  uint16_t cur_pos = dxl.readControlTableItem(PRESENT_POSITION, dynid);
  uint16_t goal_speed = goal_position - cur_pos;
  goal_speed = (goal_speed / 0.1);
  goal_speed = ((goal_speed / 651.7587) * 84.0731);
  goal_speed = round(goal_speed);
  if (goal_speed > 1023)
  {
    goal_speed = 1023;
  }
  uint16_t overshoot_position = cur_pos + (cur_pos - goal_position) * 1.5;
  dxl.writeControlTableItem(MOVING_SPEED, dynid, goal_speed);
  dxl.writeControlTableItem(GOAL_POSITION, dynid, overshoot_position);
}
void recieveCommands()
{
  // read dynamixel control table commands and pass them on to the Dynamixels
  if (COMPUTER_SERIAL.available() >= 3)
  {
    // reads two bytes of header and one byte for the payload. If the two header bytes are equal to 60000 then read in a number of bytes equal to the payload.
    char check_buffer[3];
    COMPUTER_SERIAL.readBytes(check_buffer, 3);
    uint16_t check = INT_JOIN_BYTE(check_buffer[1], check_buffer[0]);
    if (check != 60000)
    {
        serialFlush();
    }
    else
    {
      uint8_t payload = check_buffer[2];
      uint8_t message_buffer[payload];
      COMPUTER_SERIAL.readBytes(message_buffer, payload);
      for (int i = 0; i < payload - 4; i += 4)
      {
        //writes a control table item command using motor id, command id and value.
        uint8_t id = message_buffer[i];
        uint8_t command = message_buffer[i + 1];
        uint16_t full_byte = INT_JOIN_BYTE(message_buffer[i + 3], message_buffer[i + 2]);
        if ((command == GOAL_POSITION) && (full_byte <= 4095))
        {
            dynamixelWrite(id, command, full_byte);
            //dxl.writeControlTableItem(command, id, full_byte);
         
          
          //setCurrentPositionAndVelocity(id, full_byte);
        }else {
          dxl.writeControlTableItem(command, id, full_byte);
        }
      }
      // flushes the serial buffer if the footer doesn't equal 244
      if (message_buffer[payload - 1] != 244)
      {
          serialFlush();
      }
    }
  }
}


unsigned long last_serial = 0;
unsigned long time_now = 0;
void loop()
{
  //sends positions every 100 milliseconds
  time_now = millis();
  if ((time_now - last_serial) >= 100)
  {
    //sendPositions();
    last_serial = time_now;
    
  }

    recieveCommands();
}
