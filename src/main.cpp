/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
#include <Arduino.h>
#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#include <SoftwareSerial.h>
    SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
    #define DXL_SERIAL Serial
    #define DEBUG_SERIAL soft_serial
    const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
    #define DXL_SERIAL Serial
    #define DEBUG_SERIAL SerialUSB
    const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
    #define DXL_SERIAL Serial1
    #define DEBUG_SERIAL SerialUSB
    const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
    #define DXL_SERIAL Serial3       //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
    #define DEBUG_SERIAL Serial
    const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
    // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
    // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
    #define DXL_SERIAL Serial3
    #define DEBUG_SERIAL Serial
    const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#else // Other boards when using DynamixelShield
    #define DXL_SERIAL Serial1
    #define DEBUG_SERIAL Serial
    const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

// const uint8_t current_id = 1;
const float DXL_PROTOCOL_VERSION = 1.0;

int incomingByte = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
#define INT_JOIN_BYTE(u, l) (u << 8) | l
void setup()
{
  // put your setup code here, to run once:

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  //Serial.begin(9600);
  
  for (int current_id = 2; current_id < 6; current_id++)
  {
    //dxl.ping(current_id);

    // Turn off torque when configuring items in EEPROM area
    //dxl.torqueOff(current_id);
    // dxl.writeControlTableItem(ID, current_id, 2);
    // dxl.writeControlTableItem(BAUD_RATE, current_id, 1);
    dxl.setOperatingMode(current_id, OP_POSITION);
    dxl.torqueOff(current_id);
  }

  //long position = 0; //random(0, 4095);
  //long speed =  200;// random(0, 1023);
  // dxl.writeControlTableItem(GOAL_POSITION, current_id, );

  //for(int current_id = 2; current_id < 6; current_id++)
  //{
    //dxl.writeControlTableItem(MOVING_SPEED, current_id, speed);
  //}
  //for(int current_id = 2; current_id < 6; current_id++)
  //{
    //dxl.writeControlTableItem(GOAL_POSITION, current_id, 0);
  //}
    ////dxl.writeControlTableItem(MOVING_SPEED, 02, 0900);
  
} 

unsigned long interval = 100;
unsigned long last_time = 0;


void loop()
{

  unsigned long timenow = millis();
  if (timenow > (last_time + interval)){
      last_time = timenow;
      //uint16_t positions[4] ;
      String positions;

    for(int i = 2; i < 6; i++){
        uint16_t position = dxl.readControlTableItem(PRESENT_POSITION, i);

        if (i == 2){
            positions += String(position);
        } else {
            positions += "," + String(position);
        }
        //positions[i - 2] = position;

        //int bytes = DEBUG_SERIAL.write(positions);
        //DEBUG_SERIAL.println(bytes);
        

    }

    DEBUG_SERIAL.println(positions);

    //DEBUG_SERIAL.println(positions);

  }
  //unsigned long timenow = millis();
  //if (timenow > (last_time + interval)){
      //last_time = timenow;
 
    //uint8_t buffer[4];
    //if(DEBUG_SERIAL.available() > 0){
        //DEBUG_SERIAL.readBytes(buffer, 4);
        //DEBUG_SERIAL.println(int(buffer));
    //}

    //if(buffer[1] == 1){
        //if((INT_JOIN_BYTE(buffer[2], buffer[3])) == 0){
        //dxl.torqueOff(buffer[0]);
        //delay(2000);
        //}
        //if((INT_JOIN_BYTE(buffer[2], buffer[3])) == 1){
        //dxl.torqueOn(buffer[0]);
        //delay(2000);
        //}
    //}

    //if(buffer[1] == 2){
        //dxl.writeControlTableItem(MOVING_SPEED, buffer[0], INT_JOIN_BYTE(buffer[2], buffer[3]));
    //}
    //if(buffer[1] == 3){
        //dxl.writeControlTableItem(GOAL_POSITION, buffer[0], INT_JOIN_BYTE(buffer[2], buffer[3]));
    //delay(2000);
    //}
  //}
}
