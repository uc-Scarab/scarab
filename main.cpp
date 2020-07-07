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

/*
* Please refer to each DYNAMIXEL eManual(http://emanual.robotis.com/docs/en/dxl/) for more information regarding Torque.
*/

#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>

// Please modify it to suit your hardware.s
#define INT_JOIN_BYTE(u, l) (u << 8) | l

#define UPPER_BYTE(b) (b >> 8) //defines byte structure 
#define LOWER_BYTE(b) (b & 0xff)

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
void transferData(){
  for(int i=0; i<5;i++){
  uint8_t id = i;
  uint16_t valueOfDyna = dxl.readControlTableItem(PRESENT_POSITION, id);
  //uint16_t valueOfDyna = 1000;
  uint8_t outBuffer[7];   

  outBuffer[0] = lowByte(60000);
  outBuffer[1] = highByte(60000);
  outBuffer[2] = 4;
  outBuffer[3] = i;
  outBuffer[4] = lowByte(valueOfDyna);
  outBuffer[5] = highByte(valueOfDyna);
  outBuffer[6] = 244;

  DEBUG_SERIAL.write(outBuffer, 7); 
  }

}

void recieveData(){
    if(DEBUG_SERIAL.available() >= 3){
        uint8_t check_buffer[3];
        DEBUG_SERIAL.readBytes(check_buffer, 3);
        uint16_t check = INT_JOIN_BYTE(check_buffer[1], check_buffer[0]);
        if(int(check) != 60000){
            DEBUG_SERIAL.flush();
            digitalWrite(LED_BUILTIN, LOW);
        } else {
            digitalWrite(LED_BUILTIN, HIGH);
            int payload = int(check_buffer[2]);
            uint8_t message_buffer[payload];
            DEBUG_SERIAL.readBytes(message_buffer, payload);

            for(int i=0;i<payload -3;i+=3){
                int id = int(message_buffer[i]);
                uint16_t full_byte = INT_JOIN_BYTE(message_buffer[i + 2], message_buffer[i + 1]);
                dxl.writeControlTableItem(GOAL_POSITION, id ,int(full_byte));


                }

                    //if (message_buffer[payload - 1] != 244){
                    //DEBUG_SERIAL.flush();
                    //}
 
        }
    }
}
void setup() {
  // put your setup code here, to run once:

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  DEBUG_SERIAL.begin(115200);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Turn on the output Torque.
  dxl.torqueOn(DXL_ID);
  delay(2000);
  
  // Turn off the output Torque.
  dxl.torqueOff(DXL_ID);
  delay(2000);
}

