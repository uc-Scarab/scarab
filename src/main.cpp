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

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

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

  for (int current_id = 2; current_id < 6; current_id++)
  {
    dxl.ping(current_id);

    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(current_id);
    // dxl.writeControlTableItem(ID, current_id, 2);
    // dxl.writeControlTableItem(BAUD_RATE, current_id, 1);
    dxl.setOperatingMode(current_id, OP_POSITION);
    dxl.torqueOn(current_id);
  }
}

void loop()
{
  // int current_id = 3;
  // put your main code here, to run repeatedly:

  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value.
  // Set Goal Position in RAW value
  //  dxl.setGoalPosition(current_id, 0);
  //  delay(1000);
  //  dxl.setGoalPosition(current_id, 1000);
  //  while (dxl.readControlTableItem(MOVING, current_id) == 1);



  // // Print present position in raw value
  // DEBUG_SERIAL.print("Present Position(raw) : ");
  // DEBUG_SERIAL.println(dxl.getPresentPosition(current_id));
  // delay(1000);

  // // Set Goal Position in DEGREE value
  // dxl.setGoalPosition(current_id, 4095);
  // delay(1000);
  // // Print present position in degree value
  // DEBUG_SERIAL.print("Present Position(degree) : ");
  // DEBUG_SERIAL.println(dxl.getPresentPosition(current_id, UNIT_DEGREE));
  // delay(1000);

  long position = 0; //random(0, 4095);
  long speed =  500;// random(0, 1023);
  // dxl.writeControlTableItem(GOAL_POSITION, current_id, );

  for (int current_id = 2; current_id < 6; current_id++)
  {
    dxl.writeControlTableItem(MOVING_SPEED, current_id, speed);
  }
  delay(500);

  for (int current_id = 2; current_id < 6; current_id++)
  {
    dxl.writeControlTableItem(GOAL_POSITION, current_id, position);
    // while (dxl.readControlTableItem(MOVING, current_id) == 1);
    delay(500);
      
  }

    for (int current_id = 2; current_id < 6; current_id++)
  {
    dxl.writeControlTableItem(GOAL_POSITION, current_id, 500);
    // while (dxl.readControlTableItem(MOVING, current_id) == 1);
    delay(500);
      
  }

}
