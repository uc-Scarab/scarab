#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#define DEBUG_SERIAL Serial

#define UPPER_BYTE(b) (b >> 8) //defines byte structure 
#define LOWER_BYTE(b) (b & 0xff)

int time = 0;
void setup() {
  DEBUG_SERIAL.begin(115200);
}

void transferData(int time){
 uint8_t ID[4] = { 2, 3, 4, 5 }; //ID for 8 bit transfer
 // uint16_t valueOfDyna = nxl.getPresentPosition(DXL_ID[0]);
uint16_t valueOfDyna = time;

byte outBuffer[12];

 outBuffer[0] = ID[0]; 
 outBuffer[1] = LOWER_BYTE(valueOfDyna); 
 outBuffer[2] = UPPER_BYTE(valueOfDyna); 

 outBuffer[3] = ID[1]; 
 outBuffer[4] = LOWER_BYTE(valueOfDyna); 
 outBuffer[5] = UPPER_BYTE(valueOfDyna);

 outBuffer[6] = ID[2]; 
 outBuffer[7] = LOWER_BYTE(valueOfDyna); 
 outBuffer[8] = UPPER_BYTE(valueOfDyna);

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
transferData(time); 
time += 1;
delay(100);
int time_since = millis();
if (time_since >= 10000){
    time = 0;
}

}
