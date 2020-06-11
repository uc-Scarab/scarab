#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#define DEBUG_SERIAL Serial
#define INT_JOIN_BYTE(u, l) (u << 8) | l

#define UPPER_BYTE(b) (b >> 8) //defines byte structure 
#define LOWER_BYTE(b) (b & 0xff)

bool alt = true;
//int payload = 12;
void setup() {
  DEBUG_SERIAL.begin(115200);
}
//void receiveData(uint8_t in_buffer[3]){
        //int payload = int(in_buffer[2]);
        //uint8_t data_buffer[payload];
        //DEBUG_SERIAL.readBytes(data_buffer, payload);
        //int value = int(INT_JOIN_BYTE(data_buffer[0], data_buffer[1])) + 1;
        //uint8_t new_payload[payload + 3];
        //new_payload[0] = LOWER_BYTE(65336);
        //new_payload[1] = UPPER_BYTE(65336);
        //new_payload[2] = payload;
        
        //for(int i=3; i<(payload - 3); i+=3){
            //new_payload[i] = int(data_buffer[i]);
            //new_payload[i + 1] = LOWER_BYTE(value);
            //new_payload[i + 2] = UPPER_BYTE(value);
        //}

        //if (int(data_buffer[payload - 1]) != 244){
            //DEBUG_SERIAL.flush();
        //}
        
        //new_payload[payload - 1] = 244;
        //DEBUG_SERIAL.write(new_payload, sizeof(new_payload));   
    
//}

//void transferData(byte new_payload){
 //uint8_t ID[15] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 }; //ID for 8 bit transfer
 // uint16_t valueOfDyna = nxl.getPresentPosition(DXL_ID[0]);
//uint16_t valueOfDyna = time;

//byte outBuffer[sizeof(new_payload)];
//outBuffer[0] = LOWER_BYTE(new_payload[0]);
//outBuffer[1] = UPPER_BYTE(new_payload[0]);
//outBuffer[2] = new_payload[1];
//for(int i=3;int i<payload.length - 3; i+=3){
    //outBuffer[i] = new_payload[i];
    //outBuffer[i + 1] = LOWER_BYTE(new_payload[i + 1]);
    //outBuffer[i + 2] = UPPER_BYTE(new_payload[i + 1]);

//}
 //outBuffer[0] = LOWER_BYTE(start);    
 //outBuffer[1] = UPPER_BYTE(start);
 //outBuffer[2] = payload;
 //outBuffer[3] = ID[0]; 
 //outBuffer[4] = LOWER_BYTE(valueOfDyna); 
 //outBuffer[5] = UPPER_BYTE(valueOfDyna); 

 //outBuffer[6] = ID[1]; 
 //outBuffer[7] = LOWER_BYTE(valueOfDyna); 
 //outBuffer[8] = UPPER_BYTE(valueOfDyna);

 //outBuffer[9] = ID[2]; 
 //outBuffer[10] = LOWER_BYTE(valueOfDyna); 
 //outBuffer[11] = UPPER_BYTE(valueOfDyna);

 //outBuffer[12] = ID[3]; 
 //outBuffer[13] = LOWER_BYTE(valueOfDyna); 
 //outBuffer[14] = UPPER_BYTE(valueOfDyna);
 //outBuffer[15] = 244;
 

//DEBUG_SERIAL.write(outBuffer,16); 
//DEBUG_SERIAL.flush(); 



/*
 byte outBuffer[3]; 
 outBuffer[0] = ID[0]; 
 outBuffer[1] = LOWER_BYTE(valueOfDyna); 
 outBuffer[2] = UPPER_BYTE(valueOfDyna); 

DEBUG_SERIAL.write(outBuffer,3); 
*/ 





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
    uint8_t check_buffer[3];
    DEBUG_SERIAL.readBytes(check_buffer, 3);
    uint16_t check = INT_JOIN_BYTE(check_buffer[1], check_buffer[0]);

    if(check != 60000){
        DEBUG_SERIAL.flush();
    }

    payload = int(check_buffer[2]);
    uint8_t message_buffer[payload];
    DEBUG_SERIAL.readBytes(message_buffer, payload);


    message = payload -1;
    int ids[message/3];
    int positions[message/3];

    for(int i =0; i<message;i+=3){
        
    }


    uint8_t trash[4];
    DEBUG_SERIAL.readBytes(trash, 4);
    //if(int(check) == 60000){
        //DEBUG_SERIAL.println("check");
    //}
    

    //uint8_t test[9];
    //if(alt){

    //test[0] = highByte(60000);
    //test[1] = lowByte(60000);
    //test[2] = 4;
    //test[3] = 5;
    //test[4] = highByte(3000);
    //test[5] = lowByte(3000);
    //test[6] = 244;
    //DEBUG_SERIAL.write(test, 7);
    //}else {
    //test[0] = highByte(60000);
    //test[1] = lowByte(60000);
    //test[2] = 7;
    //test[3] = 5;
    //test[4] = highByte(2000);
    //test[5] = lowByte(2000);
    //test[6] = 6;
    //test[7] = highByte(5000);
    //test[8] = lowByte(5000);
    //test[9] = 244;
    //DEBUG_SERIAL.write(test, 10);

    //}
   
//alt = !alt;
delay(100);
}

