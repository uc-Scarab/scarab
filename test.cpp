#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#define DEBUG_SERIAL Serial
#define INT_JOIN_BYTE(u, l) (u << 8) | l

#define UPPER_BYTE(b) (b >> 8) //defines byte structure 
#define LOWER_BYTE(b) (b & 0xff)

int time = 0;
int start = 65336;
int payload = 12;
void setup() {
  DEBUG_SERIAL.begin(115200);
}
int receiveData(uint8_t in_buffer[3]){
        int payload = int(in_buffer[2]);
        uint8_t data_buffer[payload];
        DEBUG_SERIAL.readBytes(data_buffer, payload);
        int value = int(INT_JOIN_BYTE(data_buffer[4], data_buffer[5])) + 1;
        byte new_payload[payload + 3];
        new_payload[0] = LOWER_BYTE(65336);
        new_payload[1] = UPPER_BYTE(65336);
        new_payload[2] = payload;
        
        for(int i=3; i<(payload - 3); i+=3){
            new_payload[i] = int(data_buffer[i]);
            new_payload[i + 1] = LOWER_BYTE(value);
            new_payload[i + 2] = UPPER_BYTE(value);
        }

        if (int(data_buffer[payload - 1]) != 244){
            DEBUG_SERIAL.flush();
        }
        

        new_payload[payload - 1] = 244;

        return(new_payload);
    
}

void transferData(byte new_payload){
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
 
 DEBUG_SERIAL.write(new_payload);   

//DEBUG_SERIAL.write(outBuffer,16); 
//DEBUG_SERIAL.flush(); 



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

uint8_t in_buffer[3];
DEBUG_SERIAL.read(in_buffer, 3);
uint8_t check = INT_JOIN_BYTE(in_buffer[1], in_buffer[0]);
if(int(check) == 65336){
    byte new_stuff = recieveData(in_buffer);
    transferData(new_stuff);
   }
}

time += 1;
delay(100);
}

}
