#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#define DEBUG_SERIAL Serial
#define INT_JOIN_BYTE(u, l) (u << 8) | l

#define UPPER_BYTE(b) (b >> 8) //defines byte structure 
#define LOWER_BYTE(b) (b & 0xff)

void setup() {
  DEBUG_SERIAL.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void readBytes(){
    if(DEBUG_SERIAL.available() >= 3){
        uint8_t check_buffer[3];
        DEBUG_SERIAL.readBytes(check_buffer, 3);
        uint16_t check = INT_JOIN_BYTE(check_buffer[1], check_buffer[0]);
        if(int(check) != 60000){
            DEBUG_SERIAL.flush();
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, HIGH);
            int payload = int(check_buffer[2]);
            uint8_t message_buffer[payload];
            DEBUG_SERIAL.readBytes(message_buffer, payload);

            //for(int i=0;i<payload -3;i+=3){
                //uint16_t full_byte = INT_JOIN_BYTE(message_buffer[i + 2], message_buffer[i + 1]);
                //}

                    if (message_buffer[payload - 1] != 244){
                    DEBUG_SERIAL.flush();
                    }
 
        }
    }
}

void writeBytes(){
    uint8_t message[7];

    message[0] = lowByte(60000);
    message[1] = highByte(60000);
    message[2] = 4;
    message[3] = 2;
    message[4] = lowByte(1000);
    message[5] = highByte(1000);
    message[6] = 244;

    DEBUG_SERIAL.write(message, 7);

}

void loop(){
    //readBytes();
    writeBytes();
    readBytes();
    delay(100);
}
