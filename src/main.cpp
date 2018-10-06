#include <Arduino.h>
#include <SoftwareSerial.h>


SoftwareSerial swSer(25, 14, false, 255);
#define SW_BAUD 38400

uint8_t state = 0;
uint8_t buf[4];
uint16_t lidar1 = 0, lidar2 = 0;

void setup() {
    Serial.begin(115200);
    swSer.begin(SW_BAUD);

    Serial.println("\nSoftware serial test started");
}

void StateMachine (uint16_t &_lidar1, uint16_t &_lidar2, uint8_t &_state, uint8_t *buf) {
    byte inByte = 0;
    while (swSer.available() > 0) {
        inByte = swSer.read();
        Serial.print("read byte ");
        Serial.print(inByte, DEC);
        Serial.print("\t state ");
        Serial.print(_state);  
        Serial.print("\tbytes available ");
        Serial.println(swSer.available());
    
    
        if (_state == 0 && inByte == 0x0F) {
            _state = 1;
            break;
        }

        if (_state == 1 && inByte == 0x0F)  {
            _state = 2;
            break;
        }
        else {
            _state = 0; 
            break;
        }
        
        if (_state >= 2) {
            buf[_state-2] = inByte;
            _state++;
            Serial.println(_state);
            break;
        }
        
        if (_state == 5) {
            memcpy (&lidar1, buf, 2);
            memcpy (&lidar2, buf + 2, 2);
            _state = 0;
            break;
        }
    }
}

void loop() {
    // StateMachine (lidar1, lidar2, state, buf);
    // if (state == 5) {
    //     Serial.print(lidar1);
    //     Serial.print('\t');
    //     Serial.println(lidar2);
    // }

    // uint8_t outByte = 0x0F;
    // Serial.print ("Sent:");
    // for (int i = 0; i < 10; i++) {
    //     buf[i] = rand();
    //     Serial.print (buf[i], DEC);
    //     Serial.print (' ');
    // }
    
    // Serial1.write(outByte);

    // Serial.println();
    // Serial.print("Read: ");

    bool printed = false;
    int inbytes = swSer.available();
    if (inbytes) {
      Serial.print (inbytes);
      Serial.print ('\t');
    }
    uint8_t buf[256];
    swSer.readBytes(buf, inbytes);
    for (int i = 0; i < inbytes; i++) {
        Serial.print (buf[i]); 
        Serial.print (' ');
        printed = true;
    }
    if (printed) {
        Serial.println();
    }
    
    // delay (100);
    
    // Serial.print ("Available: ");
    // Serial.println (swSer.available());
    // delay(100);
}
