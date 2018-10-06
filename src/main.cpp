#include <Arduino.h>
#include <SoftwareSerial.h>


SoftwareSerial swSer(16, 12, false, 255);
#define SW_BAUD 38400

uint8_t state = 0;
uint8_t buf[4];
uint16_t lidar1 = 0, lidar2 = 0;

void setup() {
    Serial.begin(115200);
    swSer.begin(SW_BAUD);

    Serial.println("\nSoftware serial test started");
}

void StateMachine (uint16_t &_lidar1, uint16_t &_lidar2, uint8_t &_state, uint8_t *buf, bool &flag) {
    byte inByte = 0;
    while (swSer.available() > 0) {
        inByte = swSer.read();
        Serial.print("read byte ");
        Serial.print(inByte, DEC);
        Serial.print("\t state ");
        Serial.println(_state); 
    
        if (_state == 6) {
            uint8_t temp;

            // changing endians
            temp = buf[0];
            buf[0] = buf[1];
            buf[1] = temp;

            temp = buf[2];
            buf[2] = buf[3];
            buf[3] = temp;
            memcpy (&lidar1, buf, 2);
            memcpy (&lidar2, buf + 2, 2);

            state = 0;
            flag = true;
        }

        if (_state >= 2 && state < 6) {
            buf[_state-2] = inByte;
            _state++;
        }

        if (_state == 1) {
            if (inByte == 0x0F)  {
                _state = 2;
            }
            else {
                _state = 0; 
            }
        }

        if (_state == 0 && inByte == 0x0F) {
            _state = 1;
        }
    }
}

void loop() {
    bool f = false;
    StateMachine (lidar1, lidar2, state, buf, f);
    if (f) {
        Serial.print(lidar1);
        Serial.print('\t');
        Serial.println(lidar2);
    }
}
