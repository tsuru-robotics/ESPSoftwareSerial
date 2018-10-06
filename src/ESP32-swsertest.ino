
// #include <Arduino.h>
// #include <Esp.h>

// extern "C" {
//     #include "esp32-hal-gpio.h"
// }

//    int m_rxPin, m_txPin, m_txEnablePin;
//    bool m_rxValid, m_rxEnabled;
//    bool m_txValid, m_txEnableValid;
//    bool m_invert;
//    bool m_overflow;
//    unsigned long m_bitTime;
//    unsigned int m_inPos, m_outPos;
//    int m_buffSize;
//    uint8_t *m_buffer;


// #define MAX_PIN 35
// #define DEFAULT_BUAD_RATE 19200


// #define BAUD_RATE 38400


// void setup() {
//   Serial.begin(115200);
//   begin(38400);

//   Serial.println("\nSoftware serial test started");
//   confSoftwareSerial(14, 12, false, 64);
// }

// void loop() {
//   /*
//   Serial.write("test");
//   delay(100);
//   return;
//   */

//   bool newStr = false;
//   while (available() > 0) {
//     Serial.print(read());
//     Serial.print(' ');
//     yield();
//     newStr = true;
//   }  
// //   read();
//   if (newStr)
//     Serial.println();

// }

// void begin(long speed) {
//    // Use getCycleCount() loop to get as exact timing as possible
//    m_bitTime = ESP.getCpuFreqMHz()*1000000/speed;

//    if (!m_rxEnabled)
//      enableRx(true);
// }

// long baudRate() {
//    return ESP.getCpuFreqMHz()*1000000/m_bitTime;
// }

// int read() {
//    if (!m_rxValid || (m_inPos == m_outPos)) return -1;
//    uint8_t ch = m_buffer[m_outPos];
//    m_outPos = (m_outPos+1) % m_buffSize;
//    return ch;
// }

// int available() {
//    if (!m_rxValid) return 0;
//    int avail = m_inPos - m_outPos;
//    if (avail < 0) avail += m_buffSize;
//    return avail;
// }

// void confSoftwareSerial(int receivePin, int transmitPin, bool inverse_logic, unsigned int buffSize) {
//    m_rxValid = m_txValid = m_txEnableValid = false;
//    m_buffer = NULL;
//    m_invert = inverse_logic;
//    m_overflow = false;
//    m_rxEnabled = false;
//    if (isValidGPIOpin(receivePin)) {
//       m_rxPin = receivePin;
//       m_buffSize = buffSize;
//       m_buffer = (uint8_t*)malloc(m_buffSize);
//       if (m_buffer != NULL) {
//          m_rxValid = true;
//          m_inPos = m_outPos = 0;
//          pinMode(m_rxPin, INPUT);
//         //  Serial.println("1");
//         //  Serial.println(m_rxPin);
//          enableRx(true);
//       }
//    }
//    if (isValidGPIOpin(transmitPin)) {
//       m_txValid = true;
//       m_txPin = transmitPin;
//       pinMode(m_txPin, OUTPUT);
//       digitalWrite(m_txPin, !m_invert);
//    }
//    // Default speed
//    //begin(DEFAULT_BUAD_RATE);
// }

// bool isValidGPIOpin(int pin) {
//    return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= MAX_PIN);
// }

// #define WAIT { while (ESP.getCycleCount()-start < wait); wait += m_bitTime; }

// void IRAM_ATTR rxRead() {
//    // Advance the starting point for the samples but compensate for the
//    // initial delay which occurs before the interrupt is delivered
//    unsigned long wait = m_bitTime + m_bitTime/3 - 500;
//    unsigned long start = ESP.getCycleCount();
//    uint8_t rec = 0;
//    for (int i = 0; i < 8; i++) {
//      WAIT;
//      rec >>= 1;
//      if (digitalRead(m_rxPin))
//        rec |= 0x80;
//    }
//    if (m_invert) rec = ~rec;
//    // Stop bit
//    WAIT;
//    // Store the received value in the buffer unless we have an overflow
//    int next = (m_inPos+1) % m_buffSize;
//    if (next != m_outPos) {
//       m_buffer[m_inPos] = rec;
//       m_inPos = next;
//    } else {
//       m_overflow = true;
//    }
   
//    // Must clear this bit in the interrupt register,
//    // it gets set even when interrupts are disabled
//    GPIO.status_w1tc = 0;
//     return;
// }

// void enableRx(bool on) {
//    if (m_rxValid) {
//       if (on)
//          attachInterrupt(m_rxPin, rxRead, m_invert ? RISING : FALLING);
//       else
//          detachInterrupt(m_rxPin);
//       m_rxEnabled = on;
//    }
//    Serial.println("4");
// }


