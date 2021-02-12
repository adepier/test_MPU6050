#ifndef _ARDUINO_SERIAL_H_
#define _ARDUINO_SERIAL_H_

#include "mbed.h"

enum Format { BIN, OCT, DEC, HEX, NUMBER };

class ArduinoSerial
{
public:
    ArduinoSerial() {}
    // ArduinoSerial() : serial(USBTX, USBRX) {}
    // ArduinoSerial(int baudrate) : serial(USBTX, USBRX) {
    //     baud(baudrate);
    // }
    // ArduinoSerial(PinName tx, PinName rx, int baudrate = 9600) : serial(tx, rx) {
    //     baud(baudrate);
    // }

private:
    //Serial serial;

public:
    void begin(int baudrate) {
        //baud(baudrate);
        
    }

    void inline print(const char* x) {
        printf("%s", x);
    }
    
    template <typename T>
    void inline print(T x) {
        printf("%i", (float)x);
    }
    
    template <typename T>
    void inline print(T x, Format fmt) {
        if (fmt == BIN) {
            printf("We aren't supporting this format: %d", x);
        } else if(fmt == OCT) {
            printf("%o", x);
        } else if (fmt == DEC) {
            printf("%d", x);
        } else if (fmt == HEX) {
            printf("%x", x);
        } else {
            printf("%g", x);
        }
    }
    
    template <typename T>
    void inline println(T x) {
        ArduinoSerial::print(x);
        printf("\r\n");
    }
    
    template <typename T>
    void inline println(T x, Format fmt) {
        ArduinoSerial::print(x, fmt);
        printf("\r\n");
    }

    void inline write(const uint8_t packet) {
        //putc(packet);
    }

    void inline write(const uint8_t* packet, uint8_t length) {
        for (int i = 0; i < length; ++i) {
            //putc(packet[i]);
        }
    }
};

#endif /* _ARDUINO_SERIAL_H_ */