#ifndef SERIALCOMMUNICATION_HPP
#define SERIALCOMMUNICATION_HPP

#include <Arduino.h>

/*  SerialCommunation Protocol
    For RPI to MCU   */
/*  */
/*  
    Pocket:   { 0x3E  0x00  0x00  0x00  0x00  }
              {  '>'   ' '   ' '   ' '   ' '  }

    |     1st Bit     |     2nd Bit     |     3rd Bit     |     4th Bit     |     5th Bit     |
    |     StartBit    |     EStopBit    |     WBDirBit    |     WBPwrBit    |     CheckSum    |
*/

#define POCKET_SIZE 5

// Define the Bit for the Protocol
#define StartBit 0x3E

// Wheelbase Enable Bits
#define EStopEnabledBit  0xA1
#define EstopDisabledBit 0xA2

// Wheelbase Direction Bits
#define WBDirStopBit  0xA0
#define WBDirForwardBit  0xA1
#define WBDirBackwardBit 0xA2
#define WBDirClockwiseBit 0xA3
#define WBDirAntiClkwiseBit 0xA4

// Wheelbase Power Bits
#define WheelbasePowerBit(x) map(x,0,100,0x00,0xFF)


class SerialInterface
{
  private:
    HardwareSerial* _serial_ptr;
    uint8_t _start_bit;
    uint8_t _pocket_size;
    uint8_t _rx_buffer[POCKET_SIZE];
    uint8_t _tx_buffer[POCKET_SIZE];

  public:
    SerialInterface();
    SerialInterface( uint8_t startbit, HardwareSerial* serial_ptr, uint8_t pocket_size );

    uint8_t getStartBit() const;
    uint8_t getPocketSize() const;
    uint8_t getRxBuffer( uint8_t bit_num ) const;
    uint8_t getTxBuffer( uint8_t bit_num ) const;

    void Init();

    uint8_t getCheckSum( uint8_t* data, uint8_t size );
    bool checkCheckSum( uint8_t* data, uint8_t size );

    void set( uint8_t startbit, HardwareSerial* serial_ptr, uint8_t pocket_size);

};


#endif