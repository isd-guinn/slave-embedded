#include "HardwareSerial.h"
#ifndef SERIALCOMMUNICATION_HPP
#define SERIALCOMMUNICATION_HPP

#include <Arduino.h>

/*  SerialCommunation Protocol
    For RPI to MCU   */
/*  */

/*  
    MOSI:
      Virtual Estop
      FOC Modes (Disengage/ Engage)
      Control Mode (Angle , Speed)
      Target angle / current angle
      Target speed(x,y) / current speed (x,y)
      Vacuum Mode ( I,O )

    MOSI Pocket:   { 0x3E  0x00  0x00  0x00  0x00  }
              {  '>'   ' '   ' '   ' '   ' '  }

    |     1st Byte     |     2nd Byte     |     3rd Byte     |     4th Byte     |     5th Btye     |
    |     StartBit    |     EStopBit    |     WBDirBit    |     WBPwrBit    |     CheckSum    |
*/

/*  
    MISO
      Debug msg
      FOC Angles (by 2)


    MISO Pocket:   { 0x3E  0x00  0x00  0x00  0x00  }
              {  '>'   ' '   ' '   ' '   ' '  }

    |     1st Byte     |     2nd Byte     |     3rd Byte     |     4th Byte     |     5th Btye     |
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

    uint8_t* _rx_buffer_ptr;
    uint8_t _rx_counter;

    uint8_t* _tx_buffer_ptr;
    uint8_t _tx_counter;

  public:
    SerialInterface( HardwareSerial* serial_ptr, uint8_t pocket_size, uint8_t startbit );
    ~SerialInterface();

    uint8_t getStartBit() const;
    uint8_t getPocketSize() const;
    uint8_t* getRxBufferPtr() const;
    uint8_t getRxCounter() const;
    uint8_t* getTxBufferPtr() const;

    void Init();
    uint8_t available();

    uint8_t rxPush();
    void rxClear( bool keepStartBit );

    uint8_t getCheckSum( uint8_t* data, uint8_t size );
    bool checkCheckSum( uint8_t* data, uint8_t size );

    bool onRecievedCommand();


    void set( HardwareSerial* serial_ptr,  uint8_t pocket_size, uint8_t startbit , uint8_t* rx_buffer_ptr, uint8_t* tx_buffer_ptr );

};


#endif