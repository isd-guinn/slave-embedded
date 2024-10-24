#include "SerialCommunication.hpp"

SerialInterface::SerialInterface( HardwareSerial* serial_ptr, uint8_t rx_packet_size, uint8_t tx_packet_size, uint8_t startbit = '>',  uint8_t endbit = 0x3F)
{ 
  uint8_t* rx_buffer_ptr = new uint8_t[rx_packet_size];
  _rx_counter = 0;
  uint8_t* tx_buffer_ptr = new uint8_t[tx_packet_size];
  _tx_counter = 0;

  _is_packet = false;
  set( serial_ptr, rx_packet_size, tx_packet_size, startbit, endbit, rx_buffer_ptr, tx_buffer_ptr);
};

SerialInterface::~SerialInterface()
{
  delete[] _rx_buffer_ptr;
  delete[] _tx_buffer_ptr;
}

uint8_t SerialInterface::getStartBit() const
{
  return _start_bit;
};

uint8_t SerialInterface::getRxPacketSize() const
{
  return _rx_packet_size;
};

uint8_t SerialInterface::getTxPacketSize() const
{
  return _tx_packet_size;
};


uint8_t* SerialInterface::getRxBufferPtr() const
{
  return _rx_buffer_ptr;
};

uint8_t SerialInterface::getRxCounter() const
{
  return _rx_counter;
}

uint8_t SerialInterface::getTxCounter() const
{
  return _tx_counter;
}

uint8_t* SerialInterface::getTxBufferPtr() const
{
  return _tx_buffer_ptr;
};

uint8_t SerialInterface::rxPush( )
{
  uint8_t byte = _serial_ptr->read();
  _rx_buffer_ptr[_rx_counter++] = byte;
  return byte;
}

void SerialInterface::rxClear( bool keepStartBit )
{
  _rx_counter = (keepStartBit)?1:0 ;
}

//push tx return if suceess
bool SerialInterface::txPush( uint8_t inByte )
{
  if (_tx_counter>=_tx_packet_size) return false;
  _tx_buffer_ptr[_tx_counter++] = inByte;
  return true;
}

//send data to tx
void SerialInterface::txSend()
{
  for( int i = 0; i < _tx_packet_size; i++ )
  {
    Serial.printf("%x ",_tx_buffer_ptr[i]);
    _serial_ptr->write(_tx_buffer_ptr[i]);
  }
  Serial.println("|| Sent");
}

void SerialInterface::txClear( bool keepStartBit )
{
  _tx_counter = (keepStartBit)?1:0 ;
}


uint8_t SerialInterface::getCheckSum( uint8_t* data, uint8_t size )
{

  uint8_t check_sum = 0; 
  for( int i = 0; i < size-2; i++ ) check_sum += data[i];
  return check_sum;

};

bool SerialInterface::checkCheckSum( uint8_t* data, uint8_t size )
{

  if( getCheckSum(data, size) == data[size - 2] ) return true;
  else return false; 

};

bool SerialInterface::onRecievedCommand()
{ 

  // If there is nothing in the hardware buffer
  if( _serial_ptr->available() == 0 ) 
  {
    // Serial.println("Nothing");
    return false;
  }

  else if( !_is_packet)
  {
    if (rxPush() ==_start_bit)
    { 
      _is_packet=true;
      rxClear(true);
      // Serial.println("Start bit get");
      return false;
    }

    rxClear(false);
    return false;
    
  }

  // If we know that there somthing sending in the middle of the packet
  else
  {
    // Target size of bytes for a packet is reached
    if(_serial_ptr->available() >= _rx_packet_size - 1)
    {
      // Serial.println("Reached Desired Amount of bytes");
      //Recieve _rx_packet_size-1 bytes
      for(int i=1;i<_rx_packet_size;i++)
      {
        rxPush();
      }

      _is_packet = false;

      if(checkCheckSum( _rx_buffer_ptr, _rx_packet_size ) && _rx_buffer_ptr[_rx_packet_size-1] == _end_bit)
      {
        return true;
      }

      // if not packet
      rxClear(false);
      return false;
    }
    // Serial.printf("Not enough!!!!! %d \n", _serial_ptr->available());
    return false;
  }
  
  return false;
}

void SerialInterface::set( HardwareSerial* serial_ptr,  uint8_t rx_packet_size, uint8_t tx_packet_size, uint8_t startbit, uint8_t endbit, uint8_t* rx_buffer_ptr, uint8_t* tx_buffer_ptr )
{
  _start_bit = startbit;
  _end_bit = endbit;
  _serial_ptr = serial_ptr;
  _rx_packet_size = rx_packet_size;
  _tx_packet_size = tx_packet_size;
  _rx_buffer_ptr = rx_buffer_ptr;
  _tx_buffer_ptr = tx_buffer_ptr;
};
