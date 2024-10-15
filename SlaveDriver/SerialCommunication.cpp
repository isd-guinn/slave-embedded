#include "SerialCommunication.hpp"

SerialInterface::SerialInterface( HardwareSerial* serial_ptr, uint8_t pocket_size, uint8_t startbit = '>')
{ 
  uint8_t* rx_buffer_ptr = new uint8_t[pocket_size];
  _rx_counter = 0;
  uint8_t* tx_buffer_ptr = new uint8_t[pocket_size];
  _tx_counter = 0;
  set( serial_ptr, pocket_size, startbit, rx_buffer_ptr, tx_buffer_ptr);
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

uint8_t SerialInterface::getPocketSize() const
{
  return _pocket_size;
};


uint8_t* SerialInterface::getRxBufferPtr() const
{
  return _rx_buffer_ptr;
};

uint8_t SerialInterface::getRxCounter() const
{
  return _rx_counter;
}

uint8_t* SerialInterface::getTxBufferPtr() const
{
  return _tx_buffer_ptr;
};

void SerialInterface::Init(){};

uint8_t SerialInterface::available()
{
  return _serial_ptr->available();
}

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



uint8_t SerialInterface::getCheckSum( uint8_t* data, uint8_t size )
{

  uint8_t check_sum = 0; 
  for( int i = 0; i < size - 1; i++ ) check_sum += data[i];
  return check_sum;

};

bool SerialInterface::checkCheckSum( uint8_t* data, uint8_t size )
{

  if( getCheckSum(data, size) == data[size - 1] ) return true;
  else return false; 

};

bool SerialInterface::onRecievedCommand()
{
  if ( _serial_ptr->available() == 0 || _rx_counter >= _pocket_size)
  { 
    return false; 
  }
  
  if ( rxPush() == _start_bit )
  {
    rxClear(true);
  }

  else if ( _rx_counter <= 1 )
  {
    rxClear(false);
  }

  else if ( _rx_counter >= _pocket_size )
  {
    return true;
  }
  return false;
}



void SerialInterface::set( HardwareSerial* serial_ptr,  uint8_t pocket_size, uint8_t startbit , uint8_t* rx_buffer_ptr, uint8_t* tx_buffer_ptr ){
  _start_bit = startbit;
  _serial_ptr = serial_ptr;
  _pocket_size = pocket_size;
  _rx_buffer_ptr = rx_buffer_ptr;
  _tx_buffer_ptr = tx_buffer_ptr;
};
