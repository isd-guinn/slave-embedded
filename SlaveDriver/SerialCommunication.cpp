#include "SerialCommunication.hpp"

SerialInterface::SerialInterface( HardwareSerial* serial_ptr, uint8_t pocket_size, uint8_t startbit = '>')
{
  set( serial_ptr, pocket_size, startbit );
};

uint8_t SerialInterface::getStartBit() const
{
  return _start_bit;
};

uint8_t SerialInterface::getPocketSize() const
{
  return _pocket_size;
};


uint8_t SerialInterface::getRxBuffer() const
{
  return _rx_buffer;
};

uint8_t SerialInterface::getTxBuffer() const
{
  return _tx_buffer;
};

void SerialInterface::Init(){};

uint8_t SerialInterface::available()
{
  return _serial_ptr->available();
}

uint8_t SerialInterface::read()
{
  _rx_buffer = _serial_ptr->read();
  return _rx_buffer;
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


void SerialInterface::set( HardwareSerial* serial_ptr,  uint8_t pocket_size, uint8_t startbit ) {
  _start_bit = startbit;
  _serial_ptr = serial_ptr;
  _pocket_size = pocket_size;
};
