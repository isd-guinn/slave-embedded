#include "SerialCommunication.hpp"

SerialInterface::SerialInterface()
{
  set(StartBit, &Serial, POCKET_SIZE);
};

SerialInterface::SerialInterface( uint8_t startbit, HardwareSerial* serial_ptr, uint8_t pocket_size )
{
  set(startbit, serial_ptr, pocket_size);
};

uint8_t SerialInterface::getStartBit() const
{
  return _start_bit;
};

uint8_t SerialInterface::getPocketSize() const
{
  return _pocket_size;
};


uint8_t SerialInterface::getRxBuffer( uint8_t bit_num ) const
{
  if ( bit_num >= getPocketSize() ) return 0xff;
  return _rx_buffer[bit_num];
};

uint8_t SerialInterface::getTxBuffer( uint8_t bit_num ) const
{
  if ( bit_num >= getPocketSize() ) return 0xff;
  return _tx_buffer[bit_num];
};

void SerialInterface::Init(){};


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

void SerialInterface::set( uint8_t startbit, HardwareSerial* serial_ptr, uint8_t pocket_size ){
  _start_bit = startbit;
  _serial_ptr = serial_ptr;
  _pocket_size = pocket_size;
};
