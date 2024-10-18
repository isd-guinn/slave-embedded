#include "SerialCommunication.hpp"

SerialInterface::SerialInterface( HardwareSerial* serial_ptr, uint8_t rx_packet_size, uint8_t tx_packet_size, uint8_t startbit = '>')
{ 
  uint8_t* rx_buffer_ptr = new uint8_t[rx_packet_size];
  _rx_counter = 0;
  uint8_t* tx_buffer_ptr = new uint8_t[tx_packet_size];
  _tx_counter = 0;
  set( serial_ptr, rx_packet_size, tx_packet_size, startbit, rx_buffer_ptr, tx_buffer_ptr);
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
  for( int i = 0; i < _tx_packet_size; i++ ) _serial_ptr->write(_tx_buffer_ptr[i]);
}

void SerialInterface::txClear( bool keepStartBit )
{
  _tx_counter = (keepStartBit)?1:0 ;
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

// This boolean operation indicates if a command with a proper startbit is sent and shd be put in a loop
bool SerialInterface::onRecievedCommand()
{ 
  // If there is no byte in the hardware buffer or the current command is recieved but havnt been deal with i.e. the virtual buffer have not been reset, return false.
  if ( _serial_ptr->available() == 0 || _rx_counter >= _rx_packet_size)
  { 
    return false; 
  }
  
  // If a start byte is recieved, the virtual buffer is ready to recieved the 2nd byte, i.e. clear the virtual buffer except for the first byte. Return false.
  if ( rxPush() == _start_bit )
  {
    rxClear(true);
  }

  // If a byte except a start byte if recieved while it being the first byte in the packet, clear the whole virtual buffer and return false.
  else if ( _rx_counter <= 1 )
  {
    rxClear(false);
  }

  // Return True when a complete command is reached.
  else if ( _rx_counter >= _rx_packet_size )
  {
    // if(checkCheckSum( _rx_buffer_ptr, _rx_packet_size )) return true;
    // else rxClear(false);

    return true;
  }
  return false;
}



void SerialInterface::set( HardwareSerial* serial_ptr,  uint8_t rx_packet_size, uint8_t tx_packet_size, uint8_t startbit , uint8_t* rx_buffer_ptr, uint8_t* tx_buffer_ptr ){
  _start_bit = startbit;
  _serial_ptr = serial_ptr;
  _rx_packet_size = rx_packet_size;
  _tx_packet_size = tx_packet_size;
  _rx_buffer_ptr = rx_buffer_ptr;
  _tx_buffer_ptr = tx_buffer_ptr;
};
