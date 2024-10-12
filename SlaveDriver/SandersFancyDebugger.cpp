#include "SandersFancyDebugger.hpp"

FancyDebugger::FancyDebugger( HardwareSerial* serial_ptr )
{
  set(serial_ptr);
}

// void FancyDebugger::Init()
// {
//   _serial_ptr->begin(115200);
// }


void FancyDebugger::msg( String prefix, String content )
{
  _serial_ptr->print( prefix );
  _serial_ptr->print( "\t||\t" );
  _serial_ptr->print( content );
  _serial_ptr->print( '\n' );
}

void FancyDebugger::err( String content )
{
  _serial_ptr->print( "ERROR" );
  _serial_ptr->print( "\t||\t" );
  _serial_ptr->print( content );
  _serial_ptr->print( '\n' );

}

bool FancyDebugger::boolReccursiveInit( bool op, String op_name, long timeout)
{

  msg("Init", "Start Init:");
  msg("Init", op_name);

  timeout += micros();
  while ( micros() < timeout )
  {
    if( op ){ msg("Done", op_name); return true; }
    else { err("Failed Init: "); err(op_name); }
  }
  err("Init Timeout.");
  return false;

}

bool FancyDebugger::boolReccursiveInit( bool op, String init_content, String err_content, String done_content, long timeout)
{
  msg("Init", init_content);

  timeout += micros();
  while ( micros() < timeout )
  {
    if( op ){ msg("Done", done_content); return true; }
    else err(err_content);
  }
  err("Init Timeout.");
  return false;

}


void FancyDebugger::set( HardwareSerial* serial_ptr )
{
  _serial_ptr = serial_ptr;
}