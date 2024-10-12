#ifndef SANDERSFANCYDEBUGGER_HPP
#define SANDERSFANCYDEBUGGER_HPP

#include <Arduino.h>

class FancyDebugger
{
  private:
    HardwareSerial* _serial_ptr;

  public:
    FancyDebugger( HardwareSerial* serial_ptr );

    void Init();
    void msg( String prefix, String content );
    void err( String content );
    bool boolReccursiveInit( bool op, String op_name, long timeout); 
    bool boolReccursiveInit( bool op, String init_content, String err_content, String done_content, long timeout); 

    void set( HardwareSerial* serial_ptr );

};

#endif