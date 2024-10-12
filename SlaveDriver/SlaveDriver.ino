//#include "isd-dev-pinout.hpp"
#include "WheelsControl.hpp"
#include "SerialCommunication.hpp"

#define LED_BUILTIN 48

HardwareSerial MasterSerial(0);
SerialInterface MasterSerialInterface(&MasterSerial, 5, '>');

/*
    Blinking
*/
uint32_t xBlinking_stack = 2048;
TaskHandle_t xBlinking_handle = NULL;
void xBlinking( void* pv )
{

  for( ; ; )
  {
    // digitalWrite(LED_BUILTIN,LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // digitalWrite(LED_BUILTIN,HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}

/*
    Echo
*/
uint32_t xEcho_stack = 2048;
TaskHandle_t xEcho_handle = NULL;
void xEcho( void* pv )
{ 
  int counter = 0;
  int data[5];

  for( ; ; )
  { 

    if ( MasterSerial.available() == 0 )
    { 
      vTaskDelay(50 / portTICK_PERIOD_MS); 
      continue; 
    }

    // If the comming bit is the Starting Bit, start input:
    if ( MasterSerialInterface.read() == MasterSerialInterface.getStartBit() )
    { 
      data[0] = MasterSerialInterface.getRxBuffer();
      counter = 1;
    }

    else if (counter > 0)
    { 
      data[counter] = MasterSerialInterface.getRxBuffer();
      counter++;

      if (counter >= MasterSerialInterface.getPocketSize())
      {
        for(int i=0;i<counter;i++)
        {
          MasterSerial.print(data[i]);
        }
        MasterSerial.print('\n');
        counter = 0;
      }
      
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }

}

void setup() 
{
  //Serial.begin(115200);
  MasterSerial.begin(115200, SERIAL_8N1, 44, 43);

  //Serial.println("INIT\t||\tBlinking");
  MasterSerial.write('c');
  MasterSerial.write('\n');

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  //Serial.println("DONE\t||\tBlinking");

  xTaskCreatePinnedToCore(  xEcho,
                            "Blinking",
                            xEcho_stack,
                            NULL,
                            1,
                            &xEcho_handle,
                            1 );

  xTaskCreatePinnedToCore(  xBlinking,
                            "Blinking",
                            xBlinking_stack,
                            NULL,
                            1,
                            &xBlinking_handle,
                            1 );

  vTaskDelay(500 / portTICK_PERIOD_MS);

}

void loop(){}

