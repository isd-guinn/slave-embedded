#include "isd-dev-pinout.hpp"
#include "WheelsControl.hpp"
#include "SandersFancyDebugger.hpp"

FancyDebugger debugger;

uint32_t task_blinking_stack = 2048;
xTaskHandle task_blinking_handle = NULL;
void task_blinking(void * pv)
{
  for( ; ; )
  {
    debugger.msg( "Status", "Blinking" );
    digitalWrite(LED_BUILTIN,HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN,LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
  }
}

void setup() 
{

  debugger.msg( "Init", "Blinking Task." );
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(LED_BUILTIN,LOW);
  debugger.msg( "Done", "Blinking Task." );

  xTaskCreatePinnedToCore( task_blinking, "Blinking", task_blinking_stack, NULL, 1, &task_blinking_handle, 1 );
  
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

void loop() {}

