#include "isd-dev-pinout.hpp"
#include "WheelsControl.hpp"
#include "SerialCommunication.hpp"

//#define LED_BUILTIN 48
// #define LED_BUILTIN 2

HardwareSerial MasterSerial(0);
SerialInterface master(&MasterSerial, 5, '>');


uint32_t xBlinking_stack = 2048;
TaskHandle_t xBlinking_handle = NULL;
void xBlinking( void* pv );

uint32_t xWriteBuffer_stack = 2048;
TaskHandle_t xWriteBuffer_handle = NULL;
void xWriteBuffer( void* pv );

uint32_t xEchoBuffer_stack = 2048;
TaskHandle_t xEchoBuffer_handle = NULL;
void xEchoBuffer( void* pv );

uint32_t xVacuum_stack = 2048;
TaskHandle_t xVacuum_handle = NULL;
void xVacuum( void* pv );

/*
    Blinking
*/
void xBlinking( void* pv )
{

  for( ; ; )
  {
    digitalWrite(LED_BUILTIN,LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN,HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}

/*
    Echo
*/
void xWriteBuffer( void* pv )
{ 
  int counter = 0;
  int data[5];

  for( ; ; )
  { 
    
    if (master.onRecievedCommand())
    {
      vTaskSuspend(xEchoBuffer_handle);
      MasterSerial.print( "Command Check Sum: " );
      MasterSerial.print( master.getCheckSum( master.getRxBufferPtr(), master.getPocketSize() ) );
      MasterSerial.println( " Enter the Correct version of the Command. " );

      master.rxClear(false);

      while ( !master.onRecievedCommand() )
      { 
        vTaskResume(xEchoBuffer_handle);
        vTaskDelay(50 / portTICK_PERIOD_MS);
      }

      vTaskSuspend(xEchoBuffer_handle);

      if( master.checkCheckSum( master.getRxBufferPtr(), master.getPocketSize() ) )
      {
        MasterSerial.println( "Correct Command." );
      }
      else MasterSerial.println( "False Command." );

      vTaskResume(xEchoBuffer_handle);

      master.rxClear(false);
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }

}

void xEchoBuffer( void* pv )
{
  for( ; ; )
  {
    for(int i=0;i<master.getPocketSize();i++)
    {
      MasterSerial.print( master.getRxBufferPtr()[i] );
      MasterSerial.print(" ");
    }
    MasterSerial.println("| Not full: ");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void xVacuum( void* pv )
{
  for( ; ; )
  { 
    Serial.println("Set speed");
    ledcWrite(SERVO0_PIN,329);
    ledcWrite(SERVO1_PIN,329);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void setup() 
{
  Serial.begin(115200);
  //MasterSerial.begin(115200, SERIAL_8N1, 44, 43); //ESP32 Dev kit
  // MasterSerial.begin(115200, SERIAL_8N1, 3, 1); //ESP32S



  //Vacuum INIT Begin
  while(ledcAttach(SERVO0_PIN, 50, 12)==false){
    Serial.println("Attaching...");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  while(ledcAttach(SERVO1_PIN, 50, 12)==false){
    Serial.println("Attaching...");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  Serial.println("Init stage 2");
  ledcWrite(SERVO0_PIN,410);
  ledcWrite(SERVO1_PIN,410);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  Serial.println("Init stage 3");
  ledcWrite(SERVO0_PIN,308);
  ledcWrite(SERVO1_PIN,308);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  //Vacuum INIT End



  //Serial.println("INIT\t||\tBlinking");

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  //Serial.println("DONE\t||\tBlinking");

  // xTaskCreatePinnedToCore(  xEchoBuffer,
  //                           "Blinking",
  //                           xEchoBuffer_stack,
  //                           NULL,
  //                           1,
  //                           &xEchoBuffer_handle,
  //                           1 );

  // xTaskCreatePinnedToCore(  xWriteBuffer,
  //                           "Blinking",
  //                           xWriteBuffer_stack,
  //                           NULL,
  //                           1,
  //                           &xWriteBuffer_handle,
  //                           1 );

  xTaskCreatePinnedToCore(  xBlinking,
                            "Blinking",
                            xBlinking_stack,
                            NULL,
                            1,
                            &xBlinking_handle,
                            1 );

  xTaskCreatePinnedToCore(  xVacuum,
                            "Vacuum",
                            xVacuum_stack,
                            NULL,
                            1,
                            &xVacuum_handle,
                            1 );

  vTaskDelay(500 / portTICK_PERIOD_MS);


}

void loop(){}

