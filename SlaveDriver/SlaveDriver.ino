/*
  ISD Dev board will be used
*/

#include "WheelsControl.hpp"
#include "SerialCommunication.hpp"
#include "MasterSerialProtocol.hpp"


#include "isd-dev-pinout.hpp" //isd Dev board
#define LED_BUILTIN 48 //ESP32S3 Dev Kit
// #define LED_BUILTIN 2 //Remote


#define LEFTWHEELS_IN1    35
#define LEFTWHEELS_IN2    36
#define RIGHTWHEELS_IN1   37
#define RIGHTWHEELS_IN2   38
#define LEFTWHEELS_EN     47
#define RIGHTWHEELS_EN    48


HardwareSerial MasterSerial(0);
SerialInterface master(&MasterSerial, 6, 6, '>');
Wheelbase wheelbase( 24.0, 24.0, LEFTWHEELS_IN1, RIGHTWHEELS_IN1, LEFTWHEELS_IN2, RIGHTWHEELS_IN2, LEFTWHEELS_EN, RIGHTWHEELS_EN);

// Store all the required states for updates the modules
struct RobotState
{
  control_mode_t control_mode;
  float speed_current;
  float speed_target;
  float angle_current;
  float angle_target;

  float v_pump;
};

struct RobotState rs{ NULL_CONTROL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

/*////////////////////////////////////////////////////////////////
RTOS Tasks Setup
////////////////////////////////////////////////////////////////*/

uint32_t xBlinking_stack = 2048;
TaskHandle_t xBlinking_handle = NULL;
void xBlinking( void* pv );

uint32_t xPhraseCommand_stack = 10000;
TaskHandle_t xPhraseCommand_handle = NULL;
void xPhraseCommand( void* pv );

uint32_t xEchoBuffer_stack = 2048;
TaskHandle_t xEchoBuffer_handle = NULL;
void xEchoBuffer( void* pv );

uint32_t xVacuum_stack = 2048;
TaskHandle_t xVacuum_handle = NULL;
void xVacuum( void* pv );

uint32_t xUpdateWheelbase_stack = 10000;
TaskHandle_t xUpdateWheelbase_handle = NULL;
void xUpdateWheelbase( void* pv );


/*////////////////////////////////////////////////////////////////
RTOS Tasks Definition
////////////////////////////////////////////////////////////////*/

/*
    Blinking
*/
void xBlinking( void* pv )
{

  // Serial.println("INIT\t||\tBlinking");

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  // Serial.println("DONE\t||\tBlinking");

  for( ; ; )
  {
    // Serial.println("DEBUG\t||\tBlinking");
    digitalWrite(LED_BUILTIN,LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN,HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}
/*
    Echo
*/
void xPhraseCommand( void* pv )
{ 

  for( ; ; )
  { 
    
    if (master.onRecievedCommand())
    {
      rs.speed_target = REINTERPRET_AS_FLOAT(master.getRxBufferPtr(),1);
      MasterSerial.printf("Target Speed: %f\n", rs.speed_target);
      master.rxClear(false);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }

}

void xEchoBuffer( void* pv )
{
  for( ; ; )
  {
    for(int i=0;i<master.getRxPocketSize();i++)
    {
      MasterSerial.print( master.getRxBufferPtr()[i] );
      MasterSerial.print(" ");
    }
    MasterSerial.print("| Size: ");
    MasterSerial.println( master.getRxCounter() );
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void xVacuum( void* pv )
{ 

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

  for( ; ; )
  { 
    Serial.println("Set speed");
    ledcWrite(SERVO0_PIN,329);
    ledcWrite(SERVO1_PIN,329);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void xUpdateWheelbase( void* pv )
{ 
  for( ; ; )
  { 
    
    switch (rs.control_mode)
    {
      case NULL_CONTROL:
        break;
      
      case SPEED_CONTROL:
        if (rs.speed_target > 0) wheelbase.foward(rs.speed_target);
        else if (rs.speed_target < 0) wheelbase.backward(rs.speed_target);
        else wheelbase.stop();
        break;
      
      case ANGLE_CONTROL:
        break;

      default:
        break;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
}


/*////////////////////////////////////////////////////////////////
Main Program
////////////////////////////////////////////////////////////////*/

void setup() 
{
  // Serial.begin(115200);
  MasterSerial.begin(115200, SERIAL_8N1, 44, 43); //ESP32 Dev kit
  // MasterSerial.begin(115200, SERIAL_8N1, 3, 1); //ESP32S


  // Initialise wheelbase
  if(!wheelbase.init(false))
  { 
    while(true)
    {
      // Serial.println("ERROR\t||\tINIT FAILED");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
  // !!!!!!!! Stuck at Here if failed

  rs.control_mode = SPEED_CONTROL;
  rs.speed_target = 12.0f;

  xTaskCreatePinnedToCore(  xEchoBuffer,
                            "Blinking",
                            xEchoBuffer_stack,
                            NULL,
                            1,
                            &xEchoBuffer_handle,
                            1 );

  xTaskCreatePinnedToCore(  xPhraseCommand,
                            "Phrase Command",
                            xPhraseCommand_stack,
                            NULL,
                            1,
                            &xPhraseCommand_handle,
                            1 );

  xTaskCreatePinnedToCore(  xBlinking,
                            "Blinking",
                            xBlinking_stack,
                            NULL,
                            1,
                            &xBlinking_handle,
                            1 );

  // xTaskCreatePinnedToCore(  xVacuum,
  //                           "Vacuum",
  //                           xVacuum_stack,
  //                           NULL,
  //                           1,
  //                           &xVacuum_handle,
  //                           1 );

  // xTaskCreatePinnedToCore(  xUpdateWheelbase,
  //                           "UpdateWheelbase",
  //                           xUpdateWheelbase_stack,
  //                           NULL,
  //                           1,
  //                           &xUpdateWheelbase_handle,
  //                           1 );

  vTaskDelay(500 / portTICK_PERIOD_MS);


}

void loop(){}

