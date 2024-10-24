#define SMALL_ENDIAN
// #define BIG_ENDIAN

/*
  ISD Dev board will be used
*/

#include "WheelsControl.hpp"
#include "SerialCommunication.hpp"
#include "MasterSerialProtocol.hpp"


#include "isd-dev-pinout.hpp" //isd Dev board

#define LEFTWHEELS_IN1    37
#define LEFTWHEELS_IN2    36
#define RIGHTWHEELS_IN1   13
#define RIGHTWHEELS_IN2   12
#define LEFTWHEELS_EN     38
#define RIGHTWHEELS_EN    14


// HardwareSerial MasterSerial(0);
HardwareSerial MasterSerial(2);
SerialInterface master(&MasterSerial, M2S_PACKET_SIZE, S2M_PACKET_SIZE, START_BIT, END_BIT);
Wheelbase wheelbase( 24.0, 24.0, LEFTWHEELS_IN1, RIGHTWHEELS_IN1, LEFTWHEELS_IN2, RIGHTWHEELS_IN2, LEFTWHEELS_EN, RIGHTWHEELS_EN);

/*////////////////////////////////////////////////////////////////
Global Variable/ Robot State Setup
////////////////////////////////////////////////////////////////*/
struct RobotState
{ 
  bool v_estop;
  control_mode_t control_mode;
  float speed_target;
  float speed_current;
  float angle_target;
  float angle_current;
  float angle_speed_target;
  float angular_speed_current;
  float vacuum_voltage;
  bool foc_engaged;
};

struct RobotState rs{ false, NULL_CONTROL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false };

uint8_t local_pkt[M2S_PACKET_SIZE];

int trig_count = 0;

void vomitRxBuffer()
{
      for(int i=0;i<master.getRxPacketSize();i++)
    {
      Serial.print( master.getRxBufferPtr()[i] , HEX);
      Serial.print(" ");
      vTaskDelay(1);
    }
    Serial.print("| Size: ");
    Serial.println( master.getRxCounter() );
    vTaskDelay(1);
}

/*////////////////////////////////////////////////////////////////
RTOS Tasks Setup
////////////////////////////////////////////////////////////////*/

/*    Blinking || Core 1   */
void xBlinking( void* pv );
uint32_t xBlinking_stack = 2048;
TaskHandle_t xBlinking_handle = NULL;

/*    Phrase Command || Core 0   */
void xPhraseCommand( void* pv );
uint32_t xPhraseCommand_stack = 10000;
TaskHandle_t xPhraseCommand_handle = NULL;


/*    Update State || Core 0   */
void xUpdateState( void* pv );
uint32_t xUpdateState_stack = 10000;
TaskHandle_t xUpdateState_handle = NULL;

/*    Echo Buffer || Core 1   */
void xEchoBuffer( void* pv );
uint32_t xEchoBuffer_stack = 2048;
TaskHandle_t xEchoBuffer_handle = NULL;

/*    Vacuum || Core 1   */
void xVacuum( void* pv );
uint32_t xVacuum_stack = 2048;
TaskHandle_t xVacuum_handle = NULL;

/*    Update Wheelbase || Core 1   */
void xUpdateWheelbase( void* pv );
uint32_t xUpdateWheelbase_stack = 10000;
TaskHandle_t xUpdateWheelbase_handle = NULL;

void xVomitState( void* pv );
uint32_t xVomitState_stack = 10000;
TaskHandle_t xVomitState_handle = NULL;


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
    Phrase Command Task
      Phrase the command packet to either update the state or do a certain task.
*/
void xPhraseCommand( void* pv )
{ 
  for( ; ; )
  { 
    if ( master.onRecievedCommand() )
    {
      for(int i=0;i<M2S_PACKET_SIZE;i++) 
      {
        local_pkt[i] = master.getRxBufferPtr()[i];
      }
      trig_count++;
      // Serial.println("Triggered!!!!!!!!!!!!!");
      // vomitRxBuffer();
      master.rxClear(false);
    }
    vTaskDelay( 50 / portTICK_PERIOD_MS );
  }

}

void xUpdateState( void* pv )
{
  for ( ; ; )
  {
      rs.v_estop              = (local_pkt[BYTE_POS_M2S_VESTOP] == V_ESTOP_EN_CODE)? true:false;
      rs.control_mode         = local_pkt[BYTE_POS_M2S_CONTROLMODE];
      rs.speed_target         = ByteUtil::reconFloat( local_pkt, BYTE_POS_M2S_TARGETSPEED );
      rs.speed_current        = ByteUtil::reconFloat( local_pkt, BYTE_POS_M2S_CURRENTSPEED );
      rs.angle_target         = ByteUtil::reconFloat( local_pkt, BYTE_POS_M2S_TARGETANGLE );
      rs.angle_current        = ByteUtil::reconFloat( local_pkt, BYTE_POS_M2S_CURRENTANGLE );
      rs.angle_speed_target   = ByteUtil::reconFloat( local_pkt, BYTE_POS_M2S_TARANGSPEED );
      rs.angular_speed_current= ByteUtil::reconFloat( local_pkt, BYTE_POS_M2S_CURANGSPEED );
      rs.vacuum_voltage       = ByteUtil::reconFloat( local_pkt, BYTE_POS_M2S_VACUUMVOLTAGE );
      rs.foc_engaged = (local_pkt[BYTE_POS_M2S_FOCMODE] == FOC_EN_CODE)? true:false;
      //Serial.printf("LEFT !!!!!: %f\tRIGHT !!!!!: %f\n", rs.speed_target, rs.angle_target);
      // MasterSerial.printf("Target Speed: %f\n", rs.speed_target);
      vTaskDelay( 100 / portTICK_PERIOD_MS );
  }
}

void xEchoBuffer( void* pv )
{
  for( ; ; )
  {
    vomitRxBuffer();
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
    // Serial.printf("Control Mode: %x\n", rs.control_mode);

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

      case MANUAL_CONTROL:
        // Serial.printf("Left Wheel: %f\tRight Wheel: %f\n", rs.speed_target, rs.angle_target);
        if(rs.speed_target > 0) wheelbase.wheelClockwise(0, rs.speed_target);
        else wheelbase.wheelAntiClockwise(0, -rs.speed_target);
        if(rs.angle_target > 0) wheelbase.wheelAntiClockwise(1, rs.angle_target);
        else wheelbase.wheelClockwise(1, -rs.angle_target);
        break;

      default:
        break;
    }
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void xVomitState( void* pv )
{ 

  for( ; ; )
  {
    Serial.println("----------------------------------------");
    Serial.printf("v_estop: %x\n",                rs.v_estop);
    Serial.printf("control_mode: %x\n",           rs.control_mode);
    Serial.printf("speed_target: %f\n",           rs.speed_target);
    Serial.printf("speed_current: %f\n",          rs.speed_current);
    Serial.printf("angle_target: %f\n",          rs.angle_target);
    Serial.printf("angle_current: %f\n",          rs.angle_current);
    Serial.printf("angle_speed_target: %f\n",     rs.angle_speed_target);
    Serial.printf("angular_speed_current: %f\n",  rs.angular_speed_current);
    Serial.printf("vacuum_voltage: %f\n",         rs.vacuum_voltage);
    Serial.printf("foc_engaged: %x\n",            rs.foc_engaged);
    Serial.println("----------------------------------------");
    
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}


/*////////////////////////////////////////////////////////////////
Main Program
////////////////////////////////////////////////////////////////*/

void setup() 
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  MasterSerial.setRxBufferSize(132);
  Serial.begin(115200);
  MasterSerial.begin(115200, SERIAL_8N1, 42, 41); //ISD Dev board Serial2
  // MasterSerial.begin(115200, SERIAL_8N1, 44, 43); //ESP32 Dev kit
  // MasterSerial.begin(115200, SERIAL_8N1, 3, 1); //ESP32S

  Serial.println("INIT\t||\START SETUP");

  // Initialise wheelbase
  if(!wheelbase.init(false))
  { 
    while(true)
    {
      Serial.println("ERROR\t||\tINIT FAILED");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
  // !!!!!!!! Stuck at Here if failed

  rs.control_mode = MANUAL_CONTROL;
  rs.speed_target = 12.0f;
  rs.angle_target = 12.0f;

  // xTaskCreatePinnedToCore(  xEchoBuffer,
  //                           "Blinking",
  //                           xEchoBuffer_stack,
  //                           NULL,
  //                           1,
  //                           &xEchoBuffer_handle,
  //                           1 );

  xTaskCreatePinnedToCore(  xPhraseCommand,
                            "Phrase Command",
                            xPhraseCommand_stack,
                            NULL,
                            1,
                            &xPhraseCommand_handle,
                            0 );

  xTaskCreatePinnedToCore(  xUpdateState,
                            "Update State",
                            xUpdateState_stack,
                            NULL,
                            2,
                            &xUpdateState_handle,
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

  xTaskCreatePinnedToCore(  xUpdateWheelbase,
                            "UpdateWheelbase",
                            xUpdateWheelbase_stack,
                            NULL,
                            1,
                            &xUpdateWheelbase_handle,
                            1 );

  xTaskCreatePinnedToCore(  xVomitState,
                            "Vomit State",
                            xVomitState_stack,
                            NULL,
                            1,
                            &xVomitState_handle,
                            1 );


  vTaskDelay(500 / portTICK_PERIOD_MS);


}

void loop(){}

