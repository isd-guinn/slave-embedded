#define SMALL_ENDIAN
// #define BIG_ENDIAN

/*
  ISD Dev board will be used
*/

#include <Adafruit_MCP2515.h>

#include "WheelsControl.hpp"
#include "MasterCanProtocol.hpp"
#include "isd-dev-pinout.hpp" //isd Dev board

#include "REG.h"
#include "wit_c_sdk.h"

#define LEFTWHEELS_IN1    36
#define LEFTWHEELS_IN2    35
#define RIGHTWHEELS_IN1   38
#define RIGHTWHEELS_IN2   37
#define LEFTWHEELS_EN     11
#define RIGHTWHEELS_EN    12

#define VACUUM_PIN  40
#define VACUUM_FREQ   50
#define VACUUM_RES    12

#define IMU_RX        42
#define IMU_TX        41
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

#define CS_PIN      6
#define MOSI_PIN    15
#define MISO_PIN    16
#define SCK_PIN     7
#define QUARTZ_FREQUENCY 8000000UL
#define CAN_BAUDRATE 100000
#define CAN_DATA_LENGTH 8

/*////////////////////////////////////////////////////////////////
    Global Variable/ Robot State Setup
////////////////////////////////////////////////////////////////*/
const float hw_v_supply = 24.0f;
const float hw_v_limit  = 22.0f;

const int vacuum_duty = 329;

Wheelbase wheelbase( hw_v_supply, hw_v_supply, LEFTWHEELS_IN1, RIGHTWHEELS_IN1, LEFTWHEELS_IN2, RIGHTWHEELS_IN2, LEFTWHEELS_EN, RIGHTWHEELS_EN);

Adafruit_MCP2515 mcp(CS_PIN,MOSI_PIN,MISO_PIN,SCK_PIN);


struct RobotState{

  bool v_estop;
  float speed_target;
  float speed_current;
  float angle_target;
  float angle_current;
  float angle_speed_target;
  float angular_speed_current;
  float vacuum_voltage;

  float acc[3];
  float gyro[3];
  float angle[3];
};

struct RobotState rs;

struct CANMsg{
  int id;
  uint8_t data[CAN_DATA_LENGTH];
};

struct CANMsg buf;
QueueHandle_t xCANqueue;


HardwareSerial ImuSerial(2);
static volatile char s_cDataUpdate = 0;

/*////////////////////////////////////////////////////////////////
    Helper Function
////////////////////////////////////////////////////////////////*/

namespace ByteUtil{
  inline float reconFloat(uint8_t *packet, uint8_t pos, bool isSmallEndian = true){
    if (isSmallEndian){
      uint8_t ctn[4] = {packet[pos+3], packet[pos+2], packet[pos+1], packet[pos]};
      return *(float*)&ctn;
    }
    else{
      uint8_t ctn[4] = {packet[pos], packet[pos+1], packet[pos+2], packet[pos+3]};
      return *(float*)&ctn;
    }              
  }
}

// namespace IMU{
  static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
  {
    ImuSerial.write(p_data, uiSize);
    ImuSerial.flush();
  }
  static void Delayms(uint16_t ucMs)
  {
    vTaskDelay(ucMs / portTICK_PERIOD_MS);
  }
  static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
  {
    int i;
      for(i = 0; i < uiRegNum; i++)
      {
        switch(uiReg)
        {
          case AZ:
            s_cDataUpdate |= ACC_UPDATE;
            break;
          case GZ:
            s_cDataUpdate |= GYRO_UPDATE;
            break;
          case HZ:
            s_cDataUpdate |= MAG_UPDATE;
            break;
          case Yaw:
            s_cDataUpdate |= ANGLE_UPDATE;
            break;
          default:
            s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
      }
  }
// }

/*////////////////////////////////////////////////////////////////
    RTOS Tasks Setup
////////////////////////////////////////////////////////////////*/

/*    Vacuum || Core 1   */
void xVacuum( void* pv );
uint32_t xVacuum_stack = 2048;
TaskHandle_t xVacuum_handle = NULL;

/*    Update Wheelbase || Core 1   */
void xUpdateWheelbase( void* pv );
uint32_t xUpdateWheelbase_stack = 10000;
TaskHandle_t xUpdateWheelbase_handle = NULL;

/*    Blinking || Core 1   */
void xBlinking( void* pv );
uint32_t xBlinking_stack = 800;
TaskHandle_t xBlinking_handle = NULL;

/*    VomitState || Core 1   */
void xVomitState( void* pv );
uint32_t xVomitState_stack = 10000;
TaskHandle_t xVomitState_handle = NULL;

/*    MasterCanProcess || Core 0   */
void xCanProcess( void* pv );
uint32_t xCanProcess_stack = 10000;
TaskHandle_t xCanProcess_handle = NULL;

/*    MasterCanSend || Core 0   */
void xCanSend( void* pv );
uint32_t xCanSend_stack = 10000;
TaskHandle_t xCanSend_handle = NULL;

/*    xControlPanel || Core 1   */
void xControlPanel( void* pv );
uint32_t xControlPanel_stack = 10000;
TaskHandle_t xControlPanel_handle = NULL;

/*    xImuProcess || Core 0   */
void xImuProcess( void* pv );
uint32_t xImuProcess_stack = 35000;
TaskHandle_t xImuProcess_handle = NULL;

/*    Stack Monitor || Core 1   */
void xStackMonitor( void* pv );

/*////////////////////////////////////////////////////////////////
RTOS Tasks Definition
////////////////////////////////////////////////////////////////*/

void xVacuum( void* pv ){ 

  for( ; ; ){ 
    // Serial.println("Set speed");
    ledcWrite(VACUUM_PIN,vacuum_duty);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}

void xUpdateWheelbase( void* pv ){ 

  for( ; ; ){ 


    if(rs.speed_target<1.0f && rs.speed_target>-1.0f) \
      wheelbase.wheelStop(0);
    if(rs.angle_target<1.0f && rs.angle_target>-1.0f) \
      wheelbase.wheelStop(1);
    if(rs.speed_target > 0) \ 
      wheelbase.wheelClockwise(0, rs.speed_target);
    else \ 
      wheelbase.wheelAntiClockwise(0, -rs.speed_target);
    if(rs.angle_target > 0) \ 
      wheelbase.wheelAntiClockwise(1, rs.angle_target);
    else \ 
      wheelbase.wheelClockwise(1, -rs.angle_target);

    
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}

/*  Blinking  */
void xBlinking( void* pv ){

  for( ; ; ){
    // Serial.println("DEBUG\t||\tBlinking");
    digitalWrite(LED_BUILTIN,LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN,HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  
}

void xVomitState( void* pv ){

  for( ; ; )
  {
    Serial.println("----------------------------------------");
    // Serial.printf("v_estop: %x\n",                rs.v_estop);
    // Serial.printf("speed_target: %f\n",           rs.speed_target);
    // Serial.printf("speed_current: %f\n",          rs.speed_current);
    // Serial.printf("angle_target: %f\n",           rs.angle_target);
    // Serial.printf("angle_current: %f\n",          rs.angle_current);
    // Serial.printf("angle_speed_target: %f\n",     rs.angle_speed_target);
    // Serial.printf("angular_speed_current: %f\n",  rs.angular_speed_current);
    // Serial.printf("vacuum_voltage: %f\n",         rs.vacuum_voltage);

    Serial.printf("acc x:%f\t\ty:%f\t\tz:%f\n",         rs.acc[0],rs.acc[1],rs.acc[2]);
    Serial.printf("gyro x:%f\t\ty:%f\t\tz:%f\n",        rs.gyro[0],rs.gyro[1],rs.gyro[2]);
    Serial.printf("angle x:%f\t\ty:%f\t\tz:%f\n",        rs.angle[0],rs.angle[1],rs.angle[2]);


    Serial.println("----------------------------------------");


    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void xCanProcess( void* pv ){

  struct CANMsg* pbuf;
  xCANqueue = xQueueCreate( 5, sizeof(struct CANMsg*));
  if( xCANqueue == 0 ){
     // Failed to create the queue.
      Serial.println("wtf brooop");
  }

  for( ; ; ){

    // Try to Parse Packet
    int size = mcp.parsePacket();

    // Packet Recieved
    if(size){

      // Get ID
      buf.id = mcp.packetId();

      // Get Date. Drain the data if overflow;
      for(int i=0; mcp.available(); i++) {
        if (i<8){
          buf.data[i] = mcp.read();
        }
        else mcp.read();
      }

      pbuf=&buf;
      xQueueSend( xCANqueue, (void *) &pbuf, ( TickType_t ) 0 );
    }

    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

void xCanSend( void* pv ){
  for( ; ; ){

    struct CANMsg msg1 = { ID_IMU_ACC_XY, {0,0,0,0,    0,0,0,0} };
    struct CANMsg msg2 = { ID_IMU_QUAT_ZW, {1,1,1,1, 1,1,1,1} };

    mcp.beginPacket(msg1.id);
    for(int i=0;i<CAN_DATA_LENGTH;i++){
      mcp.write(msg1.data[i]);
      }
    mcp.endPacket();
    // Serial.printf("Sucess? %d\n",mcp.endPacket());


    vTaskDelay(500 / portTICK_PERIOD_MS);

    mcp.beginPacket(msg2.id);
    for(int i=0;i<CAN_DATA_LENGTH;i++){
      mcp.write(msg2.data[i]);
      }
    mcp.endPacket();
    // Serial.printf("Sucess? %d\n",mcp.endPacket());

    vTaskDelay(500 / portTICK_PERIOD_MS);
    
  }
}

void xControlPanel( void* pv ){

  struct CANMsg* pkt;

  for ( ; ; ){


    // Receive a message on the created queue.  Block for 10 ticks if a
    // message is not immediately available.
    if( xQueueReceive( xCANqueue, &(pkt), ( TickType_t ) 10) ){
        // pcRxedMessage now points to the struct AMessage variable posted
        // by vATask.

      Serial.printf("Packet ID 0x%x: ",pkt->id);
      for(int i=0;i<CAN_DATA_LENGTH;i++){
        Serial.printf("%x ",pkt->data[i]);
      }
      Serial.printf("To Float: %f, %f",ByteUtil::reconFloat(pkt->data,0),ByteUtil::reconFloat(pkt->data,4));
      Serial.println();

    }
      

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void xImuProcess( void* pv ){
  int i;
  for( ; ; ){

    if (ImuSerial.available()){ 
      WitSerialDataIn(ImuSerial.read());
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
		else if(s_cDataUpdate){
			for(i = 0; i < 3; i++){
				rs.acc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				rs.gyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				rs.angle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			if(s_cDataUpdate & ACC_UPDATE){
				// Serial.print("acc:");
				// Serial.print(rs.acc[0], 3);
				// Serial.print(" ");
				// Serial.print(rs.acc[1], 3);
				// Serial.print(" ");
				// Serial.print(rs.acc[2], 3);
				// Serial.print("\r\n");
				s_cDataUpdate &= ~ACC_UPDATE;
			}
			if(s_cDataUpdate & GYRO_UPDATE){
				// Serial.print("gyro:");
				// Serial.print(fGyro[0], 1);
				// Serial.print(" ");
				// Serial.print(fGyro[1], 1);
				// Serial.print(" ");
				// Serial.print(fGyro[2], 1);
				// Serial.print("\r\n");
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE){
				// Serial.print("angle:");
				// Serial.print(fAngle[0], 3);
				// Serial.print(" ");
				// Serial.print(fAngle[1], 3);
				// Serial.print(" ");
				// Serial.print(fAngle[2], 3);
				// Serial.print("\r\n");
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
			if(s_cDataUpdate & MAG_UPDATE){
				// Serial.print("mag:");
				// Serial.print(sReg[HX]);
				// Serial.print(" ");
				// Serial.print(sReg[HY]);
				// Serial.print(" ");
				// Serial.print(sReg[HZ]);
				// Serial.print("\r\n");
				s_cDataUpdate &= ~MAG_UPDATE;
			}
      s_cDataUpdate = 0;
      vTaskDelay(10 / portTICK_PERIOD_MS);
		}

    
  }
}

/*////////////////////////////////////////////////////////////////
    Inits
////////////////////////////////////////////////////////////////*/

bool led_init(){
  // Serial.println("INIT\t||\tBlinking");
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  // Serial.println("DONE\t||\tBlinking");
  return true;
}

bool vacuum_init(){
  //Vacuum INIT Begin

  ledcAttach(VACUUM_PIN, 50, 12);
  ledcWrite(VACUUM_PIN,410);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  ledcWrite(VACUUM_PIN,308);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  return true;
  //Vacuum INIT End
}

/*////////////////////////////////////////////////////////////////
    Main Program
////////////////////////////////////////////////////////////////*/

void setup() 
{
  Serial.begin(115200);
  Serial.println("Joyful Aqua Cleanr Slave -- BEGIN...");

  // // CAN Init
  // Serial.println("Init");
  // mcp.setClockFrequency(QUARTZ_FREQUENCY);
  // while (!mcp.begin(CAN_BAUDRATE)) {
  //   Serial.println("Error initializing MCP2515.");
  //   vTaskDelay(100 / portTICK_PERIOD_MS);
  // }
  // Serial.println("MCP2515 chip found");

  // IMU Init
  ImuSerial.begin(9600, SERIAL_8N1, IMU_RX, IMU_TX); //ESP32S3
  WitSetUartBaud(WIT_BAUD_9600);
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);

  // Serial.println("INIT\t||\START SETUP");
  // wheelbase.init(false);
  // wheelbase.stop();

  // vacuum_init();
  led_init();

  // !!!!!!!! Stuck at Here if failed
  Serial.println("Joyful Aqua Cleanr Slave -- READY...");

  // xTaskCreatePinnedToCore(  xVacuum,    "Vacuum",   xVacuum_stack, NULL,1,&xVacuum_handle,1 );
  // xTaskCreatePinnedToCore(  xUpdateWheelbase, "UpdateWheelbase",  xUpdateWheelbase_stack, NULL, 1 , &xUpdateWheelbase_handle, 1 );
  xTaskCreatePinnedToCore( xBlinking, "Blinking", xBlinking_stack,  NULL, 1,  &xBlinking_handle, 1 );
  xTaskCreatePinnedToCore( xVomitState, "Vomit State",  xVomitState_stack, NULL, 1,  &xVomitState_handle, 1 );
  // xTaskCreatePinnedToCore( xCanProcess, "Master CAN Process",  xCanProcess_stack, NULL, 1,  &xCanProcess_handle, 0 );
  // xTaskCreatePinnedToCore( xCanSend, "Master CAN Send",  xCanSend_stack, NULL, 1,  &xCanSend_handle, 0 );
  // xTaskCreatePinnedToCore( xControlPanel, "Control Panel",  xControlPanel_stack, NULL, 1,  &xControlPanel_handle, 1 );
  xTaskCreatePinnedToCore( xImuProcess, "IMU Process",  xImuProcess_stack, NULL, 1,  &xImuProcess_handle, 0 );

  // xTaskCreatePinnedToCore( xStackMonitor, "Stack Monitor",  10000, NULL, 1,  &xImuProcess_handle, NULL );

  vTaskDelay(500 / portTICK_PERIOD_MS);

  rs.speed_target = 0.00f;
  rs.angle_target = 0.00f;

}

void loop(){}

void xStackMonitor( void* pv ){
  for( ; ; ){
    Serial.println(uxTaskGetStackHighWaterMark(xBlinking_handle));
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

