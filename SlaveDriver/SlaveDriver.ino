#define SMALL_ENDIAN
// #define BIG_ENDIAN

/*
  ISD Dev board will be used
*/

#include <SimpleFOC.h>

#include "WheelsControl.hpp"
#include "SerialCommunication.hpp"
#include "MasterSerialProtocol.hpp"
#include "isd-dev-pinout.hpp" //isd Dev board

#define I2C1_SDA    4
#define I2C1_SCK    15   
#define I2C2_SDA    19
#define I2C2_SCK    18 

#define LEFTWHEELS_IN1    36
#define LEFTWHEELS_IN2    35
#define RIGHTWHEELS_IN1   38
#define RIGHTWHEELS_IN2   37
#define LEFTWHEELS_EN     11
#define RIGHTWHEELS_EN    12

#define BLDC1_IN1   13
#define BLDC1_IN2   12
#define BLDC1_IN3   14
#define BLDC2_IN1   33
#define BLDC2_IN2   32
#define BLDC2_IN3   35

#define VACUUM_PIN  40

#define VACUUM_FREQ   50
#define VACUUM_RES    12

/*////////////////////////////////////////////////////////////////
    Global Variable/ Robot State Setup
////////////////////////////////////////////////////////////////*/
const float hw_v_supply = 24.0f;
const float hw_v_limit  = 22.0f;

const int vacuum_duty = 329;

TwoWire i2c1(1);
TwoWire i2c2(2);

BLDCMotor foc_motor_l = BLDCMotor(7,5.1f,220);
BLDCMotor foc_motor_r = BLDCMotor(7,5.1f,220);

BLDCDriver3PWM foc_driver_l = BLDCDriver3PWM(BLDC1_IN1, BLDC1_IN2, BLDC1_IN3);
BLDCDriver3PWM foc_driver_r = BLDCDriver3PWM(BLDC2_IN1, BLDC2_IN2, BLDC2_IN3);

MagneticSensorI2C foc_sensor_l = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C foc_sensor_r = MagneticSensorI2C(AS5600_I2C);


HardwareSerial MasterSerial(2);
SerialInterface master(&MasterSerial, M2S_PACKET_SIZE, S2M_PACKET_SIZE, START_BIT, END_BIT);
Wheelbase wheelbase( hw_v_supply, hw_v_supply, LEFTWHEELS_IN1, RIGHTWHEELS_IN1, LEFTWHEELS_IN2, RIGHTWHEELS_IN2, LEFTWHEELS_EN, RIGHTWHEELS_EN);


struct RobotState{

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
  action_t direction;

  float probe_angle_left;
  float probe_tar_angle_left;
  float probe_tar_angle_left_origin;
  float probe_angle_right;
  float probe_tar_angle_right;
  float probe_tar_angle_right_origin;

  };

struct RobotState rs{

  false, NULL_CONTROL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, 

  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f 

  };

uint8_t local_pkt_m2s[M2S_PACKET_SIZE];
uint8_t local_pkt_s2m[S2M_PACKET_SIZE];

/*////////////////////////////////////////////////////////////////
    Helper Function
////////////////////////////////////////////////////////////////*/

void vomitRxBuffer(){
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

/*    Phrase Command || Core 0   */
void xPhraseCommand( void* pv );
uint32_t xPhraseCommand_stack = 10000;
TaskHandle_t xPhraseCommand_handle = NULL;

/*    Send Command || Core 0   */
void xSendCommand( void* pv );
uint32_t xSendCommand_stack = 10000;
TaskHandle_t xSendCommand_handle = NULL;

/*    Update State || Core 0   */
void xUpdateState( void* pv );
uint32_t xUpdateState_stack = 10000;
TaskHandle_t xUpdateState_handle = NULL;

/*    Vacuum || Core 1   */
void xVacuum( void* pv );
uint32_t xVacuum_stack = 2048;
TaskHandle_t xVacuum_handle = NULL;

/*    Update Wheelbase || Core 1   */
void xUpdateWheelbase( void* pv );
uint32_t xUpdateWheelbase_stack = 10000;
TaskHandle_t xUpdateWheelbase_handle = NULL;

/*    FOC routine || Core 0   */
void xFOCroutine( void* pv );
uint32_t xFOCroutine_stack = 10000;
TaskHandle_t xFOCroutine_handle = NULL;

/*    Blinking || Core 1   */
void xBlinking( void* pv );
uint32_t xBlinking_stack = 2048;
TaskHandle_t xBlinking_handle = NULL;

/*    Echo Buffer || Core 1   */
void xEchoBuffer( void* pv );
uint32_t xEchoBuffer_stack = 2048;
TaskHandle_t xEchoBuffer_handle = NULL;

/*    VomitState || Core 1   */
void xVomitState( void* pv );
uint32_t xVomitState_stack = 10000;
TaskHandle_t xVomitState_handle = NULL;


/*////////////////////////////////////////////////////////////////
RTOS Tasks Definition
////////////////////////////////////////////////////////////////*/

/*  Phrase the command packet to either update the state or do a certain task. */
void xPhraseCommand( void* pv ){

  MasterSerial.flush();

  for( ; ; ){


    if ( master.onRecievedCommand() ){
      for(int i=0;i<M2S_PACKET_SIZE;i++) {
        local_pkt_m2s[i] = master.getRxBufferPtr()[i];
      }

      Serial.println("Triggered!!!!!!!!!!!!!");
      // vomitRxBuffer();

      master.rxClear(false);
    }
    vTaskDelay( 2 / portTICK_PERIOD_MS );

  }

}

void xSendCommand( void* pv ){

  for( ; ; ){

    master.txPush(START_BIT);           // 00  |   StartBit
    master.txPush(DEBUG_);              // 01  |   DebugCode

    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.probe_angle_left,0) ); // 02  |   LeftFOCAngle   (1st Byte)
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.probe_angle_left,1) );
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.probe_angle_left,2) );
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.probe_angle_left,3) );

    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.probe_angle_right,0) );   // 06  |   RightFOCAngle   (1st Byte)
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.probe_angle_right,1) );
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.probe_angle_right,2) );
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.probe_angle_right,3) );
    
    master.txPush(master.getCheckSum(master.getTxBufferPtr(),master.getTxPacketSize())); // 10  |   CheckSum
    master.txPush(END_BIT); // 11  |   endbit

    master.txSend();
    master.txClear(false);

    vTaskDelay( 70 / portTICK_PERIOD_MS );
  }

}

void xUpdateState( void* pv ){

  for ( ; ; ){
      rs.v_estop              = (local_pkt_m2s[BYTE_POS_M2S_VESTOP] == V_ESTOP_EN_CODE)? true:false;
      rs.control_mode         = local_pkt_m2s[BYTE_POS_M2S_CONTROLMODE];
      rs.speed_target         = ByteUtil::reconFloat( local_pkt_m2s, BYTE_POS_M2S_TARGETSPEED );
      rs.speed_current        = ByteUtil::reconFloat( local_pkt_m2s, BYTE_POS_M2S_CURRENTSPEED );
      rs.angle_target         = ByteUtil::reconFloat( local_pkt_m2s, BYTE_POS_M2S_TARGETANGLE );
      rs.angle_current        = ByteUtil::reconFloat( local_pkt_m2s, BYTE_POS_M2S_CURRENTANGLE );
      rs.angle_speed_target   = ByteUtil::reconFloat( local_pkt_m2s, BYTE_POS_M2S_TARANGSPEED );
      rs.angular_speed_current= ByteUtil::reconFloat( local_pkt_m2s, BYTE_POS_M2S_CURANGSPEED );
      rs.vacuum_voltage       = ByteUtil::reconFloat( local_pkt_m2s, BYTE_POS_M2S_VACUUMVOLTAGE );
      rs.foc_engaged = (local_pkt_m2s[BYTE_POS_M2S_FOCMODE] == FOC_EN_CODE)? true:false;
      //Serial.printf("LEFT !!!!!: %f\tRIGHT !!!!!: %f\n", rs.speed_target, rs.angle_target);
      // MasterSerial.printf("Target Speed: %f\n", rs.speed_target);
      vTaskDelay( 100 / portTICK_PERIOD_MS );
  }

}

void xVacuum( void* pv ){ 

  for( ; ; ){ 
    // Serial.println("Set speed");
    ledcWrite(VACUUM_PIN,vacuum_duty);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}

void xUpdateWheelbase( void* pv ){ 

  for( ; ; ){ 

    switch (rs.control_mode){
      case NULL_CONTROL:
        wheelbase.stop();
        // wheelbase.wheelClockwise(0, 2.0f);
        // wheelbase.wheelAntiClockwise(1, 2.0f);
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

        // wheelbase.wheelClockwise(0, 2.0f);
        // wheelbase.wheelAntiClockwise(1, 2.0f);

        if(rs.speed_target<1.0f && rs.speed_target>-1.0f)wheelbase.wheelStop(0);
        if(rs.angle_target<1.0f && rs.angle_target>-1.0f)wheelbase.wheelStop(1);
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

void xFOCroutine( void* pv ){
  for( ; ; ){
    foc_motor_l.loopFOC();
    foc_motor_l.move(rs.probe_tar_angle_left);
    rs.probe_angle_left = foc_motor_l.shaftAngle();

    foc_motor_r.loopFOC();
    foc_motor_r.move(rs.probe_tar_angle_right);
    rs.probe_angle_right = foc_motor_r.shaftAngle();

    // Serial.printf("%f %f\n",foc_motor_l.shaftAngle(),foc_motor_r.shaftAngle());

    vTaskDelay(2);
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

void xEchoBuffer( void* pv ){

  for( ; ; ){
    vomitRxBuffer();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    Serial.printf("angle_target: %f\n",           rs.angle_target);
    Serial.printf("angle_current: %f\n",          rs.angle_current);
    Serial.printf("angle_speed_target: %f\n",     rs.angle_speed_target);
    Serial.printf("angular_speed_current: %f\n",  rs.angular_speed_current);
    Serial.printf("vacuum_voltage: %f\n",         rs.vacuum_voltage);
    // Serial.printf("foc_engaged: %x\n",            rs.foc_engaged);
    // Serial.printf("foc_l: %f\n",            rs.probe_angle_left);
    // Serial.printf("foc_r: %f\n",            rs.probe_angle_right);
    Serial.println("----------------------------------------");
    
    vTaskDelay(200 / portTICK_PERIOD_MS);
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
  while(ledcAttach(VACUUM_PIN, 50, 12)==false){
    Serial.println("Attaching...");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  ledcWrite(VACUUM_PIN,410);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  ledcWrite(VACUUM_PIN,308);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  return true;
  //Vacuum INIT End
}
bool master_serial_init(){
  MasterSerial.setRxBufferSize(140);
  MasterSerial.begin(115200, SERIAL_8N1, 42, 41); //ISD Dev board Serial2
  return true;
}
bool foc_init(bool debug = true){
  i2c1.begin( I2C1_SDA, I2C1_SCK, 400000UL );
  i2c2.begin( I2C2_SDA, I2C2_SCK, 400000UL );
  if(debug) SimpleFOCDebug::enable(&Serial);

  foc_sensor_l.init(&i2c1);
  foc_sensor_r.init(&i2c2);

  foc_motor_l.linkSensor(&foc_sensor_l);
  foc_motor_r.linkSensor(&foc_sensor_r);

  foc_driver_l.voltage_power_supply = hw_v_supply;
  foc_driver_l.voltage_limit = hw_v_limit;
  foc_driver_r.voltage_power_supply = hw_v_supply;
  foc_driver_r.voltage_limit = hw_v_limit;

  if(!foc_driver_l.init()){
    Serial.println("Driver 1 init failed!");
    return false;
  }

  if(!foc_driver_r.init()){
    Serial.println("Driver 2 init failed!");
    return false;
  }

  foc_motor_l.linkDriver(&foc_driver_l);
  foc_motor_r.linkDriver(&foc_driver_r);

  foc_motor_l.voltage_limit = hw_v_limit;
  foc_motor_l.foc_modulation = FOCModulationType::SpaceVectorPWM;
  foc_motor_l.torque_controller = TorqueControlType::voltage;
  foc_motor_l.controller = MotionControlType::angle;

  foc_motor_r.voltage_limit = hw_v_limit;
  foc_motor_r.foc_modulation = FOCModulationType::SpaceVectorPWM;
  foc_motor_r.torque_controller = TorqueControlType::voltage;
  foc_motor_r.controller = MotionControlType::angle;

  foc_motor_l.PID_velocity.P = 1.20f;
  foc_motor_l.PID_velocity.I = 0.00f;
  foc_motor_l.PID_velocity.D = 0;

  foc_motor_r.PID_velocity.P = 1.20f;
  foc_motor_r.PID_velocity.I = 0.00f;
  foc_motor_r.PID_velocity.D = 0;

  foc_motor_l.LPF_velocity.Tf = 0.03f;
  foc_motor_r.LPF_velocity.Tf = 0.03f;

  foc_motor_l.P_angle.P = 20.0f;
  foc_motor_l.P_angle.I = 0.00f;
  foc_motor_l.P_angle.D = 0.02f;

  foc_motor_r.P_angle.P = 25.0f;
  foc_motor_r.P_angle.I = 0.00f;
  foc_motor_r.P_angle.D = 0.005f;

  // foc_motor_l.velocity_limit = 9.42f;
  // foc_motor_r.velocity_limit = 9.42f;

  foc_motor_l.voltage_limit = hw_v_limit; // Volts -  default driver.voltage_limit
  foc_motor_r.voltage_limit = hw_v_limit; // Volts -  default driver.voltage_limit

  foc_motor_l.useMonitoring(Serial);
  foc_motor_r.useMonitoring(Serial);

  if(!foc_motor_l.init()){
    Serial.println("Motor init failed!");
    return false;
  }

  if(!foc_motor_r.init()){
    Serial.println("Motor init failed!");
    return false;
  }

  while(!foc_motor_l.initFOC()){
    Serial.println("FOC init failed!");
  }

  while(!foc_motor_r.initFOC()){
    Serial.println("FOC init failed!");
  }

  rs.probe_tar_angle_left = -2.50f;
  rs.probe_tar_angle_right = -3.30f;

  return true;

}

/*////////////////////////////////////////////////////////////////
    Main Program
////////////////////////////////////////////////////////////////*/

void setup() 
{
  // vTaskDelay(500 / portTICK_PERIOD_MS);

  Serial.begin(115200);
  // master_serial_init();

  Serial.println("INIT\t||\START SETUP");
  // wheelbase.init(false);
  // wheelbase.stop();

  vacuum_init();
  // while(!foc_init(true));
  led_init();

  // !!!!!!!! Stuck at Here if failed

  // xTaskCreatePinnedToCore(  xPhraseCommand,   "Phrase Command",   xPhraseCommand_stack,NULL,3,&xPhraseCommand_handle,0 );
  // xTaskCreatePinnedToCore(  xSendCommand,   "Send Command",   xSendCommand_stack,NULL,1,&xSendCommand_handle,1 );
  // xTaskCreatePinnedToCore(  xUpdateState,   "Update State",   xUpdateState_stack,NULL,2,&xUpdateState_handle,1 );
  xTaskCreatePinnedToCore(  xVacuum,    "Vacuum",   xVacuum_stack, NULL,1,&xVacuum_handle,1 );
  // xTaskCreatePinnedToCore(  xUpdateWheelbase, "UpdateWheelbase",  xUpdateWheelbase_stack, NULL, 1 , &xUpdateWheelbase_handle, 1 );
  // xTaskCreatePinnedToCore( xFOCroutine, "FOC Routine",  xFOCroutine_stack,  NULL, 1,  &xFOCroutine_handle,  0 );
  xTaskCreatePinnedToCore( xBlinking, "Blinking", xBlinking_stack,  NULL, 1,  &xBlinking_handle, 1 );
  // xTaskCreatePinnedToCore( xEchoBuffer, "Echo Buffer", xEchoBuffer_stack,  NULL, 1,  &xEchoBuffer_handle, 1 );
  // xTaskCreatePinnedToCore( xVomitState, "Vomit State",  xVomitState_stack, NULL, 1,  &xVomitState_handle, 1 );


  vTaskDelay(500 / portTICK_PERIOD_MS);

  rs.probe_angle_left = 0.00f;
  rs.probe_angle_right = 0.00f;
  rs.speed_target = 0.00f;
  rs.angle_target = 0.00f;

}

void loop(){

}

