#include "SlaveDriver/MasterSerialProtocol.hpp"
#include <iostream>
#include <stdint.h>
using namespace std;

// x is the array, y is the index for the first byte of the float
#define REINTERPRET_AS_FLOAT(x,y) (*(float*)(&x[y]))
// x is the 4 Byte Value(e.g. float), y is the index of the byte to extract
#define EXTRACT_BYTE_FROM_4BYTE_VALUE(x,y) (*((uint8_t*)(&x)+y))

/*
This code is a debugging tool to generate packets as a emulator of the master device.

This code is not intended to be used in the final product.

*/

uint8_t getCheckSum( uint8_t* data, uint8_t size )
{

  uint8_t check_sum = 0; 
  for( int i = 0; i < size - 1; i++ ) check_sum += data[i];
  return check_sum;

};

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

struct RobotState rs_test= RobotState();

int main()
{
    rs_test.v_estop = false;
    rs_test.control_mode = SPEED_CONTROL;
    rs_test.speed_target = 12.0;
    rs_test.speed_current = 0.0;
    rs_test.angle_target = 0.0;
    rs_test.angle_current = 0.0;
    rs_test.angle_speed_target = 0.0;
    rs_test.angular_speed_current = 0.0;
    rs_test.vacuum_voltage = 0.0;
    rs_test.foc_engaged = false;
    
    uint8_t packet[M2S_PACKET_SIZE];
    packet[BYTE_POS_M2S_STARTBIT] = START_BIT;
    packet[BYTE_POS_M2S_VESTOP] = rs_test.v_estop ? V_ESTOP_EN_CODE : V_ESTOP_DIS_CODE;
    packet[BYTE_POS_M2S_CONTROLMODE] = rs_test.control_mode;
    packet[BYTE_POS_M2S_TARGETSPEED] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.speed_target,0);
    packet[BYTE_POS_M2S_TARGETSPEED+1] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.speed_target,1);
    packet[BYTE_POS_M2S_TARGETSPEED+2] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.speed_target,2);
    packet[BYTE_POS_M2S_TARGETSPEED+3] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.speed_target,3);
    packet[BYTE_POS_M2S_CURRENTSPEED] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.speed_current,0);
    packet[BYTE_POS_M2S_CURRENTSPEED+1] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.speed_current,1);
    packet[BYTE_POS_M2S_CURRENTSPEED+2] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.speed_current,2);
    packet[BYTE_POS_M2S_CURRENTSPEED+3] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.speed_current,3);
    packet[BYTE_POS_M2S_TARGETANGLE] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_target,0);
    packet[BYTE_POS_M2S_TARGETANGLE+1] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_target,1);
    packet[BYTE_POS_M2S_TARGETANGLE+2] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_target,2);     
    packet[BYTE_POS_M2S_TARGETANGLE+3] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_target,3);
    packet[BYTE_POS_M2S_CURRENTANGLE] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_current,0);
    packet[BYTE_POS_M2S_CURRENTANGLE+1] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_current,1);
    packet[BYTE_POS_M2S_CURRENTANGLE+2] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_current,2);
    packet[BYTE_POS_M2S_CURRENTANGLE+3] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_current,3);
    packet[BYTE_POS_M2S_TARANGSPEED] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_speed_target,0);
    packet[BYTE_POS_M2S_TARANGSPEED+1] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_speed_target,1);
    packet[BYTE_POS_M2S_TARANGSPEED+2] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_speed_target,2);
    packet[BYTE_POS_M2S_TARANGSPEED+3] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angle_speed_target,3);
    packet[BYTE_POS_M2S_CURANGSPEED] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angular_speed_current,0);
    packet[BYTE_POS_M2S_CURANGSPEED+1] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angular_speed_current,1);
    packet[BYTE_POS_M2S_CURANGSPEED+2] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angular_speed_current,2);
    packet[BYTE_POS_M2S_CURANGSPEED+3] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.angular_speed_current,3);
    packet[BYTE_POS_M2S_VACUUMVOLTAGE] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.vacuum_voltage,0);
    packet[BYTE_POS_M2S_VACUUMVOLTAGE+1] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.vacuum_voltage,1);
    packet[BYTE_POS_M2S_VACUUMVOLTAGE+2] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.vacuum_voltage,2);
    packet[BYTE_POS_M2S_VACUUMVOLTAGE+3] = EXTRACT_BYTE_FROM_4BYTE_VALUE(rs_test.vacuum_voltage,3);
    packet[BYTE_POS_M2S_FOCMODE] = rs_test.foc_engaged ? FOC_EN_CODE : FOC_DIS_CODE;
    packet[BYTE_POS_M2S_CHECKSUM] = getCheckSum(packet, M2S_PACKET_SIZE);

    // Print the packet as a double digit hex number
    for(int i = 0; i < M2S_PACKET_SIZE; i++)
    {
        cout << hex << (int)packet[i] << " ";
    }
    cout << endl;
}