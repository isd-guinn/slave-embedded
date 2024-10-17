#ifndef MASTERSERIALPROTOCOL_HPP 
#define MASTERSERIALPROTOCOL_HPP

#include <Arduino.h>
#include <SerialCommunication.hpp>

/*
  Packet from Master:
  
  00  |   StartBit
  01  |   VirtualEStop
  02  |   ControlMode

  03  |   TargetSpeed   (1st Byte)
  04  |   TargetSpeed   (2nd Byte)
  05  |   TargetSpeed   (3rd Byte)
  06  |   TargetSpeed   (4th Byte)

  07  |   CurrentSpeed  (1st Byte)
  08  |   CurrentSpeed  (2nd Byte)
  09  |   CurrentSpeed  (3rd Byte)
  10  |   CurrentSpeed  (4th Byte)

  11  |   TargetAngle   (1st Byte)
  12  |   TargetAngle   (2nd Byte)
  13  |   TargetAngle   (3rd Byte)
  14  |   TargetAngle   (4th Byte)

  15  |   CurrentAngle  (1st Byte)
  16  |   CurrentAngle  (2nd Byte)
  17  |   CurrentAngle  (3rd Byte)
  18  |   CurrentAngle  (4th Byte)

  19  |   VacuumVoltage (1st Byte)
  20  |   VacuumVoltage (2nd Byte)
  21  |   VacuumVoltage (3rd Byte)
  22  |   VacuumVoltage (4th Byte)

  23  |   FOCMode

  24  |   CheckSum
*/

/*
  Packet from Slave:
  
  00  |   StartBit
  01  |   DebugCode

  02  |   LeftFOCAngle
  03  |   RightFOCAngle
  
  04  |   CheckSum
*/

#define M2S_POCKET_SIZE   25
#define S2M_POCKET_SIZE   5

#define START_BIT         0x3E

#define V_ESTOP_EN_CODE   0xA1
#define V_ESTOP_DIS_CODE  0xA2

typedef uint8_t control_mode_t;
#define NULL_CONTROL      0x00
#define SPEED_CONTROL     0x01
#define ANGLE_CONTROL     0x02

#define FOC_EN_CODE       0xB1
#define FOC_DIS_CODE      0xB2

typedef uint8_t debug_code_t;
#define DEBUG_            0x01

#endif


