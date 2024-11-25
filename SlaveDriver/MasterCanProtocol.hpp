#ifndef MASTER_CAN_PROTOCOL_HPP  
#define MASTER_CAN_PROTOCOL_HPP

// #ifdef BIG_ENDIAN
// #define EXTRACT_BYTE_FROM_4BYTE_VALUE(x,y) (*((uint8_t*)(&x)+y))
// #endif

#ifdef SMALL_ENDIAN
#define EXTRACT_BYTE_FROM_4BYTE_VALUE(x,y) (*((uint8_t*)(&x)+(3-(y))))
#endif

/* MAX 8 BYTES USING CAN2.0 */
#define CAN_FRAME_DLC 8

/* FRAME ID - S2M */
#define ID_IMU_ACC_XY      0x101
#define ID_IMU_ACC_Z       0x102

#define ID_IMU_ANGVEL_XY   0x103
#define ID_IMU_ANGVEL_Z    0x104

#define ID_IMU_ANG_XY      0x105
#define ID_IMU_ANG_Z       0x106

#define ID_IMU_QUAT_XY     0x107
#define ID_IMU_QUAT_ZW     0x108

#define ID_FOC             0x200

/* FRAME ID - M2S */
#define ID_MOTOR_VOLTAGE   0x300
#define ID_ACTION          0x400

/*  Byte Position Macros - IMU DATA

  00  |   Data_x OR Data_z  (1st Byte)
  01  |   Data_x OR Data_z  (2nd Byte)
  02  |   Data_x OR Data_z  (3rd Byte)
  03  |   Data_x OR Data_z  (4th Byte)

  04  |   Data_y OR Data_w  (1st Byte)
  05  |   Data_y OR Data_w  (2nd Byte)
  06  |   Data_y OR Data_w  (3rd Byte)
  07  |   Data_y OR Data_w  (4th Byte)
*/

/*  Byte Position Macros - FOC ANGLE
  
  00  |   FOC Angle Left    (1st Byte)
  01  |   FOC Angle Left    (2nd Byte)
  02  |   FOC Angle Left    (3rd Byte)
  03  |   FOC Angle Left    (4th Byte)

  04  |   FOC Angle Right   (1st Byte)
  05  |   FOC Angle Right   (2nd Byte)
  06  |   FOC Angle Right   (3rd Byte)
  07  |   FOC Angle Right   (4th Byte)
*/

/*  Byte Position Macros - ACTION
  
  00  |   Action            (1st Byte)
*/

/*  Byte Position Macros - ACTION
  
  00  |   MotorVolt Left    (1st Byte)
  01  |   MotorVolt Left    (2nd Byte)
  02  |   MotorVolt Left    (3rd Byte)
  03  |   MotorVolt Left    (4th Byte)

  04  |   MotorVolt Right   (1st Byte)
  05  |   MotorVolt Right   (2nd Byte)
  06  |   MotorVolt Right   (3rd Byte)
  07  |   MotorVolt Right   (4th Byte)
*/

typedef uint8_t action_t;
#define STOP              0x00
#define FORWARD           0x01
#define BACKWARD          0x02
#define ANTI_CLOCKWISE    0x03
#define CLOCKWISE         0x04

/* IMU DATA BYTE */
#define BYTE_POS_IMU_XZ 0
#define BYTE_POS_IMU_YW 4

/* FOC ANGLE */
#define BYTE_POS_FOC_LEFT 0
#define BYTE_POS_FOC_RIGHT 4

/* ACTION */
#define BYTE_POS_ACTION 0

/* MOTOR VOLTAGE */
#define BYTE_POS_MOTOR_VOLT_LEFT 0
#define BYTE_POS_MOTOR_VOLT_RIGHT 4

#endif