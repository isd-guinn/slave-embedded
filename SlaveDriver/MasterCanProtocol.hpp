
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
#define ID_IMU_ACC_XY      101 // 0x65
#define ID_IMU_ACC_Z       102 // 0x66

#define ID_IMU_ANGVEL_XY   103 // 0x67
#define ID_IMU_ANGVEL_Z    104 // 0x68

#define ID_IMU_ANG_XY      105 // 0x69
#define ID_IMU_ANG_Z       106 // 0x6A

#define ID_IMU_QUAT_XY     107 // 0x6B
#define ID_IMU_QUAT_ZW     108 // 0x6C

#define ID_FOC             200 // 0xC8

/* FRAME ID - M2S */
#define ID_MOTOR_VOLTAGE   300 // 0x12C
#define ID_ACTION          400 // 0x190

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
