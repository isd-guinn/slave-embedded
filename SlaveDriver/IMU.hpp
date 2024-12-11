#ifndef _IMU_HPP_
#define _IMU_HPP_

#include "REG.h"
#include "wit_c_sdk.h"
#include "Arduino.h"

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

class IMU
{
    private:
        uint32_t s_cDataUpdate;
        HardwareSerial imuSerial;
        float acc[3];
        float gyro[3];
        float angle[3];
        
        static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
        static void Delayms(uint16_t ucMs);
        static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
    
    public:
        IMU(HardwareSerial serial);
        void init();
        void process();

}

#endif // _IMU_HPP_