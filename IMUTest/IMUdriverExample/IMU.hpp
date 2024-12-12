
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

#define MOVING_AVERAGE_SIZE 3
#define TRIGO_RESOLUTION 4096

typedef struct {
    float P;
    float G;
    float Q;
    float R;
    float Output;
}KFType;

class IMU{
    private:
        static inline uint32_t s_cDataUpdate;
        static inline HardwareSerial* imuSerial;

        KFType kf_acc[3];

        long ms_last;
        static inline long dt;

        int16_t acc_offset[3];
        float acc_last[3];

        float* acc;
        float* vel;
        float* gyro;
        float* angle;
        
        static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
        static void Delayms(uint16_t ucMs);
        static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);

        static inline float karmanFilter(KFType* kf, float input);
        static inline int16_t karmanFilter(KFType* kf, int16_t input);


    public:
        IMU(HardwareSerial* serial, float* acc, float* vel, float* gyro, float* angle);
        void init();
        void calibrate();
        bool process();
};

#endif // _IMU_HPP_