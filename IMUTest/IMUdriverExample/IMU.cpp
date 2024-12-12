#include "IMU.hpp"

IMU::IMU(HardwareSerial* serial, float* acc, float* vel, float* gyro, float* angle){
    IMU::imuSerial = serial;
    this->acc = acc;
    this->vel = vel;
    this->gyro = gyro;
    this->angle = angle;
}

void IMU::init(){
    WitSetUartBaud(WIT_BAUD_115200);
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(IMU::SensorUartSend);
    WitRegisterCallBack(IMU::SensorDataUpdata);
    WitDelayMsRegister(IMU::Delayms);
    ms_last = millis();

    for(int i = 0; i < 3; i++){
        kf_acc[i].P = 0.1;
        kf_acc[i].G = 0;
        kf_acc[i].Q = 1;
        kf_acc[i].R = 5;
        kf_acc[i].Output = 0;
    }
    
    kf_acc[2].Output = 2048;

    
    Serial.println("IMU Calibration...");
    IMU::calibrate();
}


void IMU::calibrate(){
    int i;
    for(i = 0; i < 3; i++){
        acc_offset[i] = 0;
    }
    int32_t sum[3] = {0, 0, 0};
    int32_t ang_sum[3] = {0, 0, 0};
    for(i = 0; i < 20; i++){
        Serial.printf("Cal at %d\n", i);
        while(!IMU::process());
        sum[0] += sReg[AX];
        sum[1] += sReg[AY];
        sum[2] += sReg[AZ];
        ang_sum[0] += sReg[Roll];
        ang_sum[1] += sReg[Roll+1];
        ang_sum[2] += sReg[Roll+2];
    }
    acc_offset[0] = sum[0] / 20;
    acc_offset[1] = sum[1] / 20;
    acc_offset[2] = sum[2] / 20;

    acc_offset[0] = 0;
    acc_offset[1] = 0;
    acc_offset[2] = 2048;

    vel[0] = 0;
    vel[1] = 0;
    vel[2] = 0;
}

bool IMU::process(){
    int i;
    float temp;
    
    while (IMU::imuSerial->available()){ 
        WitSerialDataIn(IMU::imuSerial->read());
    }
    if(IMU::s_cDataUpdate){
        IMU::dt = millis() - ms_last;

        for(i = 0; i < 3; i++){

            //Kalman filter
            acc_last[i] = acc[i];
            // acc[i] = IMU::karmanFilter(&kf_acc[i], sReg[AX+i] / 32768.0f * 16.0f * 9.81f); 
            acc[i] = IMU::karmanFilter(&kf_acc[i], sReg[AX+i]) * 0.00479f;

            vel[i] = vel[i] + (acc[i] + acc_last[i] - acc_offset[i] * 0.00958f) / 2.0 * IMU::dt / 1000.0f; // Normal
            // Gyroscope
            gyro[i] = sReg[GX+i] / 32768.0f * 2000.0f; // Normal

            // Angle
            angle[i] = sReg[Roll+i] / 32768.0f * 180.0f; // Normal

        }
        if(IMU::s_cDataUpdate & ACC_UPDATE){
            IMU::s_cDataUpdate &= ~ACC_UPDATE;
        }
        if(IMU::s_cDataUpdate & GYRO_UPDATE){
            IMU::s_cDataUpdate &= ~GYRO_UPDATE;
        }
        if(IMU::s_cDataUpdate & ANGLE_UPDATE){
            IMU::s_cDataUpdate &= ~ANGLE_UPDATE;
        }
        if(IMU::s_cDataUpdate & MAG_UPDATE){
            IMU::s_cDataUpdate &= ~MAG_UPDATE;
        }
        IMU::s_cDataUpdate = 0;
      
        ms_last = ms_last + IMU::dt;

        return true;
    }
    return false;
}



void IMU::SensorUartSend(uint8_t *p_data, uint32_t uiSize){
    IMU::imuSerial->write(p_data, uiSize);
    IMU::imuSerial->flush();
}
void IMU::Delayms(uint16_t ucMs){
    vTaskDelay(ucMs / portTICK_PERIOD_MS);
}
void IMU::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum){
    int i;
    for(i = 0; i < uiRegNum; i++){
        switch(uiReg){
            case AZ:
                IMU::s_cDataUpdate |= ACC_UPDATE;
                break;
            case GZ:
                IMU::s_cDataUpdate |= GYRO_UPDATE;
                break;
            case HZ:
                IMU::s_cDataUpdate |= MAG_UPDATE;
                break;
            case Yaw:
                IMU::s_cDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                IMU::s_cDataUpdate |= READ_UPDATE;
                break;
        }
        uiReg++;
    }
}

float IMU::karmanFilter(KFType* kf, float input){
    kf->P = kf->P + kf->Q;
    kf->G = kf->P / (kf->P + kf->R);
    kf->Output = kf->Output + kf->G * (input - kf->Output);
    kf->P = (1 - kf->G) * kf->P;
    return kf->Output;
}

int16_t IMU::karmanFilter(KFType* kf, int16_t input){
    kf->P = kf->P + kf->Q;
    kf->G = kf->P / (kf->P + kf->R);
    kf->Output = kf->Output + kf->G * (input - kf->Output);
    kf->P = (1 - kf->G) * kf->P;
    return kf->Output;
}