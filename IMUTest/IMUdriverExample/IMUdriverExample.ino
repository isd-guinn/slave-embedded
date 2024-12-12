#include "IMU.hpp"

#define IMU_RX        42
#define IMU_TX        41

float acc[3];
float vel[3];
float gyro[3];
float angle[3];

HardwareSerial ImuSerial(2);
IMU imu(&ImuSerial, acc, vel, gyro, angle);

/*    xImuProcess || Core 0   */
void xImuProcess( void* pv );
uint32_t xImuProcess_stack = 50000;
TaskHandle_t xImuProcess_handle = NULL;

/*    Anything || Core 0   */
void xAnything( void* pv );
uint32_t xAnything_stack = 20000;
TaskHandle_t xAnything_handle = NULL;

void xImuProcess( void* pv ){
  vel[0] = 0;
  vel[1] = 0;
  vel[2] = 0;
  for( ; ; ){
    imu.process();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void xAnything( void* pv ){

  for( ; ; ){
    

    ///***************************///
    // Put Code here for task
    /*
          Data:
          float acc[3];
          float vel[3];
          float gyro[3];
          float angle[3];
    */
    ///***************************///

    Serial.println("----------------------------------------");
    Serial.printf("acc   x:%10.3f\t\ty:%10.3f\t\tz:%10.3f\n",        acc[0],acc[1],acc[2]);
    Serial.printf("vel   x:%10.3f\t\ty:%10.3f\t\tz:%10.3f\n",        vel[0],vel[1],vel[2]);
    Serial.printf("gyro  x:%10.3f\t\ty:%10.3f\t\tz:%10.3f\n",        gyro[0],gyro[1],gyro[2]);
    Serial.printf("angle x:%10.3f\t\ty:%10.3f\t\tz:%10.3f\n",        angle[0],angle[1],angle[2]);
    Serial.println("----------------------------------------");
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


void setup() {

  Serial.begin(115200);

  ImuSerial.begin(115200, SERIAL_8N1, IMU_RX, IMU_TX); //ESP32S3
  imu.init();

  xTaskCreatePinnedToCore( xImuProcess, "IMU Process",  xImuProcess_stack, NULL, 1,  &xImuProcess_handle, 0 );
  xTaskCreatePinnedToCore( xAnything, "Anything",  xAnything_stack, NULL, 1,  &xAnything_handle, 1 );

}

void loop() {}
