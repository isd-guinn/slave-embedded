// LEFT CLOCKWISE -> FORWARD
// RIGHT ANTICLOCKWISE -> FORWARD

#ifndef WHEELSCONTROL_HPP
#define WHEELSCONTROL_HPP

#include <Arduino.h>

#define PWM_FREQ 50
#define PWM_RES 12

#define INIT_TIMEOUT 5000

class Wheelbase
{
  private:
    uint8_t _pin[2][3]; // 0: Left 1: Right;
    float _v_supply;
    float _v_limit;
    bool _debug_mode;

  public:
    Wheelbase( 
      float v_supply, 
      float v_limit, 

      uint8_t pin_L_1, 
      uint8_t pin_R_1, 

      uint8_t pin_L_2, 
      uint8_t pin_R_2, 

      uint8_t pin_L_en, 
      uint8_t pin_R_en
      );
    
    float getSupplyVoltage() const;
    float getLimitVoltage() const;

    long calDuty(float x, float min, float max, int res);

    void wheelStop( uint8_t select );
    void wheelClockwise( uint8_t select, float v_target );
    void wheelAntiClockwise( uint8_t select, float v_target );

    void stop();
    void foward(float v_target);
    void backward(float v_target);
    void left(float v_target);
    void right(float v_target);

    bool init(bool debug);
    void debugMode(bool on);

    void set( 
      float v_supply, 
      float v_limit, 

      uint8_t pin_L_1, 
      uint8_t pin_R_1, 

      uint8_t pin_L_2, 
      uint8_t pin_R_2, 

      uint8_t pin_L_en, 
      uint8_t pin_R_en
      );

};

#endif