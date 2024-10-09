#ifndef WHEELSCONTROL_HPP
#define WHEELSCONTROL_HPP

#include <Arduino.h>

class Wheelbase
{
  private:
    uint8_t _pin[2][3]; // 0: Left 1: Right;
    float _v_supply;
    float _v_limit;

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

    void wheelStop();
    void wheelClockwise( uint8_t select, uint8_t power );
    void wheelAntiClockwise( uint8_t select, uint8_t power );

    void stop();
    void foward(uint8_t power);
    void backward(uint8_t power);
    void left(uint8_t power);
    void right(uint8_t power);

    bool init();

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