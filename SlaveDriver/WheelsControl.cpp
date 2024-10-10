#include "WheelsControl.hpp"

Wheelbase::Wheelbase( 
                      float v_supply, 
                      float v_limit, 

                      uint8_t pin_L_1, 
                      uint8_t pin_R_1, 

                      uint8_t pin_L_2, 
                      uint8_t pin_R_2, 

                      uint8_t pin_L_en, 
                      uint8_t pin_R_en

                      )
{
  set( v_supply, v_limit, pin_L_1, pin_R_1, pin_L_2, pin_R_2, pin_L_en, pin_R_en);
};

float Wheelbase::getSupplyVoltage() const
{
  return _v_supply;
}

float Wheelbase::getLimitVoltage() const
{
  return _v_limit;
}

void Wheelbase::wheelStop( uint8_t select )
{
  
  for(int i=2;i>=0;i--) digitalWrite(_pin[select][i], LOW);

}

void Wheelbase::wheelClockwise( uint8_t select, uint8_t power )
{
  digitalWrite(_pin[select][2], HIGH);
  digitalWrite(_pin[select][0], HIGH);
  digitalWrite(_pin[select][1], LOW);
}

void Wheelbase::wheelAntiClockwise( uint8_t select, uint8_t power )
{
  digitalWrite(_pin[select][2], HIGH);
  digitalWrite(_pin[select][0], LOW);
  digitalWrite(_pin[select][1], HIGH);
}

void Wheelbase::stop()
{
  for(int i=0;i<2;i++) wheelStop(i);
}

void Wheelbase::foward(uint8_t power)
{
  wheelClockwise(0,100);
  wheelAntiClockwise(1,100);
}

void Wheelbase::backward(uint8_t power)
{
  wheelClockwise(1,100);
  wheelAntiClockwise(0,100);
}

void Wheelbase::left(uint8_t power)
{
  wheelAntiClockwise(0,100);
  wheelAntiClockwise(1,100);
}

void Wheelbase::right(uint8_t power)
{
  wheelClockwise(0,100);
  wheelClockwise(1,100);
}

bool Wheelbase::init()
{ 

  for(int i=0;i<2;i++){ for(int j=0;j<2;j++){ pinMode(_pin[i][j], OUTPUT); } }
  stop();

}

void Wheelbase::set( 
                      float v_supply, 
                      float v_limit, 

                      uint8_t pin_L_1, 
                      uint8_t pin_R_1, 

                      uint8_t pin_L_2, 
                      uint8_t pin_R_2, 

                      uint8_t pin_L_en, 
                      uint8_t pin_R_en
                      
                      )
{

  _v_supply = v_supply;
  _v_limit = v_limit;

  _pin[0][0] = pin_L_1;
  _pin[0][1] = pin_L_2;
  _pin[0][2] = pin_L_en;

  _pin[1][0] = pin_L_1;
  _pin[1][1] = pin_L_1;
  _pin[1][2] = pin_L_1;

}


