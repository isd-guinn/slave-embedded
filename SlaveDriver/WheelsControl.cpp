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
  _debug_mode = false;

};

inline float Wheelbase::getSupplyVoltage() const
{
  return _v_supply;
}

inline float Wheelbase::getLimitVoltage() const
{
  return _v_limit;
}

inline long Wheelbase::calDuty(float x, float min, float max, int res)
{
  return (x-min) / (max-min) * ( pow(2,res)-1 );
}


void Wheelbase::wheelStop( uint8_t select )
{
  digitalWrite( _pin[select][2], LOW);
  ledcWrite( _pin[select][0], 0);
  ledcWrite( _pin[select][1], 0);

  if(_debug_mode) Serial.printf("DEBUG\t||\tWheel STOPPED: %s WHEEL\n",(select)?"RIGHT":"LEFT");
}

void Wheelbase::wheelClockwise( uint8_t select, float v_target)
{
  v_target = constrain(v_target, 0, _v_limit);
  long duty = calDuty(v_target, 0, _v_limit, PWM_RES);
  ledcWrite( _pin[select][0], duty);
  ledcWrite( _pin[select][1], 0);
  digitalWrite( _pin[select][2], HIGH);

  if(_debug_mode) Serial.printf("DEBUG\t||\tWheel CLOCKWISE: %s WHEEL -- Voltage: %f -- Duty: %d\n",(select)?"RIGHT":"LEFT",v_target, duty);
}

void Wheelbase::wheelAntiClockwise( uint8_t select, float v_target)
{
  v_target = constrain(v_target, 0, _v_limit);
  long duty = calDuty(v_target, 0, _v_limit, PWM_RES);
  ledcWrite( _pin[select][0], 0 );
  ledcWrite( _pin[select][1], duty);
  digitalWrite( _pin[select][2], HIGH);

  if(_debug_mode) Serial.printf("DEBUG\t||\tWheel ANTICLOCKWISE: %s WHEEL -- Voltage: %f -- Duty: %d\n",(select)?"RIGHT":"LEFT",v_target, duty);
}

void Wheelbase::stop()
{
  wheelStop(0);
  wheelStop(1);

  if(_debug_mode) Serial.println("DEBUG\t||\tWheelbase STOPPED");
  vTaskDelay(10);
}

void Wheelbase::foward(float v_target)
{
  wheelAntiClockwise(0,v_target);
  wheelClockwise(1,v_target);

  if(_debug_mode) Serial.printf("DEBUG\t||\tWheelbase FORWARD: Target Voltage: %f\n", v_target);
  vTaskDelay(10);
}

void Wheelbase::backward(float v_target)
{
  wheelClockwise(0,v_target);
  wheelAntiClockwise(1,v_target); 

  if(_debug_mode) Serial.printf("DEBUG\t||\tWheelbase BACKWARD: Target Voltage: %f\n", v_target);
  vTaskDelay(10);
}

void Wheelbase::left(float v_target)
{
  wheelClockwise(0,v_target);
  wheelClockwise(1,v_target);

  if(_debug_mode) Serial.printf("DEBUG\t||\tWheelbase LEFT: Target Voltage: %f\n", v_target);
  vTaskDelay(10);
}

void Wheelbase::right(float v_target)
{
  wheelAntiClockwise(0,v_target);
  wheelAntiClockwise(1,v_target); 

  if(_debug_mode) Serial.printf("DEBUG\t||\tWheelbase RIGHT: Target Voltage: %f\n", v_target);
  vTaskDelay(10);
}

bool Wheelbase::init(bool debug=false)
{ 
  long last = esp_timer_get_time();
  bool ready = true;

  // Config DEBUG mode from parameters
  debugMode(debug);

  if(_debug_mode) Serial.println("DEBUG\t||\tWheelbase Start Init");

  // INIT for wheels in both sides
  for(int i=0;i<2;i++)
  { 

    // INIT LOOP of PWM for IN1 pin
    while( ledcAttach(_pin[i][0], 50, 12) == false )
    { 
      // Failed Conditions
      if(_debug_mode) Serial.printf("ERROR\t||\t PIN%d Init Failed\n", _pin[i][0]);

      // Break if timeout
      if( (esp_timer_get_time() - last) > INIT_TIMEOUT)
      {
        if(_debug_mode) Serial.printf("ERROR\t||\t PIN%d Init TIMEOUT\n", _pin[i][0]);
        ready = false;
        break;
      }
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    //update last time
    last = esp_timer_get_time();

    // INIT LOOP of PWM for IN2 pin
    while( ledcAttach(_pin[i][1], 50, 12) == false )
    {
      // Failed Conditions
      if(_debug_mode) Serial.printf("ERROR\t||\t PIN%d Init Failed\n", _pin[i][1]);

      // Break if timeout
      if( (esp_timer_get_time() - last) > INIT_TIMEOUT)
      {
        if(_debug_mode) Serial.printf("ERROR\t||\t PIN%d Init TIMEOUT\n", _pin[i][1]);
        ready = false;
        break;
      }
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    //update last time
    last = esp_timer_get_time();

    // INIT for EN pin
    pinMode(_pin[i][2], OUTPUT);
  }

  if(_debug_mode) Serial.println("DEBUG\t||\tWheelbase End Init");

  //Stop the motors on init
  stop();

  //return True
  return ready;

}

void Wheelbase::debugMode(bool on)
{
  _debug_mode = on;
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

  _pin[1][0] = pin_R_1;
  _pin[1][1] = pin_R_2;
  _pin[1][2] = pin_R_en;

}


