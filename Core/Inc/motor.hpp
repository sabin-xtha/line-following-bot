#ifndef ROBOTLIB_MOTOR_HPP_
#define ROBOTLIB_MOTOR_HPP_

enum Direction
{
  CLOCKWISE,
  ANTI_CLOCKWISE
};

#include "stm32f4xx.h"

Direction reverse_direction(Direction dir);

class Motor
{
public:
  TIM_HandleTypeDef *pwm_timer;
  GPIO_TypeDef *direction_port1;
  GPIO_TypeDef *direction_port2;
  uint16_t pwm_full_signal;
  uint16_t pwm_signal;
  int pwm_timer_channel;
  uint16_t direction_pin1;
  uint16_t direction_pin2;
  Direction direction;

  Motor(TIM_HandleTypeDef *_pwm_timer, GPIO_TypeDef *_direction_port1,
        int _pwm_timer_channel, uint16_t _direction_pin1, GPIO_TypeDef *_direction_port2,
        uint16_t _direction_pin2, int _pwm_full_signal = 20999, int _pwm_signal = 0,
        Direction _direction = ANTI_CLOCKWISE)
      : pwm_timer(_pwm_timer), direction_port1(_direction_port1), direction_port2(_direction_port2),
        pwm_full_signal(_pwm_full_signal), pwm_signal(_pwm_signal),
        pwm_timer_channel(_pwm_timer_channel), direction_pin1(_direction_pin1), direction_pin2(_direction_pin2),
        direction(_direction) {}

  Motor() {}

  Motor(const Motor &_motor)
  {
    pwm_timer = _motor.pwm_timer;
    direction_port1 = _motor.direction_port1;
    direction_port2 = _motor.direction_port2;
    pwm_full_signal = _motor.pwm_full_signal;
    pwm_signal = _motor.pwm_signal;
    pwm_timer_channel = _motor.pwm_timer_channel;
    direction_pin1 = _motor.direction_pin1;
    direction_pin2 = _motor.direction_pin2;
    direction = _motor.direction;
  }

  void set_direction(Direction _direction);

  void set_speed(float speed);

  void init(void);

  void update();
};

#endif // ROBOTLIB_MOTOR_HPP_
