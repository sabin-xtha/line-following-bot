#ifndef ROBOT_H_
#define ROBOT_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"

#ifdef __cplusplus
#include "motor.hpp"
#include "encoder.hpp"
#include "pid.hpp"

enum UpNext
{
  Left = 1,
  Right = 2,
  Straight = 4,
  Back = 8
};
class Node
{
public:
  float dist;
  float x, y, theta;
  uint8_t alt;// = 0;//Straight;//Left | Right | Back;
  Node *up;
  Node *down;
  Node *left;
  Node *right;
  UpNext next = Straight;

  Node()
  {
    x = 0;  
    y = 0;
    theta = 0;
    alt = 0;
    up = NULL;
    down = NULL;
    left = NULL;
    right = NULL;
  }
  Node *getNode(int dist)
  {
    Node *node = new Node();
    // node->prev = this;
    // node->left = NULL;
    // node->right = NULL;
    // node->x = this->x + dist * cos(this->theta);
    // node->y = this->y + dist * sin(this->theta);
    return node;
  }
  void update(uint8_t alt, int dist)
  {
    this->alt = alt;
    // this->dist = dist;
    // this->x = prev->x + dist * cos(this->theta);
    // this->y = prev->y + dist * sin(this->theta);
  }
};

class Robot
{
public:
  Motor motor[2];
  Encoder enc[2];
  uint8_t lfactor;

  uint8_t input;
  void init();
  void control();
  void left(float);
  void right(float);
  void forward(float);
  void backward(float);
  void stop();

  Node start;
  // void rotate_pi(float);

  PID stering_pid;
};

#endif

#ifdef __cplusplus
extern "C"
{
#endif
  void init_robot();
  void operate_robot();
#ifdef __cplusplus
}
#endif

#endif // ROBOT_H_
