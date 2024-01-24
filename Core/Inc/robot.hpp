#ifndef ROBOT_H_
#define ROBOT_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"

#ifdef __cplusplus
#include "motor.hpp"
#include "encoder.hpp"
#include "pid.hpp"
#include "vector"
#include <cstdlib>
#include <stdio.h>

#define pi 3.1415
uint32_t prevtime;
int state;

enum UpNext
{
  NORTH = 90,
  EAST = 0,
  SOUTH = 270,
  WEST = 180
};

class Node
{
public:
  float dist;
  float x, y;
  int theta = 0;
  uint8_t alt; // = 0;//Straight;//Left | Right | Back;
  Node *north;
  Node *south;
  Node *east;
  Node *west;
  UpNext dir = NORTH;

  int state;    // 1->left, 2->right, 3->T, dead-end->255, +->
  int type = 0; // 0->0, 1->L,R, 2->T, 3->+
  bool visited = false;
  Node()
  {
    x = 0;
    y = 0;
    theta = 0;
    alt = 0;
    north = NULL;
    south = NULL;
    west = NULL;
    east = NULL;
  }
  // node->x = this->x + dist * cos(this->theta);
  // node->y = this->y + dist * sin(this->theta);
  void update(uint8_t alt, int dist)
  {
    this->alt = alt;
    // this->dist = dist;
    // this->x = prev->x + dist * cos(this->theta);
    // this->y = prev->y + dist * sin(this->theta);
  }

  // bool checknochild(int angle)
  // {
  // }
};
Node dynamic_node[100];
int ptr = 0;
Node *getNode()
{
  Node *node = &dynamic_node[ptr];
  ptr++;
  return node;
}
void getchild(int angle, Node *t)
{
  if ((t->dir + angle) % 360 == NORTH)
  {
    t->north = getNode();
    t->north->south = t;
  }
  if ((t->dir + angle) % 360 == EAST)
  {
    t->east = getNode();
    t->east->west = t;
  }
  if ((t->dir + angle) % 360 == SOUTH)
  {
    t->south = getNode();
    t->south->north = t;
  }
  if ((t->dir + angle) % 360 == WEST)
  {
    t->west = getNode();
    t->west->east = t;
  }
}

class Robot
{
public:
  Motor motor[2];
  Encoder enc[2];
  uint8_t lfactor;
  bool mode = 0;
  bool is_end = false;
  float speed;

  uint8_t input;
  void init();
  void control();
  void left();
  void right();
  void uturn();
  void forward(float);
  void backward(float);
  void stop();
  void check_further();

  Node *nodes[100];
  Node *current_node;
  Node *temp_node;

  Robot() {}

  // void rotate_pi(float);

  PID stering_pid;
};

#endif

#ifdef __cplusplus
extern "C"
{
#endif
  void init_robot();
  void traverse();
  void operate_robot();
#ifdef __cplusplus
}
#endif

#endif // ROBOT_H_