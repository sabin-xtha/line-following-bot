#include "robot.h"
#include "gpio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdint.h>

uint8_t data = 0;

void Robot::forward(float speed)
{
    motor[0].set_direction(CLOCKWISE);
    motor[1].set_direction(CLOCKWISE);
    motor[0].set_speed(speed);
    motor[1].set_speed(speed);
}
void Robot::backward(float speed)
{
    motor[0].set_direction(CLOCKWISE);
    motor[1].set_direction(CLOCKWISE);
    motor[0].set_speed(speed);
    motor[1].set_speed(speed);
}
void Robot::left(float speed)
{
    motor[0].set_direction(CLOCKWISE);
    motor[1].set_direction(CLOCKWISE);
    motor[0].set_speed(speed);
    motor[1].set_speed(-speed);
}
void Robot::right(float speed)
{
    motor[0].set_direction(CLOCKWISE);
    motor[1].set_direction(CLOCKWISE);
    motor[0].set_speed(speed);
    motor[1].set_speed(speed);
}
void Robot::stop()
{
    motor[0].set_direction(CLOCKWISE);
    motor[1].set_direction(CLOCKWISE);
    motor[0].set_speed(0);
    motor[1].set_speed(0);
}
void Robot::init()
{
    HAL_UART_Receive_DMA(&huart1, &input, 1);

    stering_pid = PID(1, 0, 0, P_ON_E, DIRECT);
    stering_pid.Init();
    stering_pid.SetOutputLimits(-50, 50);
    stering_pid.SetTunings(1, 0, 0);
    stering_pid.SetSampleTime(50);
    stering_pid.SetMode(AUTOMATIC);

    motor[0] = Motor(&htim9, GPIOE, TIM_CHANNEL_1, GPIO_PIN_3);
    motor[1] = Motor(&htim9, GPIOC, TIM_CHANNEL_2, GPIO_PIN_14);
    enc[0] = Encoder(&htim1, 600);
    enc[1] = Encoder(&htim5, 600);
}
Robot bot;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance)
    {
        HAL_UART_Receive_DMA(&huart1, &bot.input, 1);
    }
}
void init_robot()
{
    bot.init();
}
void Robot::control()
{
    if (input == 0b00000000)
    {
        // no line
    }
    else if (input == 0b11111111)
    {
        // junction or end point
    }
    else if ((input >> 6) > 2)
    {
        // rotate left
    }
    else if(1){

    }
    else if ((input & 0b00000011) > 0)
    {
        //rotate right
    }
    else
    {
        stering_pid.Setpoint = 35;
        stering_pid.Compute();
        float output = stering_pid.Output / 70;
        forward(output);
    }
    // else if (input >= 0 && input <= 25)
    // {
    // }
    // else if (input >= 45 && input <= 70)
    // {
    // }
    // else if ()
    // {
    // }

    // float input;
    //
    // if (output <= 1)
    // {
    //
    // }
}
void operate_robot()
{
    uint32_t prev_time = 0;
    while (HAL_GetTick() - prev_time)
    {
        prev_time = HAL_GetTick();
    }
}