#include "robot.hpp"
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
uint8_t dist_tolerence = 10;

void Robot::forward(float diff = 0)
{
    printf("forward\n");
    motor[0].set_speed((motor_pid[0].Output / 50) - diff); // right 0
    motor[1].set_speed((motor_pid[1].Output / 50) + diff); // left 1

    printf("%lf\t%lf\t%lf\n", motor_pid[1].Input, motor_pid[1].Setpoint, motor_pid[1].Output);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
}
void Robot::backward(float diff)
{
    printf("backward\n");
    motor[0].set_speed(speed - diff);
    motor[1].set_speed(speed + diff);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
}
void Robot::left()
{
    printf("left\n");
    prevtime = HAL_GetTick();
    while ((HAL_GetTick() - prevtime) < 20)
    {
        motor[0].set_speed(speed);
        motor[1].set_speed(speed);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
        // prevtime = HAL_GetTick();
    }
}
void Robot::right()
{
    printf("right\n");
    prevtime = HAL_GetTick();
    while ((HAL_GetTick() - prevtime) < 20)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
        // prevtime = HAL_GetTick();
    }
}
void Robot::stop()
{
    printf("stop\n");

    // motor[0].set_speed(0);
    // motor[1].set_speed(0);
    motor[0].set_speed((motor_pid[0].Output / 50)); // right 0
    motor[1].set_speed((motor_pid[1].Output / 50)); // left 1
    // speed = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
}
void Robot::uturn()
{
    printf("uturn\n");
    prevtime = HAL_GetTick();
    while ((HAL_GetTick() - prevtime) < 20)
    {
        stop();
        motor[0].set_speed(speed);
        motor[1].set_speed(speed);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
        // prevtime = HAL_GetTick();
    }
}
void Robot::init()
{
    HAL_UART_Receive_DMA(&huart1, &input, 1);

    motor_pid[0] = PID(1, 0, 0, P_ON_E, DIRECT);
    motor_pid[0].Init();
    motor_pid[0].SetOutputLimits(0, 50);
    motor_pid[0].SetTunings(2, 15, 0.03);
    motor_pid[0].SetSampleTime(40);
    motor_pid[0].SetMode(AUTOMATIC);

    motor_pid[1] = PID(1, 0, 0, P_ON_E, DIRECT);
    motor_pid[1].Init();
    motor_pid[1].SetOutputLimits(0, 50);
    motor_pid[1].SetTunings(2, 15, 0.03);
    motor_pid[1].SetSampleTime(40);
    motor_pid[1].SetMode(AUTOMATIC);

    stering_pid = PID(1, 0, 0, P_ON_E, DIRECT);
    stering_pid.Init();
    stering_pid.SetOutputLimits(-6, 6);
    // stering_pid.SetTunings(4.5, .5, 1);
    stering_pid.SetTunings(10, 1, 0);

    stering_pid.SetSampleTime(40);
    stering_pid.SetMode(AUTOMATIC);

    motor[0] = Motor(&htim2, GPIOA, TIM_CHANNEL_4, GPIO_PIN_0, 65535);
    motor[1] = Motor(&htim2, GPIOE, TIM_CHANNEL_1, GPIO_PIN_7, 65535);
    motor[0].init();
    motor[1].init();
    enc[0] = Encoder(&htim4, 1425);
    enc[1] = Encoder(&htim1, 1425);
    enc[0].init();
    enc[1].init();

    printf("init\n");
}

void Robot::check_further()
{
    // move forward for 100ms
    if ((HAL_GetTick() - prevtime) < 100)
    {
        forward(0.5);
        // prevtime = HAL_GetTick();
    }

    // left turn--->90, right turn--->
    if (input == 0b00000000 && state == 1)
    { // L
        printf("Left L\n");
        left();
        current_node = getNode();
        current_node->theta += 90;
        current_node->type = 1;
        getchild(90, current_node);
    }
    else if (input == 0b11111111 && state == 1)
    {
        // LT
        printf("Left T\n");
        left();
        current_node = getNode();
        current_node->theta += 90;
        current_node->type = 2;
        getchild(90, current_node);
        getchild(0, current_node);
    }
    else if (input == 0b00000000 && state == 2)
    {
        // R
        right();
        printf("Right T\n");
        current_node = getNode();
        current_node->theta -= 90;
        current_node->type = 1;
        getchild(-90, current_node);
    }
    else if (input == 0b00000000 && state == 2)
    {
        // RT
        printf("Right T\n");
        forward();
        current_node = getNode();
        current_node->theta -= 90;
        current_node->type = 2;
        getchild(-90, current_node);
        getchild(0, current_node);
    }
    else if (input == 0b11111111 && state == 3)
    {
        printf("End\n");
        is_end = true; // end
        stop();
    }
    else if (input == 0b00000000 && state == 3)
    { // T
        printf("T only\n");
        left();
        current_node = getNode();
        current_node->theta = 0;
        current_node->type = 2;
        getchild(90, current_node);
        getchild(-90, current_node);
    }
    else
    { // +
        printf("+\n");
        left();
        current_node = getNode();
        current_node->type = 3;
        getchild(90, current_node);
        getchild(-90, current_node);
        getchild(0, current_node);
    }
}
Robot bot;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
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
        // dead end
        current_node = getNode();
        current_node->type = 0;
        current_node->theta += 180;
        // current_node = current_node;
        uturn();
    }
    else if (input == 0b11111111)
    {
        // maybe T or end of maze
        state = 3;
        check_further();
    }
    else if ((input >> 7) > 0)
    {
        // right L
        state = 2;
        check_further();
    }
    else if ((input & 0b00000001) > 0)
    {
        // left L
        state = 1;
        check_further();
    }
    else
    {
        stering_pid.Setpoint = 24;
        stering_pid.Input = input;
        stering_pid.Compute();
        float output = stering_pid.Output / 255;
        forward(output);
    }
}

// int size = 0;
// void DFS(Node *v)
// {
//     // check if visited?
//     for (int i = 0; i < size; i++)
//     {
//         if ((fabs(bot.current_node->x - bot.nodes[i]->x) <= dist_tolerence) && (fabs(bot.current_node->y - bot.nodes[i]->y) <= dist_tolerence))
//         {
//             bot.current_node = bot.nodes[i];
//         }
//     }
//     bot.current_node->visited = true;
//     bot.nodes[size] = bot.current_node;
// }

// void traverse()
// {
//     while (bot.mode)
//     { // dry run
//     }
// }
void operate_robot()
{
    uint32_t prev_time = HAL_GetTick();
    while (1)
    {
        if ((HAL_GetTick() - prev_time) > 40)
        {

            bot.motor_pid[0].Input = -bot.enc[0].get_omega();
            bot.motor_pid[1].Input = bot.enc[1].get_omega();
            bot.motor_pid[0].Setpoint = bot.speed;
            bot.motor_pid[1].Setpoint = bot.speed;

            printf("%lf\n", bot.motor_pid[1].Input);
            bot.motor_pid[0].Compute();
            bot.motor_pid[1].Compute();
            // if (bot.input == 0)
            // {
            //     bot.motor_pid[0].Input = -bot.enc[0].get_omega();
            //     bot.motor_pid[1].Input = bot.enc[1].get_omega();
            //     bot.motor_pid[0].Setpoint = 0.0;
            //     bot.motor_pid[1].Setpoint = 0.0;

            //     // printf("%lf\n", bot.motor_pid[1].Input);
            //     bot.motor_pid[0].Compute();
            //     bot.motor_pid[1].Compute();
            //     bot.stop();
            //     continue;
            // }
            bot.speed = 15;

            int nl_inp = bot.input;
            float linear_input = 0;
            for (int i = 1; i < 7; i++)
            {
                if (((nl_inp >> i) & 0b00000001))
                {
                    linear_input += 3.5 - (i - 1);
                }
            }
            bot.stering_pid.Setpoint = 0.0;
            bot.stering_pid.Input = (double)(linear_input);
            bot.stering_pid.Compute();
            float output = bot.stering_pid.Output / bot.speed;
            // bot.forward(output);
            // bot.control();

            prev_time = HAL_GetTick();
            // printf("%lf\t%lf\t%lf\n", bot.stering_pid.Input, bot.stering_pid.Output / 60, bot.stering_pid.Setpoint);
            // printf("%f\t%f", bot.enc[0].get_omega(), bot.enc[1].get_omega());
        }
    }
}