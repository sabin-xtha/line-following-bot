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
    motor[0].set_speed(speed - diff); // right 0
    motor[1].set_speed(speed + diff); // left 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
}
void Robot::backward(float diff)
{
    printf("backward\n");
    motor[0].set_speed(speed - diff);
    motor[1].set_speed(speed + diff);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
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
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
        // prevtime = HAL_GetTick();
    }
}
void Robot::right()
{
    printf("right\n");
    prevtime = HAL_GetTick();
    while ((HAL_GetTick() - prevtime) < 20)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
        // prevtime = HAL_GetTick();
    }
}
void Robot::stop()
{
    printf("stop\n");

    motor[0].set_speed(0);
    motor[1].set_speed(0);
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
        motor[0].set_speed(speed);
        motor[1].set_speed(speed);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
        // prevtime = HAL_GetTick();
    }
}
void Robot::init()
{
    HAL_UART_Receive_DMA(&huart1, &input, 1);

    stering_pid = PID(4, 0, 0, P_ON_E, DIRECT);
    stering_pid.Init();
    stering_pid.SetOutputLimits(-6, 6);
    stering_pid.SetTunings(1, 0, 0);
    stering_pid.SetSampleTime(20);
    stering_pid.SetMode(AUTOMATIC);

    motor[0] = Motor(&htim2, GPIOA, TIM_CHANNEL_4, GPIO_PIN_0, 65535);
    motor[1] = Motor(&htim2, GPIOE, TIM_CHANNEL_1, GPIO_PIN_7, 65535);
    motor[0].init();
    motor[1].init();
    enc[0] = Encoder(&htim1, 1425);
    enc[1] = Encoder(&htim4, 1425);
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
    }

    // left turn--->90, right turn--->
    if (input == 0b00000000 && state == 1)
    { // L
        printf("Left L\n");
        left();
        maze.check_new_point_or_not();

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
    else if ((input >> 6) > 0)
    {
        // right L
        state = 2;
        check_further();
    }
    else if ((input & 0b00000011) > 0)
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
        if ((HAL_GetTick() - prev_time) > 20)
        {

            // if(bot.input == 0){
            //     continue;
            // }
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
            // printf("%X\n", bot.input);
            bot.speed = 0.3;

            int nl_inp = bot.input;
            float linear_input = 0;
            for (int i = 0; i < 8; i++)
            {
                if (((nl_inp >> i) & 0b00000001))
                {
                    linear_input += 3.5 - i;
                }
            }
            // bot.forward();
            bot.control();
            bot.stering_pid.Setpoint = 0.0;
            bot.stering_pid.Input = (double)(linear_input);
            bot.stering_pid.Compute();
            float output = bot.stering_pid.Output / 40;
            // bot.forward(output);
            prev_time = HAL_GetTick();
            printf("%lf\t%lf\t%lf\n", bot.stering_pid.Input, bot.stering_pid.Output / 60, bot.stering_pid.Setpoint);
            // printf("%f\t%f", bot.enc[0].get_omega(), bot.enc[1].get_omega());
        }
    }
}