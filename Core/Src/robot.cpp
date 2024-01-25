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

#define SAMPLE_TIME 10
#define PATH GPIO_PIN_SET
#define NOT_PATH GPIO_PIN_RESET

uint8_t data = 0;
uint8_t dist_tolerence = 10;

uint32_t uart_receive_time;
void Robot::forward()
{
    int nl_inp = input;
    linear_input = 0.0;
    for (int i = 0; i < 8; i++)
    {
        if (((nl_inp >> i) & 0b00000001))
        {
            linear_input -= 3.5 - (i);
        }
    }

    stering_pid.Setpoint = 0.0;
    stering_pid.Input = (double)(linear_input);
    stering_pid.SetOutputLimits(-(speed / 3.0), (speed / 3.0));
    stering_pid.Compute();
    double output = stering_pid.Output;

    motor_pid[0].Input = -enc[0].get_omega();
    motor_pid[1].Input = enc[1].get_omega();
    motor_pid[0].Setpoint = speed - output;
    motor_pid[1].Setpoint = speed + output;
    motor_pid[0].Compute();
    motor_pid[1].Compute();
    // printf("forward\n");
    motor[0].set_speed((motor_pid[0].Output) / 50); // right 0
    motor[1].set_speed((motor_pid[1].Output) / 50); // left 1

    // printf("pid_outputs:::%lf\t%lf\t%f\n", (motor_pid[0].Output - output) / 50, (motor_pid[1].Output + output) / 50, output);
    // printf("motor speeds:::%f\t%f\n", motor_pid[0].Input, motor_pid[1].Input);
}
void Robot::backward(float output)
{
}
void Robot::left()
{
    prev_count1 = enc[0].get_count();
    uint32_t prev_time = HAL_GetTick();
    while (abs(enc[0].get_count() - prev_count1) < 1610)
    {
        if ((HAL_GetTick() - prev_time) > SAMPLE_TIME)
        {
            printf("left, %d\n", enc[0].get_count());

            motor_pid[0].Setpoint = 10;
            motor_pid[1].Setpoint = 0.0;
            motor_pid[0].Input = -enc[0].get_omega();
            motor_pid[1].Input = enc[1].get_omega();
            motor_pid[0].Compute();
            motor_pid[1].Compute();
            motor[0].set_speed(motor_pid[0].Output / 50);
            motor[1].set_speed(motor_pid[1].Output / 50);
            prev_time = HAL_GetTick();
        }
    }
}
void Robot::right()
{
    prev_count2 = enc[1].get_count();
    uint32_t prev_time = HAL_GetTick();
    while (abs(enc[1].get_count() - prev_count2) < 1610)
    {
        if ((HAL_GetTick() - prev_time) > SAMPLE_TIME)
        {
            printf("left, %d\n", enc[1].get_count());

            motor_pid[0].Setpoint = 0.0;
            motor_pid[1].Setpoint = 10;
            motor_pid[0].Input = -enc[0].get_omega();
            motor_pid[1].Input = enc[1].get_omega();
            motor_pid[0].Compute();
            motor_pid[1].Compute();
            motor[0].set_speed(motor_pid[0].Output / 50);
            motor[1].set_speed(motor_pid[1].Output / 50);
            prev_time = HAL_GetTick();
        }
    }
}
void Robot::stop()
{
    printf("stop\n");
    motor_pid[0].outputSum = 0;
    motor_pid[1].outputSum = 0;

    motor_pid[0].Setpoint = 0;
    motor_pid[1].Setpoint = 0;

    motor[0].set_speed(0.0); // right 0
    motor[1].set_speed(0.0); // left 1
}
void Robot::uturn()
{
    prev_count1 = enc[0].get_count();
    prev_count2 = enc[1].get_count();

    uint32_t prev_time = HAL_GetTick();
    while (((input & 0b00100000)) != 0b00100000)
    {
        // while (abs(enc[0].get_count() - prev_count1) < 1610 && abs(enc[1].get_count() - prev_count2) < 1610){
        if ((HAL_GetTick() - prev_time) > SAMPLE_TIME)
        {
            printf("left, %d\n", enc[0].get_count());

            motor_pid[0].Setpoint = 10;
            motor_pid[1].Setpoint = -10;
            motor_pid[0].Input = -enc[0].get_omega();
            motor_pid[1].Input = enc[1].get_omega();
            motor_pid[0].Compute();
            motor_pid[1].Compute();
            motor[0].set_speed(motor_pid[0].Output / 50);
            motor[1].set_speed(motor_pid[1].Output / 50);
            prev_time = HAL_GetTick();
        }
    }
}
void Robot::init()
{
    HAL_UART_Receive_DMA(&huart1, &input, 1);
    uart_receive_time = HAL_GetTick();

    motor_pid[0] = PID(1, 0, 0, P_ON_E, REVERSE);
    motor_pid[0].Init();
    motor_pid[0].SetOutputLimits(-40, 40);
    motor_pid[0].SetTunings(2, 15, 0.03);
    motor_pid[0].SetSampleTime(SAMPLE_TIME);
    motor_pid[0].SetMode(AUTOMATIC);

    motor_pid[1] = PID(1, 0, 0, P_ON_E, DIRECT);
    motor_pid[1].Init();
    motor_pid[1].SetOutputLimits(-40, 40);
    motor_pid[1].SetTunings(1.5, 13, 0.03);
    motor_pid[1].SetSampleTime(SAMPLE_TIME);
    motor_pid[1].SetMode(AUTOMATIC);

    stering_pid = PID(1, 0, 0, P_ON_E, DIRECT);
    stering_pid.Init();
    stering_pid.SetOutputLimits(-(speed / 4.0), (speed / 4.0));
    stering_pid.SetTunings(1, 0.02, .4); //(4.5, .5, 1);
    stering_pid.SetSampleTime(SAMPLE_TIME);
    stering_pid.SetMode(AUTOMATIC);

    motor[0] = Motor(&htim2, GPIOA, TIM_CHANNEL_4, GPIO_PIN_0, GPIOA, GPIO_PIN_1, 65535);
    motor[1] = Motor(&htim2, GPIOE, TIM_CHANNEL_1, GPIO_PIN_7, GPIOE, GPIO_PIN_14, 65535);
    motor[0].init();
    motor[1].init();
    enc[0] = Encoder(&htim4, 1425);
    enc[1] = Encoder(&htim1, 1425);
    enc[0].init();
    enc[1].init();

    printf("init\n");
}

Robot bot;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
        HAL_UART_Receive_DMA(&huart1, &bot.input, 1);
        uart_receive_time = HAL_GetTick();
    }
}
void init_robot()
{
    bot.init();
}
void Robot::control()
{
    GPIO_PinState forward_ir = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);

    // if(forward_ir == GPIO_PIN_SET){
    //     printf("setttttttttttttt\n");
    // }
    // else{
    //     printf("resettttttttttttt\n");
    // }

    if (input == 0b00000000)
    {
        printf("dead end\n");
        // dead end
        // current_node = getNode();
        // current_node->type = 0;
        // current_node->theta += 180;
        // current_node = current_node;
        maze.node_found();
        stop();
        HAL_Delay(2000);
        // if ((HAL_GetTick() - time) < 2000)
        // {
        //     stop();
        //     return;
        // }
        uturn();
    }
    else if (input == 0b11111111)
    {
        if (forward_ir == PATH)
        {
            printf("+\n");
            stop();
            HAL_Delay(2000);
            left();
        }
        else
        {
            printf("T\n");
            stop();
            HAL_Delay(2000);
            left();
        }
    }
    else if (((input & 0b01111110) == 0b01111110))
    {
        printf("end\n");
        stop();
    }
    else if ((input & 0b11110000) == 0b11110000)
    {
        if (forward_ir == NOT_PATH)
        {
            printf("left\n");
            stop();
            HAL_Delay(2000);
            left();
        }
        else
        {
            printf("LT\n");
            stop();
            HAL_Delay(2000);
            left();
        }
    }
    else if ((input & 0b00001111) == 0b00001111)
    {
        if (forward_ir == NOT_PATH)
        {
            printf("right\n");
            stop();
            HAL_Delay(2000);
            right();
        }
        else
        {
            printf("RT\n");
            stop();
            HAL_Delay(2000);
            forward();
        }
    }
    else
    {
        printf("forward\n");
        forward();
    }
}

void operate_robot()
{
    uint32_t prev_time = HAL_GetTick();
    while (1)
    {
        if ((HAL_GetTick() - uart_receive_time) > 500)
        {
            HAL_UART_Receive_DMA(&huart1, &bot.input, 1);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
        }
        if ((HAL_GetTick() - prev_time) > SAMPLE_TIME)
        {
            // printf("jbbf v  %f\n", bot.input);
            // if (fabs(bot.input) <= 0.0000001)
            // {
            printf("input %X\n", bot.input);

            //     printf("stop\n");

            //     bot.speed = 0;
            //     bot.motor_pid[0].Input = -bot.enc[0].get_omega();
            //     bot.motor_pid[1].Input = bot.enc[1].get_omega();
            //     bot.motor_pid[0].Setpoint = 0.0;
            //     bot.motor_pid[1].Setpoint = 0.0;
            //     // bot.motor[0].set_speed(0.0);
            //     // bot.motor[1].set_speed(0.0);

            //     bot.motor_pid[0].Compute();
            //     bot.motor_pid[1].Compute();
            //     bot.stop();
            //     continue;
            // }

            bot.speed = 5;
            bot.control();
            // bot.forward();
            prev_time = HAL_GetTick();
            // printf("output ::: %f\n", bot.stering_pid.Input);
            // printf("steering%lf\t%lf\t%lf\t%lf\n", bot.stering_pid.Input, bot.stering_pid.Setpoint, bot.stering_pid.Output, bot.speed);
            // printf("%ld\t%ld\n", bot.enc[0].get_count(), bot.enc[1].get_count());

            // printf("speeeeeeeeed:::%lf\t%lf\n", bot.enc[0].get_omega(), bot.enc[1].get_omega());
        }
    }
}