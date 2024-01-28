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

unsigned int turn[100];
int turncount = 0;
void Robot::forward()
{
    printf("'forward\n");
    int nl_inp = input;
    linear_input = 0.0;
    int high_bit = 0;
    for (int i = 0; i < 8; i++)
    {
        if (((nl_inp >> i) & 0b00000001))
        {
            linear_input -= 3.5 - (i);
            high_bit++;
        }
    }
    if (high_bit)
    {
        linear_input /= high_bit;
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
    motor[0].set_speed((motor_pid[0].Output) / 50); // right 0
    motor[1].set_speed((motor_pid[1].Output) / 50); // left 1
    printf("input:%lf\tsetpoint:%lf\toutput:%lf", motor_pid[0].Input, motor_pid[0].Setpoint, motor_pid[0].Output);

    // printf("pid_outputs:::%lf\t%lf\t%f\n", (motor_pid[0].Output - output) / 50, (motor_pid[1].Output + output) / 50, output);
    // printf("motor speeds:::%f\t%f\n", motor_pid[0].Input, motor_pid[1].Input);
}
void Robot::backward(int delay)
{
    printf("backward\n");
    motor[0].set_speed(-0.45); // right 0
    motor[1].set_speed(-0.45); // left 1
    HAL_Delay(delay);
}
void Robot::left()
{
    printf("left\n");
    prev_count1 = enc[0].get_count();
    uint32_t prev_time = HAL_GetTick();
    while (abs(enc[0].get_count() - prev_count1) < 1350)
    {
        if ((HAL_GetTick() - prev_time) > SAMPLE_TIME)
        {
            // printf("left, %d\n", enc[0].get_count());

            motor_pid[0].Setpoint = 15;
            motor_pid[1].Setpoint = 0.0;
            motor_pid[0].Input = -enc[0].get_omega();
            motor_pid[1].Input = -enc[1].get_omega();
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
    printf("right\n");
    prev_count2 = enc[1].get_count();
    uint32_t prev_time = HAL_GetTick();
    while (abs(enc[1].get_count() - prev_count2) < 1225)
    {
        if ((HAL_GetTick() - prev_time) > SAMPLE_TIME)
        {
            // printf("left, %d\", enc[1].get_count());

            motor_pid[0].Setpoint = 0.0;
            motor_pid[1].Setpoint = 15;
            motor_pid[0].Input = -enc[0].get_omega();
            motor_pid[1].Input = -enc[1].get_omega();
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
    printf("uturn\n");
    prev_count1 = enc[0].get_count();
    prev_count2 = enc[1].get_count();

    uint32_t prev_time = HAL_GetTick();
    uint8_t bitval = 0b00001100;
    while (((input & bitval)) != bitval)
    {
        // while (abs(enc[0].get_count() - prev_count1) < 1610 && abs(enc[1].get_count() - prev_count2) < 1610){
        if ((HAL_GetTick() - prev_time) > SAMPLE_TIME)
        {
            // printf("left, %d\n", enc[0].get_count());

            motor_pid[0].Setpoint = -10;
            motor_pid[1].Setpoint = 10;
            motor_pid[0].Input = -enc[0].get_omega();
            motor_pid[1].Input = -enc[1].get_omega();
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

    HAL_UART_Receive_DMA(&huart2, &input, 1);
    uart_receive_time = HAL_GetTick();

    motor_pid[0] = PID(1, 0, 0, P_ON_E, REVERSE);
    motor_pid[0].Init();
    motor_pid[0].SetOutputLimits(-40, 40);
    motor_pid[0].SetTunings(1.8, 15, 0.03);
    motor_pid[0].SetSampleTime(SAMPLE_TIME);
    motor_pid[0].SetMode(AUTOMATIC);

    motor_pid[1] = PID(1, 0, 0, P_ON_E, REVERSE);
    motor_pid[1].Init();
    motor_pid[1].SetOutputLimits(-40, 40);
    motor_pid[1].SetTunings(1.5, 13, 0.03);
    motor_pid[1].SetSampleTime(SAMPLE_TIME);
    motor_pid[1].SetMode(AUTOMATIC);

    stering_pid = PID(1, 0, 0, P_ON_E, DIRECT);
    stering_pid.Init();
    stering_pid.SetOutputLimits(-(speed / 3.0), (speed / 3.0));
    stering_pid.SetTunings(0.8, 0, 0.4); //(4.5, .5, 1);
    stering_pid.SetSampleTime(SAMPLE_TIME);
    stering_pid.SetMode(AUTOMATIC);

    motor[0] = Motor(&htim2, GPIOB, TIM_CHANNEL_4, GPIO_PIN_10, GPIOB, GPIO_PIN_11, 65535);
    motor[1] = Motor(&htim2, GPIOE, TIM_CHANNEL_1, GPIO_PIN_7, GPIOE, GPIO_PIN_14, 65535);
    motor[0].init();
    motor[1].init();
    enc[0] = Encoder(&htim4, 1425);
    enc[1] = Encoder(&htim1, 1425);
    enc[0].init();
    enc[1].init();

    maze.init();
    printf("init\n");
}

Robot bot;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart2.Instance)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
        HAL_UART_Receive_DMA(&huart2, &bot.input, 1);

        uart_receive_time = HAL_GetTick();
    }
}
void init_robot()
{
    bot.init();
}
int cnt = 0;
bool sabai_high = false;
int dead_band = 0;
int prev_sensor = 0;
void Robot::control()
{
    if (prev_sensor == bot.input)
    {
        dead_band++;
    }
    else
    {
        dead_band = 0;
    }
    prev_sensor = bot.input;

    if (input == 0b11111111)
    {
        forward();
        sabai_high = true;
        cnt++;
        return;
    }
    // printf("%d\n", dead_band);
    if (dead_band < 5)
    {
        forward();
        return;
    }
    dead_band = 0;

    GPIO_PinState leftorright = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    int distance_moved = 10;

    if (sabai_high && cnt < 20)
    {
        stop();
        HAL_Delay(100);
        backward(300);
        sabai_high = false;
        cnt = 0;
        printf("+\n");
        maze.nodeFound(3, distance_moved);
        left();
        stop();
        HAL_Delay(100);
        // backward(200);

        turn_array[turn_count] = LEFT;
        turn_count++;

        maze.direction = (maze.direction + 3) % 4;
    }
    else if (cnt > 20)
    {
        sabai_high = false;
        cnt = 0;
        // }
        // else if (((input & 0b01111110) == 0b01111110))
        // {
        printf("end\n");
        for (int i = 0; i < turn_count; i++)
        {
            printf("turn array [%d] :: %d\n", i, turn_array[i]);
        }
        if (button_count < 50)
        {
            stop();
            backward(300);
            stop();
            // optimize until button is pressed
        }

        optimize();

        for (int i = 0; i < turn_count; i++)
        {
            printf("after optimize turn array [%d] :: [%d]\n", i, turn_array[i]);
        }
        runpath();
    }

    else if (input == 0b00000000)
    {
        printf("dead end\n");
        // dead end
        maze.nodeFound(0, distance_moved);
        uturn();
        turn_array[turn_count] = UTURN;
        turn_count++;

        stop();
        HAL_Delay(100);
        maze.direction = (maze.direction + 2) % 4;
    }
    else if ((input & 0b11110000) == 0b11110000)
    {
        if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == NOT_PATH) && (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == NOT_PATH))
        {
            printf("left\n");
            maze.nodeFound(1, distance_moved);

            stop();
            HAL_Delay(100);
            backward(200);
            left();
            stop();
            HAL_Delay(100);

            // backward(200);

            turn_array[turn_count] = LEFT;
            turn_count++;
            maze.direction = (maze.direction + 3) % 4;
        }
        else
        {
            printf("LT\n");
            maze.nodeFound(2, distance_moved);
            if (leftorright == GPIO_PIN_SET)
            {
                // stop();
                // HAL_Delay(100);
                backward(200);
                left();
                stop();
                HAL_Delay(100);
                // backward(200);
            }
            else
            {
                forward();
            }
            turn_array[turn_count] = LEFT;
            turn_count++;

            maze.direction = (maze.direction + 3) % 4;
        }
    }
    else if ((input & 0b00001111) == 0b00001111)
    {
        if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == NOT_PATH) && (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == NOT_PATH))
        {
            printf("right\n");
            maze.nodeFound(1, distance_moved);
            stop();
            HAL_Delay(100);
            backward(200);
            right();
            stop();
            HAL_Delay(100);
            // backward(200);

            turn_array[turn_count] = RIGHT;
            turn_count++;

            maze.direction = (maze.direction + 1) % 4;
        }
        else
        {
            printf("RT\n");
            maze.nodeFound(2, distance_moved);
            if (leftorright == GPIO_PIN_SET)
            {
                forward();
            }
            else
            {
                stop();
                HAL_Delay(100);
                backward(200);
                right();
                stop();
                HAL_Delay(100);
            }
            turn_array[turn_count] = FORWARD;
            turn_count++;
        }
    }
    else
    {
        // printf("forward\n");
        forward();
    }
    // HAL_Delay(500);
}
void Robot::optimize()
{
    // for (int i = 0; i < OPTIMIZATION_LEVEL; i++)
    // {

    while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET))
    {
        int temp_turn[100];
        int temp_turn_count = 0;
        for (int i = 0; i < turn_count; i++)
        {
            if (turn_array[i] == UTURN)
            {
                if (turn_array[i - 1] == turn_array[i + 1])
                {
                    temp_turn[temp_turn_count - 1] = FORWARD;
                    i++;
                    continue;
                }
                if ((turn_array[i - 1] == RIGHT && turn_array[i + 1] == LEFT) || (turn_array[i - 1] == LEFT && turn_array[i + 1] == RIGHT))
                {
                    temp_turn[temp_turn_count - 1] = UTURN;
                    i++;
                    continue;
                }
                if (turn_array[i - 1] == FORWARD && turn_array[i + 1] == RIGHT)
                {
                    temp_turn[temp_turn_count - 1] = LEFT;
                    i++;
                    continue;
                }
                if (turn_array[i - 1] == FORWARD && turn_array[i + 1] == LEFT)
                {
                    temp_turn[temp_turn_count - 1] = RIGHT;
                    i++;
                    continue;
                }
                if (turn_array[i - 1] == LEFT && turn_array[i + 1] == FORWARD)
                {
                    temp_turn[temp_turn_count - 1] = RIGHT;
                    i++;
                    continue;
                }
                if (turn_array[i - 1] == RIGHT && turn_array[i + 1] == FORWARD)
                {
                    temp_turn[temp_turn_count - 1] = LEFT;
                    i++;
                    continue;
                }
            }
            temp_turn[temp_turn_count] = turn_array[i];
            temp_turn_count++;
        }
        for (int i = 0; i < temp_turn_count; i++)
        {
            turn_array[i] = temp_turn[i];
        }
        turn_count = temp_turn_count;
    }
}
void Robot::runpath()
{
    printf("runpath\n");
    is_end = false;
    int index = 0;
    uint32_t prev_time = HAL_GetTick();
    sabai_high = false;
    cnt = 0;
    node_time = HAL_GetTick();
    while (!is_end)
    {
        if (prev_sensor == bot.input)
        {
            dead_band++;
        }
        else
        {
            dead_band = 0;
        }
        prev_sensor = bot.input;
        if ((HAL_GetTick() - prev_time) > SAMPLE_TIME)
        {
            prev_time = HAL_GetTick();
            if (input == 0b11111111)
            {
                sabai_high = true;
                cnt++;
                forward();
                continue;
            }
            // if ((HAL_GetTick() - node_time) < 250)
            // {
            //     forward();
            //     printf("cont ");
            //     continue;
            // }
            // node_time = HAL_GetTick();

            if (dead_band < 5)
            {
                forward();
                return;
            }
            dead_band = 0;

            // for (int i = 0; i < 8; i++)
            // {
            //     printf("%x", (bot.input & 0b10000000 >> i) && 0b10000000 >> i);
            // }
            // printf("\n");

            if (cnt > 20)
            {
                is_end = true;
                printf("stop\n");
                stop();
                while (1)
                    ;
            }
            else if (input == 0b00000000 || input == 0b11111111 ||
                     (input & 0b11110000) == 0b11110000 ||
                     (input & 0b00001111) == 0b00001111 || (cnt < 20 && sabai_high))
            {
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
                printf("hereee [%d] :: %d \n", i, turn_array[i]);
                if (turn_array[index] == UTURN)
                {
                    printf("uturn\n");
                    stop();
                    HAL_Delay(100);
                    uturn();
                }
                else if (turn_array[index] == RIGHT)
                {
                    printf("right\n");
                    stop();
                    HAL_Delay(100);
                    backward(200);
                    right();
                    stop();
                    HAL_Delay(100);
                }
                else if (turn_array[index] == LEFT)
                {
                    printf("left\n");
                    stop();
                    HAL_Delay(100);
                    backward(200);
                    left();
                    stop();
                    HAL_Delay(100);
                }
                else if (turn_array[i] == FORWARD)
                {
                    printf("forward\n");
                    forward();
                }
                index++;
                cnt = 0;
                sabai_high = false;
            }
            else
            {
                printf("forward\n");
                forward();
            }
        }
    }
}
void operate_robot()
{
    uint32_t prev_time = HAL_GetTick();
    while (1)
    {
        if ((HAL_GetTick() - uart_receive_time) > 100)
        {
            // HAL_UART_Receive_DMA(&huart1, &bot.input, 1);
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        }
        if ((HAL_GetTick() - prev_time) > SAMPLE_TIME)
        {
            for (int i = 0; i < 8; i++)
            {
                printf("%x", (bot.input & 0b10000000 >> i) && 0b10000000 >> i);
            }
            printf("\n");
            // if (fabs(bot.input) <= 0.0000001)
            // {
            // printf("\nir :%d ,input %X\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4), bot.input);

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
            // if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET){
            //     bot.button_count++;
            // }

            bot.speed = 10;

            // bot.control();

            // bot.turn_array[0] = 1;
            // bot.turn_array[1] = 1;
            // bot.turn_array[2] = 2;
            // bot.runpath();
            // bot.turn_array[0] = 1;

            bot.forward();
            // printf("count:%d\n",bot.enc[1].get_count());
            prev_time = HAL_GetTick();
            // printf("output ::: %f\n", bot.stering_pid.Input);
            // printf("steering%lf\t%lf\t%lf\t%lf\n", bot.stering_pid.Input, bot.stering_pid.Setpoint, bot.stering_pid.Output, bot.speed);
            // printf("%ld\t%ld\n", bot.enc[0].get_count(), bot.enc[1].get_count());
        }
    }
}