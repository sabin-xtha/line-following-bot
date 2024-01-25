#ifndef MAZE_HPP
#define MAZE_HPP

#define MAX_NODES 100

class Maze
{
public:
    unsigned int coordinate_array[MAX_NODES][2];
    unsigned int point_array[MAX_NODES];
    unsigned int type_array[MAX_NODES]; // 0=deadend 1=turn 2=T 3=+
    unsigned int explored_array[MAX_NODES];

    int total_points = 0;
    int direction = 0;
};

#endif