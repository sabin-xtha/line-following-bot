#ifndef MAZE_HPP
#define MAZE_HPP

#define MAX_NODES 100
#define TOLERANCE 0 // MM
#include <cmath>
enum DIRECTION
{
    WEST,
    NORTH,
    EAST,
    SOUTH
};

struct Point
{
    int x;
    int y;
};

class Maze
{
public:
    unsigned int node_array[MAX_NODES];
    int coordinate_array[MAX_NODES][2];
    unsigned int type_array[MAX_NODES]; // 0=deadend 1=turn 2=T 3=+
    unsigned int explored_array[MAX_NODES];
    int old_coordinate[2];
    unsigned int traverse_list[MAX_NODES];
    int total_nodes = 0;
    int direction = 0;
    int adjacencyList[1][4] = {0}; // x=each node, y=each 4parent nodes
    // first make node to all child nodes as no paths

    int count = 0;

    void init()
    {
        node_array[0] = 0;
        total_nodes = 1;
        coordinate_array[total_nodes][0] = 0;
        coordinate_array[total_nodes][1] = 0;
        traverse_list[count] = 0;
        old_coordinate[0] = 0;
        old_coordinate[1] = 0;
        count++;
        printf("maze init");
        printf("[");
        direction = NORTH;

        for (int i = 0; i < count; i++)
        {
            printf("%d,", traverse_list[i]);
        }
        printf("]");
    }

    void nodeFound(int type, int distance)
    {
        HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
        // printf("nodeFound\n");
        int oldindex = checkNode(distance);
        // printf("oldInd%d\n", oldindex);
        if (oldindex == -1)
        {
            newNode(type, distance);
        }
        else
        {
            oldNode(oldindex);
        }

        count++;
    }

    int distance(struct Point p1, struct Point p2)
    {
        if (p1.x == p2.x)
        {
            return abs(p1.y - p2.y);
        }
        else
        {
            return abs(p1.x - p2.x);
        }
    }

    void findadjacencylist()
    {
        int first = traverse_list[0], last = traverse_list[count - 1];
        struct Point p1, p2, p3;

        // first node coordinate
        p1.x = coordinate_array[0][0];
        p1.y = coordinate_array[0][1];

        // second node coordinates
        p2.x = coordinate_array[1][0];
        p2.y = coordinate_array[1][1];

        adjacencyList[first][0] = distance(p1, p2);

        // last node coordinate
        p1.x = coordinate_array[count - 1][0];
        p1.y = coordinate_array[count - 1][1];

        // Second last node coordinates
        p2.x = coordinate_array[count - 2][0];
        p2.y = coordinate_array[count - 2][1];

        adjacencyList[last][0] = distance(p1, p2);

        for (int i = 0; i < MAX_NODES - 2; i++)
        {
            int previous = traverse_list[i];
            int particular_point = traverse_list[i + 1];
            int next = traverse_list[i + 2];

            // Previous node coordinate
            p1.x = coordinate_array[i][0];
            p1.y = coordinate_array[i][1];

            // particular node coordinate
            p2.x = coordinate_array[i + 1][0];
            p2.y = coordinate_array[i + 1][1];

            // next node coordinate
            p3.x = coordinate_array[i + 2][0];
            p3.y = coordinate_array[i + 2][1];

            if (previous != next)
            {
                adjacencyList[particular_point][previous] = distance(p1, p2);
                adjacencyList[particular_point][next] = distance(p2, p3);
            }
            else
            {
                continue;
            }
        }
    }
    // check if this node is old or new;
    /*
    returns:
    true: node is new
    false: node is old
    */
    int checkNode(int distance)
    {
        // printf("checkNode\n");
        int current_coordinate[2];
        if (direction == NORTH)
        {
            current_coordinate[0] = old_coordinate[0];
            current_coordinate[1] = old_coordinate[1] + distance;
        }
        else if (direction == SOUTH)
        {
            current_coordinate[0] = old_coordinate[0];
            current_coordinate[1] = old_coordinate[1] - distance;
        }
        else if (direction == EAST)
        {
            current_coordinate[0] = old_coordinate[0] + distance;
            current_coordinate[1] = old_coordinate[1];
        }
        else if (direction == WEST)
        {
            current_coordinate[0] = old_coordinate[0] - distance;
            current_coordinate[1] = old_coordinate[1];
        }
        // printf("curr_coordx:%d,curr_coordy:%d\n", current_coordinate[0], current_coordinate[1]);
        for (int i = 0; i < total_nodes; i++)
        {
            int diffx = abs(current_coordinate[0] - coordinate_array[i][0]);
            int diffy = abs(current_coordinate[1] - coordinate_array[i][1]);
            if (diffx <= TOLERANCE && diffy <= TOLERANCE)
            {
                // printf("oldNodeHahaha%d\n",i);
                return i;
            }
        }
        // printf("newNodehahaha\n");
        return -1;
    }

    void oldNode(int oldindex)
    {
        // printf("oldNode\n");
        explored_array[oldindex] += 1;
        old_coordinate[0] = coordinate_array[oldindex][0];
        old_coordinate[1] = coordinate_array[oldindex][1];
        traverse_list[count] = oldindex;
    }
    void newNode(int type, int distance)
    {
        // printf("newNode\n");
        total_nodes += 1;
        explored_array[total_nodes-1] = 1;
        node_array[total_nodes-1] = total_nodes - 1;
        type_array[total_nodes-1] = type;
        if (direction == NORTH)
        {
            coordinate_array[total_nodes-1][0] = old_coordinate[0];
            coordinate_array[total_nodes-1][1] = old_coordinate[1] + distance;
        }
        else if (direction == SOUTH)
        {
            coordinate_array[total_nodes-1][0] = old_coordinate[0];
            coordinate_array[total_nodes-1][1] = old_coordinate[1] - distance;
        }
        else if (direction == EAST)
        {
            coordinate_array[total_nodes-1][0] = old_coordinate[0] + distance;
            coordinate_array[total_nodes-1][1] = old_coordinate[1];
        }
        else if (direction == EAST)
        {
            coordinate_array[total_nodes-1][0] = old_coordinate[0] - distance;
            coordinate_array[total_nodes-1][1] = old_coordinate[1];
        }
        old_coordinate[0] = coordinate_array[total_nodes-1][0];
        old_coordinate[1] = coordinate_array[total_nodes-1][1];
        traverse_list[count] = total_nodes-1;
    }
};

#endif