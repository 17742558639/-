#include "bfs.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// 四个方向的偏移向量
const int DIRS[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// 地图数组，0=空地，1=墙
static int map[ROWS][COLS] = {
    {0, 0, 0, 0, 0},
    {0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0},
    {0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0}
};

vBfsPath Path = {.path_len = 0, .path_index = 1, .pos = {BFS_START_X, BFS_START_Y}};

// ----------------------------------------------------------------------
// bfs_shortest_path
// 参数
//   rows, cols     : 地图的行数和列数
//   map            : 地图数组，0=空地，1=墙
//   start, goal    : 起点和终点坐标
//   path           : 存储整条路径的坐标点数组
//   max_path_len   : path数组的最大长度
// 返回值
//   最短路径长度，-1表示没有路径
// ----------------------------------------------------------------------
int bfs_shortest_path(int rows, int cols, int map[rows][cols],
                      Point start, Point goal,
                      Point *path, int max_path_len)
{
    /* 坐标必须先检查，再访问map，防止异常任务导致数组越界。 */
    if (rows <= 0 || cols <= 0 || path == NULL || max_path_len <= 0)
        return -1;
    if (start.x < 0 || start.x >= rows || start.y < 0 || start.y >= cols)
        return -1;
    if (goal.x < 0 || goal.x >= rows || goal.y < 0 || goal.y >= cols)
        return -1;

    // 1. 检查起点和终点是否是墙
    if (map[start.x][start.y] == WALL || map[goal.x][goal.y] == WALL)
        return -1;
    
    // 2. 初始化
    bool visited[ROWS][COLS];      // 记录每个点是否被访问过
    Point parent[ROWS][COLS];      // 记录每个点的父节点坐标
    memset(visited, 0, sizeof(visited));
    memset(parent, -1, sizeof(parent));  // 初始化为(-1,-1)表示无父节点

    // 3. 初始化队列
    Point queue[MAX_QUEUE];
    int front = 0, rear = 0;

    // 初始化队列
    queue[rear++] = start;
    visited[start.x][start.y] = true;
    parent[start.x][start.y] = (Point){-1, -1};  // 初始化为无父节点

    // 4. BFS 循环遍历
    while (front < rear) {
        Point cur = queue[front++];

        // 如果到达目标位置
        if (cur.x == goal.x && cur.y == goal.y) {
            // 从路径坐标点数组中回溯路径
            Point temp[MAX_QUEUE];
            int step = 0;
            Point p = cur;
            while (p.x != -1 && p.y != -1) {
                temp[step++] = p;
                p = parent[p.x][p.y];
            }
            // 如果路径长度超过最大路径长度
            if (step > max_path_len) return -1;
            // 从路径坐标点数组中回溯路径，从目标位置到起点位置的顺序
            for (int i = 0; i < step; i++) {
                path[i] = temp[step - 1 - i];
            }
            return step;
        }

        // 遍历当前点的所有邻居点
        for (int i = 0; i < 4; i++) {
            int nx = cur.x + DIRS[i][0];
            int ny = cur.y + DIRS[i][1];

            // 如果邻居点超出地图范围
            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols)
                continue;
            // 如果邻居点是墙或已被访问过
            if (map[nx][ny] == WALL || visited[nx][ny])
                continue;

            // 访问邻居点，更新父节点坐标并加入队列
            visited[nx][ny] = true;
            parent[nx][ny] = cur;
            queue[rear++] = (Point){nx, ny};
        }
    }

    // 如果没有到达目标位置，返回-1表示没有路径
    return -1;
}


void CalPath(int start_x,int start_y,int end_x,int end_y) 
{
    /* 每次重新寻路前作废旧结果，失败时不能继续使用上一次路径。 */
    Path.path_len = -1;
    Path.path_index = 1;
    memset(Path.path, 0, sizeof(Path.path));

    // 初始化起点和终点坐标
    Point start = {start_x, start_y};
    Point goal  = {end_x, end_y};

    // 计算最短路径长度
    Path.path_len = bfs_shortest_path(ROWS, COLS, map, start, goal, Path.path, ROWS*COLS);
}
