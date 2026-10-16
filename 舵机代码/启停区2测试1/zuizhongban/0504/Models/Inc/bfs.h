#ifndef _BFS_H
#define _BFS_H

#include "main.h"

// ---------- ��ͼ���� ----------
#define ROWS    5           // 地图行数           // ��ͼ����
#define COLS    5           // ��ͼ����
#define WALL    1           // �ϰ�����ֵ
#define EMPTY   0           // ��ͨ����ֵ
#define UNIT_DIS    41      // ʵ��һ����λ��Ӧ��ʵ�ʾ���(cm)
#define INTERVAL_DIS    15      //��������λ�õļ������

// ---------- λ�ö�Ӧ������ ----------
#define BFS_START_X 0
#define BFS_START_Y 4
#define BFS_START   BFS_START_X,BFS_START_Y

/* 启动校正完成后进入BFS的两个入口，同时也是回程出口。 */
#define START_ZONE1_BFS_X 0
#define START_ZONE1_BFS_Y 0
#define START_ZONE2_BFS_X 0
#define START_ZONE2_BFS_Y 4

/* 两个启停区都先到二维码区域，但粗定位模式不执行识别。 */
#define QR_SCAN_X 0
#define QR_SCAN_Y 2

/* BFS entry after correcting from stop/start zone 2. */
#define MATERIAL_COARSE_X 2
#define MATERIAL_COARSE_Y 0
#define ROUGH_COARSE_X 2
#define ROUGH_COARSE_Y 4
#define STORAGE_COARSE_X 4
#define STORAGE_COARSE_Y 2

#define PLACE_A     1,0
#define PLACE_B     3,0
#define PLACE_C     1,2
#define PLACE_D     3,2

#define PLACE_abc   2,4
#define PLACE_def   4,2


// BFS ���������������*��*2�㹻������ֱ������*�У�
#define MAX_QUEUE (ROWS * COLS)

// ����ṹ��
typedef struct {
    int x, y;
} Point;

typedef struct {
    int path_len;               // 路径总长度
    uint8_t path_index;               // 当前在路径中的索引位置
    Point pos;                  // 当前实时位置
    Point pos_tar;                  // 目标位置（终点）
    Point path[ROWS * COLS];    // 存储整条路径的坐标点，最大容量为地图面积（行×列）
}vBfsPath;

extern vBfsPath Path;

extern void CalPath(int start_x,int start_y,int end_x,int end_y);


#endif
