#ifndef WPR_CMD_COMMON_H
#define WPR_CMD_COMMON_H

#include <string>

// 显示的机器人item数量
#define ROBOT_NUM       7

// 单体操作
#define ROBOT_ACT_TELEOP        1       //遥操作
#define ROBOT_ACT_CHARGE       2       //去充电

// 航点类型
#define ITEM_TYPE_WAYPOINT             1       // 普通航点
#define ITEM_TYPE_DOCK                       2       // 充电坞

typedef struct stRobotIP
{
    int nID;
    std::string strIP;
    bool bOnline;
}stRobotIP;

#endif