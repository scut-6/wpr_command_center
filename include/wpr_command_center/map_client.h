#ifndef WPR_MAP_CLIENT_H_
#define WPR_MAP_CLIENT_H_

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include <QObject>
#include <QPixmap>
#include <QFile>

class CMapClient : public QObject
{
     Q_OBJECT
    public:
        CMapClient();
        static void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void MapFromROS(QImage inMap);
        bool LoadDefaultMap();
        QPixmap map;
        ros::Subscriber map_sub;
    
    signals:
        void signalMapUpdated();

    public slots:
        void slotSaveMap();
        void slotGetMapFromMapServer();
};

#endif //WPR_MAP_CLIENT_H_