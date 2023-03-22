#include "../include/wpr_command_center/map_client.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <math.h>

static CMapClient* map_client = NULL;
CMapClient::CMapClient()
{
    map_client = this;
}

void CMapClient::slotGetMapFromMapServer()
{
    ros::NodeHandle nh;
    map_sub = nh.subscribe("/map",10,CMapClient::MapCallback);
	ros::spinOnce();
}

void CMapClient::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // 显示地图信息
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;
    ROS_WARN("Got map from map_server (w=%d h=%d)", info.width, info.height);
    // ROS_WARN("map resolution =  %.2f", info.resolution);
    // double quatx=  info.origin.orientation.x;
    // double quaty=  info.origin.orientation.y;
    // double quatz=  info.origin.orientation.z;
    // double quatw=  info.origin.orientation.w;
    // tf::Quaternion q(quatx, quaty, quatz, quatw);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    //ROS_WARN("map origin =  ( %.2f , %.2f ) rad= %.2f", info.origin.position.x, info.origin.position.y,yaw );

    //用来统计地图有效区域的变量
    int xMax,xMin,yMax,yMin ;
    xMax=xMin= info.width/2;
    yMax=yMin=  info.height/2;
    bool bFirstPoint = true;

    // 把地图数据转换成图片
    QImage map_raw(info.width, info.height,QImage::Format_RGB888);
    map_raw.fill(Qt::gray);
    for(int y =0 ; y<info.height ; y++)
        for(int x =0 ; x<info.width ; x++)
        {
            // 可通行区域
             if(msg->data[y*info.height+x]==0)
             {
                //map_raw.setPixelColor(x, y, Qt::white);
                map_raw.setPixel(x, y, qRgb(255,255,255));
             }
             // 障碍物
            if(msg->data[y*info.height+x]==100)
             {
                //map_raw.setPixelColor(x, y,  Qt::black);
                map_raw.setPixel(x, y, qRgb(0,0,0));
             }
             // 其他
             if(msg->data[y*info.height+x] > 0 && msg->data[y*info.height+x] < 100)
             {
                float value = (float)msg->data[y*info.height+x]*0.01;
                //map_raw.setPixelColor(x, y,  QColor(value,value,value,1.0));
                int nVal = value*255;
                map_raw.setPixel(x, y, qRgb(nVal,nVal,nVal));
             }
             // 统计有效区域
             if(msg->data[y*info.height+x] != -1)
             {
                 if(bFirstPoint == true)
                 {
                    xMax=xMin= x;
                    yMax=yMin= y;
                    bFirstPoint = false;
                    continue;
                 }
                 if(x < xMin) xMin = x;
                 if(x > xMax) xMax = x;
                 if(y < yMin) yMin = y;
                 if(y > yMax) yMax = y;
             }
        }
    //ROS_WARN("xMin= %d xMax=%d yMin=%d yMax=%d",xMin,xMax,yMin,yMax);
    
    // 按照有效区域对地图进行裁剪
    int new_half_width = abs(int(info.width/2 - xMin));
    if(abs(int(info.width/2 - xMax)) > new_half_width)
        new_half_width = abs(int(info.width/2 - xMax));
    int new_half_height = abs(int(info.height/2 - yMin));
    if(abs(int(info.height/2 - yMax)) > new_half_height)
        new_half_height = abs(int(info.height/2 -yMax));
    new_half_width += 20;
    new_half_height += 20;
    int new_origin_x =  info.width/2 - new_half_width;
    int new_origin_y = info.height/2 -new_half_height;
    //ROS_WARN("new_origin(%d , %d) new_half_width=%d new_half_height=%d",new_origin_x,new_origin_y,new_half_width,new_half_height);
    QImage roi_map = map_raw.copy(new_origin_x,new_origin_y,new_half_width*2,new_half_height*2);
    roi_map = roi_map.mirrored(true, false);

    //按照地图分辨率对图片进行缩放，使得每个像素对应1厘米
    float fMapScale = 1;
    if(info.resolution > 0)
        fMapScale =  0.25/info.resolution;
    //ROS_WARN("fMapScale= %.2f",fMapScale);
    QImage new_map = roi_map.scaled(new_half_width*2*fMapScale, new_half_height*2*fMapScale);
    
    // 传递出去给MapView 显示
    map_client->MapFromROS(new_map);
}

void CMapClient::MapFromROS(QImage inMap)
{
    map = QPixmap::fromImage(inMap);
    //ROS_WARN("map_raw (w=%d h=%d)  map (w=%d h=%d) ", inMap.width(), inMap.height() ,map.width(), map.height()  );
    emit signalMapUpdated();
}

void CMapClient::slotSaveMap()
{
    if(!map.isNull())
    {
        QFile file("./wpr_cmd_map.png");
        file.open(QIODevice::WriteOnly);
        bool res = map.save(&file, "PNG");
        if(res == false)
        {
            ROS_WARN("CMapClient::slotSaveMap failed!!");
        }
    }
}

bool CMapClient::LoadDefaultMap()
{
    bool result = true;
    //map.load(":/images/map.png");
    map.load("./wpr_cmd_map.png");
    if (map.isNull())
    {
        printf("map is null !!!\n");
        result = false;
    }
    return result;
}
