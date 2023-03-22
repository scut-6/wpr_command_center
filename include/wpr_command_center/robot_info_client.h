#ifndef WPR_ROBOT_INFO_CLIENT_H_
#define WPR_ROBOT_INFO_CLIENT_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <QObject>

class CRobotInfoClient : public QObject
{
     Q_OBJECT
    public:
        CRobotInfoClient();
        void Initial();
        void SetID( int inID);
        int GetID();
        void SetFootprintName( std::string inName);
        bool GetPose();

        float pose_x;
        float pose_y;
        float pose_angle;
    private:
        int m_nID;
        std::string m_strFootprint;
        tf::TransformListener* m_tfListener;
};

#endif //WPR_ROBOT_INFO_CLIENT_H_