#include "../include/wpr_command_center/robot_info_client.h"
#include <math.h>

CRobotInfoClient::CRobotInfoClient()
{

}

void CRobotInfoClient::Initial()
{
    m_tfListener = new  tf::TransformListener();
}

 void CRobotInfoClient::SetID( int inID)
 {
     m_nID = inID;
 }

int CRobotInfoClient::GetID()
{
    return m_nID;
}

void CRobotInfoClient::SetFootprintName( std::string inName)
{
    m_strFootprint = inName;
}

bool CRobotInfoClient::GetPose()
{
    bool res = true;
    tf::StampedTransform transform;
    m_tfListener->waitForTransform("/map",m_strFootprint,ros::Time(0),ros::Duration(0.1));
    try{
        m_tfListener->lookupTransform("/map",m_strFootprint,ros::Time(0),transform); 
    }
    catch (tf::TransformException ex)
    {
        //ROS_ERROR("%s",ex.what());
        //ros::Duration(0.1).sleep();
        return false;
    }
    pose_x = -transform.getOrigin().x();
    pose_y = transform.getOrigin().y();
    //ROS_WARN("%s  pose = ( %.2f , %.2f)",m_strFootprint.c_str(),pose_x,pose_y);

     double quatx=  transform.getRotation().getX();
    double quaty=  transform.getRotation().getY();
    double quatz=  transform.getRotation().getZ();
    double quatw=  transform.getRotation().getW();
    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose_angle = yaw;
    //ROS_WARN("%s  angle = %.2f ",m_strFootprint.c_str(),pose_angle);
    return res;
}
