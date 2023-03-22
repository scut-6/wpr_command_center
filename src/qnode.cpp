/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <sstream>
#include "../include/wpr_command_center/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace wpr_command_center 
{

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
init_argc(argc),
init_argv(argv)
{
	for(int i=0;i<ROBOT_NUM;i++)
	{
		arRobotIP[i].nID = i+1;
		arRobotIP[i].bOnline = false;
	}
}

QNode::~QNode() 
{
    if(ros::isStarted()) 
	{
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() 
{
	ros::init(init_argc,init_argv,"wpr_command_center");
	if ( ! ros::master::check() ) 
	{
		return false;
	}
	init_ros_comm();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) 
{
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"wpr_command_center");
	if ( ! ros::master::check() ) 
	{
		return false;
	}
	init_ros_comm();
	return true;
}

void QNode::init_ros_comm()
{
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	//chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	navi_pose_pub =  n.advertise<wpr_warehousing_monitor::RobotPose>("/wpr_warehousing/ext_navi_pose", 10);
	robot_pose_pub =  n.advertise<wpr_warehousing_monitor::RobotPose>("/wpr_warehousing/ext_robot_pose", 10);
	robot_teleop_pub = n.advertise<std_msgs::Int64>("/wpr_warehousing/teleop_robot_id", 10);
	robot_ip_sub = n.subscribe<wpr_warehousing_monitor::RobotData>("/wpr_warehousing/robot_ip",100,&QNode::RobotIPCallback,this);
	wpr_mission_pub = n.advertise<std_msgs::String>("/wpr_warehousing/mission", 10 );
	special_task_pub = n.advertise<wpr_warehousing_monitor::RobotData>("/wpr_warehousing/special_task",10);
	path_follow_pub = n.advertise<std_msgs::Int32MultiArray>("/wpr_warehousing/ext_path_follow",10);
	path_to_pub =  n.advertise<wpr_warehousing_monitor::RobotPose>("/wpr_warehousing/ext_path_to", 10);
	cliGetWPNum = n.serviceClient<waterplus_map_tools::GetNumOfWaypoints>("/waterplus/get_num_waypoint");
    cliGetWPIndex = n.serviceClient<waterplus_map_tools::GetWaypointByIndex>("/waterplus/get_waypoint_index");
	cliGetCHNum = n.serviceClient<waterplus_map_tools::GetNumOfWaypoints>("/waterplus/get_num_charger");
    cliGetCHIndex = n.serviceClient<waterplus_map_tools::GetWaypointByIndex>("/waterplus/get_charger_index");
	arm_4_pub =  n.advertise<std_msgs::String>("/arm_4/test", 10);
	arm_5_pub =  n.advertise<std_msgs::String>("/arm_5/test", 10);
	start();
}

void QNode::run() 
{
	ros::Rate loop_rate(1);
	while ( ros::ok() ) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::RobotIPCallback(const wpr_warehousing_monitor::RobotData::ConstPtr& msg)
{
	if(msg->robot_id <= 0 || msg->robot_id > ROBOT_NUM)
		return;
	int nIndex = msg->robot_id-1;
	if(arRobotIP[nIndex].bOnline == false)
	{
		arRobotIP[nIndex].strIP = msg->data.data;
		arRobotIP[nIndex].bOnline = true;
		ROS_WARN("[QNode::RobotIPCallback] ID= %d  IP= %s",(int)msg->robot_id,arRobotIP[nIndex].strIP.c_str());
	}
}

void QNode::slotSetNaviPose(int inID, float inX, float inY, float inYaw)
{
	wpr_warehousing_monitor::RobotPose msg;
	msg.robot_id = inID;
	msg.x = inX;
	msg.y = inY;
	msg.yaw = inYaw;
	navi_pose_pub.publish(msg);
}

void QNode::slotSetPathTo(int inID, float inX, float inY, float inYaw)
{
	wpr_warehousing_monitor::RobotPose msg;
	msg.robot_id = inID;
	msg.x = inX;
	msg.y = inY;
	msg.yaw = inYaw;
	path_to_pub.publish(msg);
}

void QNode::slotSetRobotPose(int inID, float inX, float inY, float inYaw)
{
	wpr_warehousing_monitor::RobotPose msg;
	msg.robot_id = inID;
	msg.x = inX;
	msg.y = inY;
	msg.yaw = inYaw;
	robot_pose_pub.publish(msg);
}

void QNode::slotMission(std::string inMission)
{
	std_msgs::String msg;
	msg.data = inMission.c_str();
	wpr_mission_pub.publish(msg);
}

void QNode::slotSpecialTask(int inID,std::string inCmd,std::string inData)
{
	wpr_warehousing_monitor::RobotData msg;
	msg.robot_id = inID;
	msg.cmd.data = inCmd;
	msg.data.data = inData;
	special_task_pub.publish(msg);
}

void QNode::slotRobotAction(int inID, int inAction)
{
	if(inAction == ROBOT_ACT_TELEOP)
	{
		std_msgs::Int64 msg;
		msg.data = inID;
		robot_teleop_pub.publish(msg);
	}
}

void QNode::slotGetWaypoints()
{
	// 普通航点
	 waterplus_map_tools::GetNumOfWaypoints srvNum;
    if (cliGetWPNum.call(srvNum))
    {
        ROS_INFO("Num_wp = %d", (int)srvNum.response.num);
    }
    else
    {
        ROS_ERROR("Failed to call service get_num_waypoints");
    }
    waterplus_map_tools::GetWaypointByIndex srvWpIndex;
    for(int i=0;i<srvNum.response.num;i++)
    {
        srvWpIndex.request.index = i;
        if (cliGetWPIndex.call(srvWpIndex))
        {
            std::string name = srvWpIndex.response.name;
            float x = srvWpIndex.response.pose.position.x;
            float y = srvWpIndex.response.pose.position.y;
			tf::Quaternion q(srvWpIndex.response.pose.orientation.x,srvWpIndex.response.pose.orientation.y,srvWpIndex.response.pose.orientation.z,srvWpIndex.response.pose.orientation.w);
			float yaw = tf::getYaw(q);
            //ROS_INFO("Get_wp_index: name = %s (%.2f,%.2f)  %.2f", name.c_str(),x,y,yaw);
			emit signalAddWaypoint(QString(name.c_str()),ITEM_TYPE_WAYPOINT,x,y,yaw);
        }
        else
        {
            ROS_ERROR("Failed to call service get_wp_index");
        }
    }
	// 充电坞
	waterplus_map_tools::GetNumOfWaypoints srvChNum;
    if (cliGetCHNum.call(srvChNum))
    {
        ROS_INFO("Num_ch = %d", (int)srvChNum.response.num);
    }
    else
    {
        ROS_ERROR("Failed to call service get_num_waypoints");
    }
    waterplus_map_tools::GetWaypointByIndex srvChIndex;
    for(int i=0;i<srvChNum.response.num;i++)
    {
        srvChIndex.request.index = i;
        if (cliGetCHIndex.call(srvChIndex))
        {
            std::string name = srvChIndex.response.name;
            float x = srvChIndex.response.pose.position.x;
            float y = srvChIndex.response.pose.position.y;
			tf::Quaternion q(srvChIndex.response.pose.orientation.x,srvChIndex.response.pose.orientation.y,srvChIndex.response.pose.orientation.z,srvChIndex.response.pose.orientation.w);
			float yaw = tf::getYaw(q);
            //ROS_INFO("Get_ch_index: name = %s (%.2f,%.2f)  %.2f", name.c_str(),x,y,yaw);
			emit signalAddWaypoint(QString(name.c_str()),ITEM_TYPE_DOCK,x,y,yaw);
        }
        else
        {
            ROS_ERROR("Failed to call service get_ch_index");
        }
    }
}

void QNode::slotPathFollow(int inID, int inPathIndex)
{
	std_msgs::Int32MultiArray msg;
	msg.data.push_back(inID);
	msg.data.push_back(inPathIndex);
	path_follow_pub.publish(msg);
}

void QNode::SendArmTest(int inID, std::string inAction)
{
	if(inID == 4)
	{
		std_msgs::String msg;
		msg.data = inAction;
		arm_4_pub.publish(msg);
	}
	if(inID == 5)
	{
		std_msgs::String msg;
		msg.data = inAction;
		arm_5_pub.publish(msg);
	}
}

}  // namespace wpr_command_center
