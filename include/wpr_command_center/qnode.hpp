/**
 * @file /include/wpr_command_center/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef wpr_command_center_QNODE_HPP_
#define wpr_command_center_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include "common.h"
#include <string>
#include <QThread>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>
#include <wpr_warehousing_monitor/RobotPose.h>
#include <wpr_warehousing_monitor/RobotData.h>
#include <waterplus_map_tools/GetNumOfWaypoints.h>
#include <waterplus_map_tools/GetWaypointByIndex.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace wpr_command_center {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	void SendArmTest(int inID, std::string inAction);
	stRobotIP arRobotIP[ROBOT_NUM];

Q_SIGNALS:
    void rosShutdown();
    void signalAddWaypoint(QString, int, float, float, float);

public slots:
	void slotSetNaviPose(int inID, float inX, float inY, float inYaw);
	void slotSetRobotPose(int inID, float inX, float inY, float inYaw);
	void slotMission(std::string inMission);
	void slotSpecialTask(int inID,std::string inCmd,std::string inData);
	void slotRobotAction(int inID, int inAction);
	void slotGetWaypoints();
	void slotPathFollow(int inID, int inPathIndex);
	void slotSetPathTo(int inID, float inX, float inY, float inYaw);

private:
	void init_ros_comm();
	void RobotIPCallback(const wpr_warehousing_monitor::RobotData::ConstPtr& msg);
	int init_argc;
	char** init_argv;
	ros::Subscriber robot_ip_sub;
	ros::Publisher chatter_publisher;
	ros::Publisher navi_pose_pub;
	ros::Publisher robot_pose_pub;
	ros::Publisher robot_teleop_pub;
	ros::Publisher wpr_mission_pub;
	ros::Publisher special_task_pub;
	ros::Publisher path_follow_pub;
	ros::Publisher path_to_pub;
	ros::ServiceClient cliGetWPNum;
    ros::ServiceClient cliGetWPIndex;
	ros::ServiceClient cliGetCHNum;
    ros::ServiceClient cliGetCHIndex;
	ros::Publisher arm_4_pub;
	ros::Publisher arm_5_pub;
};

}  // namespace wpr_command_center

#endif /* wpr_command_center_QNODE_HPP_ */
