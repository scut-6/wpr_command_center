/**
 * @file /include/wpr_command_center/main_window.hpp
 *
 * @brief Qt based gui for wpr_command_center.
 *
 * @date November 2010
 **/
#ifndef wpr_command_center_MAIN_WINDOW_H
#define wpr_command_center_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "map_view.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace wpr_command_center {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void resizeEvent(QResizeEvent * event);
    void keyPressEvent(QKeyEvent *event);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	// ROS和地图功能按钮
	void on_button_connect_clicked(bool check );
	// void on_button_save_map_clicked(bool check );
	void on_button_get_map_clicked(bool check ) ;
	// 路径
	void on_button_draw_path_clicked(bool check ) ;
	void on_button_remove_path_clicked(bool check ) ;
	void on_button_save_path_clicked(bool check ) ;
	void on_button_load_path_clicked(bool check ) ;
	void on_button_test_path_clicked(bool check ) ;
	void on_button_path_to_clicked(bool check ) ;
	void on_button_path_all_clicked(bool check ) ;
	// 单体操作按钮
	void on_button_robot_pose_clicked(bool check ) ;
	void on_button_navi_pose_clicked(bool check ) ;
	void on_button_robot_vel_clicked(bool check ) ;
	void on_button_robot_charge_clicked(bool check ) ;
	// 外部信号
	void on_button_signal_1_clicked(bool check ) ;
	void on_button_signal_2_clicked(bool check ) ;
	void on_button_signal_3_clicked(bool check ) ;
	// 群体策略
	void on_button_stop_all_clicked(bool check ) ;
	void on_button_pose_waypoint_clicked(bool check ) ;
	void on_button_start_all_clicked(bool check ) ;
	void on_button_continue_all_clicked(bool check ) ;
	void on_button_charge_all_clicked(bool check ) ;
	void on_button_next_mission_clicked(bool check ) ;

    /******************************************
    ** Manual connections
    *******************************************/
	void slotCntTimerUp();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	bool bQNodeInited;
	CMapView* map_view;
	bool bStopCmd;
	QTimer *cnt_timer;
};

}  // namespace wpr_command_center

#endif // wpr_command_center_MAIN_WINDOW_H
