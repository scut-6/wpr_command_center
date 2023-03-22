/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/wpr_command_center/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace wpr_command_center {

	using namespace Qt;

	/*****************************************************************************
	** Implementation [MainWindow]
	*****************************************************************************/

	MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
		: QMainWindow(parent)
		, qnode(argc,argv)
	{
		ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
		QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

		map_view = new CMapView(this);
		map_view->setGeometry(ui.map_view->rect());
		map_view->setMouseTracking(true);
		setCentralWidget(map_view);

		//ReadSettings();
		setWindowIcon(QIcon(":/images/icon.png"));
		//ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
		QObject::connect(map_view, SIGNAL(signalSetNaviPos(int,float,float,float)), &qnode, SLOT(slotSetNaviPose(int,float,float,float)));
		QObject::connect(map_view, SIGNAL(signalSetRobotPos(int,float,float,float)), &qnode, SLOT(slotSetRobotPose(int,float,float,float)));
		QObject::connect(map_view, SIGNAL(signalRobotAction(int,int)), &qnode, SLOT(slotRobotAction(int,int)));
		QObject::connect(&qnode, SIGNAL(signalAddWaypoint(QString, int, float, float, float)), map_view, SLOT(slotAddWaypoint(QString, int, float, float, float)));
		QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
		QObject::connect(map_view, SIGNAL(signalSetPathTo(int,float,float,float)), &qnode, SLOT(slotSetPathTo(int,float,float,float)));
		bQNodeInited = false;
		bStopCmd = false;

		cnt_timer = new QTimer(this);
		connect(cnt_timer,SIGNAL(timeout()),this,SLOT(slotCntTimerUp()));
		cnt_timer->start(1000);
	}

	MainWindow::~MainWindow() {}

	/*****************************************************************************
	** Implementation [Slots]
	*****************************************************************************/

	void MainWindow::showNoMasterMessage() 
	{
		QMessageBox msgBox;
		msgBox.setText("Couldn't find the ros master.");
		msgBox.exec();
		close();
	}

	/*
	* These triggers whenever the button is clicked, regardless of whether it
	* is already checked or not.
	*/

	// ROS和地图功能按钮
	void MainWindow::on_button_connect_clicked(bool check ) 
	{
		if ( !qnode.init() ) 
		{
			showNoMasterMessage();
			map_view->SetInfo("ROS网络接入失败！");
		} 
		else 
		{
			ui.button_connect->setEnabled(false);
			bQNodeInited = true;
			map_view->SetInfo("");
		}
		if(bQNodeInited == true)
		{
			map_view->Initial();
			map_view->map_client.slotGetMapFromMapServer();
			// 获取航点
			qnode.slotGetWaypoints();
		}
	}

	// void MainWindow::on_button_save_map_clicked(bool check ) 
	// {
	// 	if(bQNodeInited == false)
	// 		return;
	// 	map_view->slotSaveMap();
	// }

	void MainWindow::on_button_get_map_clicked(bool check ) 
	{
		if(bQNodeInited == false)
			return;
		map_view->map_client.slotGetMapFromMapServer();
	}

	// 路径
	void MainWindow::on_button_draw_path_clicked(bool check )
	{
		map_view->SetMode(MAP_MODE_DRAW_PATH);
	}
	void MainWindow::on_button_remove_path_clicked(bool check )
	{
		map_view->RemovePath();
	}
	void MainWindow::on_button_save_path_clicked(bool check )
	{
		std::string strPathDir = map_view->path_item.pathHolder.GetPathDir();
		QString strPathFile = QString::fromStdString(strPathDir + "/path.xml");
		QString strFileName = QFileDialog::getSaveFileName(this, tr("保存路径文件"),
														strPathFile,
														tr("路径文件(*xml)"),
														0,QFileDialog::DontUseNativeDialog);
		if(strFileName != "")
		{
			map_view->path_item.pathHolder.SavePathToFile(strFileName.toStdString());
			map_view->scene->update();
		}
	}
	void MainWindow::on_button_load_path_clicked(bool check )
	{
		QString strPathDir = QString::fromStdString(map_view->path_item.pathHolder.GetPathDir());
		QString strFileName = QFileDialog::getOpenFileName(this,
                                                    "读取路径文件",
                                                    strPathDir,
                                                     "路径文件(*xml)",
													 0,QFileDialog::DontUseNativeDialog);
		if(strFileName != "")
		{
			map_view->RemovePath();
			map_view->LoadPathFromFile(strFileName.toStdString());
		}
	}
	void MainWindow::on_button_test_path_clicked(bool check ) 
	{
		int nRobotID = map_view->GetSelectedRobotID();
		int nPathIndex = ui.path_index_input->text().toInt();
		qnode.slotPathFollow(nRobotID,nPathIndex);
	}
	void MainWindow::on_button_path_to_clicked(bool check ) 
	{
		if(bStopCmd == false)
			map_view->SetMode(MAP_MODE_PATH_TO);
	}
	void MainWindow::on_button_path_all_clicked(bool check ) 
	{
		map_view->ShowAllPath();
	}
	// 单体操作按钮
	void MainWindow::on_button_robot_pose_clicked(bool check ) 
	{
		if(bStopCmd == false)
			map_view->SetMode(MAP_MODE_ROBOT_POSE);
	}

	void MainWindow::on_button_navi_pose_clicked(bool check ) 
	{
		if(bStopCmd == false)
			map_view->SetMode(MAP_MODE_NAVI_POSE);
	}

	void MainWindow::on_button_robot_vel_clicked(bool check ) 
	{
		if(bStopCmd == false)
			map_view->RobotSelectedAction(ROBOT_ACT_TELEOP);
	}

	void MainWindow::on_button_robot_charge_clicked(bool check ) 
	{
		if(bStopCmd == false)
		{
			int nSelectedRobotID = map_view->GetSelectedRobotID();
			if(nSelectedRobotID > 0)
			{
				std::stringstream stream_id;
    			stream_id << nSelectedRobotID;
				std::string strChargerName = "c"+ stream_id.str();
				qnode.slotSpecialTask(nSelectedRobotID,"charging",strChargerName);
			}
		}
	}
	// 外部信号
	void MainWindow::on_button_signal_1_clicked(bool check )
	{
		if(bStopCmd == false)
			qnode.slotMission("signal_1");
	}
	void MainWindow::on_button_signal_2_clicked(bool check )
	{
		if(bStopCmd == false)
			qnode.slotMission("signal_2");
	}
	void MainWindow::on_button_signal_3_clicked(bool check )
	{
		if(bStopCmd == false)
			qnode.slotMission("signal_3");
	}
	// 群体策略
	void  MainWindow::on_button_stop_all_clicked(bool check ) 
	{
		bStopCmd = true;
		qnode.slotMission("stop");
		qnode.slotRobotAction(0,ROBOT_ACT_TELEOP);
		map_view->SetInfo("任务停止！请按键盘'S'键恢复...");
	}
	void MainWindow::on_button_pose_waypoint_clicked(bool check ) 
	{
			qnode.slotMission("pose_waypoint");
	}
	void  MainWindow::on_button_start_all_clicked(bool check ) 
	{
		if(bStopCmd == false)
			qnode.slotMission("start");
	}
	void MainWindow::on_button_continue_all_clicked(bool check ) 
	{
		qnode.slotMission("continue");
		map_view->SetInfo("");
		bStopCmd = false;
	}
	void  MainWindow::on_button_charge_all_clicked(bool check )
	{
		if(bStopCmd == false)
			qnode.slotMission("charge");
	}
	void  MainWindow::on_button_next_mission_clicked(bool check )
	{
		if(bStopCmd == false)
			qnode.slotMission("next_mission");
	}
	/*****************************************************************************
	** Implemenation [Slots][manually connected]
	*****************************************************************************/
	void MainWindow::slotCntTimerUp()
	{
		if(bQNodeInited == true)
		{
			cnt_timer->stop();
		}
		else
			on_button_connect_clicked(true);
	}
	/*****************************************************************************
	** Implementation [Menu]
	*****************************************************************************/

	void MainWindow::on_actionAbout_triggered() 
	{
		QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
	}

	/*****************************************************************************
	** Implementation [Configuration]
	*****************************************************************************/

	void MainWindow::ReadSettings() 
	{
		QSettings settings("Qt-Ros Package", "wpr_command_center");
		restoreGeometry(settings.value("geometry").toByteArray());
		restoreState(settings.value("windowState").toByteArray());
	}

	void MainWindow::WriteSettings() 
	{
		QSettings settings("Qt-Ros Package", "wpr_command_center");
		settings.setValue("geometry", saveGeometry());
		settings.setValue("windowState", saveState());
	}

	void MainWindow::keyPressEvent(QKeyEvent *event)
	{
		//ROS_WARN("[MainWindow]keyPressEvent");
		switch(event->key())
		{
			case Qt::Key_R:
				if(bStopCmd == false)
					map_view->SetMode(MAP_MODE_ROBOT_POSE);
				break;
			case Qt::Key_G:
				if(bStopCmd == false)
					map_view->SetMode(MAP_MODE_NAVI_POSE);
				break;
			case Qt::Key_P:
				if(bStopCmd == false)
					map_view->SetMode(MAP_MODE_PATH_TO);
				break;
			case Qt::Key_Escape:
				if(bStopCmd == false)
					map_view->SetMode(MAP_MODE_NORMAL);
				break;
			case Qt::Key_Space:	//不建议使用空格作为紧急停止按钮，因为点击界面按钮后，空格是该按钮的Pressed事件。
				//on_button_stop_all_clicked(true ); 
				break;
			case Qt::Key_Shift:	//使用Shift键作为紧急停止按钮
				on_button_stop_all_clicked(true ); 
				break;
			case Qt::Key_S:
				on_button_continue_all_clicked(true ) ;
				break;
		}
		QMainWindow::keyPressEvent(event);
	}

	void MainWindow::closeEvent(QCloseEvent *event)
	{
		WriteSettings();
		QMainWindow::closeEvent(event);
	}

	void MainWindow::resizeEvent(QResizeEvent * event)
	{
		// ROS_WARN("ui.map_widget = %d",ui.map_widget);
		// ROS_WARN("[main resize] ( %d , %d )  map_view(%d , %d)",(int)this->width(),this->height(),ui.map_widget->rect().width(),ui.map_widget->rect().height());
		// map_view->scene->setSceneRect(QRectF(QPointF(-1000, 1000), QSizeF(2000, 2000)));
		//map_view->setGeometry(ui.map_widget->rect());
		 map_view->update();
		// map_view->viewport()->update();
		// map_view->scene->invalidate(map_view->scene->sceneRect());
		// map_view->scene->update();
		// map_view->viewport()->repaint();
	}
}  // namespace wpr_command_center

