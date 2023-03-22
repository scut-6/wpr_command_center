#include "../include/wpr_command_center/map_view.h"
#include <QtGui>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QFrame>
#include <math.h>

static const double Pi = 3.14159265358979323846264338327950288419717;

CMapView::CMapView(QWidget *parent)
{
    nMode = MAP_MODE_NORMAL;
    bRobotSelected = false;
    nSelectedRobotIndex = 0;
    bLeftPressed = false;
    zoom = 50;
	QObject::connect(&map_client, SIGNAL(signalMapUpdated()), this, SLOT(slotReadMap()));

    scene = new QGraphicsScene(this);
    setScene(scene);
    setCacheMode(CacheBackground);

    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    //setDragMode(QGraphicsView::ScrollHandDrag);           //手型拖动

    // 加载上一次保存的地图
    // bool res = map_client.LoadDefaultMap();
    // if(res == true)
    //     slotReadMap();

    QFrame *coordFrame = new QFrame;
    labelInfo = new QLabel;
    labelInfo->setStyleSheet("font-size:40px; color:yellow; background-color: gray;border-style: outset;border-width: 2px;border-radius: 10px;border-color: beige");
    labelInfo->setText("尚未接入ROS网络，请点击右侧面板按钮进行连接！");
    QGridLayout *grid = new QGridLayout;
    grid->addWidget(labelInfo, 0, 0);
    grid->setSizeConstraint(QLayout::SetFixedSize);
    coordFrame->setLayout(grid);

    QVBoxLayout *coordLayout = new QVBoxLayout;
    coordLayout->addWidget(coordFrame);
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(coordLayout);
    layout->addStretch();
    layout->setMargin(5);
    layout->setSpacing(3);
    setLayout(layout);

    // setMinimumSize(600,400);

    // 机器人item
    for(int i=0;i<ROBOT_NUM;i++)
    {
       arRobotItem[i].hide();
    }
    arRobotItem[0].SetID(1);
    arRobotItem[0].setPos(QPoint(0,0));
    scene->addItem(&arRobotItem[0]);
    arRobotItem[1].SetID(2);
    arRobotItem[1].setPos(QPoint(0,10000));
    scene->addItem(&arRobotItem[1]);
    arRobotItem[2].SetID(3);
    arRobotItem[2].setPos(QPoint(10000,0));
    arRobotItem[2].setRotation(45);
    arRobotItem[2].SetAngle(45);
    scene->addItem(&arRobotItem[2]);

    arRobotItem[3].SetID(4);
    arRobotItem[3].setPos(QPoint(10000,0));
    arRobotItem[3].setRotation(45);
    arRobotItem[3].SetAngle(45);
    scene->addItem(&arRobotItem[3]);

    arRobotItem[4].SetID(5);
    arRobotItem[4].setPos(QPoint(10000,0));
    arRobotItem[4].setRotation(45);
    arRobotItem[4].SetAngle(45);
    scene->addItem(&arRobotItem[4]);

    arRobotItem[5].SetID(6);
    arRobotItem[5].setPos(QPoint(10000,0));
    arRobotItem[5].setRotation(45);
    arRobotItem[5].SetAngle(45);
    scene->addItem(&arRobotItem[5]);

    arRobotItem[6].SetID(7);
    arRobotItem[6].setPos(QPoint(10000,0));
    arRobotItem[6].setRotation(45);
    arRobotItem[6].SetAngle(45);
    scene->addItem(&arRobotItem[6]);    

    // 箭头item
    arrow_item.setPos(QPoint(0,0));
    scene->addItem(&arrow_item);
    arrow_item.hide();
    
    // 路径item
    path_item.setPos(QPoint(0,0));
    path_item.setZValue(-9999);
    scene->addItem(&path_item);

    scene->clearSelection();
}

void CMapView::Initial()
{
    for(int i=0;i<ROBOT_NUM;i++)
    {
        robot_client[i].Initial();
    }
    robot_client[0].SetID(1);
    robot_client[0].SetFootprintName("wpb_mani_1/base_footprint");
    robot_client[1].SetID(2);
    robot_client[1].SetFootprintName("wpb_mani_2/base_footprint");
    robot_client[2].SetID(3);
    robot_client[2].SetFootprintName("wpb_mani_3/base_footprint"); 
    robot_client[3].SetID(4);
    robot_client[3].SetFootprintName("wpb_mani_4/base_footprint");
    robot_client[4].SetID(5);
    robot_client[4].SetFootprintName("wpb_mani_5/base_footprint");
    robot_client[5].SetID(6);
    robot_client[5].SetFootprintName("wpb_mani_6/base_footprint");
    robot_client[6].SetID(7);
    robot_client[6].SetFootprintName("wpb_mani_7/base_footprint");    

	QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(slotUpdateRobot()));
    timer.start(100);
}

void CMapView::SetMode(int inMode)
{
    nMode = inMode;
    switch(nMode)
    {
        case MAP_MODE_NORMAL:
            labelInfo->setText("");
            break;
        case MAP_MODE_NAVI_POSE:
            if(RobotSelected() == true)
            {
                arrow_item.penArrow = QPen(Qt::magenta, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
                labelInfo->setText("设置导航目标");
            }
            else
                labelInfo->setText("请先选择机器人");
            break;
        case MAP_MODE_ROBOT_POSE:
           if(RobotSelected() == true)
            {
                arrow_item.penArrow = QPen(Qt::green, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
                labelInfo->setText("修正机器人位置");
            }
            else
                labelInfo->setText("请先选择机器人");
            break;
        case MAP_MODE_DRAW_PATH:
            path_item.bDrawMouse = true;
            labelInfo->setText("绘制路径");
            path_item.nPathMode = PATH_SINGLE;
            scene->update();
            break;
        case MAP_MODE_PATH_TO:
            if(RobotSelected() == true)
            {
                arrow_item.penArrow = QPen(Qt::magenta, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
                labelInfo->setText("设置沿路径行走的目标");
             }
            else
                labelInfo->setText("请先选择机器人");
            break;
        default:
            arrow_item.penArrow = QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
            break;
    }
    if(inMode != MAP_MODE_DRAW_PATH)
    {
        path_item.bDrawMouse = false;
        scene->update();
    }
}

void CMapView::SetInfo(QString inStr)
{
    labelInfo->setText(inStr);
}

bool CMapView::RobotSelected()
{
    bRobotSelected = false;
    for(int i=0;i<ROBOT_NUM;i++)
    {
        if(arRobotItem[i].bSelected == true)
        {
            nSelectedRobotIndex = i;
            bRobotSelected = true;
            break;
        }
    }
    return bRobotSelected;
}

void CMapView::RobotSelectedAction(int inAction)
{
    // 判断是否有机器人被选中
    if(RobotSelected() == false)
        return;
    
    emit signalRobotAction(nSelectedRobotIndex + 1, inAction);
}

// read map information
void CMapView::slotReadMap()
{
    map_background = map_client.map;
    int width = map_background.width();
    int height = map_background.height();
    ROS_WARN("[CMapView] New map  = %d  x %d ",width,height);
    scene->setSceneRect(-width/2, -height/2, width, height);
    scene->update();
}

void CMapView::slotSaveMap()
{
    // 保存地图图片
    map_client.slotSaveMap();
    // 保存缩放系数和视口偏移（待实现）
}

void CMapView::slotZoom(int value)
{
    qreal s;
    if (value > zoom)
        s = pow(1.01, (value-zoom));
    else
        s = pow((1/1.01), (zoom-value));

    scale(s, s);
    zoom = value;
}

void CMapView::drawBackground(QPainter *painter, const QRectF &rect)
{
    painter->fillRect(rect,Qt::gray);
    painter->drawPixmap(int(sceneRect().left()), int(sceneRect().top()), map_background);
}

#define ZOOM_IN_TIMES       -30
#define ZOOM_OUT_TIMES  30
void CMapView::wheelEvent(QWheelEvent *event)
{
    qreal s=0;
    bool in = true;
    bool out = true; 
 
    if(event->delta() >= 0)//delta返回滑动的距离
    {
        scaleValue += 1;
    }
    if(event->delta() < 0)//delta返回滑动的距离
    {
        scaleValue -= 1;
    }
 
    if( scaleValue < ZOOM_IN_TIMES )
    {
        scaleValue = ZOOM_IN_TIMES;
        in = false;
        return;
    }
    else if( scaleValue > ZOOM_OUT_TIMES )
    {
        scaleValue = ZOOM_OUT_TIMES;
        out = false;
        return;
    }
 
    if(in)
    {
        s = qPow(1.01,event->delta()/10);
    }
    if(out)
    {
        s = qPow(1/1.01,-event->delta()/10);
    }
    qDebug()<<"scaleValue="<<scaleValue;
    scale(s,s);
}

void CMapView::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        bLeftPressed = true;
        if(nMode  == MAP_MODE_NAVI_POSE || nMode == MAP_MODE_ROBOT_POSE || nMode  == MAP_MODE_PATH_TO)
        {
            // 判断是否有机器人被选中
            bRobotSelected = false;
            for(int i=0;i<ROBOT_NUM;i++)
            {
                if(arRobotItem[i].bSelected == true)
                {
                    //ROS_WARN("[mousePressEvent] RobotIndex %d  selected!",i);
                    nSelectedRobotIndex = i;
                    bRobotSelected = true;
                    break;
                }
            }

            //RobotSelected状态下，点击第一次左键，进入显示朝向箭头状态
            if(bRobotSelected == true )
            {
                pntPressed = mapToScene(event->pos());
                arrow_item.pntSrc = pntPressed;
                arrow_item.pntDest = pntPressed;
                arrow_item.show();
                //ROS_WARN("Navi first point  (%.2f,%.2f)",pntPressed.x(),pntPressed.y());
            }
            else
            {
                 labelInfo->setText("请先选择机器人");
            }
        }

        if(nMode  == MAP_MODE_DRAW_PATH)
        {
            pntCurrent = mapToScene(event->pos());
            path_item.pathHolder.SetPoint(pntCurrent.x()*-0.01,pntCurrent.y()*0.01);
            scene->update();
            //ROS_WARN("MAP_MODE_DRAW_PATH (%.2f , %.2f ) size = %d",pntCurrent.x(),pntCurrent.y(), path_item.arPoint.size() );
        }
    }
     if(event->button() == Qt::RightButton)
    {
            SetMode(MAP_MODE_NORMAL);
    }
    QGraphicsView::mousePressEvent(event);
}

void CMapView::mouseMoveEvent(QMouseEvent *event)
{
    if(bLeftPressed == true)
    {
        if(nMode  == MAP_MODE_NAVI_POSE || nMode == MAP_MODE_ROBOT_POSE || nMode  == MAP_MODE_PATH_TO)
        {
            pntCurrent = mapToScene(event->pos());
            arrow_item.pntDest = pntCurrent;
            scene->update();
            //ROS_WARN("arrow angle = %.2f", CalArrowAngle(pntPressed,pntCurrent));
        }
    }

    if(nMode  == MAP_MODE_DRAW_PATH)
    {
        pntCurrent = mapToScene(event->pos());
        path_item.pntMouse = pntCurrent;
        scene->update();
        //ROS_WARN("[MouseMove] MAP_MODE_DRAW_PATH (%.2f , %.2f ) ",pntCurrent.x(),pntCurrent.y());
    }

    QGraphicsView::mouseMoveEvent(event);
}

void CMapView::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        bLeftPressed = false;
        if(nMode == MAP_MODE_NAVI_POSE)
        {
            pntCurrent = mapToScene(event->pos());
            //ROS_WARN("mouseReleaseEvent  (%.2f,%.2f) -> (%.2f,%.2f)",pntPressed.x(),pntPressed.y(),pntCurrent.x(),pntCurrent.y());
            // 发送导航坐标
            float pose_angle = CalArrowAngle(pntPressed,pntCurrent);
            //ROS_WARN("pose_angle = %.2f",pose_angle);
            emit signalSetNaviPos(arRobotItem[nSelectedRobotIndex].m_nID, pntPressed.x()*-0.01, pntPressed.y()*0.01,pose_angle );
            // 恢复到普通模式
            bRobotSelected = false;
            nMode = MAP_MODE_NORMAL;
            labelInfo->setText("");
            scene->clearSelection();
            arrow_item.hide();
            scene->update();
        }
        if(nMode == MAP_MODE_ROBOT_POSE)
        {
            pntCurrent = mapToScene(event->pos());
            //ROS_WARN("mouseReleaseEvent  (%.2f,%.2f) -> (%.2f,%.2f)",pntPressed.x(),pntPressed.y(),pntCurrent.x(),pntCurrent.y());
            // 发送机器人坐标
            float pose_angle = CalArrowAngle(pntPressed,pntCurrent);
            //ROS_WARN("pose_angle = %.2f",pose_angle);
            emit signalSetRobotPos(arRobotItem[nSelectedRobotIndex].m_nID, pntPressed.x()*-0.01, pntPressed.y()*0.01,pose_angle );
            // 恢复到普通模式
            bRobotSelected = false;
            nMode = MAP_MODE_NORMAL;
            labelInfo->setText("");
            scene->clearSelection();
            arrow_item.hide();
            scene->update();
        }
        if(nMode == MAP_MODE_PATH_TO)
        {
            pntCurrent = mapToScene(event->pos());
            //ROS_WARN("mouseReleaseEvent  (%.2f,%.2f) -> (%.2f,%.2f)",pntPressed.x(),pntPressed.y(),pntCurrent.x(),pntCurrent.y());
            // 发送移动目标坐标
            float pose_angle = CalArrowAngle(pntPressed,pntCurrent);
            emit signalSetPathTo(arRobotItem[nSelectedRobotIndex].m_nID, pntPressed.x()*-0.01, pntPressed.y()*0.01,pose_angle );
            // 恢复到普通模式
            bRobotSelected = false;
            nMode = MAP_MODE_NORMAL;
            labelInfo->setText("");
            scene->clearSelection();
            arrow_item.hide();
            scene->update();
        }
    }
    QGraphicsView::mouseReleaseEvent(event);
}

void CMapView::mouseDoubleClickEvent(QMouseEvent *event)
{
    QGraphicsView::mouseDoubleClickEvent(event);
}

void CMapView::keyPressEvent(QKeyEvent *event)
{
    //ROS_WARN("[CMapView]keyPressEvent");
    QGraphicsView::keyPressEvent(event);
}

// map scene coordinate to map
QPointF CMapView::mapToMap(QPointF p)
{
    QPointF latLon;
    qreal w = sceneRect().width();
    qreal h = sceneRect().height();
    qreal lon = y1 - ((h/2 + p.y())*abs(y1-y2)/h);
    qreal lat = x1 + ((w/2 + p.x())*abs(x1-x2)/w);
    latLon.setX(lat);
    latLon.setY(lon);

    return latLon;
}

float CMapView::CalArrowAngle(QPointF inSrc, QPointF inDest)
{
    float res = 0;
	int dx = inSrc.x() - inDest.x();
	float dy = inSrc.y() - inDest.y();
	if (dx == 0)
	{
		if (dy > 0)
		{
			res = 180 - 90;
		} 
		else
		{
			res = 0 - 90;
		}
	} 
	else
	{
		double fTan = dy / dx;
		res = atan(fTan)*180/Pi;
		if (dx < 0)
		{
			res = res - 180;
		}
	}
	res -= 90;
	if (res < 0)
	{
		res += 360;
	}
	if (res > 360)
	{
		res -= 360;
	}

    res *= Pi/180;
    res = 1.5*Pi -res;
    if(res < 0)
    {
        res += 2*Pi;
    }
	return res;
}
int CMapView::GetSelectedRobotID()
{
    int nRobotSelectedID = -1;
    for(int i=0;i<ROBOT_NUM;i++)
    {
        if(arRobotItem[i].bSelected == true)
        {
            nRobotSelectedID = i + 1;
            break;
        }
    }
    return nRobotSelectedID;
}

qreal CMapView::GetMinZValue()
{
    qreal qMinZ = 9999;
    for(int i=0;i<ROBOT_NUM;i++)
    {
        if(arRobotItem[i].zValue() <qMinZ)
            qMinZ = arRobotItem[i].zValue();
    }
    if(arrow_item.zValue() <qMinZ)
        qMinZ = arrow_item.zValue();
}

void CMapView::slotAddWaypoint(QString inStrName, int inType, float inMapX, float inMapY, float inMapYaw)
{
    ROS_WARN("[CMapView::slotAddWaypoint] %s t=%d (%.2f , %.2f) %.2f",inStrName.toStdString().c_str(),inType, inMapX, inMapY,inMapYaw);
    CWaypointItem* newWaypointItem = new CWaypointItem();
    newWaypointItem->SetName(inStrName);
    newWaypointItem->SetType(inType);
    newWaypointItem->setPos(QPoint(inMapX*-100,inMapY*100));
    float angle = -90 -inMapYaw*180/Pi;
    newWaypointItem->setRotation(angle);
    newWaypointItem->SetAngle(angle);
    if(inType == ITEM_TYPE_WAYPOINT)
        newWaypointItem->setOpacity(0.2);
    qreal qMinZ = GetMinZValue();
    newWaypointItem->setZValue(qMinZ-1);
    newWaypointItem->show();
    scene->addItem(newWaypointItem);
    scene->update();
}

void CMapView::slotUpdateRobot()
{
    for(int i=0;i<ROBOT_NUM;i++)
    {
        bool resGetPose = robot_client[i].GetPose();
        arRobotItem[i].setPos(QPoint(robot_client[i].pose_x*100,robot_client[i].pose_y*100));
        float angle = -90 -robot_client[i].pose_angle*180/Pi;
        arRobotItem[i].setRotation(angle);
        arRobotItem[i].SetAngle(angle);
        if(resGetPose == true &&arRobotItem[i].isVisible() == false)
        {
             arRobotItem[i].show();
        }
    }
}

bool CMapView::LoadPathFromFile(std::string inFilename)
{
    path_item.pathHolder.LoadPathFromFile(inFilename);
    path_item.nPathMode = PATH_SINGLE;
    scene->update();
}

void CMapView::RemovePath()
{
    path_item.pathHolder.arPoint.clear();
    path_item.pathHolder.strPathName = "";
    path_item.nPathMode = PATH_SINGLE;
    scene->update();
}
    
void CMapView::ShowAllPath()
{
    path_item.LoadAllPath();
    path_item.nPathMode = PATH_ALL;
    scene->update();
}