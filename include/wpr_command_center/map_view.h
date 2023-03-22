#ifndef WPR_MAP_VIEW_H
#define WPR_MAP_VIEW_H

#include <QGraphicsView>
#include <QTimer>
#include <QFileDialog>
#include "common.h"
#include "map_client.h"
#include "robot_info_widget.h"
#include "robot_item.h"
#include  "robot_info_client.h"
#include "arrow_item.h"
#include "waypoint_item.h"
#include "path_item.h"

class QPixmap;
class QLabel;
class QPointF;

#define MAP_MODE_NORMAL            0
#define MAP_MODE_NAVI_POSE       1
#define MAP_MODE_ROBOT_POSE  2
#define MAP_MODE_DRAW_PATH     3
#define MAP_MODE_PATH_TO            4

class CMapView : public QGraphicsView
{
    Q_OBJECT
public:
    CMapView(QWidget *parent = Q_NULLPTR);
    void Initial();
    QPointF mapToMap(QPointF);
    float CalArrowAngle(QPointF inSrc, QPointF inDest);
    void SetMode(int inMode);
    void SetInfo(QString inStr);
    bool RobotSelected();
    void RobotSelectedAction(int inAction);
    qreal GetMinZValue();
    int GetSelectedRobotID();
    bool LoadPathFromFile(std::string inFilename);
    void RemovePath();
    void ShowAllPath();
    QGraphicsScene *scene;
    CMapClient map_client;
    CRobotItem arRobotItem[ROBOT_NUM];
    CRobotInfoClient robot_client[ROBOT_NUM];
    CPathItem path_item;

public slots:
    void slotZoom(int value);
    void slotReadMap();
    void slotSaveMap();
    void slotUpdateRobot();
    void slotAddWaypoint(QString inStrName, int inType, float inMapX, float inMapY, float inMapYaw);

signals:
    void signalSetNaviPos(int,float,float,float);
    void signalSetRobotPos(int,float,float,float);
    void signalRobotAction(int,int);
    void signalSetPathTo(int,float,float,float);

protected:
    void drawBackground(QPainter *painter, const QRectF &rect);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);

private:
    QPixmap map_background;
    QTimer timer;
    qreal zoom;
    QLabel *labelInfo;

    double x1, y1;
    double x2, y2;    
    int scaleValue;

    int nMode;
    bool bRobotSelected;
    CArrowItem arrow_item;
    bool bLeftPressed;
    QPointF pntPressed;
    QPointF pntCurrent;
    int nSelectedRobotIndex;
};

#endif // WPR_MAP_VIEW_H