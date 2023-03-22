#ifndef ROBOT_INFO_WIDGET_H
#define ROBOT_INFO_WIDGET_H

#include <QWidget>
#include <QPainter>

class CRobotInfoWidget : public QWidget
{
     Q_OBJECT
public:
    CRobotInfoWidget();

protected:
    virtual void paintEvent(QPaintEvent * event);
    virtual void resizeEvent(QResizeEvent * event);
};

#endif //ROBOT_INFO_WIDGET_H