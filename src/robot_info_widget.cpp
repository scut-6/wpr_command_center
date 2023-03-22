#include "../include/wpr_command_center/robot_info_widget.h"

CRobotInfoWidget::CRobotInfoWidget()
{

}

void CRobotInfoWidget::paintEvent(QPaintEvent * event)
{
    QPainter painter(this);
    qreal width = this->width();
    qreal height = this->height();
    //ROS_WARN("width = %d     height = %d",(int)this->width(),this->height());

    painter.setRenderHints(QPainter::Antialiasing|QPainter::TextAntialiasing);
    painter.setPen(QColor(60,60,255));
    painter.setBrush(QColor(255, 160, 90));
    painter.drawRect(0, 0, width, height);

    // 设置字体：微软雅黑、点大小50、斜体
    QFont font;
    font.setFamily("Microsoft YaHei");
    font.setPointSize(50);
    painter.setFont(font);

    // 绘制文本
    painter.drawText(rect(), Qt::AlignCenter, "Robot Info");
}

void CRobotInfoWidget::resizeEvent(QResizeEvent * event)
{

}