#include "../include/wpr_command_center/robot_item.h"

CRobotItem::CRobotItem()
{
    m_nID = 0;
    m_nRadius = 25;
    m_fAngle = 0;
    bSelected = false;
    clrOutline = Qt::black;
    clrOutlineSelected = Qt::red;
    clrID = Qt::yellow;
    clrBackground = Qt::blue;
    clrBackgroundSelected =Qt::darkGreen;
    setFlag(QGraphicsItem::ItemIsSelectable );
}

CRobotItem::~CRobotItem()
{

}

void CRobotItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * /* widget */)
{
    painter->setRenderHint(QPainter::Antialiasing, true);

    //绘制机器人朝向标记
    QPen penAngle(Qt::red);
    penAngle.setWidth(5);
    painter->setPen(penAngle);
    painter->drawLine(0, 0, 0, -(m_nRadius*2*0.8)); 

    QPen penOutline(clrOutline);
    penOutline.setWidth(2);
    painter->setBrush(clrBackground);
    if(option->state & QStyle::State_Selected)
    {
        penOutline.setWidth(4);
        penOutline.setColor(clrOutlineSelected);
        painter->setBrush(clrBackgroundSelected);
    }
    painter->setPen(penOutline);
    painter->drawEllipse(-m_nRadius,-m_nRadius,m_nRadius*2,m_nRadius*2);

    painter->setFont(QFont("Arial",24,QFont::Bold));
    painter->setPen(QPen(clrID));
    painter->save();
    painter->rotate(-m_fAngle);
    painter->drawText(-m_nRadius,-m_nRadius,m_nRadius*2,m_nRadius*2,Qt::AlignCenter,m_strID);
    painter->restore();

}

 void CRobotItem::SetID(int inID)
 {
    m_nID = inID;
    m_strID = QString("%1").arg(m_nID);
 }

void CRobotItem::SetAngle(float inAngle)
{
    m_fAngle = inAngle;
}

 QRectF CRobotItem::boundingRect() const
 {
     QRectF rect(-m_nRadius,-m_nRadius*2*0.9,m_nRadius*2,m_nRadius+m_nRadius*2*0.9);
     return rect;
 }

 QVariant CRobotItem::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (change == ItemSelectedChange ) 
    {
        bSelected = value.toBool();
        //printf("robot %d  is selected\n",m_nID);
    }
    return QGraphicsItem::itemChange(change, value);
}