#include "../include/wpr_command_center/waypoint_item.h"

static const double Pi = 3.14159265358979323846264338327950288419717;
static double TwoPi = 2.0 * Pi;
static float arrowSize = 15.0;

static float fDockBase = 20.0;
static float fDockWing = 20.0;

CWaypointItem::CWaypointItem()
{
    m_nRadius = 15;
    m_fAngle = 0;
    clrOutline = Qt::magenta;
    clrName = Qt::blue;
    clrBackground = Qt::magenta;
}

CWaypointItem::~CWaypointItem()
{

}

void CWaypointItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * /* widget */)
{
    painter->setRenderHint(QPainter::Antialiasing, true);
    
    // 普通航点绘制
    if(m_nType == ITEM_TYPE_WAYPOINT)
    {
        //绘制航点朝向标记
        QPointF pntSrc = QPointF(0,-m_nRadius);
        QPointF pntDest = QPointF(0, -(m_nRadius*2.3));
        QLineF line(pntSrc,pntDest);
        QPen penAngle(clrOutline);
        penAngle.setWidth(2);
        painter->setPen(penAngle);
        painter->drawLine(line); 
        double angle = ::acos(line.dx() / line.length());
        if (line.dy() >= 0)
            angle = TwoPi - angle;
        QPointF destArrowP1 = pntDest + QPointF(sin(angle-Pi/3)*arrowSize,cos(angle-Pi/3)* arrowSize);
        QPointF destArrowP2 = pntDest + QPointF(sin(angle-Pi+Pi/3)*arrowSize,cos(angle-Pi+Pi/3)*arrowSize);
        painter->drawLine(QLineF(destArrowP1,pntDest));
        painter->drawLine(QLineF(destArrowP2,pntDest));

        QPen penOutline(clrOutline);
        penOutline.setWidth(2);
        painter->setBrush(clrBackground);
        painter->setPen(penOutline);
        painter->drawEllipse(-m_nRadius,-m_nRadius,m_nRadius*2,m_nRadius*2);

        painter->setFont(QFont("Arial",12,QFont::Bold));
        painter->setPen(QPen(clrName));
        painter->save();
        painter->rotate(-m_fAngle);
        painter->drawText(-m_nRadius,-m_nRadius,m_nRadius*2,m_nRadius*2,Qt::AlignCenter,m_strName);
        painter->restore();
    }

    // 充电坞绘制
    if(m_nType == ITEM_TYPE_DOCK)
    {
        // 背板
        QPointF pntLeft = QPointF(fDockBase,0);
        QPointF pntRight = QPointF(-fDockBase,0);
        QLineF line(pntLeft,pntRight);
        QPen penDock( Qt::blue);
        penDock.setWidth(5);
        painter->setPen(penDock);
        painter->drawLine(line); 
        double angle = Pi;
        // 左侧翼
        QPointF pntLeftWing = pntLeft + QPointF(sin(angle-Pi/6)*fDockWing,cos(angle-Pi/6)* fDockWing);
        painter->drawLine(QLineF(pntLeft,pntLeftWing));
         // 右侧翼
        QPointF pntRightWing = pntRight +  QPointF(sin(angle+Pi/6)*fDockWing,cos(angle+Pi/6)*fDockWing);
        painter->drawLine(QLineF(pntRight,pntRightWing));
        // 名称
        painter->setFont(QFont("Arial",12,QFont::Bold));
        painter->setPen(QPen(clrName));
        painter->save();
        painter->rotate(-m_fAngle);
        painter->drawText(-fDockBase,0,fDockBase*2,fDockBase,Qt::AlignCenter,m_strName);
        painter->restore();
    }

}

 void CWaypointItem::SetName(QString inName)
 {
    m_strName = inName;
 }

void CWaypointItem::SetType(int inType)
{
    m_nType = inType;
}

void CWaypointItem::SetAngle(float inAngle)
{
    m_fAngle = inAngle;
}

 QRectF CWaypointItem::boundingRect() const
 {
   if(m_nType == ITEM_TYPE_WAYPOINT)
    {
        QRectF rect(-m_nRadius,-m_nRadius*2.5,m_nRadius*2.5,m_nRadius+m_nRadius*2.5);
        return rect;
    }

    if(m_nType == ITEM_TYPE_DOCK)
    {
        QRectF rect(-fDockBase-fDockWing,-fDockWing,(fDockBase+fDockWing)*2,fDockWing*2);
        return rect;
    }
 }
