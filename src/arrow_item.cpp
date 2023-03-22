#include "../include/wpr_command_center/arrow_item.h"

static const double Pi = 3.14159265358979323846264338327950288419717;
static double TwoPi = 2.0 * Pi;
static float arrowSize = 15.0;

CArrowItem::CArrowItem()
{
    pntSrc = QPointF(0,0);
    pntDest = QPointF(0,0);
}

CArrowItem::~CArrowItem()
{

}

void CArrowItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * /* widget */)
{
    painter->setRenderHint(QPainter::Antialiasing, true);
    QLineF line(pntSrc,pntDest);
    if (qFuzzyCompare(line.length(), qreal(0.)))
    {
        return;
    }
    painter->setPen(penArrow);
    painter->drawLine(line);
    double angle = ::acos(line.dx() / line.length());
    if (line.dy() >= 0)
        angle = TwoPi - angle;
    QPointF destArrowP1 = pntDest + QPointF(sin(angle-Pi/3)*arrowSize,cos(angle-Pi/3)* arrowSize);
    QPointF destArrowP2 = pntDest + QPointF(sin(angle-Pi+Pi/3)*arrowSize,cos(angle-Pi+Pi/3)*arrowSize);
    painter->drawLine(QLineF(destArrowP1,pntDest));
    painter->drawLine(QLineF(destArrowP2,pntDest));
}

 QRectF CArrowItem::boundingRect() const
 {
    float left = pntSrc.x();
    if(left > pntDest.x())
        left = pntDest.x();
    float top = pntSrc.y();
    if(top > pntDest.y())
        top = pntDest.y();
    float right = pntSrc.x();
    if(right < pntDest.x())
        right = pntDest.x();
    float bottom = pntSrc.y();
    if(bottom < pntDest.y())
        bottom = pntDest.y();
    float width = right - left;
    float height = bottom - top;
    QRectF rect(left,top,width,height);
     return rect;
 }
