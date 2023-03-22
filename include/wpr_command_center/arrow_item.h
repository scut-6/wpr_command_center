#ifndef     WPR_M_ARROW_ITEM
#define     WPR_M_ARROW_ITEM

#include <QGraphicsItem>
#include <QStyleOptionGraphicsItem>
#include <QPen>
#include <QStyle>
#include <QPainter>
#include <math.h>

class CArrowItem : public QGraphicsItem
{
    public:
        CArrowItem();
        ~CArrowItem();
        QPointF pntSrc;
        QPointF pntDest;
        QPen penArrow;

    private:
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * /* widget */);
        QRectF boundingRect() const;
};

#endif //WPR_M_ARROW_ITEM