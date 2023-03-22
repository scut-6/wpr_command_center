#ifndef     WPR_M_WAYPOINT_ITEM
#define     WPR_M_WAYPOINT_ITEM

#include <QGraphicsItem>
#include <QStyleOptionGraphicsItem>
#include <QPen>
#include <QStyle>
#include <QPainter>
#include <math.h>
#include "common.h"

class CWaypointItem : public QGraphicsItem
{
    public:
        CWaypointItem();
        ~CWaypointItem();
        void SetName(QString inName);
        void SetType(int inType);
        void SetAngle(float inAngle);

        QString m_strName;
        int m_nType;
        int m_nRadius;
        float m_fAngle;
        QColor clrOutline;
        QColor clrName;
        QColor clrBackground;

    private:
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * /* widget */);
        QRectF boundingRect() const;
};

#endif //WPR_M_WAYPOINT_ITEM