#ifndef     WPR_M_ROBOT_ITEM
#define     WPR_M_ROBOT_ITEM

#include <QGraphicsItem>
#include <QStyleOptionGraphicsItem>
#include <QPen>
#include <QStyle>
#include <QPainter>

class CRobotItem : public QGraphicsItem
{
    public:
        CRobotItem();
        ~CRobotItem();
        void SetID(int inID);
        void SetAngle(float inAngle);

        int m_nID;
        QString m_strID;
        int m_nRadius;
        float m_fAngle;
        bool bSelected;
        QColor clrOutline;
        QColor clrOutlineSelected;
        QColor clrID;
        QColor clrBackground;
        QColor clrBackgroundSelected;

    private:
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * /* widget */);
        QRectF boundingRect() const;
        QVariant itemChange(GraphicsItemChange change, const QVariant &value);
};

#endif //WPR_M_ROBOT_ITEM