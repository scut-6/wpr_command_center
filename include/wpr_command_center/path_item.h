#ifndef     WPR_M_PATH_ITEM
#define     WPR_M_PATH_ITEM

#include <dirent.h>
#include <unistd.h>
#include <QGraphicsItem>
#include <QStyleOptionGraphicsItem>
#include <QPen>
#include <QStyle>
#include <QVector>
#include <QPainter>
#include<QTime>
#include <math.h>
#include "PathHolder.h"

#define PATH_SINGLE 0
#define PATH_ALL        1

class CPathItem : public QGraphicsItem
{
    public:
        CPathItem();
        ~CPathItem();
        void LoadAllPath();
        QPen penPath;
        QPen penMouse;
        CPathHolder pathHolder;
        QVector<CPathHolder> arPathHolder;
        QVector<QPen> arPathPen;
        int nPathMode;
        QPointF pntLast;
        QPointF pntCur;
        QPointF pntMouse;
        bool bDrawMouse;

    private:
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * /* widget */);
        QRectF boundingRect() const;
        std::string GetPathDir();
};

#endif //WPR_M_PATH_ITEM