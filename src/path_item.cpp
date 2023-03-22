#include "../include/wpr_command_center/path_item.h"

static const double Pi = 3.14159265358979323846264338327950288419717;
static double TwoPi = 2.0 * Pi;
static float arrowSize = 15.0;

CPathItem::CPathItem()
{
    qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
    penPath.setWidth(2); 
    penPath.setColor(Qt::red);
    penMouse.setWidth(1); 
    penMouse.setColor(Qt::red);
    penMouse.setStyle(Qt::DashLine);
    bDrawMouse = false;
    nPathMode = PATH_SINGLE;
}

CPathItem::~CPathItem()
{

}

std::string CPathItem::GetPathDir()
{
    std::string strPathDir;
    char const* home = getenv("HOME");
    strPathDir = home;
    strPathDir += "/WPR_Path";
    return strPathDir;
}

 void CPathItem::LoadAllPath()
 {
     arPathHolder.clear();
     arPathPen.clear();
    // 读取目录下的所有路径
    DIR *dir;
    struct dirent *ptr;
    char base[1000];
    std::string basePath =  GetPathDir();
    if ((dir=opendir(basePath.c_str())) == NULL)
    {
        ROS_WARN("[PathFinder] %s 路径文件夹打开失败 ... \n",basePath.c_str());
    }
    else
    {
        while ((ptr=readdir(dir)) != NULL)
        {
            if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
                continue;
            else if(ptr->d_type == 8)    ///file
            {
                std::string filename(ptr->d_name);
                CPathHolder path_holder;
                path_holder.LoadPathFromFile(basePath + "/" + filename);
                arPathHolder.push_back(path_holder);
                QPen penTmp;
                penTmp.setWidth(2); 
                penTmp.setColor(QColor(qrand()%200, qrand()%200, qrand()%200, 255));
                arPathPen.push_back(penTmp);
            }
        }
        closedir(dir);
    }
 }

void CPathItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * /* widget */)
{
    QFont font;
    // font.setFamily("Microsoft YaHei");
    font.setPointSize(20);
    painter->setFont(font);
    QFontMetrics fm = painter->fontMetrics();

    // [1] 单条路径
    if(nPathMode == PATH_SINGLE)
    {
        if(pathHolder.arPoint.size() >0)
        {
            painter->setRenderHint(QPainter::Antialiasing, true);
            painter->setPen(penPath); 

            float tmp_x = pathHolder.arPoint[0].x*-100;
            float tmp_y = pathHolder.arPoint[0].y*100;
            // 路径起始点画个圆圈
            float rStart =  10;
            QRectF rectStart(tmp_x - rStart, tmp_y - rStart, rStart*2, rStart*2);
            painter->drawEllipse(rectStart);

            // 写上路径名称
            QRectF rectTxt(tmp_x -  fm.width(pathHolder.strPathName)/2 , tmp_y - rStart - fm.height() , fm.width(pathHolder.strPathName) ,fm.height());
            painter->drawText(rectTxt,pathHolder.strPathName);

            // 画路径点之间的连线
            pntLast = QPointF(tmp_x,tmp_y);
            int nNumOfPoints = pathHolder.arPoint.size();
            for(int i=1;i<nNumOfPoints;i++)
            {
                float tmp_x = pathHolder.arPoint[i].x*-100;
                float tmp_y = pathHolder.arPoint[i].y*100;
                pntCur =  QPointF(tmp_x,tmp_y);
                QLineF line(pntLast,pntCur);
                painter->drawLine(line);
                // 路径末端画个箭头
                if(i == nNumOfPoints-1)
                {
                    double angle = ::acos(line.dx() / line.length());
                    if (line.dy() >= 0)
                        angle = TwoPi - angle;
                    QPointF destArrowP1 = pntCur + QPointF(sin(angle-Pi/3)*arrowSize,cos(angle-Pi/3)* arrowSize);
                    QPointF destArrowP2 = pntCur + QPointF(sin(angle-Pi+Pi/3)*arrowSize,cos(angle-Pi+Pi/3)*arrowSize);
                    painter->drawLine(QLineF(destArrowP1,pntCur));
                    painter->drawLine(QLineF(destArrowP2,pntCur));
                }
                pntLast = pntCur;
            }
            if(bDrawMouse == true)
            {
                painter->setPen(penMouse); 
                QLineF line(pntLast,pntMouse);
                painter->drawLine(line);
            }
        }
    }
    // [2] 所有路径
    if(nPathMode == PATH_ALL)
    {
        painter->setRenderHint(QPainter::Antialiasing, true);
        int nNumOfPath = arPathHolder.size();
        if(nNumOfPath > 0)
        {
            for(int i=0; i<nNumOfPath ; i++)
            {
                int nNumOfPoints = arPathHolder[i].arPoint.size();
                if(nNumOfPoints > 0)
                {
                    painter->setPen(arPathPen[i]); 
                    float tmp_x = arPathHolder[i].arPoint[0].x*-100;
                    float tmp_y = arPathHolder[i].arPoint[0].y*100;
                    // 路径起始点画个圆圈
                    float rStart =  10;
                    QRectF rectStart(tmp_x - rStart, tmp_y - rStart, rStart*2, rStart*2);
                    painter->drawEllipse(rectStart);

                    // 写上路径名称
                    QRectF rectTxt(tmp_x -  fm.width(arPathHolder[i].strPathName)/2 , tmp_y - rStart - fm.height() , fm.width(arPathHolder[i].strPathName) ,fm.height());
                     painter->drawText(rectTxt,arPathHolder[i].strPathName);

                    // 画路径点之间的连线
                    pntLast = QPointF(tmp_x,tmp_y);
                    
                    for (int j=0; j< nNumOfPoints ; j++)
                    {
                        float tmp_x = arPathHolder[i].arPoint[j].x*-100;
                        float tmp_y = arPathHolder[i].arPoint[j].y*100;
                        pntCur =  QPointF(tmp_x,tmp_y);
                        QLineF line(pntLast,pntCur);
                        painter->drawLine(line);
                        // 路径末端画个箭头
                        if(j == nNumOfPoints-1)
                        {
                            double angle = ::acos(line.dx() / line.length());
                            if (line.dy() >= 0)
                                angle = TwoPi - angle;
                            QPointF destArrowP1 = pntCur + QPointF(sin(angle-Pi/3)*arrowSize,cos(angle-Pi/3)* arrowSize);
                            QPointF destArrowP2 = pntCur + QPointF(sin(angle-Pi+Pi/3)*arrowSize,cos(angle-Pi+Pi/3)*arrowSize);
                            painter->drawLine(QLineF(destArrowP1,pntCur));
                            painter->drawLine(QLineF(destArrowP2,pntCur));
                        }
                        pntLast = pntCur;
                     }
                }
            }
        }
    }
}

 QRectF CPathItem::boundingRect() const
 {
    float left =0;
    float right = 0;
    float top = 0;
    float bottom = 0;
    if(pathHolder.arPoint.size() >0)
    {
        for(int i=0;i<pathHolder.arPoint.size();i++)
        {
            float tmp_x = pathHolder.arPoint[i].x*-100;
            float tmp_y = pathHolder.arPoint[i].y*100;
            if(left > tmp_x)
                left = tmp_x;
            if(top > tmp_y)
                top = tmp_y;
            if(right < tmp_x)
                right = tmp_x;
            if(bottom < tmp_y)
                bottom = tmp_y;
        }
         if(bDrawMouse == true)
         {
            float tmp_x = pntMouse.x();
            float tmp_y = pntMouse.y();
            if(left > tmp_x)
                left = tmp_x;
            if(top > tmp_y)
                top = tmp_y;
            if(right < tmp_x)
                right = tmp_x;
            if(bottom < tmp_y)
                bottom = tmp_y;
         }
    }
    float width = right - left;
    float height = bottom - top;
    QRectF rect(left,top,width,height);
     return rect;
 }
