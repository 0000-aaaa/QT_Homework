
#include "rope.h"

Rope::Rope()
{

}

Rope::Rope(QPoint _pos, QPixmap _pixmap, float M):Polygon(_pos,_pixmap,M)
{
    setPoints(Point(_pos.x()+0.0,_pos.y()+0.0));
    setPoints(Point(_pos.x()+0.0,_pos.y()+25.0));
    setPoints(Point(_pos.x()+5.0,_pos.y()+25.0));
    setPoints(Point(_pos.x()+5.0,_pos.y()+0.0));
    caculateIP();
    setAngularV(0.0);
}

Rope::~Rope()
{

}

