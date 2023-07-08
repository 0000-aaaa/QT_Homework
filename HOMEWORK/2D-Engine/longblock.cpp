
#include "longblock.h"

LongBlock::LongBlock()
{

}

LongBlock::LongBlock(QPoint _pos, QPixmap _pixmap, float M):Polygon(_pos,_pixmap,M)
{
    setPoints(Point(_pos.x()+0.0,_pos.y()+0.0));
    setPoints(Point(_pos.x()+0.0,_pos.y()+250.0));
    setPoints(Point(_pos.x()+50.0,_pos.y()+250.0));
    setPoints(Point(_pos.x()+50.0,_pos.y()+0.0));
    caculateIP();
    setAngularV(0.0);//setVelocity(QVector2D(1,0));
}


