
#include "block.h"

Block::Block()
{

}

Block::Block(QPoint _pos, QPixmap _pixmap, float M):Polygon(_pos,_pixmap,M)
{
    setPoints(Point(_pos.x()+0.0,_pos.y()+0.0));
    setPoints(Point(_pos.x()+0.0,_pos.y()+50.0));
    setPoints(Point(_pos.x()+50.0,_pos.y()+50.0));
    setPoints(Point(_pos.x()+50.0,_pos.y()+0.0));
    caculateIP();
    setAngularV(0.0);
}

