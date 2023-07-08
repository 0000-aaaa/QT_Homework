
#ifndef LONGBLOCK_H
#define LONGBLOCK_H
#include "polygon.h"



class LongBlock:public Polygon
{
public:
    LongBlock();
    LongBlock(QPoint _pos, QPixmap _pixmap, float M = 0.5);
};

#endif // LONGBLOCK_H
