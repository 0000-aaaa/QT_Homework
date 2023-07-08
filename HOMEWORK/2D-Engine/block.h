
#ifndef BLOCK_H
#define BLOCK_H
#include "polygon.h"



class Block:public Polygon
{
public:
    Block();
    Block(QPoint _pos, QPixmap _pixmap, float M = 1);
};

#endif // BLOCK_H
