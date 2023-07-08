
#ifndef ROPE_H
#define ROPE_H
#include "polygon.h"



class Rope:public Polygon
{
public:
    Rope();
    Rope(QPoint _pos, QPixmap _pixmap, float M = 1);
    ~Rope();
};

#endif // ROPE_H
