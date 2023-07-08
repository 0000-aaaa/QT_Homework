
#ifndef FLOOR_H
#define FLOOR_H
#include "polygon.h"



class Floor:public Polygon
{
public:
    Floor();
    Floor(QPoint _pos, QPixmap _pixmap, float M = 0);
};

#endif // FLOOR_H
