
#ifndef CIRCLE_H
#define CIRCLE_H
#include "polygon.h"



class Circle:public Polygon
{
public:
    float F;
    Circle();
    Circle(QPoint _pos, QPixmap _pixmap, float M = 1);
    void controlMove(int direction);
};

#endif // CIRCLE_H
