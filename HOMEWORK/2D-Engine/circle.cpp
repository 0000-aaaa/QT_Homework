
#include "circle.h"
#include <QtMath>
Circle::Circle()
{

}

Circle::Circle(QPoint _pos, QPixmap _pixmap, float M):Polygon(_pos,_pixmap,M)
{
    for (int i = 1; i < 36; ++i) {
        setPoints(Point(_pos.x()+50.0+50.0*qCos(3.14159/18*i),_pos.y()+50.0+50.0*qSin(3.14159/18*i)));
    }
    caculateIP();
    setAngularV(0.0);
    F = 0.1;
}

void Circle::controlMove(int direction)
{
    switch(direction)
    {
    case Qt::Key_W:addForce(0,-F,0,0);break;
    case Qt::Key_S:addForce(0,F,0,0);break;
    case Qt::Key_A:addForce(-F,0,0,0);break;
    case Qt::Key_D:addForce(F,0,0,0);break;
    default:break;
    }
    actForce();
}
