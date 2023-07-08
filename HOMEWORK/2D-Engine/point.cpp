
#include "point.h"
#include <QDebug>
Point::Point():QVector2D(0,0),Rx(0),Ry(0){}

Point::Point(const Point &copy)
{
    setX(copy.x());
    setY(copy.y());
    Rx = copy.Rx;
    Ry = copy.Ry;
}

Point::Point(float x,float y):Rx(0),Ry(0)
{
    setX(x);
    setY(y);
}

Point::Point(const QVector2D &copy)
{
    setX(copy.x());
    setY(copy.y());
    Rx = 0;
    Ry = 0;
}

Point::~Point(){}

void Point::setRXY(const QVector2D &centerP)
{
    QVector2D R = (*this)-centerP;
    Rx = R.x();
    Ry = R.y();
}

void Point::move(const QVector2D &centerP, const float angularV)
{
    float tempx,tempy;
    tempx =Rx*qCos(angularV*DTIME) - Ry*qSin(angularV*DTIME);
    tempy =Rx*qSin(angularV*DTIME) + Ry*qCos(angularV*DTIME);
    QVector2D WR(tempx,tempy);
    QVector2D temp = centerP + WR;
    Rx = tempx;
    Ry = tempy;
    setX(temp.x());
    setY(temp.y());
    //qDebug()<<"PPPPPPP"<<temp;
}

void Point::operator=(const Point &copy)
{
    setX(copy.x());
    setY(copy.y());
    Rx = copy.Rx;
    Ry = copy.Ry;
}

void Point::vertical()
{
    float tempY = y();
    float tempX = x();
    setX(-tempY);
    setY(tempX);
}

float Point::cross(const Point &v) const
{
    return x()*v.y()-y()*v.x();
}

