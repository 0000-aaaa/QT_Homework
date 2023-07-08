
#ifndef POINT_H
#define POINT_H
#include <QVector2D>
#include <QtMath>
#include "Const.h"
class Point:public QVector2D
{
public:
    float Rx,Ry;
    Point();
    Point(const Point& copy);
    Point(float x,float y);
    Point(const QVector2D& copy);
    ~Point();
    void setRXY(const QVector2D& centerP);
    void move(const QVector2D& centerP,const float angularV);
    void operator=(const Point& copy);
    void vertical();
    float cross(const Point& v)const;
};

#endif // POINT_H
