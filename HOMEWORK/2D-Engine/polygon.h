
#ifndef POLYGON_H
#define POLYGON_H
#include <QVector>
#include <QVector2D>
#include "point.h"
#include <QtMath>
#include <QGraphicsPixmapItem>
#include <QtMinMax>
#include "precisebouncejudge.h"
#include "Const.h"
struct forcement
{
public:
    Point force;
    Point actP;
    forcement():force(0,0),actP(0,0){}
    forcement(const Point& F,const Point& pos):force(F),actP(pos){}
    forcement(const QVector2D& F,const QVector2D& pos):force(F),actP(pos){}
    forcement(float fx,float fy,float px,float py):force(fx,fy),actP(px,py){}
    ~forcement(){};
};
class Polygon:public QGraphicsPixmapItem
{
public:
    QVector<Point> points;//构成多边形的点
    QVector<Point> companyPoints;//伴随多边形运动点
    QVector<forcement> forces;
    QPixmap map;//贴图
    QVector2D mapPos;
    QVector2D position;
    QVector2D velocity;
    float angularV;
    float Mass,Inertia;
    float inverseMass,inverseInertia;
    float left,right,up,down;//包围盒
    float S;//包围盒面积
    Polygon();
    Polygon(QPoint _pos, QPixmap _pixmap, float M = 1);
    ~Polygon();
    bool overlaps(const Polygon& q)const;//检测包围盒是否重叠
    void caculateIP();
    void setPoints(Point newPoint);
    void addPoints(Point newPoint);//添加伴随点
    void setVelocity(QVector2D v);
    void setAngularV(float w);
    void addForce(float fx,float fy,float px,float py);
    void addForce(const QVector2D& F,const QVector2D& ActP);
    void actForce();
    void move();
    bool checkBounce(const Polygon& object)const;
    void actBounce(Polygon& object);
};

#endif // POLYGON_H
