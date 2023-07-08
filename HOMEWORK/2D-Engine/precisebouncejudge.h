
#ifndef PRECISEBOUNCEJUDGE_H
#define PRECISEBOUNCEJUDGE_H
#include "point.h"
#include <QVector>
struct Minkowski_t
{
    Point m_body1;
    Point m_body2;
    Point m_result;
    Minkowski_t(const Point& body1,const Point& body2,const Point& result):
        m_body1(body1),m_body2(body2),m_result(result){}
    Minkowski_t(){};
    Minkowski_t(const Minkowski_t& copy){
        m_body1 = copy.m_body1;
        m_body2 = copy.m_body2;
        m_result = copy.m_result;
    }
    void operator=(const Minkowski_t& copy){
        m_body1 = copy.m_body1;
        m_body2 = copy.m_body2;
        m_result = copy.m_result;
    }
};

class PreciseBounceJudge
{
public:
    QVector<Minkowski_t> s;
    QVector<Minkowski_t> Nearsest;
    QVector<Point> bouncePoint;
    Point U;
    int mFind(QVector<Point> sim,Point p);
    PreciseBounceJudge();
    ~PreciseBounceJudge();
    Minkowski_t compare(Point v21,Point v22,Point v1);
    Point support(const QVector<Point>& shape,const Point& direction);
    Point nearestSimplex();
    bool GJK(const QVector<Point>& p,const QVector<Point>& q,Point init_direction);
    void EPA(const QVector<Point>& p,const QVector<Point>& q);
    void createBouncePoint();
    void createBouncePointPair(const QVector<Point>& p,const QVector<Point>& q);
};

#endif // PRECISEBOUNCEJUDGE_H

