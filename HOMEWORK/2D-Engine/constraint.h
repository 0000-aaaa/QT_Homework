
#ifndef CONSTRAINT_H
#define CONSTRAINT_H
#include "bvh.h"
#include "precisebouncejudge.h"
#include "polygon.h"
#include <QVector>
struct Constraints{
    Polygon* body[2];
    int p,q;
    float R0,k;
    Constraints():p(0),q(0),R0(0),k(0){
        body[0] = NULL;
    }
    Constraints(Polygon* pp,Polygon* qq,int pointp,int pointq,float _R0 = 0,float _k = 0){
        body[0] = pp;
        body[1] = qq;
        p = pointp;
        q = pointq;
        R0 = _R0;
        k = _k;
    }
    ~Constraints(){
        body[0] = NULL;
        body[1] = NULL;
    }
};

class Constraint
{
public:
    PotentialContact potentialBound[CONTACTNUM];
    QVector<Constraints> rigids;
    QVector<Constraints> elastics;
    QVector<Constraints> bars;
    int potentialBoundNum;
    Constraint();
    void setRopeConstraint(Polygon* p, Polygon* q, const Point& ppoint, const Point& qpoint, float _R0=0);
    void setElasticConstraint(Polygon* p, Polygon* q, const Point& ppoint,const Point& qpoint, float _R0=0, float _k=0);
    void setBarConstraint(Polygon* p, Polygon* q, const Point& ppoint, const Point& qpoint, float _R0=0);
    void getPotentialBound(BVH& bvhNode);
    void actRopeConstraint();
    void actElasticConstraint();
    void actBarConstraint();
    void actBounce();
};

#endif // CONSTRAINT_H
