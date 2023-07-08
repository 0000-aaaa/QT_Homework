
#ifndef BVH_H
#define BVH_H
#include <QVector>
#include "polygon.h"


struct PotentialContact{
    Polygon* body[2];
    PotentialContact(){
        body[0] = NULL;
        body[1] = NULL;
    }
    PotentialContact(Polygon* _p,Polygon* _q){
        body[0] = _p;
        body[1] = _q;
    }
    ~PotentialContact(){
        body[0] = NULL;
        body[1] = NULL;
    }
};
struct BoundingVolume{
    float left,right,up,down;
    BoundingVolume(float _left,float _right,float _up,float _down):
        left(_left),right(_right),up(_up),down(_down){}
    BoundingVolume():left(0),right(0),up(0),down(0){}
    BoundingVolume(const BoundingVolume& copy){
        left = copy.left;
        right = copy.right;
        up = copy.up;
        down = copy.down;
    }
    BoundingVolume(const Polygon& copy){
        left = copy.left;
        right = copy.right;
        up = copy.up;
        down = copy.down;
    }
    ~BoundingVolume(){}
    bool overlaps(const BoundingVolume& q)const{
        if((q.left>=left&&q.left<=right)||(q.right>=left&&q.right<=right)){
            return ((q.up>=down&&q.up<=up)||(q.down>=down&&q.down<=up));
        }
        return false;
    }
    float getSize()const{
        return (right - left)*(up - down);
    }
    float getGrowth(const BoundingVolume& newValue)const{
        float tempL,tempR,tempU,tempD;
        tempL = left<=newValue.left?left:newValue.left;
        tempR = right>=newValue.right?right:newValue.right;
        tempU = up>=newValue.up?up:newValue.up;
        tempD = down<=newValue.down?down:newValue.down;
        return (tempR - tempL)*(tempU - tempD);
    }
    void operator=(const BoundingVolume& copy){
        left = copy.left;
        right = copy.right;
        up = copy.up;
        down = copy.down;
    }
};


class BVH
{
public:
    BVH * children[2];
    BVH * parent;
    BoundingVolume volume;
    Polygon * body;
    BVH();
    BVH(BVH* _parent,BoundingVolume _volume, Polygon* _body);
    ~BVH();
    bool isLeaf()const;
    bool overlaps(const BVH * other)const;
    unsigned getPotentialContactsWith(const BVH* other,PotentialContact* contact,unsigned limit)const;
    unsigned getPotentialContacts(PotentialContact* contacts,unsigned limit);
    void recaculateBoundingVolume();
    void insert(Polygon* body,const BoundingVolume &volume);
    void clear();
    BVH* changeNode();
    void moveNode();
    void operator=(const BVH& copy);
};

#endif // BVH_H
