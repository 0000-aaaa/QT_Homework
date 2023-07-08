
#include "polygon.h"
#include <qDebug>
Polygon::Polygon(){}

Polygon::Polygon(QPoint _pos, QPixmap _pixmap, float M)
{
    this->setPos(_pos);
    this->setPixmap(_pixmap);
    mapPos.setX(_pos.x());
    mapPos.setY(_pos.y());
    map = _pixmap;
    Mass = M;
    if(Mass)inverseMass = 1/M;
    else inverseMass = 0;
    angularV = 0.0;
    left = 0.0;
    right = 0.0;
    up = 0.0;
    down = 0.0;
    S = 0.0;
}

Polygon::~Polygon(){}

bool Polygon::overlaps(const Polygon &q) const
{
    if((q.left>=left&&q.left<=right)||(q.right>=left&&q.right<=right)){
        return ((q.up>=down&&q.up<=up)||(q.down>=down&&q.down<=up));
    }
    return false;
}

void Polygon::caculateIP()
{
    QVector<Point> lines;
    for(int i=1;i<points.size();i++){
        lines.push_back(points[i]-points[0]);
    }
    float Area = 0;
    for(int i=0;i<lines.size()-1;i++){
        float temp = lines[i].cross(lines[i+1]);
        if(temp<0)temp = -temp;
        position += temp*(points[0]+points[i+1]+points[i+2])/3;
        Area += temp;
    }
    if(Area)position /= Area;
    float MinX = 9999999, MinY = 9999999;
    float MaxX = -9999999, MaxY = -9999999;
    for(int i=0;i<points.size();i++){
        points[i].setRXY(position);
        if(points[i].Rx<MinX)MinX = points[i].Rx;
        if(points[i].Ry<MinY)MinY = points[i].Ry;
        if(points[i].Rx>MaxX)MaxX = points[i].Rx;
        if(points[i].Ry>MaxY)MaxY = points[i].Ry;
    }
    left = position.x()+MinX;
    right = position.x()+MaxX;
    up = position.y()+MaxY;
    down = position.y()+MinY;
    S = (right - left)*(up - down);
    if(!lines.empty())lines.clear();
    for(int i=0;i<points.size();i++){
        lines.push_back(points[i]-position);
    }
    int len = lines.size();
    float sum1 = 0, sum2 = 0;
    float cross;
    for(int i=0;i<len;i++){
        int j = (i+1)%len;
        cross = lines[i].cross(lines[j]);
        if(cross<0)cross=-cross;
        sum1 += cross *( lines[i].dotProduct(lines[i],lines[i])
                           +lines[i].dotProduct(lines[i],lines[j])
                         +lines[j].dotProduct(lines[j],lines[j]));
        sum2 += cross;
    }
    Inertia = Mass/6*sum1/sum2;
    if(Inertia)inverseInertia = 1/Inertia;
    else inverseInertia = 0;
}

void Polygon::setPoints(Point newPoint)
{
    points.push_back(newPoint);
}

void Polygon::addPoints(Point newPoint)
{
    Point tempPoint = newPoint + mapPos;
    companyPoints.push_back(tempPoint);
    int t = companyPoints.size()-1;
    companyPoints[t].setRXY(position);
}

void Polygon::setVelocity(QVector2D v)
{
    velocity = v;
}

void Polygon::setAngularV(float w)
{
    angularV = w;
}

void Polygon::addForce(float fx, float fy, float px, float py)
{
    forces.push_back(forcement(fx,fy,px,py));
}

void Polygon::addForce(const QVector2D &F, const QVector2D &ActP)
{
    forces.push_back(forcement(F,ActP));
}

void Polygon::actForce()
{
    int total = forces.size();
    for(int i=0;i<total;i++){
        Point r = forces[i].actP;
        angularV += r.cross(forces[i].force)*inverseInertia*DTIME;
        velocity += forces[i].force * inverseMass*DTIME;
    }
    if(!forces.empty())forces.clear();
}

void Polygon::move()
{
    if(Mass)velocity += QVector2D(0,0.1);
    velocity *= DURATION;
    angularV *= ANGULAR_DURATION;
    this->moveBy(velocity.x(),velocity.y());
    position += velocity * DTIME;
    mapPos += velocity * DTIME;
    QVector2D temp = position - mapPos;
    this->setTransformOriginPoint(temp.x(),temp.y());
    float angle = angularV*DTIME*180/3.14159265;
    this->setRotation(rotation() + angle);
    float MinX = 9999999, MinY = 9999999;
    float MaxX = -9999999, MaxY = -9999999;
    for(int i=0;i<points.size();i++){
        points[i].move(position,angularV);
        if(points[i].Rx<MinX)MinX = points[i].Rx;
        if(points[i].Ry<MinY)MinY = points[i].Ry;
        if(points[i].Rx>MaxX)MaxX = points[i].Rx;
        if(points[i].Ry>MaxY)MaxY = points[i].Ry;
    }
    for(int j=0;j<companyPoints.size();j++){
        companyPoints[j].move(position,angularV);
    }
    left = position.x()+MinX;
    right = position.x()+MaxX;
    up = position.y()+MaxY;
    down = position.y()+MinY;
    S = (right - left)*(up - down);
}

bool Polygon::checkBounce(const Polygon &object) const
{
    PreciseBounceJudge judge;
    bool temp = judge.GJK(this->points,object.points,object.position - this->position);
    return temp;
}

void Polygon::actBounce(Polygon &object)
{
    PreciseBounceJudge bounce;
    bool judge = bounce.GJK(this->points,object.points,object.position - this->position);
    if(judge){
        bounce.EPA(this->points,object.points);
        bounce.createBouncePoint();
        //bounce.createBouncePointPair(this->points,object.points);
        Point ra = bounce.bouncePoint[0] - position;
        Point rb = bounce.bouncePoint[1] - object.position;
        Point n = -bounce.U;//
        n.normalize();
        Point tangent = n;tangent.vertical();
        float im_a = inverseMass;
        float im_b = object.inverseMass;
        float ii_a = inverseInertia;
        float ii_b = object.inverseInertia;
        float rn_a = ra.cross(n);
        float rn_b = rb.cross(n);
        float kNormal = im_a +ii_a * rn_a *rn_a +im_b + ii_b * rn_b * rn_b;
        float effectiveMassNormal = kNormal == 0.0 ? 0 : 1.0/kNormal;
        QVector2D wa(-angularV*ra.y(),angularV*ra.x());
        QVector2D wb(-object.angularV*rb.y(),object.angularV*rb.x());
        QVector2D va = velocity + wa;
        QVector2D vb = object.velocity +wb;
        QVector2D dv = va - vb;
        float bias = m_biasFactor*qMax(0.0,bounce.U.length()-m_maxPenetration)/DTIME;
        float jv = n.dotProduct(n,dv);
        if(jv>0)jv=0;
        float jvb = -jv +bias;
        float lambda_n = effectiveMassNormal * jvb;
        QVector2D impulse_n = lambda_n * n;
        addForce(impulse_n,ra);
        object.addForce(-impulse_n,rb);
        if(bounce.bouncePoint.size()>2){
            ra = bounce.bouncePoint[2] - position;
            rb = bounce.bouncePoint[3] - object.position;
            rn_a = ra.cross(n);
            rn_b = rb.cross(n);
            kNormal = im_a +ii_a * rn_a *rn_a +im_b + ii_b * rn_b * rn_b;
            effectiveMassNormal = kNormal == 0.0 ? 0 : 1.0/kNormal;
            QVector2D wa(-angularV*ra.y(),angularV*ra.x());
            QVector2D wb(-object.angularV*rb.y(),object.angularV*rb.x());
            QVector2D va = velocity + wa;
            QVector2D vb = object.velocity +wb;
            QVector2D dv = va - vb;
            Point tempU = bounce.bouncePoint[2] - bounce.bouncePoint[3];
            float bias = m_biasFactor*qMax(0.0,tempU.length()-m_maxPenetration)/DTIME;
            float jv = n.dotProduct(n,dv);
            if(jv>0)jv=0;
            float jvb = -jv +bias;
            float lambda_n = effectiveMassNormal * jvb;
            QVector2D impulse_n = lambda_n * n;
            addForce(impulse_n,ra);
            object.addForce(-impulse_n,rb);
        }
        actForce();
        object.actForce();
    }
}





