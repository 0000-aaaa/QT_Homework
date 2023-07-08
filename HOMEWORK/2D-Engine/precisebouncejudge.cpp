
#include "precisebouncejudge.h"
#include <QDebug>
int PreciseBounceJudge::mFind(QVector<Point> sim, Point p)
{
    int total = sim.size();
    for(int i=0;i<total;i++){
        if(sim[i]==p)return i;
    }
    return total;
}

PreciseBounceJudge::PreciseBounceJudge()
{
    if(!s.empty())s.clear();
    if(!Nearsest.empty())Nearsest.clear();
    if(!bouncePoint.empty())bouncePoint.clear();
}

PreciseBounceJudge::~PreciseBounceJudge()
{
    if(!s.empty())s.clear();
    if(!Nearsest.empty())Nearsest.clear();
    if(!bouncePoint.empty())bouncePoint.clear();
}

Minkowski_t PreciseBounceJudge::compare(Point v21, Point v22, Point v1)
{
    Point u1 = v21 - v1;
    Point u2 = v22 - v1;
    float x1 = u1.dotProduct(u1,U);
    float x2 = u2.dotProduct(u2,U);
    if(x1<0)x1=-x1;
    if(x2<0)x2=-x2;
    if(x1<=x2)return Minkowski_t(v1,v21,u1);
    else return Minkowski_t(v1,v22,u2);
}

Point PreciseBounceJudge::support(const QVector<Point> &shape, const Point &direction)
{
    float Max = -9999999;
    Point ans;
    for(int i=0;i<shape.size();i++){
        float temp = direction.dotProduct(direction,shape[i]);
        if(temp > Max){
            Max = temp;
            ans = shape[i];
        }
    }
    return ans;
}

Point PreciseBounceJudge::nearestSimplex()
{
    Point ans;
    QVector<Point> sim;
    int total = s.size();
    for(int i=0;i<total;i++){
        sim.push_back(s[i].m_result);
    }
    if(total<3){
        Point temp1 = sim[1] - sim[0];
        temp1.vertical();
        Point temp2 = -temp1;
        if(sim[0].dotProduct(sim[0],temp1)<0)ans = temp1;
        else ans = temp2;
    }
    else{
        float sum1 = 0,sum2;
        for(int i=1;i<3;i++){
            sum1 += sim[i].cross(sim[i-1]);
        }
        if(sum1<0)sum1 = -sum1;
        Point l1 = sim[2]-sim[0],l2 = sim[1]-sim[0];
        sum2 = l1.cross(l2);
        if(sum2<0)sum2=-sum2;
        if(sum2<sum1){
            Point Origin(0,0);
            float d,Min=99999;
            int t = 0;
            for(int i=0;i<3;i++){
                Point ttemp = sim[(i+1)%total]-sim[i];
                ttemp.normalize();
                d = Origin.distanceToLine(sim[i],ttemp);
                if(d<Min){
                    Min=d;t=i;
                }
            }
            t = 3 - (t+(t+1)%3);
            s.erase(s.begin()+t);
            sim.erase(sim.begin()+t);
            Point temp1 = sim[1]-sim[0];
            temp1.vertical();
            Point temp2 = -temp1;
            if(sim[0].dotProduct(sim[0],temp1)<0)ans = temp1;
            else ans = temp2;
        }
    }
    return ans;
}

bool PreciseBounceJudge::GJK(const QVector<Point> &p, const QVector<Point> &q, Point init_direction)
{
    Point Ap = support(p,init_direction);
    Point Aq = support(q,-init_direction);
    Point A = Ap - Aq;
    s.push_back(Minkowski_t(Ap,Aq,A));
    Point D = -A;
    while(1){
        Ap = support(p,D);
        Aq = support(q,-D);
        A = Ap - Aq;
        if(A.dotProduct(A,D)<0)return false;
        s.push_back(Minkowski_t(Ap,Aq,A));
        D = nearestSimplex();
        if(D == Point(0,0))return true;
    }
    return false;
}

void PreciseBounceJudge::EPA(const QVector<Point> &p, const QVector<Point> &q)
{
    Point ans(0,0);
    int total = s.size();
    QVector<Point> sim;
    Point Origin(0,0);
    for(int i =0 ;i<total;i++){
        sim.push_back(s[i].m_result);
    }
    float d,Min = 9999999;
    int t = 0;
    for(int i=0;i<total;i++){
        Point ttemp = sim[(i+1)%total]-sim[i];
        ttemp.normalize();
        d = Origin.distanceToLine(sim[i],ttemp);
        if(d<Min){
            Min=d;t=i;
        }
    }
    Point u = sim[(t+1)%total]-sim[t];
    u.vertical();
    if(u.dotProduct(u,sim[t])<0)u = -u;
    u.normalize();
    ans = u * Min;
    Point Ap = support(p,u);
    Point Aq = support(q,-u);
    Point A = Ap - Aq;
    int it = mFind(sim,A);
    while(it == sim.size()){
        sim.insert(sim.begin()+t+1,A);
        s.insert(s.begin()+t+1,Minkowski_t(Ap,Aq,A));
        total = sim.size();
        Min = 99999;
        for(int i=0;i<total;i++){
            Point ttemp = sim[(i+1)%total]-sim[i];
            ttemp.normalize();
            d = Origin.distanceToLine(sim[i],ttemp);
            if(d<Min){
                Min=d;t=i;
            }
        }
        u = sim[(t+1)%total]-sim[t];
        u.vertical();
        u.normalize();
        if(u.dotProduct(u,sim[t])<0)u = -u;
        ans = u * Min;
        Ap = support(p,u);
        Aq = support(q,-u);
        A = Ap - Aq;
        it = mFind(sim,A);
    }
    if(!Nearsest.empty())Nearsest.clear();
    Nearsest.push_back(s[t]);
    Nearsest.push_back(s[(t+1)%total]);
    U = ans;
}

void PreciseBounceJudge::createBouncePoint()
{
    Point dir1 = Nearsest[1].m_body1-Nearsest[0].m_body1;
    Point dir2 = Nearsest[1].m_body2-Nearsest[0].m_body2;
    if(dir1.lengthSquared()<dir2.lengthSquared()){
        bouncePoint.push_back(Nearsest[1].m_body1);
        bouncePoint.push_back(Nearsest[1].m_body1 - U);
    }
    else{
        bouncePoint.push_back(Nearsest[1].m_body2 + U);
        bouncePoint.push_back(Nearsest[1].m_body2);
    }
}

void PreciseBounceJudge::createBouncePointPair(const QVector<Point> &p, const QVector<Point> &q)
{
    if(!bouncePoint.empty())bouncePoint.clear();
    bool jjj = false;
    Minkowski_t ref, inc;
    Point n = U;
    n.normalize();
    Point p1 = support(p,n);
    Point q1 = support(q,-n);
    int p1Num = mFind(p,p1);int pNum = p.size();
    int q1Num = mFind(q,q1);int qNum = q.size();
    Minkowski_t Mp = compare(p[(p1Num+pNum-1)%pNum],p[(p1Num+1)%pNum],p1);
    Minkowski_t Mq = compare(q[(q1Num+qNum-1)%qNum],q[(q1Num+1)%qNum],q1);
    float mp = n.dotProduct(n,Mp.m_result);
    float mq = n.dotProduct(n,Mq.m_result);
    if(mp<0)mp=-mp;
    if(mq<0)mq=-mq;
    if(mp<=mq){
        ref = Mp;
        inc = Mq;
        n = -n;
    }
    else{
        ref = Mq;
        inc = Mq;
        jjj = true;
    }
    float Ref1 = ref.m_body1.dotProduct(ref.m_body1,ref.m_result);
    float Inc1 = inc.m_body1.dotProduct(inc.m_body1,ref.m_result) - Ref1;
    float Inc2 = inc.m_body2.dotProduct(inc.m_body2,ref.m_result) - Ref1;
    if(Inc1<0){
        inc.m_body1 = inc.m_body1 + inc.m_result * ((-Inc1)/(Inc2-Inc1));
        inc.m_result = inc.m_body2 - inc.m_body1;
    }
    if(Inc2<0){
        inc.m_body2 = inc.m_body1 + inc.m_result * ((-Inc1)/(Inc1-Inc2));
        inc.m_result = inc.m_body2 - inc.m_body1;
    }
    float Ref2 = ref.m_body2.dotProduct(ref.m_body2,-ref.m_result);
    Inc1 = inc.m_body1.dotProduct(inc.m_body1,-ref.m_result) - Ref2;
    Inc2 = inc.m_body2.dotProduct(inc.m_body2,-ref.m_result) - Ref2;
    if(Inc1<0){
        inc.m_body1 = inc.m_body1 + inc.m_result * ((-Inc1)/(Inc2-Inc1));
        inc.m_result = inc.m_body2 - inc.m_body1;
    }
    if(Inc2<0){
        inc.m_body2 = inc.m_body1 + inc.m_result * ((-Inc1)/(Inc1-Inc2));
        inc.m_result = inc.m_body2 - inc.m_body1;
    }
    float Ref3 = n.dotProduct(n,ref.m_body1);
    Inc1 = n.dotProduct(n,inc.m_body1) - Ref3;
    Inc2 = n.dotProduct(n,inc.m_body2) - Ref3;
    if(Inc1>0){
        ref.m_body1 = inc.m_body1 + (-n)*Inc1;
    }
    if(Inc2>0){
        ref.m_body2 = inc.m_body2 + (-n)*Inc2;
    }
    if(jjj){
        bouncePoint.push_back(inc.m_body1);
        bouncePoint.push_back(ref.m_body1);
        bouncePoint.push_back(inc.m_body2);
        bouncePoint.push_back(ref.m_body2);
    }
    else{
        bouncePoint.push_back(ref.m_body1);
        bouncePoint.push_back(inc.m_body1);
        bouncePoint.push_back(ref.m_body2);
        bouncePoint.push_back(inc.m_body2);
    }
}

