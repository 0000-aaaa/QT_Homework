
#include "constraint.h"

Constraint::Constraint()
{
    potentialBoundNum = 0;
}

void Constraint::setRopeConstraint(Polygon *p, Polygon *q,const Point &ppoint,const Point &qpoint, float _R0)
{
    int tempp,tempq;
    p->addPoints(ppoint);
    tempp = p->companyPoints.size()-1;
    q->addPoints(qpoint);
    tempq = q->companyPoints.size()-1;
    rigids.push_back(Constraints(p,q,tempp,tempq,_R0));
}

void Constraint::setElasticConstraint(Polygon *p, Polygon *q, const Point &ppoint, const Point &qpoint,float _R0, float _k)
{
    int tempp,tempq;
    p->addPoints(ppoint);
    tempp = p->companyPoints.size()-1;
    q->addPoints(qpoint);
    tempq = q->companyPoints.size()-1;
    elastics.push_back(Constraints(p,q,tempp,tempq,_R0,_k));
}

void Constraint::setBarConstraint(Polygon *p, Polygon *q,const Point &ppoint,const Point &qpoint, float _R0)
{
    int tempp,tempq;
    p->addPoints(ppoint);
    tempp = p->companyPoints.size()-1;
    q->addPoints(qpoint);
    tempq = q->companyPoints.size()-1;
    bars.push_back(Constraints(p,q,tempp,tempq,_R0));
}

void Constraint::getPotentialBound(BVH &bvhNode)
{
    potentialBoundNum = bvhNode.getPotentialContacts(potentialBound,CONTACTNUM);
}

void Constraint::actRopeConstraint()
{
    for (int i = 0; i < rigids.size(); ++i) {
        Point R = rigids[i].body[0]->companyPoints[rigids[i].p]
                   -rigids[i].body[1]->companyPoints[rigids[i].q];
        Point ra = rigids[i].body[0]->companyPoints[rigids[i].p]
                     -rigids[i].body[0]->position;
        Point rb = rigids[i].body[1]->companyPoints[rigids[i].q]
                     -rigids[i].body[1]->position;
        float dr = R.length();
        if(dr>=rigids[i].R0){
            Point n = -R;
            n.normalize();
            float im_a = rigids[i].body[0]->inverseMass;
            float im_b = rigids[i].body[1]->inverseMass;
            float ii_a = rigids[i].body[0]->inverseInertia;
            float ii_b = rigids[i].body[1]->inverseInertia;
            float rn_a = ra.cross(n);
            float rn_b = rb.cross(n);
            float kNormal = im_a +ii_a * rn_a *rn_a +im_b + ii_b * rn_b * rn_b;
            float effectiveMassNormal = kNormal == 0.0 ? 0 : 1.0/kNormal;
            float bias = /*m_biasFactor*/(dr-rigids[i].R0);
            if(bias<0)bias=0;
            QVector2D wa(-rigids[i].body[0]->angularV*ra.y(),rigids[i].body[0]->angularV*ra.x());
            QVector2D wb(-rigids[i].body[1]->angularV*rb.y(),rigids[i].body[1]->angularV*rb.x());
            QVector2D va = rigids[i].body[0]->velocity + wa;
            QVector2D vb = rigids[i].body[1]->velocity + wb;
            QVector2D dv = va - vb;
            float jv = n.dotProduct(n,dv);
            if(jv>0)jv=0;
            float jvb = -jv+bias;
            float lambda_n = effectiveMassNormal * jvb;
            QVector2D impulse_n = lambda_n * n;
            rigids[i].body[0]->addForce(impulse_n,ra);
            rigids[i].body[1]->addForce(-impulse_n,rb);
        }
    }
}

void Constraint::actElasticConstraint()
{
    for(int i=0;i<elastics.size();i++){
        Point R = elastics[i].body[0]->companyPoints[elastics[i].p]
                  -elastics[i].body[1]->companyPoints[elastics[i].q];
        Point actP = elastics[i].body[0]->companyPoints[elastics[i].p]
                     -elastics[i].body[0]->position;
        Point actQ = elastics[i].body[1]->companyPoints[elastics[i].q]
                     -elastics[i].body[1]->position;
        float dr = R.length()-elastics[i].R0;
        float F = elastics[i].k*dr;
        R.normalize();
        elastics[i].body[0]->addForce(-R*F,actP);
        elastics[i].body[1]->addForce(R*F,actQ);
    }
}

void Constraint::actBarConstraint()
{
    for (int i = 0; i < bars.size(); ++i) {
        Point R = bars[i].body[0]->companyPoints[bars[i].p]
                  -bars[i].body[1]->companyPoints[bars[i].q];
        Point ra = bars[i].body[0]->companyPoints[bars[i].p]
                   -bars[i].body[0]->position;
        Point rb = bars[i].body[1]->companyPoints[bars[i].q]
                   -bars[i].body[1]->position;
        float dr = R.length();
        Point n = -R;
        n.normalize();
        float im_a = bars[i].body[0]->inverseMass;
        float im_b = bars[i].body[1]->inverseMass;
        float ii_a = bars[i].body[0]->inverseInertia;
        float ii_b = bars[i].body[1]->inverseInertia;
        float rn_a = ra.cross(n);
        float rn_b = rb.cross(n);
        float kNormal = im_a +ii_a * rn_a *rn_a +im_b + ii_b * rn_b * rn_b;
        float effectiveMassNormal = kNormal == 0.0 ? 0 : 1.0/kNormal;
        QVector2D wa(-bars[i].body[0]->angularV*ra.y(),bars[i].body[0]->angularV*ra.x());
        QVector2D wb(-bars[i].body[1]->angularV*rb.y(),bars[i].body[1]->angularV*rb.x());
        QVector2D va = bars[i].body[0]->velocity + wa;
        QVector2D vb = bars[i].body[1]->velocity + wb;
        QVector2D dv = va - vb;
        float bias = m_biasFactor*(dr-bars[i].R0);
        float jv = n.dotProduct(n,dv);
        float jvb = -jv + bias;
        float lambda_n = effectiveMassNormal * jvb;
        QVector2D impulse_n = lambda_n * n;
        bars[i].body[0]->addForce(impulse_n,ra);
        bars[i].body[1]->addForce(-impulse_n,rb);
    }
}

void Constraint::actBounce()
{
    for(int i=0;i<potentialBoundNum;i++){
        PreciseBounceJudge bounce;
        bool judge = bounce.GJK(potentialBound[i].body[0]->points,
                                potentialBound[i].body[1]->points,
                                potentialBound[i].body[1]->position - potentialBound[i].body[0]->position);
        if(judge){
            bounce.EPA(potentialBound[i].body[0]->points,
                       potentialBound[i].body[1]->points);
            bounce.createBouncePoint();
            /*bounce.createBouncePointPair(potentialBound[i].body[0]->points,
                                         potentialBound[i].body[1]->points);*/
            Point ra1 = bounce.bouncePoint[0] - potentialBound[i].body[0]->position;
            Point rb1 = bounce.bouncePoint[1] - potentialBound[i].body[1]->position;
            Point n = -bounce.U;//
            n.normalize();
            Point tangent = n;tangent.vertical();
            float im_a = potentialBound[i].body[0]->inverseMass;
            float im_b = potentialBound[i].body[1]->inverseMass;
            float ii_a = potentialBound[i].body[0]->inverseInertia;
            float ii_b = potentialBound[i].body[1]->inverseInertia;
            float rn_a1 = ra1.cross(n);
            float rn_b1 = rb1.cross(n);
            float kNormal = im_a +ii_a * rn_a1 *rn_a1 +im_b + ii_b * rn_b1 * rn_b1;
            float effectiveMassNormal = kNormal == 0.0 ? 0 : 1.0/kNormal;
            QVector2D wa(-potentialBound[i].body[0]->angularV*ra1.y(),
                         potentialBound[i].body[0]->angularV*ra1.x());
            QVector2D wb(-potentialBound[i].body[1]->angularV*rb1.y(),
                         potentialBound[i].body[1]->angularV*rb1.x());
            QVector2D va = potentialBound[i].body[0]->velocity + wa;
            QVector2D vb = potentialBound[i].body[1]->velocity +wb;
            QVector2D dv = va - vb;
            float bias = m_biasFactor*qMax(0.0,bounce.U.length()-m_maxPenetration)/DTIME;
            float jv = n.dotProduct(n,dv);
            if(jv>0)jv=0;
            float jvb = -jv +bias;
            float lambda_n = effectiveMassNormal * jvb;
            QVector2D impulse_n = lambda_n * n;
            potentialBound[i].body[0]->addForce(impulse_n,ra1);
            potentialBound[i].body[1]->addForce(-impulse_n,rb1);
            if(bounce.bouncePoint.size()>2){
                Point ra2 = bounce.bouncePoint[2] - potentialBound[i].body[0]->position;
                Point rb2 = bounce.bouncePoint[3] - potentialBound[i].body[1]->position;
                float rn_a2 = ra2.cross(n);
                float rn_b2 = rb2.cross(n);
                float kNormal = im_a +ii_a * rn_a2 *rn_a2 +im_b + ii_b * rn_b2 * rn_b2;
                float effectiveMassNormal = kNormal == 0.0 ? 0 : 1.0/kNormal;
                QVector2D wa(-potentialBound[i].body[0]->angularV*ra2.y(),
                             potentialBound[i].body[0]->angularV*ra2.x());
                QVector2D wb(-potentialBound[i].body[1]->angularV*rb2.y(),
                             potentialBound[i].body[1]->angularV*rb2.x());
                QVector2D va = potentialBound[i].body[0]->velocity + wa;
                QVector2D vb = potentialBound[i].body[1]->velocity +wb;
                QVector2D dv = va - vb;
                Point U2 = bounce.bouncePoint[2] - bounce.bouncePoint[3];
                float bias = m_biasFactor*qMax(0.0,U2.length()-m_maxPenetration)/DTIME;
                float jv = n.dotProduct(n,dv);
                if(jv>0)jv=0;
                float jvb = -jv +bias;
                float lambda_n = effectiveMassNormal * jvb;
                QVector2D impulse_n = lambda_n * n;
                potentialBound[i].body[0]->addForce(impulse_n,ra2);
                potentialBound[i].body[1]->addForce(-impulse_n,rb2);
            }
        }
    }
}

