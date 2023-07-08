
#include "widget.h"
#include "ui_widget.h"

#include <QDebug>
Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    ,circle(QPoint(352.5,480),QPixmap(":/new/img/CIRCLE.png"))
    ,floor(QPoint(0,700),QPixmap(":/new/img/FLOOR.png"))
    ,ceiling(QPoint(0,-50),QPixmap(":/new/img/FLOOR.png"))
    ,Rope1(QPoint(400,60),QPixmap(":/new/img/ROPE.png"))
    ,Rope2(QPoint(400,60),QPixmap(":/new/img/ROPE.png"))
    ,Rope3(QPoint(400,90),QPixmap(":/new/img/ROPE.png"))
    ,Rope4(QPoint(400,120),QPixmap(":/new/img/ROPE.png"))
    ,Rope5(QPoint(400,150),QPixmap(":/new/img/ROPE.png"))
    ,Rope6(QPoint(400,180),QPixmap(":/new/img/ROPE.png"))
    ,Rope7(QPoint(400,210),QPixmap(":/new/img/ROPE.png"))
    ,Rope8(QPoint(400,240),QPixmap(":/new/img/ROPE.png"))
    ,Rope9(QPoint(400,270),QPixmap(":/new/img/ROPE.png"))
    ,Rope10(QPoint(400,300),QPixmap(":/new/img/ROPE.png"))
    ,Rope11(QPoint(400,330),QPixmap(":/new/img/ROPE.png"))
    ,Rope12(QPoint(400,360),QPixmap(":/new/img/ROPE.png"))
    ,Rope13(QPoint(400,390),QPixmap(":/new/img/ROPE.png"))
    ,Rope14(QPoint(400,420),QPixmap(":/new/img/ROPE.png"))
    ,Rope15(QPoint(400,450),QPixmap(":/new/img/ROPE.png"))
    ,block(QPoint(500,450),QPixmap(":/new/img/LONG.png"))
{
    ui->setupUi(this);
    this->setFixedSize(1024,768);
    mGameView.setSceneRect(QRect(0,0,1024,768));
    mScene.setSceneRect(QRect(0,0,1024,768));
    mScene.addItem(&Rope1);mScene.addItem(&Rope2);mScene.addItem(&Rope3);
    mScene.addItem(&Rope4);mScene.addItem(&Rope5);mScene.addItem(&Rope6);
    mScene.addItem(&Rope7);mScene.addItem(&Rope8);mScene.addItem(&Rope9);
    mScene.addItem(&Rope10);mScene.addItem(&Rope11);mScene.addItem(&Rope12);
    mScene.addItem(&Rope13);mScene.addItem(&Rope14);mScene.addItem(&Rope15);
    mScene.addItem(&circle);
    mScene.addItem(&block);
    pconstraints.setRopeConstraint(&ceiling,&Rope1,Point(402.5,100),Point(2.5,0.0),10);
    pconstraints.setRopeConstraint(&Rope1,&Rope2,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope2,&Rope3,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope3,&Rope4,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope4,&Rope5,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope5,&Rope6,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope6,&Rope7,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope7,&Rope8,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope8,&Rope9,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope9,&Rope10,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope10,&Rope11,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope11,&Rope12,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope12,&Rope13,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope13,&Rope14,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope14,&Rope15,Point(2.5,25.0),Point(2.5,0.0),5);
    pconstraints.setRopeConstraint(&Rope15,&circle,Point(2.5,25.0),Point(50,0.0),5);
    mScene.addItem(&floor);
    mScene.addItem(&ceiling);
    mGameView.setParent(this);
    mGameView.setScene(&mScene);
    mGameView.show();
    BVHNode = BVH(NULL,BoundingVolume(circle),&circle);
    BVHNode.insert(&block,BoundingVolume(block));
    BVHNode.insert(&floor,BoundingVolume(floor));
    mPlaneMoveTimer = new QTimer(this);
    mPlaneMoveTimer->start(10);
    connect(mPlaneMoveTimer,&QTimer::timeout,this,&Widget::PlaneMove);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::keyPressEvent(QKeyEvent *event)
{
    mKeyList.append(event->key());
    if(mKeyList2.contains(event->key()))mKeyList2.removeOne(event->key());
}

void Widget::keyReleaseEvent(QKeyEvent *event)
{
    mKeyList2.append(event->key());
    if(mKeyList.contains(event->key()))
    {
        mKeyList.removeOne(event->key());
    }
}
void Widget::PlaneMove()
{
    if(block.position.x()>1024||block.position.x()<0){
        block.velocity = -block.velocity;
    }
    for(int keyCode : mKeyList)
    {
        circle.controlMove(keyCode);
    }
    pconstraints.getPotentialBound(BVHNode);
    //pconstraints.actBounce();
    block.actBounce(floor);
    block.actBounce(circle);
    pconstraints.actRopeConstraint();
    Rope1.actForce();Rope1.move();
    Rope2.actForce();Rope2.move();
    Rope3.actForce();Rope3.move();
    Rope4.actForce();Rope4.move();
    Rope5.actForce();Rope5.move();
    Rope6.actForce();Rope6.move();
    Rope7.actForce();Rope7.move();
    Rope8.actForce();Rope8.move();
    Rope9.actForce();Rope9.move();
    Rope10.actForce();Rope10.move();
    Rope11.actForce();Rope11.move();
    Rope12.actForce();Rope12.move();
    Rope13.actForce();Rope13.move();
    Rope14.actForce();Rope14.move();
    Rope15.actForce();Rope15.move();
    circle.actForce();circle.move();
    block.actForce();block.move();
    BVHNode.clear();
    BVHNode = BVH(NULL,BoundingVolume(circle),&circle);
    BVHNode.insert(&block,BoundingVolume(block));
    BVHNode.insert(&floor,BoundingVolume(floor));
}
