
#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QWidget>
#include <QGraphicsPixmapItem>//图形
#include <QGraphicsView>//视图
#include <QGraphicsScene>//场景
#include <QTimer>
#include <QKeyEvent>
#include <QList>
#include <QVector>
#include <QPushButton>
#include <QToolButton>
#include <QtMath>
#include "rope.h"
#include "circle.h"
#include "block.h"
#include "floor.h"
#include "constraint.h"
#include "bvh.h"
#include "longblock.h"
QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget

{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();
    void PlaneMove();
    void PlaneBounce();
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
private:
    Ui::Widget *ui;
    QGraphicsView mGameView;//游戏视图
    QGraphicsScene mScene;//场景
    Circle circle;
    Floor floor,ceiling;
    Rope Rope1,Rope2,Rope3,Rope4,Rope5,Rope6,Rope7,Rope8,Rope9,Rope10,Rope11,Rope12,Rope13,Rope14,Rope15;
    Block block1,block2,block3,block4,block5;
    LongBlock block;
    BVH BVHNode;
    Constraint pconstraints;
    QTimer* mPlaneMoveTimer;
    QTimer* mBounce;
    QList<int> mKeyList;
    QList<int> mKeyList2;
};

#endif // WIDGET_H
