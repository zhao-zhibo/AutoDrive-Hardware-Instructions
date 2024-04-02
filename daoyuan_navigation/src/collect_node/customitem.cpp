#include <QKeyEvent>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <cmath>
#include "customitem.h"

// 自定义 Item
CustomItem::CustomItem(QGraphicsItem *parent)
    : QGraphicsEllipseItem (parent),
      azimuth(0)
{
}

CustomItem::CustomItem(const QRectF &rect, QGraphicsItem *parent)
    : QGraphicsEllipseItem (rect,parent),
      azimuth(0){

}
CustomItem::CustomItem(qreal x, qreal y, qreal w, qreal h, QGraphicsItem *parent)
    :QGraphicsEllipseItem (x,y,w,h,parent),
      azimuth(0){

}


