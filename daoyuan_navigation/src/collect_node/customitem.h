#ifndef CUSTOMITEM_H
#define CUSTOMITEM_H
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>
class CustomItem : public QGraphicsEllipseItem
{
public:
    explicit CustomItem(QGraphicsItem *parent = nullptr);
    explicit CustomItem(const QRectF &rect, QGraphicsItem *parent = nullptr);
    explicit CustomItem(qreal x, qreal y, qreal w, qreal h, QGraphicsItem *parent = nullptr);
    double azimuth;
};

#endif // CUSTOMITEM_H
