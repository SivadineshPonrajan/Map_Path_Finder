#include "house.h"

#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

house::house(const QColor &color, int x, int y, int radius, int nodeid)
{

    this->x = x-radius;
    this->y = y-radius;
    this->color = color;
    this->radius = radius;
    this->nodeid = nodeid;

    setZValue(1000);
}

QRectF house::boundingRect() const {
    return QRectF(0,0,this->radius,this->radius);
}

void house::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget){
    QRectF rec = boundingRect();
    QBrush brush (this->color);

    painter->fillRect(rec,brush);
    painter->drawRect(rec);
}

