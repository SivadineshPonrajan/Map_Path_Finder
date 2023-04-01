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
    return QRectF(x,y,this->radius*2,this->radius*2);
//    return QRectF(x,y, 5, 5);
}

void house::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget){
    QRectF rec = boundingRect();
    QGraphicsEllipseItem ellipse{rec};

    QBrush brush (this->color);
    painter->setBrush(brush);
//    painter->fillRect(rec,brush);
    painter->drawEllipse(rec);

//    painter->drawRect(ellipse);
}

