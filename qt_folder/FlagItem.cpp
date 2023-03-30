#include "flagItem.h"

#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

flagItem::flagItem(const int x, const int y, const QColor &color)
{
    this->x = x;
    this->y = y;
    this->color = color;

    setZValue(1000);
}

QRectF flagItem::boundingRect() const
{
    return QRectF(this->x,this->y,20,10);
}

void flagItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QRectF rec = boundingRect();
    QBrush brush (this->color);
    painter->fillRect(rec,brush);
    painter->drawRect(rec);
}
