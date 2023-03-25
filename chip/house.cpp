#include "house.h"

#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

class data{
public :
    int xter = 10;
    void right(){
        this->xter = 23;
    }
};

house::house(const QColor &color, int x, int y, int radius, int nodeid)
{

    this->x = x-radius;
    this->y = y-radius;
    this->color = color;
    this->radius = radius;
    this->nodeid = nodeid;

    setZValue(1000);
    pressed = false;
    ishovered = false;
    setFlag(ItemIsSelectable);
    setAcceptHoverEvents(true);
}

QRectF house::boundingRect() const {
    return QRectF(0,0,this->radius,this->radius);
}

void house::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget){
    QRectF rec = boundingRect();
    QBrush brush (this->color);

//    QColor fillColor = (option->state & QStyle::State_Selected) ? color.darker(150) : color;
//    if (option->state & QStyle::State_MouseOver)
//        fillColor = fillColor.lighter(125);

    if(pressed == true){
        brush.setColor(Qt::green);
        qDebug() << this->nodeid;

    }else if(ishovered == true){

    }
//    else{
//        brush.setColor(this->color);
//    }

    painter->fillRect(rec,brush);
    painter->drawRect(rec);
}

void house::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    pressed = true;
    update();
    QGraphicsItem::mousePressEvent(event);
}

void house::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    pressed = false;
    update();
    QGraphicsItem::mouseReleaseEvent(event);
}

void house::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
    ishovered = true;
    update(); // Call update() to trigger a repaint
}

void house::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
    ishovered = false;
    update(); // Call update() to trigger a repaint
}
