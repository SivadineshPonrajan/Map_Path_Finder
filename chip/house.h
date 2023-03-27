#ifndef HOUSE_H
#define HOUSE_H

#include <QColor>
#include <QGraphicsItem>
#include <QRectf>

class house : public QGraphicsItem
{
public:
    house(const QColor &color, int x, int y, int radius, int nodeid);
    QRectF boundingRect() const;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = nullptr);
//    bool pressed;
//protected:
//    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
//    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
private:
    int x;
    int y;
    int radius;
    int nodeid;
    QColor color;
};

#endif // HOUSE_H
