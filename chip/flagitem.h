#ifndef FLAGITEM_H
#define FLAGITEM_H

#include <QColor>
#include <QGraphicsItem>
#include <QRectf>

class flagItem : public QGraphicsItem
{
public:
    flagItem(const int x, const int y, const QColor &color);
    QRectF boundingRect() const;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = nullptr);
private:
    int x;
    int y;
    QColor color;
};

#endif // FLAGITEM_H
