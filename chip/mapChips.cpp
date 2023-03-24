#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <qgraphicsitem.h>

class mapChips : public QGraphicsItem
{
private:
    int x;
    int y;
    int radius;
    int nodeid;
    QColor color;
    QList<QPointF> stuff;

public:
    mapChips(const QColor &color, int x, int y, int radius, int nodeid) : QGraphicsItem() {
        this->x = x-radius;
        this->y = y-radius;
        this->color = color;
        this->radius = radius;
        this->nodeid = nodeid;

        setZValue((x + y) % 2);

        setFlags(ItemIsSelectable | ItemIsMovable);
        setAcceptHoverEvents(true);
    }

    QRectF boundingRect() const override {
        return QRectF(0, 0, this->radius, this->radius);
    }

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = nullptr) override {
        painter->setBrush(Qt::red);
        painter->drawRect(boundingRect());
    }
};
