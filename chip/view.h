// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef VIEW_H
#define VIEW_H

#include <QComboBox>
#include <QPushButton>
#include <QFrame>
#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QLabel;
class QSlider;
class QToolButton;
QT_END_NAMESPACE

class View;

class GraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    GraphicsView(View *v) : QGraphicsView(), view(v) { }

protected:
#if QT_CONFIG(wheelevent)
    void wheelEvent(QWheelEvent *) override;
#endif

private:
    View *view;
};

class View : public QFrame
{
    Q_OBJECT
public:
    explicit View(const QString &name, QWidget *parent = nullptr);

    QGraphicsView *view() const;

    QComboBox* getComboBox() const { return comboBox; }

public slots:
    void zoomIn();
    void zoomOut();
    void zoomInBy(int level);
    void zoomOutBy(int level);

private slots:
    void resetView();
    void setResetButtonEnabled();
    void setupMatrix();
    void togglePointerMode();
    void toggleAntialiasing();
    void print();
    void rotateLeft();
    void rotateRight();
    void mapSelected();
    void mapResetted();
    void onComboBoxSelected(int index);

private:
    GraphicsView *graphicsView;
    QLabel *label;
    QLabel *label2;
    QComboBox *comboBox;
    QToolButton *selectModeButton;
    QToolButton *dragModeButton;
    QToolButton *antialiasButton;
    QToolButton *printButton;
    QToolButton *resetButton;
    QPushButton *map;
    QPushButton *resetMap;
    QPushButton *reset;
    QSlider *zoomSlider;
    QSlider *rotateSlider;
};

#endif // VIEW_H
