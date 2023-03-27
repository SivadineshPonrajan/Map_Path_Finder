// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "house.h"
#include "mainwindow.h"
#include "qmessagebox.h"
#include "view.h"
#include "flagitem.h"
#include <QFileDialog>
#include <QHBoxLayout>
#include <QSplitter>
#include <QMouseEvent>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <limits>

std::vector<QPointF> mapPoints;
QGraphicsItem* startFlag;
QGraphicsItem* endFlag;
QGraphicsItem* startItem;
QGraphicsItem* endItem;

int thres = 30;
int margin = 100;
int radius = thres/10;

using namespace std;
View* view;
MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent), scene(new QGraphicsScene(this))
    , h1Splitter(new QSplitter(this)), h2Splitter(new QSplitter(this))
{
    populateScene();

    QSplitter *vSplitter = new QSplitter;
    vSplitter->setOrientation(Qt::Vertical);
    vSplitter->addWidget(h1Splitter);
    vSplitter->addWidget(h2Splitter);

    view = new View("Map Explorer");
    view->view()->viewport()->installEventFilter(this);
    scene->setBackgroundBrush(Qt::black);
    view->view()->setScene(scene);
    h1Splitter->addWidget(view);

    QHBoxLayout *layout = new QHBoxLayout;
    layout->addWidget(vSplitter);
    setLayout(layout);

    setWindowTitle(tr("Embedded C++ Project"));
}

struct Vertex {
    int id;
    double latitude;
    double longitude;
    int x;
    int y;
    int xx;
    int yy;
};

struct Edge {
    int src;
    int dest;
    double dist;
    std::string name;
};

struct Point {
    double x, y;
};

std::map<int, Vertex> vertices;

double project2map(double data, double min, int thres){
    return (int)((data-min)*thres*1000);
}

double distance(const QPointF& p1, const QPointF& p2)
{
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::sqrt(dx*dx + dy*dy);
}

QPointF findNearest(QPointF point){
    Vertex* nearestVertex = nullptr;
     double minDistance = std::numeric_limits<double>::max();

     for (auto& vertexPair : vertices) {
         Vertex& vertex = vertexPair.second;
         QPointF vertexPoint(vertex.xx, vertex.yy);
         double d = distance(point, vertexPoint);
         if (d < minDistance) {
             minDistance = d;
             nearestVertex = &vertex;
         }
     }
     return QPointF(nearestVertex->xx,nearestVertex->yy);
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::MouseButtonPress){

    QMouseEvent* mouseEvent = dynamic_cast<QMouseEvent*>(event);
    QGraphicsView* graphicsView = view->view();

    if (mouseEvent->button() == Qt::LeftButton) {
      QPointF nearest;
      QPointF point = graphicsView->mapToScene(mouseEvent->pos());
      qDebug() <<"Click at: "<<point.x()<< " " <<point.y();
//      extern std::vector<QPointF> mapPoints;
      if(mapPoints.size()==0){
          mapPoints.push_back(point);
          qDebug() << "Start Element";

//          extern QGraphicsRectItem* startFlag;
//          startFlag = new QGraphicsRectItem(point.x(), point.y(), 20, 10);
//          QPen fpen(Qt::black);
//          fpen.setWidth(2);
//          startFlag->setPen(fpen);
//          startFlag->setBrush(QBrush(Qt::red));
//          scene->addItem(startFlag);

          startFlag = new flagItem(point.x(), point.y(), Qt::red);
          scene->addItem(startFlag);
//          scene->update();

          //          start
           nearest = findNearest(point);
           startItem = new house(Qt::red, nearest.x()-radius, nearest.y()-radius, 2*radius+1, 1);
           startItem->setPos(QPointF(nearest.x()-radius, nearest.y()-radius));
           scene->addItem(startItem);
           scene->update();
          //          end
      }
      else if(mapPoints.size()==1){
          mapPoints.push_back(point);
          qDebug() << "Destiny Element";

          endFlag = new flagItem(point.x(), point.y(), Qt::green);
          scene->addItem(endFlag);
//          scene->update();

          //          start
           nearest = findNearest(point);
           endItem = new house(Qt::green, nearest.x()-radius, nearest.y()-radius, 2*radius+1, 1);
           endItem->setPos(QPointF(nearest.x()-radius, nearest.y()-radius));
           scene->addItem(endItem);
           scene->update();
          //          end
      }
      else{
          QMessageBox::StandardButton reply;
          reply = QMessageBox::question(NULL, "Reset Flags Alert", "Want to reset the start and the end flag points?", QMessageBox::Yes|QMessageBox::No);

              if (reply == QMessageBox::Yes) {
                  mapPoints.clear();
                  qDebug() << "Removed all Elements";
                  scene->removeItem(startFlag);
                  scene->removeItem(startItem);
                  scene->removeItem(endFlag);
                  scene->removeItem(endItem);
              }
              else {
                  qDebug() << "Nothing happened";
              }

      }
      qDebug() << "The map Vector" << mapPoints;
    }else
    qDebug() << "clicked here";
    }
}

void MainWindow::populateScene(){

//    QString filePath = QFileDialog::getOpenFileName(this, "Open File", QDir::homePath(), "Text Files (*.txt)");
//    std::ifstream file(filePath.toStdString());

    std::ifstream file("C:/Users/sivad/Desktop/Gitnow/Map_Path_Finder/chip/graph_dc_area.2022-03-11.txt");

    if (!file.is_open()) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","Failed to open the file");
        messageBox.setFixedSize(500,200);
    }


    std::vector<Vertex> nodeVec;
    std::vector<Edge> edgeVec;
    std::vector<Point> newPlot;


    std::vector<Edge> edges;
    std::multimap<int, Edge> edgeLookUp;

    double minLat = std::numeric_limits<double>::max();
    double maxLat = std::numeric_limits<double>::lowest();
    double minLon = std::numeric_limits<double>::max();
    double maxLon = std::numeric_limits<double>::lowest();

    std::string line;
    while (std::getline(file, line)) {
        if(line[0]=='V'){
            std::istringstream iss(line);
            std::string v, vid, lat, lon, x, y;
            std::getline(iss, v, ',');
            std::getline(iss, vid, ',');
            std::getline(iss, lat, ',');
            std::getline(iss, lon, ',');
            std::getline(iss, x, ',');
            std::getline(iss, y, ',');

//            if(std::stoi(vid)>=193 && std::stoi(vid)<=196){
                Vertex vnode{ std::stoi(vid), std::stod(lat), std::stod(lon), 0, 0, 0, 0 };
                vertices[std::stoi(vid)] = vnode;
                if (std::stod(lat) < minLat) {
                    minLat = std::stod(lat);
                }
                if (std::stod(lat) > maxLat) {
                    maxLat = std::stod(lat);
                }
                if (std::stod(lon) < minLon) {
                    minLon = std::stod(lon);
                }
                if (std::stod(lon) > maxLon) {
                    maxLon = std::stod(lon);
                }
//            }
        }else if(line[0]=='E'){
            std::istringstream iss(line);
            std::string e, esrc, edest, edist, ename, e0, e1;
            std::getline(iss, e, ',');
            std::getline(iss, esrc, ',');
            std::getline(iss, edest, ',');
            std::getline(iss, edist, ',');
            std::getline(iss, ename, ',');
            std::getline(iss, e0, ',');
            std::getline(iss, e1, ',');

//            if(std::stoi(esrc)>=193 && std::stoi(esrc)<=196 && std::stoi(edest)>=193 && std::stoi(edest)<=196){
            Edge enode{ std::stoi(esrc), std::stoi(edest), std::stod(edist), ename };
            edgeLookUp.insert({std::stoi(esrc), enode});
//            }
        }
    }

    qDebug() << "Min latitude: " << minLat;
    qDebug() << "Max latitude: " << maxLat;
    qDebug() << "Min longitude: " << minLon;
    qDebug() << "Max longitude: " << maxLon;

    int xdiff = (maxLon - minLon)*1000;
    int ydiff = (maxLat - minLat)*1000;

    qDebug() << "X diff: "<< xdiff;
    qDebug() << "Y diff: "<< ydiff;

    QImage image(((2*margin)+(ydiff*thres)), ((2*margin)+(xdiff*thres)), QImage::Format_RGB32);
    image.fill(Qt::black);

    QColor gold(255, 215, 0);
    QColor red(255, 0, 0);
    QColor green(0, 255, 0);
    QColor neonBlue(102, 255, 255);

    QPainter painter(&image);
    QPen pen(gold);
    painter.setPen(pen);

    qDebug() << "Img Width: " << image.width();
    qDebug() << "Img Height: " << image.height();

    int count = 0;

    for (auto& [id, v] : vertices) {

        v.y = project2map(v.latitude, minLat, thres)+margin;
        v.x = project2map(v.longitude , minLon, thres)+margin;

        for (int x = v.y - radius; x <= v.y + radius; x++) {
            for (int y = v.x - radius; y <= v.x + radius; y++) {
                if (x >= 0 && x < image.width() && y >= 0 && y < image.height()) {

//                    image.setPixelColor(x, image.height()-y, red);

                    if(x==v.y && y==v.x){
                        v.yy = image.height()-y;
                        v.xx = x;
                        QGraphicsItem *item = new house(neonBlue, v.xx-radius, v.yy-radius, 2*radius+1, v.id);
                        item->setPos(QPointF(v.xx-radius, v.yy-radius));
                        scene->addItem(item);
                        scene->update();
//                        qDebug() << "id: " << v.id << " ; x: " << x << " ; y: " << image.height()-y;
                        count = count + 1;
                    }
                }
            }
        }
    }

    for (auto& [id, e] : edgeLookUp) {
        int src = e.src;
        int dest = e.dest;
        painter.drawLine(vertices[src].y, image.height()-vertices[src].x, vertices[dest].y, image.height()-vertices[dest].x);
    }

    qDebug() << "Actual Vertices count: " << vertices.size() ;

    painter.end();

    QPixmap pixmap = QPixmap::fromImage(image);
    QGraphicsPixmapItem *pixmapItem = scene->addPixmap(pixmap);
}
