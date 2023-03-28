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
#include "graph.h"

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
QGraphicsTextItem *coords;

QGraphicsScene* backup_scene;

int temp = 0;
int thres = 30;
int iheight = 0;
int margin = 100;
int startNode = 0;
int endNode = 0;
int radius = thres/10;

using namespace std;
View* view;
MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent), scene(new QGraphicsScene(this))
    , h1Splitter(new QSplitter(this)), h2Splitter(new QSplitter(this))
{
    populateScene(0);

    QSplitter *vSplitter = new QSplitter;
    vSplitter->setOrientation(Qt::Vertical);
    vSplitter->addWidget(h1Splitter);
    vSplitter->addWidget(h2Splitter);

    view = new View("Map Explorer");
    view->view()->viewport()->installEventFilter(this);
    scene->setBackgroundBrush(Qt::black);
    view->view()->setScene(scene);
    backup_scene = scene;
    h1Splitter->addWidget(view);

    QHBoxLayout *layout = new QHBoxLayout;
    layout->addWidget(vSplitter);
    setLayout(layout);

    setWindowTitle(tr("Embedded C++ Project"));
}

struct plotVertex {
    int id;
    double latitude;
    double longitude;
    int x;
    int y;
    int xx;
    int yy;
};

struct plotEdge {
    int src;
    int dest;
    double dist;
    std::string name;
};

struct Point {
    double x, y;
};

std::map<int, plotVertex> vertices;

double project2map(double data, double min, int thres){
    return (int)((data-min)*thres*1000);
}

double distance(const QPointF& p1, const QPointF& p2)
{
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::sqrt(dx*dx + dy*dy);
}

int findNearest(QPointF point){
    plotVertex* nearestplotVertex = nullptr;
     double minDistance = std::numeric_limits<double>::max();

     for (auto& plotVertexPair : vertices) {
         plotVertex& plotVertex = plotVertexPair.second;
         QPointF plotVertexPoint(plotVertex.x, plotVertex.y);
         double d = distance(point, plotVertexPoint);
         if (d < minDistance) {
             minDistance = d;
             nearestplotVertex = &plotVertex;
         }
     }
     return nearestplotVertex->id;
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
      if(mapPoints.size()==0){
          mapPoints.push_back(point);
          qDebug() << "Start Element";

          startFlag = new flagItem(point.x(), point.y(), Qt::red);
          scene->addItem(startFlag);
//          scene->update();

          //          start
          startNode = findNearest(point);
          qDebug() << "Selected node: " << startNode;
           startItem = new house(Qt::red, vertices[startNode].x-radius, vertices[startNode].y-radius, 2*radius+1, 1);
           startItem->setPos(QPointF(vertices[startNode].x-radius, vertices[startNode].y-radius));
           scene->addItem(startItem);

           coords = new QGraphicsTextItem("Start Point = "+QString::number(startNode));
           coords->setDefaultTextColor(Qt::white);
           QFont font("Arial", 100);
           coords->setFont(font);
           coords->setPos(250, iheight);
           scene->addItem(coords);

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
           endNode = findNearest(point);
           qDebug() << "Selected node: " << endNode;
           endItem = new house(Qt::green, vertices[endNode].x-radius, vertices[endNode].y-radius, 2*radius+1, 1);
           endItem->setPos(QPointF(vertices[endNode].x-radius, vertices[endNode].y-radius));
           scene->addItem(endItem);

           coords->setPlainText(coords->toPlainText()+", End Point = "+QString::number(endNode));

           scene->update();
          //          end
      }
      else{
          qDebug() << "Nothing happened";
      }
      qDebug() << "The map Vector" << mapPoints;
    }else
    qDebug() << "clicked here";
    }
}

void MainWindow::SelectAlgo(int index){
    qDebug() << "Selected Algo: " << index;
    QMessageBox::StandardButton reply;
    if(mapPoints.size()==2){
        if(view->getComboBox()->currentIndex()==0){
            reply = QMessageBox::information(this, "Select Algorithm : Warning", "Select the algorithmn in the dropdown before mapping", QMessageBox::Ok);

        }else{
            addToScene(startFlag);
            addToScene(startItem);
            addToScene(endFlag);
            addToScene(endItem);
            addToScene(coords);
            populateScene(view->getComboBox()->currentIndex());
    //        reply = QMessageBox::information(this, "Al", "Message", QMessageBox::Ok);
        }
    }else{
        reply = QMessageBox::information(this, "Select Algorithm : Warning", "Start nodes and the end nodes are not selected in the map", QMessageBox::Ok);
    }
}

void MainWindow::removeFromScene(QGraphicsItem* itemToRemove){
    QList<QGraphicsItem*> itemList = scene->items();
    if(itemList.contains(itemToRemove))
    {
        scene->removeItem(itemToRemove);
    }
}

void MainWindow::addToScene(QGraphicsItem* itemToAdd){
    QList<QGraphicsItem*> itemList = scene->items();
    if(!itemList.contains(itemToAdd))
    {
        scene->removeItem(itemToAdd);
    }
}

void MainWindow::ResetAlgo(){
//    QGraphicsView* graphicsView = view->view();
//    graphicsView->scene()->removeItem(startFlag);

    mapPoints.clear();
    removeFromScene(coords);
    removeFromScene(startFlag);
    removeFromScene(startItem);
    removeFromScene(endFlag);
    removeFromScene(endItem);
    scene->clear();
    qDebug() << "Map Resetted!";
    populateScene(0);
}

void MainWindow::displayPath(){
    qDebug() << "display Path";
    populateScene(view->getComboBox()->currentIndex());
}

void MainWindow::populateScene(int algo){

//    QString filePath = QFileDialog::getOpenFileName(this, "Open File", QDir::homePath(), "Text Files (*.txt)");
//    std::ifstream file(filePath.toStdString());

    std::ifstream file("C:/Users/sivad/Desktop/Gitnow/Map_Path_Finder/chip/graph_dc_area.2022-03-11.txt");

    if (!file.is_open()) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","Failed to open the file");
        messageBox.setFixedSize(500,200);
    }


    std::vector<plotVertex> nodeVec;
    std::vector<plotEdge> plotEdgeVec;
    std::vector<Point> newPlot;


    std::vector<plotEdge> plotEdges;
    std::multimap<int, plotEdge> plotEdgeLookUp;


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
                plotVertex vnode{ std::stoi(vid), std::stod(lat), std::stod(lon), 0, 0, 0, 0 };
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
            plotEdge enode{ std::stoi(esrc), std::stoi(edest), std::stod(edist), ename };
            plotEdgeLookUp.insert({std::stoi(esrc), enode});
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


        if(v.y - radius >= 0 && v.y - radius < image.width() && v.x - radius >= 0 && v.x - radius < image.height()) {
            temp = image.height()-v.x;
            v.x = v.y;
            v.y = temp;
            QGraphicsItem *item = new house(gold, v.x-radius, v.y-radius, 2*radius+1, v.id);
            item->setPos(QPointF(v.x-radius, v.y-radius));
            scene->addItem(item);
            scene->update();
        }
    }

    for (auto& [id, e] : plotEdgeLookUp) {
        int src = e.src;
        int dest = e.dest;
        painter.drawLine(vertices[src].x, vertices[src].y, vertices[dest].x, vertices[dest].y);
    }

    Graph graph("C:/Users/sivad/Desktop/Gitnow/Map_Path_Finder/chip/graph_dc_area.2022-03-11.txt");
    QPen epen(green, 7);
    radius = radius*1.4;
    if(algo == 1){
        auto path = graph.bfs(startNode, endNode);
        qDebug() << "path size: " << path.size();

        for(auto v: graph.currentlyVisitedVertices){
            QGraphicsItem *item = new house(neonBlue, vertices[v].x-radius, vertices[v].y-radius, 2*radius+1, v);
            item->setPos(QPointF(vertices[v].x-radius, vertices[v].y-radius));
            scene->addItem(item);
        }
        plotVertex edgefrom = vertices[startNode];
        plotVertex edgeto;
        painter.setPen(epen);
        for (auto & element : path) {
            auto id = vertices[element.first];

            QGraphicsItem *item = new house(red, vertices[element.first].x-radius, vertices[element.first].y-radius, 2*radius+1, element.first);
            item->setPos(QPointF(vertices[element.first].x-radius, vertices[element.first].y-radius));
            scene->addItem(item);
            edgeto = vertices[element.first];
            painter.drawLine(edgefrom.x, edgefrom.y, edgeto.x, edgeto.y);
            scene->update();
            edgefrom = edgeto;
        }
    }else if(algo == 2){
        auto path = graph.dijkstra(startNode, endNode);
        qDebug() << "path size: " << path.size();

        for(auto v: graph.currentlyVisitedVertices){
            QGraphicsItem *item = new house(neonBlue, vertices[v].x-radius, vertices[v].y-radius, 2*radius+1, v);
            item->setPos(QPointF(vertices[v].x-radius, vertices[v].y-radius));
            scene->addItem(item);
        }
        plotVertex edgefrom = vertices[startNode];
        plotVertex edgeto;
        painter.setPen(epen);
        for (auto & element : path) {
            auto id = vertices[element.first];

            QGraphicsItem *item = new house(red, vertices[element.first].x-radius, vertices[element.first].y-radius, 2*radius+1, element.first);
            item->setPos(QPointF(vertices[element.first].x-radius, vertices[element.first].y-radius));
            scene->addItem(item);
            edgeto = vertices[element.first];
            painter.drawLine(edgefrom.x, edgefrom.y, edgeto.x, edgeto.y);
            scene->update();
            edgefrom = edgeto;
        }
    }else if(algo == 3){
        auto path = graph.astar(startNode, endNode);
        qDebug() << "path size: " << path.size();

        for(auto v: graph.currentlyVisitedVertices){
            QGraphicsItem *item = new house(neonBlue, vertices[v].x-radius, vertices[v].y-radius, 2*radius+1, v);
            item->setPos(QPointF(vertices[v].x-radius, vertices[v].y-radius));
            scene->addItem(item);
        }
        plotVertex edgefrom = vertices[startNode];
        plotVertex edgeto;
        painter.setPen(epen);
        for (auto & element : path) {
            auto id = vertices[element.first];

            QGraphicsItem *item = new house(red, vertices[element.first].x-radius, vertices[element.first].y-radius, 2*radius+1, element.first);
            item->setPos(QPointF(vertices[element.first].x-radius, vertices[element.first].y-radius));
            scene->addItem(item);
            edgeto = vertices[element.first];
            painter.drawLine(edgefrom.x, edgefrom.y, edgeto.x, edgeto.y);
            scene->update();
            edgefrom = edgeto;
        }
    }

    qDebug() << "Actual Vertices count: " << vertices.size() ;

    iheight = image.height();
    radius = thres/10;

    painter.end();

    QPixmap pixmap = QPixmap::fromImage(image);
    QGraphicsPixmapItem *pixmapItem = scene->addPixmap(pixmap);

    qDebug() << ((algo==0)? "Default" : (algo==1)? "BFS" : (algo==2)? "Dijkstra" : (algo==3)? "A Star" : "Error");
}
