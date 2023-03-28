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

double minLat = 38.8135;
double maxLat = 38.9945;
double minLon = -77.1166;
double maxLon = -76.9105;

using namespace std;
View* view;
MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent), scene(new QGraphicsScene(this))
    , h1Splitter(new QSplitter(this)), h2Splitter(new QSplitter(this))
{
    graph = Graph{"/Users/Pipo/Documents/University/Embedded C++/Map_Path_Finder/dataset/graph_dc_area.2022-03-11.txt"};

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

//struct plotVertex {
//    int id;
//    double latitude;
//    double longitude;
//    int x;
//    int y;
//    int xx;
//    int yy;
//};

//struct plotEdge {
//    int src;
//    int dest;
//    double dist;
//    std::string name;
//};

struct Point {
    double x, y;
};

//std::map<int, plotVertex> vertices;

double project2map(double data, double min, int thres){
    return (int)((data-min)*thres*1000);
}

double distance(const QPointF& p1, const QPointF& p2)
{
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::sqrt(dx*dx + dy*dy);
}

int MainWindow::findNearest(QPointF point){
    Vertex* nearestplotVertex = nullptr;
     double minDistance = std::numeric_limits<double>::max();

     for (auto& plotVertexPair : graph.vertices) {
         Vertex& plotVertex = plotVertexPair.second;
         QPointF plotVertexPoint(plotVertex.xView_, plotVertex.yView_);
         double d = distance(point, plotVertexPoint);
         if (d < minDistance) {
             minDistance = d;
             nearestplotVertex = &plotVertex;
         }
     }
     return nearestplotVertex->uid_;
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
               startItem = new house(Qt::red, graph.vertices[startNode].xView_-radius, graph.vertices[startNode].yView_-radius, 2*radius+1, 1);
               startItem->setPos(QPointF(graph.vertices[startNode].xView_-radius, graph.vertices[startNode].yView_-radius));
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
               endItem = new house(Qt::green, graph.vertices[endNode].xView_-radius, graph.vertices[endNode].yView_-radius, 2*radius+1, 1);
               endItem->setPos(QPointF(graph.vertices[endNode].xView_-radius, graph.vertices[endNode].yView_-radius));
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

    std::ifstream file("/Users/Pipo/Documents/University/Embedded C++/Map_Path_Finder/dataset/graph_dc_area.2022-03-11.txt");

    if (!file.is_open()) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","Failed to open the file");
        messageBox.setFixedSize(500,200);
    }


    std::vector<Point> newPlot;


    int xdiff = (maxLat - minLat)*1000;
    int ydiff = (maxLon - minLon)*1000;

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

    for (auto& [id, v] : graph.vertices) {
        v.xView_ = project2map(v.long_, minLon, thres)+margin;
        v.yView_  = project2map(v.lat_ , minLat, thres)+margin;

        if(v.xView_ - radius >= 0 && v.xView_ - radius < image.width() && v.yView_ - radius >= 0 && v.yView_ - radius < image.height()) {
//            v.v.yView_ = image.height()-v.v.yView_;
            QGraphicsItem *item = new house(gold, v.xView_-radius, v.yView_-radius, 2*radius+1, v.uid_);
            item->setPos(QPointF(v.xView_-radius, v.yView_-radius));
            scene->addItem(item);
            scene->update();
        }
    }

    for (auto& [id, e] : graph.edgeLookUp) {
           int src = e.fromID_;
           int dest = e.toID_;
           painter.drawLine(graph.vertices[src].xView_, graph.vertices[src].yView_, graph.vertices[dest].xView_, graph.vertices[dest].yView_);
       }


    QPen epen(green, 7);
    radius = radius*1.4;
    std::vector<std::pair<int, double>> path;
    if(algo == 1){
        path = graph.bfs(startNode, endNode);
        qDebug() << "path size: " << path.size();

        for(auto v: graph.currentlyVisitedVertices){
            QGraphicsItem *item = new house(neonBlue, graph.vertices[v].xView_-radius, graph.vertices[v].yView_-radius, 2*radius+1, v);
            item->setPos(QPointF(graph.vertices[v].xView_-radius, graph.vertices[v].yView_-radius));
            scene->addItem(item);
        }
        Vertex edgefrom = graph.vertices[startNode];
        Vertex edgeto;
        painter.setPen(epen);
        for (auto & element : path) {
            auto id = graph.vertices[element.first];

            QGraphicsItem *item = new house(red, graph.vertices[element.first].xView_-radius, graph.vertices[element.first].yView_-radius, 2*radius+1, element.first);
            item->setPos(QPointF(graph.vertices[element.first].xView_-radius, graph.vertices[element.first].yView_-radius));
            scene->addItem(item);
            edgeto = graph.vertices[element.first];
            painter.drawLine(edgefrom.xView_, edgefrom.yView_, edgeto.xView_, edgeto.yView_);
            scene->update();
            edgefrom = edgeto;
        }
    }else if(algo == 2){
        path = graph.dijkstra(startNode, endNode);
        qDebug() << "path size: " << path.size();

        for(auto v: graph.currentlyVisitedVertices){
            QGraphicsItem *item = new house(neonBlue, graph.vertices[v].xView_-radius, graph.vertices[v].yView_-radius, 2*radius+1, v);
            item->setPos(QPointF(graph.vertices[v].xView_-radius, graph.vertices[v].yView_-radius));
            scene->addItem(item);
        }
        Vertex edgefrom = graph.vertices[startNode];
        Vertex edgeto;
        painter.setPen(epen);
        for (auto & element : path) {
            auto id = graph.vertices[element.first];

            QGraphicsItem *item = new house(red, graph.vertices[element.first].xView_-radius, graph.vertices[element.first].yView_-radius, 2*radius+1, element.first);
            item->setPos(QPointF(graph.vertices[element.first].xView_-radius, graph.vertices[element.first].yView_-radius));
            scene->addItem(item);
            edgeto = graph.vertices[element.first];
            painter.drawLine(edgefrom.xView_, edgefrom.yView_, edgeto.xView_, edgeto.yView_);
            scene->update();
            edgefrom = edgeto;
        }
    }else if(algo == 3){
        path = graph.astar(startNode, endNode);
        qDebug() << "path size: " << path.size();

        for(auto v: graph.currentlyVisitedVertices){
            QGraphicsItem *item = new house(neonBlue, graph.vertices[v].xView_-radius, graph.vertices[v].yView_-radius, 2*radius+1, v);
            item->setPos(QPointF(graph.vertices[v].xView_-radius, graph.vertices[v].yView_-radius));
            scene->addItem(item);
        }
        Vertex edgefrom = graph.vertices[startNode];
        Vertex edgeto;
        painter.setPen(epen);
        for (auto & element : path) {
            auto id = graph.vertices[element.first];

            QGraphicsItem *item = new house(red, graph.vertices[element.first].xView_-radius, graph.vertices[element.first].yView_-radius, 2*radius+1, element.first);
            item->setPos(QPointF(graph.vertices[element.first].xView_-radius, graph.vertices[element.first].yView_-radius));
            scene->addItem(item);
            edgeto = graph.vertices[element.first];
            painter.drawLine(edgefrom.xView_, edgefrom.yView_, edgeto.xView_, edgeto.yView_);
            scene->update();
            edgefrom = edgeto;
        }
    }

    qDebug() << "Actual Vertices count: " << graph.vertices.size() ;

    qDebug() << "PAth lenght: " << graph.getPathLength(path);


    iheight = image.height();
    radius = thres/10;

    painter.end();

    QPixmap pixmap = QPixmap::fromImage(image);
    QGraphicsPixmapItem *pixmapItem = scene->addPixmap(pixmap);


    qDebug() << ((algo==0)? "Default" : (algo==1)? "BFS" : (algo==2)? "Dijkstra" : (algo==3)? "A Star" : "Error");
}
