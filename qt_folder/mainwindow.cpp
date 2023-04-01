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
//int radius = thres/10;
int nodeRadius = 15;
int lineThickness = 5;
int coordinateOffset = 12000;

double minLat;
double maxLat;
double minLon;
double maxLon;

QMessageBox::StandardButton reply;

using namespace std;
View* view;
MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent), scene(new QGraphicsScene(this))
    , h1Splitter(new QSplitter(this)), h2Splitter(new QSplitter(this))
{

    QString filePath = QFileDialog::getOpenFileName(this, "Open File", QDir::homePath(), "Text Files (*.txt)");

    std::ifstream file(filePath.toStdString());
//    std::ifstream file{"/Users/Pipo/Documents/University/Embedded C++/Map_Path_Finder/console_application_folder/dataset/graph_dc_area.2022-03-11.txt"};

    graph = Graph{filePath.toStdString()};
//    graph = Graph{"/Users/Pipo/graph_dc_area.2022-03-11.txt"};

//    graph = Graph{"C:/Users/sivad/Desktop/Gitnow/Map_Path_Finder/dataset/graph_dc_area.2022-03-11.txt"};

    minLat = graph.minLat;
    maxLat = graph.maxLat;
    minLon = graph.minLon;
    maxLon = graph.maxLon;

    qDebug() << "Min latitude: " << minLat;
    qDebug() << "Max latitude: " << maxLat;
    qDebug() << "Min longitude: " << minLon;
    qDebug() << "Max longitude: " << maxLon;

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
         QPointF plotVertexPoint(plotVertex.xView_, iheight-plotVertex.yView_);
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
          QPointF point = graphicsView->mapToScene(mouseEvent->pos());
          qDebug() <<"Click at: "<<point.x()<< " " <<point.y();
          if(mapPoints.size()==0){
              mapPoints.push_back(point);
              qDebug() << "Start Element";

              startFlag = new flagItem(point.x(), point.y(), Qt::red);
              scene->addItem(startFlag);

              //          start
              startNode = findNearest(point);
              qDebug() << "Selected node: " << startNode;
               startItem = new house(Qt::red, graph.vertices[startNode].xView_, iheight-graph.vertices[startNode].yView_, nodeRadius, graph.vertices[startNode].uid_);
//               startItem->setPos(QPointF(graph.vertices[startNode].xView_, iheight-graph.vertices[startNode].yView_));
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

              //          start
               endNode = findNearest(point);
               qDebug() << "Selected node: " << endNode;
               endItem = new house(Qt::green, graph.vertices[endNode].xView_, iheight-graph.vertices[endNode].yView_, nodeRadius, graph.vertices[startNode].uid_);
//               endItem->setPos(QPointF(graph.vertices[endNode].xView_-radius, iheight-graph.vertices[endNode].yView_-radius));
               scene->addItem(endItem);

               coords->setPlainText(coords->toPlainText()+", End Point = "+QString::number(endNode));

               scene->update();
              //          end
          }
          qDebug() << "The map Vector" << mapPoints;
        }
        }
    }

void MainWindow::SelectAlgo(int index){
    qDebug() << "Selected Algo: " << index;
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
        }
    }else{
        reply = QMessageBox::information(this, "Select Nodes : Warning", "Start nodes and the end nodes are not selected in the map", QMessageBox::Ok);
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

void MainWindow::populateScene(int algo){

    int xdiff = (maxLat - minLat)*1000;
    int ydiff = (maxLon - minLon)*1000;

    qDebug() << "X diff: "<< xdiff;
    qDebug() << "Y diff: "<< ydiff;

    int imageSize = 50000;

//    QImage image(((2*margin)+(ydiff*thres)), ((2*margin)+(xdiff*thres)), QImage::Format_RGB32);
    QImage image(imageSize,imageSize, QImage::Format_RGB32);
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

    iheight = image.height();

    for (auto& [id, v] : graph.vertices) {
        v.xView_ = project2map(v.long_, minLon, thres)+margin;
        v.yView_  = project2map(v.lat_ , minLat, thres)+margin;

//        if(v.xView_ - radius >= 0 && v.xView_ - radius < image.width() && v.yView_ - radius >= 0 && v.yView_ - radius < image.height()) {
//            QGraphicsItem *item = new house(gold, v.xView_-radius, iheight-v.yView_-radius, 2*radius+1, v.uid_);
//            item->setPos(QPointF(v.xView_-radius, iheight-v.yView_-radius));
//            scene->addItem(item);
//            scene->update();
//        }
    }


    for (auto& [id, v] : graph.vertices) {
//        v.xView_ = project2map(v.long_, minLon, thres)+margin;
//        v.yView_  = project2map(v.lat_ , minLat, thres)+margin;


//        if(v.x_ - radius >= 0 && v.x_ - radius < image.width() && v.yView_ - radius >= 0 && v.yView_ - radius < image.height()) {
        double x = (v.x_ + coordinateOffset) * 2;
        double y = (v.y_ + coordinateOffset) * 2;
        v.xView_ = x;
        v.yView_ = y;
            QGraphicsItem *item = new house(gold, x, iheight-y, nodeRadius, v.uid_);
//            item->setPos(QPointF(x, iheight-y));
            scene->addItem(item);
//            scene->update();
//        }
    }

//    for (auto& [id, e] : graph.edgeLookUp) {
//           int src = e.fromID_;
//           int dest = e.toID_;
//           painter.drawLine(graph.vertices[src].xView_, iheight-graph.vertices[src].yView_, graph.vertices[dest].xView_, iheight-graph.vertices[dest].yView_);
//       }
//    pen.setBrush(red);
    pen.setWidth(5);
    painter.setPen(pen);
    for (auto& [id, e] : graph.edgeLookUp) {
               int src = e.fromID_;
               int dest = e.toID_;
               double xs = (graph.vertices[src].x_         + coordinateOffset)   *2    ;
               double ys = (graph.vertices[src].y_         + coordinateOffset)   *2    ;
               double xt = (graph.vertices[dest].x_        + coordinateOffset)   *2    ;
               double yt = (graph.vertices[dest].y_        + coordinateOffset)   *2    ;

               painter.drawLine(xs, iheight-ys, xt, iheight-yt);
           }

    QPen epen(green, 7);

    std::vector<std::pair<int, double>> path;
    auto start = std::chrono::high_resolution_clock::now();
    if(algo == 1){
        path = graph.bfs(startNode, endNode);
    }else if(algo == 2){
        path = graph.dijkstra(startNode, endNode);
    }else if(algo == 3){
        path = graph.astar(startNode, endNode);
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    if (algo != 0){
        for(auto v: graph.currentlyVisitedVertices){
            QGraphicsItem *item = new house(neonBlue, graph.vertices[v].xView_, iheight-graph.vertices[v].yView_, nodeRadius, v);
//            item->setPos(QPointF(graph.vertices[v].xView_-radius, iheight-graph.vertices[v].yView_-radius));
            scene->addItem(item);
        }
        Vertex edgefrom = graph.vertices[startNode];
        Vertex edgeto;
        painter.setPen(epen);
        for (auto & element : path) {
            auto id = graph.vertices[element.first];

            QGraphicsItem *item = new house(red, graph.vertices[element.first].xView_, iheight-graph.vertices[element.first].yView_, nodeRadius, element.first);
//            item->setPos(QPointF(graph.vertices[element.first].xView_-radius, iheight-graph.vertices[element.first].yView_-radius));
            scene->addItem(item);
            edgeto = graph.vertices[element.first];
            painter.drawLine(edgefrom.xView_, iheight-edgefrom.yView_, edgeto.xView_, iheight-edgeto.yView_);
//            scene->update();
            edgefrom = edgeto;
        }

        reply = QMessageBox::information(this, "Search Algorithm Result", "Info: The path compute time: " + QString::number(duration.count()) + " us." + "\nThe number of vertices in the path: " + QString::number(path.size()) + "\nThe length of the path: " + QString::number(graph.getPathLength(path)) + "\nTotal number of search vertices: " + QString::number(graph.currentlyVisitedVertices.size()+1), QMessageBox::Ok);

        qDebug() << "Info: image calculated in: " << duration.count() << " us." << "\nThe number of vertices in the path: ?" << "\nThe length of the Path: " << graph.getPathLength(path) << "\nTotal number of search vertices: " << graph.currentlyVisitedVertices.size() ;

    }
    qDebug() << "path size: " << path.size();

    qDebug() << "Actual Vertices count: " << graph.vertices.size() ;

    qDebug() << "Path length: " << graph.getPathLength(path);

    qDebug() << "Path length: " << graph.currentlyVisitedVertices.size();

    qDebug() << "Info: image calculated in: " << duration.count() << " us.";


    painter.end();
    scene->update();

    QPixmap pixmap = QPixmap::fromImage(image);
    QGraphicsPixmapItem *pixmapItem = scene->addPixmap(pixmap);

    qDebug() << ((algo==0)? "Default" : (algo==1)? "BFS" : (algo==2)? "Dijkstra" : (algo==3)? "A Star" : "Error");
}
