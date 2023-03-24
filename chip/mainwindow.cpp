// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "mapChips.cpp"
#include "chip.h"
#include "mainwindow.h"
#include "qmessagebox.h"
#include "view.h"
#include <QFileDialog>
#include <QHBoxLayout>
#include <QSplitter>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <limits>


using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent), scene(new QGraphicsScene(this))
    , h1Splitter(new QSplitter(this)), h2Splitter(new QSplitter(this))
{
    populateScene();

    QSplitter *vSplitter = new QSplitter;
    vSplitter->setOrientation(Qt::Vertical);
    vSplitter->addWidget(h1Splitter);
    vSplitter->addWidget(h2Splitter);

    View *view = new View("Map Explorer");
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
};

struct Edge {
    int src;
    int dest;
    double dist;
    std::string name;
};


#define PI 3.14159265358979323846
int R = 6371009;

struct Point {
    double x, y;
};

#define DEG2RAD(a)   ((a) / (180 / M_PI))
#define RAD2DEG(a)   ((a) * (180 / M_PI))
#define EARTH_RADIUS 6378137

/* The following functions take their parameter and return their result in degrees */

//double y2lat_d(double y)   { return RAD2DEG( atan(exp( DEG2RAD(y) )) * 2 - M_PI/2 ); }
//double x2lon_d(double x)   { return x; }

double lat2y(double lat) { return RAD2DEG( log(tan( DEG2RAD(lat) / 2 +  M_PI/4 )) ); }
double lon2x(double lon) { return EARTH_RADIUS*(RAD2DEG(lon)); }

double project2map(double data, double min, int thres){
    return (int)((data-min)*thres*1000);
}

// Compute the distance between two points in Cartesian coordinates
double distance(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}


void MainWindow::populateScene(){


    QString filePath = QFileDialog::getOpenFileName(this, "Open File", QDir::homePath(), "Text Files (*.txt)");
    std::ifstream file(filePath.toStdString());

//    std::ifstream file("C:/Users/sivad/Desktop/Gitnow/Map_Path_Finder/chip/graph_dc_area.2022-03-11.txt");

    if (!file.is_open()) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","Failed to open the file");
        messageBox.setFixedSize(500,200);
    }

    std::vector<Vertex> nodeVec;
    std::vector<Edge> edgeVec;

    std::map<int, Vertex> vertices;
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
                Vertex vnode{ std::stoi(vid), std::stod(lat), std::stod(lon), 0, 0 };
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


    int thres = 30;
    int margin = 100;
    int radius = thres/10;

    QImage image(((2*margin)+(ydiff*thres)), ((2*margin)+(xdiff*thres)), QImage::Format_RGB32);
    image.fill(Qt::black);

    QColor gold(255, 215, 0);
    QColor red(255, 0, 0);
//    QColor neonBlue(102, 255, 255);

    QPainter painter(&image);
    QPen pen(gold);
    painter.setPen(pen);


    qDebug() << "Img Width: " << image.width();
    qDebug() << "Img Height: " << image.height();

    int count = 0;

    for (auto& [id, v] : vertices) {

        v.y = project2map(v.latitude, minLat, thres)+margin;
        v.x = project2map(v.longitude , minLon, thres)+margin;

//        QGraphicsItem *item = new Chip(red, v.x, v.y, radius);
//        item->setPos(QPointF(v.x, image.height()-v.y));
//        scene->addItem(item);

        for (int x = v.y - radius; x <= v.y + radius; x++) {
            for (int y = v.x - radius; y <= v.x + radius; y++) {
                if (x >= 0 && x < image.width() && y >= 0 && y < image.height()) {

                    image.setPixelColor(x, image.height()-y, gold);

                    if(x==v.y && y==v.x){
                        QGraphicsItem *item = new Chip(red, x-radius, image.height()-y-radius, 2*radius+1, v.id);
                        item->setPos(QPointF(x-radius, image.height()-y-radius));
                        scene->addItem(item);
                        scene->update();
//                        qDebug() << "id: " << v.id << " ; x: " << x << " ; y: " << image.height()-y;
                        count = count + 1;
                    }
                }
            }
        }


//        break;
    }

    for (auto& [id, e] : edgeLookUp) {
        int src = e.src;
        int dest = e.dest;
        painter.drawLine(vertices[src].y, image.height()-vertices[src].x, vertices[dest].y, image.height()-vertices[dest].x);
    }

    qDebug() << "Vertices count: " << count ;

    qDebug() << "Actual Vertices count: " << vertices.size() ;

    painter.end();

    QPixmap pixmap = QPixmap::fromImage(image);
    QGraphicsPixmapItem *pixmapItem = scene->addPixmap(pixmap);
}
