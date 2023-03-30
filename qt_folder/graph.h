#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <math.h>

#include <queue>
#include <deque>
#include <set>

#include <chrono>

#include "graph.h"


// args
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "vertex.h"
#include "edge.h"

#define R0 6378137


// using namespace std;

// struct Graph
class Graph{
public:
    std::map<int, Vertex> vertices;
    std::vector<Edge> edges;
    std::multimap<int, Edge> edgeLookUp;
    std::set<int> currentlyVisitedVertices;
    int timeElapsed;
    double minLat;
    double maxLat;
    double minLon;
    double maxLon;

    Graph(std::string file);
    Graph(){};

    void read_file(std::string fileName);

    // TODO: eventually scale a little
    void convertToCartesian(double center_latitude, double center_longitude);

    void addVertex(std::vector<std::string> row);

    // TODO: REMOVE LENGTH
    void addEdge(std::vector<std::string> row);

    double get_edge_weight(int fromID, int toID);

    double getLength(int fromID, int toID);

    void set_all_vertex_weight_to_max_value();

    std::vector<std::pair<int, double>> backtrace(const std::map<int, int> &parents, int start, int goal);

    std::vector<std::pair<int, double>> bfs(int start, int goal);

    std::vector<std::pair<int, double>> dijkstra(int start, int goal);

    double heuristic_distance_estimator(int fromID, int toID);

    std::vector<std::pair<int, double>> astar(int start, int goal);

    void printPath(std::vector<std::pair<int,double>> path);

    double getPathLength(std::vector<std::pair<int,double>> path);
};
