#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <math.h>
#include <float.h>

#include <queue>
#include <deque>
#include <set>
#include <iomanip>

#include <chrono>

#include "graph.h"


// args
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>


#define R0 6378137


// using namespace std;

struct Vertex
{
    int uid_;
    double long_;
    double lat_;
    double x_;
    double y_;
    double weight_;
    double estimate_;
    std::set<int> adjacencyList_;


    // DEFAULT CONSTRUCTOR IS NEEDED FOR NODE MAP
    Vertex() {
        // std::cout << id << std::endl;
    }
    Vertex(int uid, double lon, double lat) : uid_(uid), long_(lon), lat_(lat), weight_(DBL_MAX), estimate_(DBL_MAX)
    {}

    friend std::ostream& operator<< (std::ostream& stream, const Vertex& node) {
        return stream << "ID: " << node.uid_ << ", weight: " << node.weight_;
    }

    double get_weight(){
        return weight_;
    }

    void set_weight(double weight){
        weight_ = weight;
    }

    double get_estimate(){
        return estimate_;
    }
    
    void set_estimate(double estimate){
        estimate_ = estimate;
    }

    friend bool operator==(const Vertex& lhs, const Vertex& rhs)
    {
        return lhs.uid_ == rhs.uid_;
    }
};

struct Edge
{
    int fromID_;
    int toID_;
    double length_;

    Edge(int from, int to, double length) : fromID_(from), toID_(to), length_(length) {}

    friend std::ostream& operator<< (std::ostream& stream, const Edge& edge) {
        return stream << "From Vertex: " << edge.fromID_ << ", to Vertex: " << edge.toID_ << ", length: " << edge.length_;
    }
};



// struct Graph
class Graph{
public:
    std::map<int, Vertex> vertices;
    std::vector<Edge> edges;
    std::multimap<int, Edge> edgeLookUp;
    
    Graph(std::string file);

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

    std::vector<std::pair<int, double>> dijkstra_priority(int start, int goal);

    std::vector<std::pair<int, double>> astar_priority(int start, int goal);

    double heuristic_distance_estimator(int fromID, int toID);

    std::vector<std::pair<int, double>> astar(int start, int goal);

    void printPath(std::vector<std::pair<int,double>> path);
};
