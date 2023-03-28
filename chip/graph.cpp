#include "graph.h"

// struct Graph

Graph::Graph(std::string file) {
    read_file(file);
}


void Graph::read_file(std::string fileName)
{
    std::ifstream fin(fileName, std::ios::in);

    std::string line;
    std::string element;
    std::vector<std::string> row;

    double sum_lat=0;
    double sum_long=0;


    while (std::getline(fin, line))
    {
        // read csv format
        row.clear();
        std::stringstream str(line);
        while(std::getline(str, element, ','))
            row.push_back(element);

        // vertex
        if (row[0] == "V")
        {
            sum_lat += std::stod(row[3]);
            sum_long += std::stod(row[2]);

            addVertex(row);
        }
        // edge
        else if (row[0] == "E")
        {
            addEdge(row);
        }
    }

    double center_latitude = sum_lat/vertices.size();
    double center_longitude = sum_long/vertices.size();

    convertToCartesian(center_latitude, center_longitude);
    std::reverse(edges.begin(), edges.end());

}

// TODO: eventually scale a little
void Graph::convertToCartesian(double center_latitude, double center_longitude){
    for (auto &v : vertices)
    {
        double latRad = v.second.lat_ * M_PI / 180.;
        double longRad = v.second.long_ * M_PI / 180.;
        double centerLatRad = center_latitude * M_PI / 180.;
        double centerLongRad =  center_longitude * M_PI / 180.;
        v.second.x_ = R0 * cos(centerLatRad) * (longRad-centerLongRad);
        v.second.y_ = R0 * log(tan(((latRad-centerLatRad)/2) + (M_PI/4)));
    }
    int fromID;
    int toID;
    double weight;
    for (auto &edge : edgeLookUp)
    {
        fromID = edge.second.fromID_;
        toID = edge.second.toID_;

        double x_term = pow((vertices[fromID].x_ - vertices[toID].x_), 2);
        double y_term = pow((vertices[fromID].y_ - vertices[toID].y_), 2);

        double weight = sqrt(x_term+y_term);
        // edge.second.weight_ = weight;
        // cout << "weight " << edge.second.weight_ << endl;
    }

}

void Graph::addVertex(std::vector<std::string> row){
    vertices[std::stoi(row[1])] = Vertex(std::stoi(row[1]), std::stod(row[2]), std::stod(row[3]));
}

// TODO: REMOVE LENGTH
void Graph::addEdge(std::vector<std::string> row){

    int fromID = std::stoi(row[1]);
    int toID = std::stoi(row[2]);
    double length = std::stod(row[3]);

    double x_term = pow((vertices[fromID].x_ - vertices[toID].x_), 2);
    // cout << "xterm " << x_term << endl;
    double y_term = pow((vertices[fromID].y_ - vertices[toID].y_), 2);

    double weight = sqrt(x_term+y_term);

    vertices[fromID].adjacencyList_.insert(toID);


    // TO SORT OR NOT TO SORT, THAT IS THE QUESTION -> no more sorting needed with set since its autosorted
    // std::sort(vertices[fromID].adjacencyList_.begin(), vertices[fromID].adjacencyList_.end());

    edgeLookUp.insert({std::stoi(row[1]), Edge(fromID, toID, length, weight)});
}



double Graph::get_edge_weight(int fromID, int toID){
    for (auto it = edgeLookUp.lower_bound(fromID); it != edgeLookUp.upper_bound(fromID); it++)
    {
        if (it->second.toID_ == toID)
        {
            return it->second.length_;
        }
    }
    return DBL_MAX;
}

double Graph::getLength(int fromID, int toID){
    for (auto it = edgeLookUp.lower_bound(fromID); it != edgeLookUp.upper_bound(fromID); it++)
    {
        if (it->second.toID_ == toID)
            return it->second.length_;
    }
    return 0.0;
}

void Graph::set_all_vertex_weight_to_max_value(){
    for (auto &v : vertices)
    {
        v.second.set_weight(DBL_MAX);
    }
}

std::vector<std::pair<int, double>> Graph::backtrace(const std::map<int, int> &parents, int start, int goal)
{
    int currentNode = goal;
    int prevNode = parents.at(goal);
    std::vector<std::pair<int, double>> path;

    while(prevNode != start)
    {
        path.push_back(std::make_pair(currentNode, getLength(prevNode, currentNode)));
        currentNode = prevNode;
        prevNode = parents.at(currentNode);
    }
    path.push_back(std::make_pair(currentNode, getLength(prevNode, currentNode)));
    path.push_back(std::make_pair(start,0));

    std::reverse(path.begin(), path.end());

    return path;

}

std::vector<std::pair<int, double>> Graph::bfs(int start, int goal){
    int length;
    int numberOfVertices = 0;

    std::deque<int> active_queue;
    std::set<int> closed_set;
    std::map<int, int> parent;

    // <node ID, length until that node>
    std::vector<std::pair<int, double>> path;

    // ID of the start vertex
    active_queue.push_back(start);

    while (active_queue.size() != 0)
    {

        int vcurrent = active_queue.front();
        if (vcurrent == goal)
        {
            int vprev;
            std::cout << "Total visited vertex: " << numberOfVertices << std::endl;
            currentlyVisitedVertices = closed_set;
            return backtrace(parent, start, goal);
        }
        active_queue.pop_front();
        closed_set.insert(vcurrent);

        numberOfVertices++;
        for (auto vnext : vertices[vcurrent].adjacencyList_)
        {
            if (closed_set.find(vnext) != closed_set.end())
            {
                continue;
            }
            if (std::find(active_queue.begin(), active_queue.end(), vnext) == active_queue.end())
            {
                active_queue.push_back(vnext);
                parent[vnext] = vcurrent;
            }
        }
    }
    std::cout << "Number of vertices visited: " << numberOfVertices << std::endl;


    return path;
}

std::vector<std::pair<int, double>> Graph::dijkstra(int start, int goal){
    std::deque<int> active_queue;
    std::set<int> closed_set;
    std::map<int, int> parent;
    int numberOfVertices = 0;

    auto mycompare = [this](int a, int b) -> bool {
        return this->vertices.at(b).get_weight() > this->vertices.at(a).get_weight();
    };

    set_all_vertex_weight_to_max_value();

    vertices[start].set_weight(0);
    active_queue.push_back(start);

    while (active_queue.size() != 0)
    {
        int vcurrent = active_queue.front();

        if (vcurrent == goal)
        {
            std::cout << "Total visited vertex: " << numberOfVertices << std::endl;
            return backtrace(parent, start, goal);
        }

        active_queue.pop_front();
        closed_set.insert(vcurrent);
        numberOfVertices++;

        auto starttime = std::chrono::high_resolution_clock::now();

        int newVerticesCount = 0;

        for (auto vnext :vertices[vcurrent].adjacencyList_)
        {
            if (closed_set.find(vnext) != closed_set.end())
            {
                continue;
            }
            auto w = vertices[vcurrent].get_weight() + get_edge_weight(vcurrent, vnext);

            if (std::find(active_queue.begin(), active_queue.end(), vnext) == active_queue.end())
            {
                vertices[vnext].set_weight(w);
                active_queue.emplace_back(vnext);
                parent[vnext] = vcurrent;
                newVerticesCount++;
            }
            else if (w < vertices[vnext].get_weight()){
                parent[vnext] = vcurrent;
                vertices[vnext].set_weight(w);
            }
        }
        std::partial_sort(active_queue.begin(), active_queue.begin()+newVerticesCount, active_queue.end(), mycompare);
        // std::sort(active_queue.begin(),  active_queue.end(), mycompare);
    }
    std::cout << "Number of vertices visited: " << numberOfVertices << std::endl;

    return std::vector<std::pair<int, double>>{};
}

double Graph::heuristic_distance_estimator(int fromID, int toID){
    double x_term = pow((vertices[fromID].x_ - vertices[toID].x_), 2);
    double y_term = pow((vertices[fromID].y_ - vertices[toID].y_), 2);

    double heuristic = sqrt(x_term+y_term);

    return heuristic;
}

std::vector<std::pair<int, double>> Graph::astar(int start, int goal){
    std::deque<int> active_queue;
    std::set<int> closed_set;
    std::map<int, int> parent;
    int numberOfVertices = 0;

    auto mycompare = [this](int a, int b) -> bool {
        return this->vertices.at(b).get_estimate() > this->vertices.at(a).get_estimate();
    };

    vertices[start].set_weight(0);
    active_queue.push_back(start);

    while (active_queue.size() != 0)
    {
        int vcurrent = active_queue.front();

        if (vcurrent == goal)
        {
            std::cout << "Total visited vertex: " << numberOfVertices << std::endl;
            return backtrace(parent, start, goal);
        }

        active_queue.pop_front();
        closed_set.insert(vcurrent);
        numberOfVertices++;

        auto starttime = std::chrono::high_resolution_clock::now();

        int newVerticesCount = 0;

        for (auto vnext :vertices[vcurrent].adjacencyList_)
        {
            if (closed_set.find(vnext) != closed_set.end())
            {
                continue;
            }
            auto g = vertices[vcurrent].get_weight() + get_edge_weight(vcurrent, vnext);
            auto f = g + heuristic_distance_estimator(vnext, goal);

            if (std::find(active_queue.begin(), active_queue.end(), vnext) == active_queue.end())
            {
                vertices[vnext].set_weight(g);
                vertices[vnext].set_estimate(f);
                active_queue.emplace_back(vnext);
                parent[vnext] = vcurrent;

                // count the number of vertices that are pushed into the active_queue for the partial sort
                newVerticesCount++;
            }
            else if (f < vertices[vnext].get_estimate()){
                parent[vnext] = vcurrent;
                vertices[vnext].set_weight(g);
                vertices[vnext].set_estimate(f);
            }
        }
        std::partial_sort(active_queue.begin(), active_queue.begin()+newVerticesCount, active_queue.end(), mycompare);
        // std::sort(active_queue.begin(),  active_queue.end(), mycompare);
    }
    std::cout << "Number of vertices visited: " << numberOfVertices << std::endl;

    return std::vector<std::pair<int, double>>{};
}

void Graph::printPath(std::vector<std::pair<int,double>> path){
    int count = 1;
    double accumulatedLength = 0;
    std::cout << "Total vertex on path from start to end = " << path.size() << std::endl;
    for (const auto node : path)
    {
        accumulatedLength += node.second;
//        std::cout << "Vertex[" << std::setw(4) << count << "]" << " = " << std::setw(8) << node.first << ", length = " << std::setw(12) << std::setprecision(2) << std::fixed << accumulatedLength << std::endl;
        count++;
    }
}
