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


#include <lemon/list_graph.h>

#define R0 6378137
// #define 

using namespace lemon;
using namespace std;

struct Vertex : public ListDigraph::Node
{
    ListDigraph::Node graphNode;
    int graphID_;
    int uid_;
    double long_;
    double lat_;
    double x_;
    double y_;
    std::vector<Vertex> adjacencyList_;
    std::vector<int> adjacencyListIDs_;


    // DEFAULT CONSTRUCTOR IS NEEDED FOR NODE MAP
    Vertex() {
        // std::cout << id << std::endl;
    }
    Vertex(int uid, double lon, double lat, ListDigraph::Node& n, int graphID) : uid_(uid), long_(lon), lat_(lat)
    {
        // std::cout << id << ", " << uid << std::endl;
        id = uid;
        graphNode = n;
        graphID_ = graphID;
        // ListDigraph::Node b = *n;
        // cout << b.id << endl;

    }
    friend std::ostream& operator<< (std::ostream& stream, const Vertex& node) {
        return stream << "ID: " << node.uid_ << ", Lat: " << node.lat_ << ", Long: " << node.long_;
    }

    void setID(int newID){
        id = newID;
    }
};

// struct VertexMap : public ListDigraph::NodeMap<Vertex>{
//     VertexMap(const ListDigraph& graph){
        
//     }
// };

struct Edge : public ListDigraph::Arc
{
    // Vertex from_;
    // Vertex to_;
    int fromID_;
    int toID_;
    double length_;

    // DEFAULT CONSTRUCTOR IS NEEDED FOR NODE MAP :: deprecated
    Edge() {}
    // Edge(Vertex from, Vertex to) : from_(from), to_(to) {}
    Edge(int from, int to, double length) : fromID_(from), toID_(to), length_(length) {}

    friend std::ostream& operator<< (std::ostream& stream, const Edge& edge) {
        return stream << "From Vertex: " << edge.fromID_ << ", to Vertex: " << edge.toID_ << ", length: " << edge.length_;
    }
};

// struct Graph
class Graph: public ListDigraph{
public:
    Graph(char *file) {
        read_file(file);
        // ListDigraph* base = dynamic_cast<ListDigraph*>(this);
        // nodeMap((ListDigraph*)this);
        ListDigraph::NodeMap<int> nodeMap(*this);
        // vertexMap(*this);
        // nmap.WriteMap(nodeMap);
    }

    std::map<int, Vertex> vertices;
    std::vector<Edge> edges;
    std::multimap<int, Edge> edgeLookUp;
    // VertexMap vertexMap;
    // ListDigraph::NodeMap<int> nmap;
    // ListDigraph::NodeMap<Vertex> nodeMap;
    // ListDigraph::ArcMap<std::string> arcMap;

    void read_file(std::string fileName)
    {
        std::ifstream fin(fileName, std::ios::in);

        std::string line;
        std::string element;
        std::vector<std::string> row;

        double sum_lat=0;
        double sum_long=0;


        while (std::getline(fin, line))
        {
            // vertex
            if (line[0] == 'V')
            {
                // read csv format
                row.clear();
                stringstream str(line);
                while(getline(str, element, ','))
                    row.push_back(element);
                
                sum_lat += std::stod(row[3]);
                sum_long += std::stod(row[2]);

                ListDigraph::Node n = this->addNode();

                // cout << "ID : " << this->id(n) << endl;

                vertices[std::stoi(row[1])] = Vertex(std::stoi(row[1]), std::stod(row[2]), std::stod(row[3]),n, this->id(n));
                vertices[std::stoi(row[1])].setID(std::stoi(row[1]));
                
            }
            // edge
            else if (line[0] == 'E')
            {
                // read csv format
                row.clear();
                stringstream str(line);
                while(getline(str, element, ','))
                    row.push_back(element);
                

                edges.push_back(Edge(std::stoi(row[1]), std::stoi(row[2]), std::stod(row[3])));
                edgeLookUp.insert({std::stoi(row[1]), Edge(std::stoi(row[1]), std::stoi(row[2]), std::stod(row[3]))});

                this->addArc(row);
                // std::cout << "Edge: " << line << std::endl;

            }
        }

        double center_latitude = sum_lat/vertices.size();
        double center_longitude = sum_long/vertices.size();
        
        // std::cout << "Center: " << center_latitude << center_longitude << std::endl;
        // for (const auto n: vertices)
        // {
        //     cout << this->id((n.second.graphNode)) << endl;
        //     std::cout << n.second << std::endl;
        // }
        // for (const auto e: edges)
        //     std::cout << e << std::endl;

        convertToCartesian(center_latitude, center_longitude);
        std::reverse(edges.begin(), edges.end());
        
    }

    // TODO: eventually scale a little
    void convertToCartesian(double center_latitude, double center_longitude){
        for (auto &v : vertices)
        {
            double latRad = v.second.lat_ * M_PI / 180.;
            double longRad = v.second.long_ * M_PI / 180.;
            double centerLatRad = center_latitude * M_PI / 180.;
            double centerLongRad =  center_longitude * M_PI / 180.;
            v.second.x_ = R0 * cos(centerLatRad) * (longRad-centerLongRad);
            v.second.y_ = R0 * log(tan(((latRad-centerLatRad)/2) + (M_PI/4)));
        }

    }
    // using ListDigraph::addNode;
    // ListDigraph::Node addNode(std::vector<std::string> row){
    //     // addNode();
    //     ListDigraph::Node v = this->addNode();
    //     // v.id = row[1];
        
    //     return this->addNode();
    // }

    using ListDigraph::addArc;
    ListDigraph::Arc addArc(std::vector<std::string> row){
        // std::stoi(row[1]), std::stoi(row[2]),

        // ListDigraph::Node u = nodeFromId(std::stoi(row[1]));
        // ListDigraph::Node v = nodeFromId(std::stoi(row[2]));
        int fromID = std::stoi(row[1]);
        int toID = std::stoi(row[2]);

        ListDigraph::Node u = vertices[fromID].graphNode;
        ListDigraph::Node v = vertices[toID].graphNode;

        vertices[fromID].adjacencyList_.push_back(vertices[toID]);
        vertices[fromID].adjacencyListIDs_.push_back(toID);

        // TO SORT OR NOT TO SORT, THAT IS THE QUESTION
        std::sort(vertices[fromID].adjacencyListIDs_.begin(), vertices[fromID].adjacencyListIDs_.end());


        return this->addArc(u, v);
    }

    ListDigraph::Node nodeFromID(int id){
        // ListDigraph::Node* node = dynamic_cast<ListDigraph::Node*>(vertex);
        return (ListDigraph::Node)vertices[id];
    }

    // int id(ListDigraph::Node node){
        
    //     auto findResult = std::find_if(std::begin(vertices), std::end(vertices), [&](const std::pair<int, Vertex> &pair)
    //     {
    //         return (ListDigraph::Node)pair.second == node;
    //     });

    //     if (findResult != std::end(vertices))
    //         return findResult->first;
    //     else
    //         return -1;
    // }
    // void populateNodeMap(ListDigraph::NodeMap<Vertex> &nodeMap){
    //     for (auto const vertex: vertices)
    //     {
    //         nodeMap.set(vertex.second, vertex.first);
    //     }
        
    // }

    // std::pair<int, int> 

    double getLength(int fromID, int toID){
        for (auto it = edgeLookUp.lower_bound(fromID); it != edgeLookUp.upper_bound(fromID); it++)
        {
            if (it->second.toID_ == toID)
                return it->second.length_;
        }
        return 0.0;

    }

    std::vector<std::pair<int, double>> backtrace(const std::map<int, int> &parents, int start, int goal)
    {
        int currentNode = goal;
        int prevNode = parents.at(goal);
        std::vector<int> path{goal};

        // std::cout << "\n \n " << getLength(prevNode, goal) << ", "<< getLength(parents.at(prevNode), prevNode) << std::endl;
        // std::vector<std::pair<int, double>> path_{std::make_pair(goal, getLength(prevNode, goal)), std::make_pair(prevNode, getLength(parents.at(prevNode), prevNode))};
        std::vector<std::pair<int, double>> path_;
        
        // path.push_back(prevNode);
        while(prevNode != start)
        {
            path_.push_back(std::make_pair(currentNode, getLength(prevNode, currentNode)));
            // int tmp = currentNode;
            currentNode = prevNode;
            prevNode = parents.at(currentNode);

            path.push_back(prevNode);

            // auto it = edgeLookUp.find(prevNode);
            // int parent = parents.at(prevNode);
            // cout << "Length from " << parent << " to " << prevNode << " has length of ";

            // path_.push_back(std::make_pair(parent, getLength(parent, prevNode)));
            // prevNode = parents.at(prevNode);

            // for (auto it = edgeLookUp.lower_bound(parent); it != edgeLookUp.upper_bound(parent); it++)
            // {
            //     // cout << it->second << std::endl;
            //     if (it->second.toID_ == prevNode)
            //     {
            //         path_.push_back(std::make_pair(parent, it->second.length_));
            //         cout << it->second.length_ << endl;
            //         break;
            //     }
            //         // cout << "found" << endl;
            // }
            // this->
        }
        path.push_back(prevNode);

        path_.push_back(std::make_pair(currentNode, getLength(prevNode, currentNode)));
        path_.push_back(std::make_pair(start,0));
        // path_.push_back(std::make_pair(start, getLength(start, prevNode)));        
        
        std::reverse(path.begin(), path.end());
        std::reverse(path_.begin(), path_.end());

        return path_;
        int count = 0;
        for (auto node : path)
        {
            count++;
            cout << node << "," << endl;
            
        }
        cout << endl;
        cout << count << endl;


    }

    std::vector<std::pair<int, double>> bfs(Vertex start, Vertex goal){
        int length;
        int numberOfVertices = 0;

        std::deque<int> active_queue;
        std::set<int> closed_set;
        std::map<int, int> parent;
        
        // <node ID, length until that node>
        std::vector<std::pair<int, double>> path;

        // ID of the start vertex
        active_queue.push_back(start.uid_);

        while (active_queue.size() != 0)
        {
            int vcurrent = active_queue.front(); 
            // std::cout << "Vertex [ " << numberOfVertices << "] = " << vcurrent <<  ", goal: " << goal.uid_ << std::endl;
            if (vcurrent == goal.uid_)
            {
                int vprev;
                cout << "Total visited vertex: " << numberOfVertices << endl;
                // cout << "Closed set: " << closed_set.size() << endl;

                return backtrace(parent, start.uid_, goal.uid_);
            }
            // cout << "test";
            active_queue.pop_front();
            closed_set.insert(vcurrent);
            // cout << vertices[vcurrent].adjacencyList_[1] << endl;
            numberOfVertices++;
            for (auto vnext : vertices[vcurrent].adjacencyListIDs_)
            {
                if (closed_set.find(vnext) != closed_set.end())
                {
                    continue;
                }
                if (std::find(active_queue.begin(), active_queue.end(), vnext) == active_queue.end())
                {
                    // cout << "Push back: " << vnext.uid_ << endl;
                    active_queue.push_back(vnext);
                    // if (vnext == 17796)
                    //     cout << vcurrent << endl;
                    parent[vnext] = vcurrent;
                }
            }
            
            // if (numberOfVertices > 28)
            //     return "";
        } 

        // backtrace(parent, start.uid_, goal.uid_);

        cout << "Number of vertices visited: " << numberOfVertices << endl;
        // cout << "Closed set: " << closed_set.size() << endl;
        
        return path;
        // std::pair<int, int> result = std::make_pair(length, numberOfVertices);
        // return result;

    }


    void printPath(std::vector<std::pair<int,double>> path){
        int count = 1;
        double accumulatedLength = 0;
        std::cout << "Total vertex on path from start to end = " << path.size() << std::endl;
        // std::cout.precision(3);
        for (const auto node : path)
        {
            accumulatedLength += node.second;
            // std::cout << accumulatedLength << " + " << node.second << std::endl;
            std::cout << "Vertex[" << std::setw(4) << count << "]" << " = " << std::setw(8) << node.first << ", length = " << std::setw(12) << setprecision(2) << fixed << accumulatedLength << std::endl;
            count++;
        }
    }

    
};


int main(int argc, char *argv[])
{
    // READ FILE

    // read_file(argv[1]);

    // create graph
    Graph g(argv[1]);

    ListDigraph::NodeMap<Vertex> nodeMap(g);
    
    
    // graph.ListDigraph::addNode();

    // ListDigraph g;
    // ListDigraph::Node u = g.addNode();
    // ListDigraph::Node v = g.addNode();
    // ListDigraph::Arc a = g.addArc(u, v);
    // ListDigraph::NodeMap<Vertex> nodeMap(g);
    // ListDigraph::ArcMap<std::string> arcMap(g);
    // nodeMap[u] = Vertex(1, 2.0, 3.0);
    // nodeMap[v] = Vertex(5, 2.0, 3.0);


    //   nodeMap[v] = "2";
    //   arcMap[a] = "3";

    cout << "Hello World! This is LEMON library here." << endl;
    cout << "We have a directed graph with " << countNodes(g) << " nodes "
         << "and " << countArcs(g) << " arc.\n" << endl;


    // for (ListDigraph::NodeIt n(g); n!= INVALID; ++n)
    //     cout << g.id(n) << endl;
    
    // for (std::map<int, Vertex>::iterator it = g.vertices.begin(); it != g.vertices.end(); it++)
    // {
    //     cout << it->first << ", x: " << it->second.x_ << ", y: " << it->second.y_ << endl;
    //     cout << "GraphNode id is: " << g.id((it->second.graphNode)) << ", " << it->second.graphID_ << endl;
    //     cout << "Adjacency list of node " << it->second.uid_ << ": ";
    //     for (const auto node : it->second.adjacencyList_)
    //         // cout <<  g.id((node.graphNode)) << "(" << node.graphID_ << ")" << ", ";
    //         cout <<  node.uid_ << "(" << node.graphID_ << ")" << ", ";
    //     cout << endl;
    // }

    // std::string result = g.bfs(g.vertices[0], g.vertices[13]);
    // for (auto v : g.vertices[86771].adjacencyList_)
    //     cout << v << endl;
    auto path = g.bfs(g.vertices[86771], g.vertices[110636]);

    g.printPath(path);
    // std::string result = g.bfs(g.vertices[1], g.vertices[11]);
    // cout << result << std::endl;

    // for (const auto n : g.vertices[17779].adjacencyListIDs_)
    //     cout << n << endl;

    return 0;
}