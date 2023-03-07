#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <math.h>


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


    // DEFAULT CONSTRUCTOR IS NEEDED FOR NODE MAP
    Vertex() {
        // std::cout << id << std::endl;
    }
    Vertex(int uid, double lon, double lat, ListDigraph::Node& n, int graphID) : uid_(uid), long_(lon), lat_(lat)
    {
        std::cout << id << ", " << uid << std::endl;
        // id = uid;
        graphNode = n;
        graphID_ = graphID;
        // ListDigraph::Node b = *n;
        // cout << b.id << endl;

    }
    friend std::ostream& operator<< (std::ostream& stream, const Vertex& node) {
        return stream << "ID: " << node.uid_ << ", Lat: " << node.lat_ << ", Long: " << node.long_;
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

    // DEFAULT CONSTRUCTOR IS NEEDED FOR NODE MAP
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

                cout << "ID : " << this->id(n) << endl;

                vertices[std::stoi(row[1])] = Vertex(std::stoi(row[1]), std::stod(row[2]), std::stod(row[3]),n, this->id(n));
                
                
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

                this->addArc(row);
                std::cout << "Edge: " << line << std::endl;

            }
        }

        double center_latitude = sum_lat/vertices.size();
        double center_longitude = sum_long/vertices.size();
        
        std::cout << "Center: " << center_latitude << center_longitude << std::endl;
        for (const auto n: vertices)
        {
            cout << this->id((n.second.graphNode)) << endl;
            std::cout << n.second << std::endl;
        }
        for (const auto e: edges)
            std::cout << e << std::endl;

        convertToCartesian(center_latitude, center_longitude);
        
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
};

double convert_to_cartesian()
{
    return 0.;
}



// std::vector<std::string> split_csv(std::string line, std::string element){

// }


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
         << "and " << countArcs(g) << " arc." << endl;
    // cout << a. << endl;
    // int id = g.id(u);

    // for (ListDigraph::NodeIt n(g); n!= INVALID; ++n)
    //     cout << g.id(n) << endl;
    
    for (std::map<int, Vertex>::iterator it = g.vertices.begin(); it != g.vertices.end(); it++)
    {
        cout << it->first << ", x: " << it->second.x_ << ", y: " << it->second.y_ << endl;
        cout << "GraphNode id is: " << g.id((it->second.graphNode)) << ", " << it->second.graphID_ << endl;
        cout << "Adjacency list of node " << it->second.uid_ << ": ";
        for (const auto node : it->second.adjacencyList_)
            cout <<  g.id((node.graphNode)) << "(" << node.graphID_ << ")" << ", ";
        cout << endl;
    }
    // for (ListDigraph::NodeIt n(g); n != INVALID; ++n)
    // {
    //     // g.id(n)
    //     // cout << "Node: " << nodeMap[n] << std::endl;
    //     cout << nodeMap[n].id_ << endl;
    // }

    return 0;
}