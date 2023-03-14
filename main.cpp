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



#include <lemon/list_graph.h>

#define R0 6378137
#define MAX_VERTEX_WEIGHT 100000000.0
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
    double weight_;
    std::vector<Vertex> adjacencyList_;
    std::vector<int> adjacencyListIDs_;
    std::vector<Vertex*> adjacencyListLinked_;


    // DEFAULT CONSTRUCTOR IS NEEDED FOR NODE MAP
    Vertex() {
        // std::cout << id << std::endl;
    }
    Vertex(int uid, double lon, double lat, ListDigraph::Node& n, int graphID) : uid_(uid), long_(lon), lat_(lat), weight_(MAX_VERTEX_WEIGHT)
    {
        // std::cout << id << ", " << uid << std::endl;
        id = uid;
        graphNode = n;
        graphID_ = graphID;
        // ListDigraph::Node b = *n;
        // cout << b.id << endl;

    }
    friend std::ostream& operator<< (std::ostream& stream, const Vertex& node) {
        return stream << "ID: " << node.uid_ << ", weight: " << node.weight_;
    }

    void setID(int newID){
        id = newID;
    }

    double get_weight(){
        return weight_;
    }

    void set_weight(double weight){
        weight_ = weight;
    }

    friend bool operator==(const Vertex& lhs, const Vertex& rhs)
    {
        return lhs.uid_ == rhs.uid_;
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
    double weight_;

    // DEFAULT CONSTRUCTOR IS NEEDED FOR NODE MAP :: deprecated
    Edge() {}
    // Edge(Vertex from, Vertex to) : from_(from), to_(to) {}
    Edge(int from, int to, double length, double weight) : fromID_(from), toID_(to), length_(length), weight_(weight) {}

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
                

                // edges.push_back(Edge(std::stoi(row[1]), std::stoi(row[2]), std::stod(row[3])));
                

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
        int fromID;
        int toID;
        double weight;
        for (auto &edge : edgeLookUp)
        {
            fromID = edge.second.fromID_;
            toID = edge.second.toID_;
            
            double x_term = pow((vertices[fromID].x_ - vertices[toID].x_), 2);
            // cout << "xterm " << x_term << endl;
            double y_term = pow((vertices[fromID].y_ - vertices[toID].y_), 2);

            double weight = sqrt(x_term+y_term);
            edge.second.weight_ = weight;
            // cout << "weight " << edge.second.weight_ << endl;
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
        double length = std::stod(row[3]);

        double x_term = pow((vertices[fromID].x_ - vertices[toID].x_), 2);
        // cout << "xterm " << x_term << endl;
        double y_term = pow((vertices[fromID].y_ - vertices[toID].y_), 2);

        double weight = sqrt(x_term+y_term);


        ListDigraph::Node u = vertices[fromID].graphNode;
        ListDigraph::Node v = vertices[toID].graphNode;

        vertices[fromID].adjacencyList_.push_back(vertices[toID]);
        vertices[fromID].adjacencyListIDs_.push_back(toID);
        vertices[fromID].adjacencyListLinked_.push_back(&vertices[toID]);


        // TO SORT OR NOT TO SORT, THAT IS THE QUESTION
        std::sort(vertices[fromID].adjacencyListIDs_.begin(), vertices[fromID].adjacencyListIDs_.end());

        edgeLookUp.insert({std::stoi(row[1]), Edge(fromID, toID, length, weight)});
        // cout << "weight: " << weight << endl;
        // edgeLookUp.insert({std::stoi(row[1]), Edge(fromID, toID, length, std::stod(row[5]))});
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

    double get_edge_weight(int fromID, int toID){
        // auto start = chrono::high_resolution_clock::now();
        // int count=0;
        for (auto it = edgeLookUp.lower_bound(fromID); it != edgeLookUp.upper_bound(fromID); it++)
        {
            // count++;
            // cout << "coutn ; " << count <<endl;
            if (it->second.toID_ == toID)
            {
                    // auto stop = chrono::high_resolution_clock::now();

                // auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
                // cout << duration.count() << endl;
                return it->second.weight_;
            }
        }

        
        return MAX_VERTEX_WEIGHT;
    }

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
            
            //     return "";
        } 

        // backtrace(parent, start.uid_, goal.uid_);

        cout << "Number of vertices visited: " << numberOfVertices << endl;
        // cout << "Closed set: " << closed_set.size() << endl;
        
        return path;
        // std::pair<int, int> result = std::make_pair(length, numberOfVertices);
        // return result;
    }

    std::vector<pair<int, double>> backtrace_dijkstra(const std::map<int, Vertex*> &parents, Vertex &start, Vertex &goal){
        Vertex currentNode = goal;
        Vertex prevNode = *parents.at(vertices[goal.uid_].uid_);
        std::vector<int> path{goal.uid_};
        std::vector<std::pair<int, double>> path_;
        
        while(prevNode != start)
        {
            path_.push_back(std::make_pair(currentNode.uid_, getLength(prevNode.uid_, currentNode.uid_)));
            currentNode = prevNode;
            prevNode = *parents.at(currentNode.uid_);

            path.push_back(prevNode.uid_);
        }
        path_.push_back(std::make_pair(currentNode.uid_, getLength(prevNode.uid_, currentNode.uid_)));
        path_.push_back(std::make_pair(start.uid_,0));

        std::reverse(path_.begin(), path_.end());

        return path_;
    }

    std::vector<pair<int, double>> dijkstra(Vertex &start, Vertex &goal){
        int length;
        int numberOfVertices = 0;
        std::map<int, Vertex> *v = &vertices;
        auto mycompare = [v](Vertex* a, Vertex* b) -> bool {
            // cout <<  v.at(b).weight_ << " <  " << v.at(a).weight_ << endl;
            // return v->at(b).weight_ > v->at(a).weight_;
            return b->weight_ > a->weight_;
            // return b < a;
        };

        std::deque<Vertex*> active_queue;
        std::set<Vertex*> closed_set;
        // std::vector<int> closed_set;
        std::map<int, Vertex*> parent;
        
        // <node ID, length until that node>
        std::vector<std::pair<int, double>> path;

        // TODO: Emplace reference/ pointer and not a copy...
        active_queue.emplace_back(&start);
        start.set_weight(0);
        cout << "Test" << endl;
        
        while (active_queue.size() != 0)
        {
            

            Vertex vcurrent = *active_queue.front();
            // path.push_back(std::make_pair(vcurrent, 1.0));
            // std::cout << "Vertex [ " << numberOfVertices << "] = " << vcurrent <<  ", goal: " << goal.uid_ << std::endl;
            // cout << "current node: " << vcurrent << endl;
            if (vcurrent == goal)
            {
                cout << "Total visited vertex: " << numberOfVertices << endl;
                // for (auto node : path)
                //     std::cout << node.first << std::endl;
                // return backtrace(parent, start, goal);
                return backtrace_dijkstra(parent, start, goal);
            }
            active_queue.pop_front();
            // closed_set.insert(closed_set.begin(), vcurrent);
            closed_set.insert(&vcurrent);
            numberOfVertices++;
            // for (auto c : active_queue)
            //     cout << "active: " << c << endl;
            // for (auto c : closed_set)
            //     cout << "closed: "  << c << endl;
            if (numberOfVertices > 20)
                break;
            int newVerticesCount = 0;
            // auto start = chrono::high_resolution_clock::now();
            
            auto starttime = chrono::high_resolution_clock::now();
            for (auto vnext :vcurrent.adjacencyListIDs_)
            {
                // cout << "vnext " << vnext << endl;
                // if (std::find(closed_set.begin(), closed_set.end(), vnext) != closed_set.end())
                if (closed_set.find(&vertices[vnext]) != closed_set.end())
                {
                    // cout << "found " << vnext << endl;
                    continue;
                }
                // auto start = chrono::high_resolution_clock::now();
                auto w = vcurrent.get_weight() + get_edge_weight(vcurrent.uid_, vnext);
                cout << vcurrent << endl;
                if (std::find(*active_queue.begin(), *active_queue.end(), vertices[vnext]) == *active_queue.end())
                {
                    // cout << w << endl;
                    vertices[vnext].set_weight(w);
                    active_queue.emplace_back(&vertices[vnext]);
                    parent[vnext] = &vcurrent;
                    newVerticesCount++;
                }
                else if (w < vertices[vnext].get_weight()){
                    cout << "else if " << vertices[vnext] << endl;
                    // std::find(active_queue.begin(), active_queue.end(), vertices[vnext])->set_weight(w);

                    // is not the same vertex anymore as the one in the active queue
                    vertices[vnext].set_weight(w);
                }
            }

            // std::partial_sort(active_queue.begin(), active_queue.begin(), active_queue.end());
            

            // cout << "Presorted: " << endl;
            // for (auto node : active_queue)
            //     cout << node << endl;
                        // auto start = chrono::high_resolution_clock::now();

            std::partial_sort(active_queue.begin(), active_queue.begin()+newVerticesCount, active_queue.end(), mycompare);

            // std::sort(active_queue.begin(), active_queue.end(), mycompare);
            // cout << "Postsorted: " << endl;
            // for (auto node : active_queue)
            //     cout << node << endl;
            // auto stop = chrono::high_resolution_clock::now();

            // auto duration = chrono::duration_cast<chrono::microseconds>(stop - starttime);
            // cout << "time: " << duration.count() << endl;
            // if (numberOfVertices > 28)
        }
        cout << "Number of vertices visited: " << numberOfVertices << endl;
    }



     std::vector<pair<int, double>> backtrace_dijkstra_V2(const std::map<int,int> &parents, int start, int goal){
        int currentNode = goal;
        int prevNode = parents.at(goal);
        std::vector<std::pair<int, double>> path_;
        
        while(prevNode != start)
        {
            path_.push_back(std::make_pair(currentNode, getLength(prevNode, currentNode)));
            currentNode = prevNode;
            prevNode = parents.at(currentNode);
        }
        path_.push_back(std::make_pair(currentNode, getLength(prevNode, currentNode)));
        path_.push_back(std::make_pair(start,0));

        std::reverse(path_.begin(), path_.end());

        return path_;
    }


    std::vector<pair<int, double>> dijkstraV2(int start, int goal){
        std::deque<int> active_queue;
        std::set<int> closed_set;
        std::map<int, int> parent;
        int numberOfVertices = 0;

        auto mycompare = [this](int a, int b) -> bool {
            return this->vertices.at(b).weight_ > this->vertices.at(a).weight_;
        };

        vertices[start].set_weight(0);
        active_queue.push_back(start);
        
        while (active_queue.size() != 0)
        {
            int vcurrent = active_queue.front();
            
            if (vcurrent == goal)
            {
                cout << "Total visited vertex: " << numberOfVertices << endl;
                return backtrace_dijkstra_V2(parent, start, goal);
            }

            active_queue.pop_front();
            closed_set.insert(vcurrent);
            numberOfVertices++;
            
            if (numberOfVertices > 20)
                break;
                            
            auto starttime = chrono::high_resolution_clock::now();

            int newVerticesCount = 0;

            for (auto vnext :vertices[vcurrent].adjacencyListIDs_)
            {
                if (closed_set.find(vnext) != closed_set.end())
                {
                    continue;
                }
                auto w = vertices[vcurrent].get_weight() + get_edge_weight(vcurrent, vnext);

                cout << vertices[vcurrent] << endl;

                if (std::find(active_queue.begin(), active_queue.end(), vnext) == active_queue.end())
                {
                    vertices[vnext].set_weight(w);
                    active_queue.emplace_back(vnext);
                    parent[vnext] = vcurrent;
                    newVerticesCount++;
                }
                else if (w < vertices[vnext].get_weight()){
                    cout << "else " << vertices[vnext] << endl;
                    vertices[vnext].set_weight(w);
                }
            }
            std::partial_sort(active_queue.begin(), active_queue.begin()+newVerticesCount, active_queue.end(), mycompare);

        }
        cout << "Number of vertices visited: " << numberOfVertices << endl;
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

    cout << "Hello World! This is LEMON library here." << endl;
    cout << "We have a directed graph with " << countNodes(g) << " nodes "
         << "and " << countArcs(g) << " arc.\n" << endl;

    auto path = g.bfs(g.vertices[86771], g.vertices[24989]);
    g.printPath(path);
    // auto path = g.bfs(g.vertices[86771], g.vertices[110636]);

    // path = g.dijkstra(g.vertices[86771], g.vertices[24989]);
    // g.printPath(path);
    path = g.dijkstraV2(86771, 110636);
    g.printPath(path);

    // auto path = g.bfs(g.vertices[0], g.vertices[6]);

    // g.printPath(path);
    // path = g.dijkstra(g.vertices[0].uid_, g.vertices[6].uid_);

    // for (const auto n : g.vertices[17779].adjacencyListIDs_)
    //     cout << n << endl;

    return 0;
}