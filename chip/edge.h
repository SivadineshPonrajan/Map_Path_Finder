#ifndef EDGE_H
#define EDGE_H
#include <iostream>

struct Edge
{
    int fromID_;
    int toID_;
    double length_;
    double weight_;

    Edge(int from, int to, double length, double weight) : fromID_(from), toID_(to), length_(length), weight_(weight) {}

    friend std::ostream& operator<< (std::ostream& stream, const Edge& edge) {
        return stream << "From Vertex: " << edge.fromID_ << ", to Vertex: " << edge.toID_ << ", length: " << edge.length_;
    }
};




#endif // EDGE_H
