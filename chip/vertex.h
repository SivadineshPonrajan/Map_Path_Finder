#ifndef VERTEX_H
#define VERTEX_H

#include <set>
#include <iostream>
#include <float.h>

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

    double xView_;
    double yView_;


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


#endif // VERTEX_H
