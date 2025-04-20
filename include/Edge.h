#ifndef PROJ_EDGE_H
#define PROJ_EDGE_H
using namespace std;
#include <unordered_map>
#include <vector>
#include <string>
#include <utility> 
#include "Node.h"
#include <chrono>

struct Time {
    int hour; // Day of the date
    int minute; // Month of the date
    int second; // Year of the date
};
class Edge {
    public: 
        Node* startingNode; // Starting node of the edge
        Time time; // Date associated with the edge
        Node* destinationNode; // Destination node of the edge
        Edge(Node* startingNode, Node* destinationNode,Time date); // Constructor to initialize the edge
};



#endif //PROJ_EDGE_H