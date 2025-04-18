#ifndef PROJ_EDGE_H
#define PROJ_EDGE_H
using namespace std;
#include <unordered_map>
#include <vector>
#include <string>
#include <utility> 
#include "Node.h"

class Edge {
    public: 
        Node* startingNode; // Starting node of the edge
        Node* destinationNode; // Destination node of the edge
        Edge(Node* startingNode, Node* destinationNode); // Constructor to initialize the edge
};



#endif //PROJ_EDGE_H