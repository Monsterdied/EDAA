#ifndef PROJ_EDGE_H
#define PROJ_EDGE_H
using namespace std;
#include <unordered_map>
#include <vector>
#include <string>
#include <utility> 
#include "Node.h"
#include <chrono>
#include "Time.h"
class Node;
class Edge {
    public: 
        Node* startingNode; // Starting node of the edge
        Time* time; // Date associated with the edge
        int travelTime; // Travel time in seconds
        Node* destinationNode; // Destination node of the edge
        Edge(Node* startingNode, Node* destinationNode,Time* date,int travelTime); // Constructor to initialize the edge
};



#endif //PROJ_EDGE_H