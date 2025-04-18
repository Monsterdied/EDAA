#include "Graph.h"
#include <iostream>

using namespace std;


void Graph::addNode(Node* node) {
    nodes[node->id] = node; // Add the node to the nodes map

    // Initialize the adjacency list for this node
    adjList[node->id] = vector<Edge>(); 

    cout << "Node added: " << node->id << ", " << node->name << ", " << node->latitude << ", " << node->longitude << endl;
}

