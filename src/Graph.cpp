#include "../include/Graph.h"
#include <iostream>

using namespace std;


void Graph::addNode(Node* node) {
    nodes[node->id] = node; // Add the node to the nodes map

    // Initialize the adjacency list for this node
    adjList[node->id] = vector<Edge*>(); 

    //cout << "Node added: " << node->id << ", " << node->name << ", " << node->latitude << ", " << node->longitude << endl;
}
unordered_map<string, Node*> Graph::getNodes() const{
    return nodes; // Return the map of nodes
}

Node* Graph::getNode(const string& id) const {
    auto it = nodes.find(id); // Find the node by its ID
    if (it != nodes.end()) {
        return it->second; // Return the node if found
    } else {
        return nullptr; // Return nullptr if not found
    }
}

void Graph::addEdge(Edge* edge){
    // Add the edge to the adjacency list of the starting node
    adjList[edge->startingNode->id].push_back(edge); 
    //cout << "Edge added from " << edge->startingNode->id << " to " << edge->destinationNode->id << endl;
}
size_t Graph::getNodeCount() const {
    return nodes.size();
}
int Graph::getEdgeCount() const {
    int edgeCount = 0;
    for (const auto& pair : adjList) {
        edgeCount += pair.second.size(); // Count the edges for each node
    }
    return edgeCount;
}
vector<Edge*> Graph::getAdjacentEdges(const string& nodeID) const{
    auto it = adjList.find(nodeID); // Find the node in the adjacency list
    if (it != adjList.end()) {
        return it->second; // Return the edges if found
    } else {
        return vector<Edge*>(); // Return an empty vector if not found
    }
}
void Graph::reset() const{
    //reset Nodes
    for (auto it = nodes.begin(); it != nodes.end(); it++) {
        Node* node = it->second;
        node->reset();
    }
}