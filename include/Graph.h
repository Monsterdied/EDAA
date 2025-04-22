#ifndef PROJ_GRAPH_H
#define PROJ_GRAPH_H
using namespace std;

#include <unordered_map>
#include "Node.h"
#include "Edge.h"
#include <vector>
#include <string>
#include <utility> 
class Graph {
    private:
        // Adjacency list representation of the graph
        unordered_map<string, vector<Edge*>> adjList; 

        // List of nodes in the graph
        unordered_map<string, Node*> nodes;	
        
    public: 
        //void Graph(); // Constructor to initialize the graph
        void addNode(Node* node); // Function to add a node to the graph~
        unordered_map<string, Node*> getNodes() const; // Function to get all nodes in the graph
        Node* getNode(const string& id) const; // Function to get a node by its ID
        void addEdge(Edge* edge); // Function to add an edge between two nodes
        size_t getNodeCount() const;
        int getEdgeCount() const ;
        vector<Edge*> getAdjacentEdges(const string& nodeID) const;

        

};



#endif //PROJ_GRAPH_H