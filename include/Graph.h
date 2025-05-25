#ifndef PROJ_GRAPH_H
#define PROJ_GRAPH_H
using namespace std;

#include <unordered_map>
#include "Node.h"
#include "Edge.h"
#include <vector>
#include <string>
#include <utility>

/**
 * @class Graph
 * @brief Represents a graph using an adjacency list, by managing a list of nodes and edges.
*/
class Graph {
    private:
        /**
        * @brief Adjacency list representation of the graph.
        * Maps a node ID (string) to a vector of Edge pointers representing outgoing edges.
        */
        unordered_map<string, vector<Edge*>> adjList; 

        /**
         * @brief Map of nodes in the graph.
         * Maps a node ID (string) to the Node object.
        */
        unordered_map<string, Node*> nodes;	
        
    public:

        /**
         * @brief Adds a node to the graph.
         * @param node Node object to be added.
         */
        void addNode(Node* node);

        /**
        * @brief Resets aspects of the graph or its nodes.
        * @note The `const` qualifier might be problematic if this method intends to modify
        * internal state of nodes (e.g., visited flags for algorithms) unless
        * those members are declared `mutable`.
        */
        void reset()const ;

        /**
        * @brief Gets all nodes in the graph.
        * @return An unordered_map where keys are node IDs and values are Node pointers.
        */
        unordered_map<string, Node*> getNodes() const;

        /**
         * @brief Gets a specific node by its ID.
         * @param id The string ID of the node to retrieve.
         * @return Pointer to the Node object if found, nullptr otherwise.
        */
        Node* getNode(const string& id) const;

        /**
        * @brief Adds an edge to the graph.
        * @param edge Pointer to the Edge object to be added.
        */
        void addEdge(Edge* edge);

        /**
         * @brief Gets the total number of nodes in the graph.
         * @return Number of nodes
        */
        size_t getNodeCount() const;

        /**
         * @brief Gets the total number of edges in the graph.
         * @return Number of edges
        */
        int getEdgeCount() const ;

        /**
         * @brief Gets all edges adjacent to a specific node.
         * @param nodeID The string ID of the node.
         * @return A vector of Edge pointers. Returns an empty vector if the node is not found or has no outgoing edges.
        */
        vector<Edge*> getAdjacentEdges(const string& nodeID) const;

        

};



#endif //PROJ_GRAPH_H