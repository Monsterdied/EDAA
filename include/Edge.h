#ifndef PROJ_EDGE_H
#define PROJ_EDGE_H
using namespace std;
#include <string>
#include "Time.h"
class Node;

/**
 * @class Edge
 * @brief Represents a connection between two nodes with associated travel information.
 *
 * The Edge class connects a starting node to a destination node and stores metadata about the connection, including travel time and type.
*/
class Edge {
    public:
    string rideName;
        /// Pointer to the starting node of this edge
        Node* startingNode;

        /// Pointer to time information associated with this edge
        const Time* time;

        /// Travel time between nodes in seconds
        int travelTime;

        /// Pointer to the destination node of this edge
        Node* destinationNode;

        /// Type of connection/edge
        std::string type;

        /**
         * @brief Construct a new Edge object
         * @param startingNode Pointer to the starting node
         * @param destinationNode Pointer to the destination node
         * @param time Pointer to time information (ownership not transferred)
         * @param travelTime Travel duration in seconds
         * @param type Type classification of the edge
        */
        Edge(Node* startingNode, Node* destinationNode,Time* time,int travelTime,string type,string rideName="");
};



#endif //PROJ_EDGE_H