#ifndef PROJ_NODE_H
#define PROJ_NODE_H
using namespace std;
#include "Edge.h"
#include "Coordinates.h"
#include <string>
#include <utility> 
#include <limits>

// Forward declaration of Edge class
class Edge;

/**
 * @class Node
 * @brief Represents a station or point in a transportation network with geographic coordinates.
 *
 * The Node class stores station information, geographic location, and algorithm-related variables for pathfinding.
*/
class Node {
    public: 
        // Station information
        std::string id;     ///< Unique identifier for the node
        std::string code;   ///< Station code or short identifier
        std::string name;   ///< Human-readable station name
        std::string desc;   ///< Description of the node
        Coordinates coordinates; ///< Geographic coordinates 
        std::string zone;   ///< Zone or area classification
        std::string type;   ///< Type of station/point

        //Algorithm Variables
        double distance =std::numeric_limits<double>::max(); ///< Current distance from start node
        Edge* previous = nullptr; ///< Previous node in the shortest path
        bool visited = false; ///< Visited flag for traversal algorithms
        double bestDistance = std::numeric_limits<double>::max(); ///< Best known distance
        Time arrivalTime; ///< Estimated arrival time at this node

        /**
         * @brief Construct a new Node object
         * @param id Unique node identifier
         * @param latitude Geographic latitude in degrees
         * @param longitude Geographic longitude in degrees
         * @param name Human-readable name
         * @param type Node type
         * @param code Station code
        */
        Node(const string& id, double latitude, double longitude, string name = "", string type = "", string code = "");

        /**
         * @brief Reset algorithm variables to their default state
        */
        void reset();

        /**
         * @brief Convert the node's coordinates to a 3D point
         * @return Point3D representation of the node's coordinates
        */
        Point3D toPoint3D() const;

        /**
         * @brief Comparison operator for priority queue ordering
         * @param other Node to compare against
         * @return true if this node has higher priority
        */
        bool operator<(const Node& other) const {
            if (bestDistance != other.bestDistance) {
                return bestDistance < other.bestDistance;
            }
            return name < other.name;
        }
};



#endif //PROJ_GRAPH_H