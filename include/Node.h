#ifndef PROJ_NODE_H
#define PROJ_NODE_H
using namespace std;
#include <unordered_map>
#include "Edge.h"
#include "Coordinates.h"
#include <vector>
#include <string>
#include <utility> 
#include <limits>
class Edge; // Forward declaration of Edge class
class Node {
    public: 
        // Station information
        string id;
        string code;
        string name;
        string desc;
        Coordinates coordinates; // Coordinates of the node
        string zone;
        string type;
        //algorithm variables
        double distance =std::numeric_limits<double>::max(); // Distance from the start node
        Edge* previous = nullptr; // Previous node in the path
        bool visited = false; // Flag to check if the node has been visited
        double bestDistance = std::numeric_limits<double>::max(); // Best distance to the node
        Node(const string& id, double latitude, double longitude, string name = "", string type = "", string code = ""); // Constructor to initialize the node
        void reset();
        Point3D toPoint3D() const; // Convert to Point3D
        bool operator<(const Node& other) const {
            if (bestDistance != other.bestDistance) {
                return bestDistance < other.bestDistance; // Lower priority comes first
            }
            return name < other.name;
        }
};



#endif //PROJ_GRAPH_H