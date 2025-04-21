#ifndef PROJ_NODE_H
#define PROJ_NODE_H
using namespace std;
#include <unordered_map>
#include "Coordinates.h"
#include <vector>
#include <string>
#include <utility> 

class Node {
    public: 
        string id;
        string code;
        string name;
        string desc;
        double latitude;
        double longitude;
        string zone;
        string type;
        Node(const string& id, double latitude, double longitude, string name = "", string type = "", string code = "", string desc = "", string zone = ""); // Constructor to initialize the node
        Point3D toPoint3D() const; // Convert to Point3D
};



#endif //PROJ_GRAPH_H