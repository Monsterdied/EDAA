#include "Graph.h"
#include <iostream>

using namespace std;


void Graph::addNode(const string& id, double latitude, double longitude, string name, string type, string code, string desc, string zone) {
    Node node;
    node.id = id;
    node.latitude = latitude;
    node.longitude = longitude;
    node.name = name;
    node.type = type;
    node.code = code;
    node.desc = desc;
    node.zone = zone;

    nodes[id] = node; // Add the node to the nodes map

    // Initialize the adjacency list for this node
    adjList[id] = vector<pair<string, double>>(); 

    cout << "Node added: " << id << ", " << name << ", " << latitude << ", " << longitude << endl;
}

