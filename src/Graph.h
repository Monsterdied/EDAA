#ifndef PROJ_GRAPH_H
#define PROJ_GRAPH_H
using namespace std;

#include <unordered_map>
#include <vector>
#include <string>
#include <utility> 

class Graph {
    private:

        struct Node {
            string id;
            string code;
            string name;
            string desc;
            double latitude;
            double longitude;
            string zone;
            string type;
        };

        // Adjacency list representation of the graph
        unordered_map<string, vector<pair<string, double>>> adjList; 

        // List of nodes in the graph
        unordered_map<string, Node> nodes;	
        
    public: 

        void addNode(const string& id, double latitude, double longitude, string name = "", string type = "", string code = "", string desc = "", string zone = "");

        

};



#endif //PROJ_GRAPH_H