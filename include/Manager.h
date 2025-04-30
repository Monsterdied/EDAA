#ifndef PROJ_MANAGER_H
#define PROJ_MANAGER_H

#pragma once
#include "Graph.h"
#include "KDTree.h"
using namespace std;


class Manager {

    private:

        Graph graph; // Graph object to manage the graph
        void ReadStations(const string& filename,string type);
        void ReadRoutesStops(const string& filename,string type);
        vector<Edge*> getKNearestFootEdges(Node* node, int k) const;

    public:
        //public for testing purposes
        KDTree kdTree; // KD-Tree object for nearest neighbor search
        void ReadGIFST(const string& filename,string type); // Function to read GIFTS data from a file
        void buildKDTree(); // Function to build a KD-Tree from the graph data
        vector<Edge*> shortestPathAstar(Node* startNode, const Coordinates& goal,double max_tentative=100000000) const;
        vector<pair<double,vector<Edge*>>> shortestPath(const Coordinates& start, const Coordinates& goal,double max_tentative=10000,
                                                        const int alternatives=15,const float a_star_multiplier=1.5) const; // Function to find the shortest path between two coordinates

        void printPath(const vector<Edge*>& path) const; // Function to print the path
        Manager(); // Constructor to initialize the Manager

        

    
};

#endif //PROJ_MANAGER_H
