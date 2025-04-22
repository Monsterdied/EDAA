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

    public:
        //public for testing purposes
        KDTree kdTree; // KD-Tree object for nearest neighbor search
        void ReadGIFST(const string& filename,string type); // Function to read GIFTS data from a file
        void buildKDTree(); // Function to build a KD-Tree from the graph data
        vector<Edge*> shortestPath(const Coordinates& start, const Coordinates& goal,double max_tentative=100000000) const;
        void printPath(const vector<Edge*>& path) const; // Function to print the path
        Manager(); // Constructor to initialize the Manager

        

    
};

#endif //PROJ_MANAGER_H
