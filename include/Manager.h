#ifndef PROJ_MANAGER_H
#define PROJ_MANAGER_H

#pragma once
#include "Graph.h"
#include "KDTree.h"
using namespace std;


class Manager {

    private:

        Graph graph; // Graph object to manage the graph
        KDTree kdTree; // KD-Tree object for spatial queries
        void ReadStations(const string& filename); 
        void ReadRoutesStops(const string& filename);

    public:
        void ReadGIFST(const string& filename); // Function to read GIFTS data from a file
        void buildKDTree(); // Function to build a KD-Tree from the graph data
        Manager(); // Constructor to initialize the Manager

        

    
};

#endif //PROJ_MANAGER_H
