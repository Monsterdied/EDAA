#ifndef PROJ_MANAGER_H
#define PROJ_MANAGER_H

#pragma once
#include "Graph.h"
#include "KDTree.h"
using namespace std;


class Manager {

    private:

        Graph graph; // Graph object to manage the graph
        void ReadStations(const string& filename); 
        void ReadRoutesStops(const string& filename);

    public:
        //public for testing purposes
        KDTree kdTree; // KD-Tree object for nearest neighbor search
        void ReadGIFST(const string& filename); // Function to read GIFTS data from a file
        void buildKDTree(); // Function to build a KD-Tree from the graph data
        Manager(); // Constructor to initialize the Manager

        

    
};

#endif //PROJ_MANAGER_H
