#ifndef PROJ_MANAGER_H
#define PROJ_MANAGER_H

#pragma once
#include "Graph.h"
#include "KDTree.h"
using namespace std;


class Manager {

    private:

        void ReadStations(const string& filename,string type);
        void ReadRoutesStops(const string& filename,string type);
        vector<Edge*> getKNearestFootEdges(Node* node, int k) const;

    public:
        Graph graph; // Graph object to manage the graph
        //public for testing purposes
        KDTree kdTree; // KD-Tree object for nearest neighbor search
        void ReadGTFS(const string& filename,string type); // Function to read GTFS data from a file
        void buildKDTree(); // Function to build a KD-Tree from the graph data
        vector<Edge*> shortestPathAstar(Node* startNode, const Coordinates& goal,double max_tentative,Time StartTime) const;
        vector<pair<double,vector<Edge*>>> shortestPath(const Coordinates& start, const Coordinates& goal,Time startTime,double max_tentative=1000000000,
                                                        const int alternatives=15,const float a_star_multiplier=2.5) const; // Function to find the shortest path between two coordinates

        Manager(); // Constructor to initialize the Manager

        void printAllNodes() const;
        void printAllEdges() const;
        void newPrintPath(const vector<Edge*>& path, Time journeyStartTime) const;

    
};

#endif //PROJ_MANAGER_H
