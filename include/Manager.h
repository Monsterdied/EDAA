#ifndef PROJ_MANAGER_H
#define PROJ_MANAGER_H


#include "Graph.h"
using namespace std;


class Manager {

    private:

        Graph graph; // Graph object to manage the graph
        void ReadStations(const string& filename); 
        void ReadRoutesStops(const string& filename);

    public:
        void ReadGIFST(const string& filename); // Function to read GIFTS data from a file
        Manager(); // Constructor to initialize the Manager

        

    
};

#endif //PROJ_MANAGER_H
