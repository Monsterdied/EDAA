#ifndef PROJ_MANAGER_H
#define PROJ_MANAGER_H


#include "Graph.h"
using namespace std;


class Manager {

    private:

        Graph graph; // Graph object to manage the graph


    public:

        Manager(); // Constructor to initialize the Manager

        void ReadMetroStations(const string& filename); 

        

    
};

#endif //PROJ_MANAGER_H
