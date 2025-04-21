#include <iostream>
#include <string>
#include <filesystem>
#include "../include/Manager.h"



using namespace std;

int main(){


    Manager manager; // Create a Manager object
    manager.ReadGIFST("../data/MetroDoPorto"); // Read the GIFST data from a file
    manager.ReadGIFST("../data/STCP"); // Read the GIFST data from a file
    manager.buildKDTree(); // Build a KD-Tree from the graph data
    return 0;
}