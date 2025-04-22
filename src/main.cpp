#include <iostream>
#include <string>
#include <filesystem>
#include "../include/Manager.h"
#include <chrono>


using namespace std;

int main(){


    Manager manager; // Create a Manager object
    manager.ReadGIFST("../data/MetroDoPorto","metro"); // Read the GIFST data from a file
    manager.ReadGIFST("../data/STCP","autocarro"); // Read the GIFST data from a file
    manager.ReadGIFST("../data/mdb-2027-202504140043","comboio");
    //manager.ReadGIFST("../data/tld-651-202504210112");
    manager.buildKDTree(); // Build a KD-Tree from the graph data

    //time
    auto start = std::chrono::high_resolution_clock::now(); // Start the timer
    Point3D point = manager.kdTree.nearestNeighbor(Point3D(0,0,0)); // Find the nearest neighbor to a given point
    auto end = std::chrono::high_resolution_clock::now(); // End the timer
    std::chrono::duration<double> elapsed = end - start; // Calculate the elapsed time
    cout << "Elapsed time: " << elapsed.count() << " seconds" << endl; // Print the elapsed time
    cout << "Nearest neighbor ID: " << point.id << endl; // Print the ID of the nearest neighbor
    return 0;
}