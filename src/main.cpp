#include <iostream>
#include <string>
#include <filesystem>
#include "../include/Manager.h"
#include <chrono>


using namespace std;

int main(){


    Manager manager; // Create a Manager object
    manager.ReadGIFST("../data/STCP","autocarro"); // Read the GIFST data from a file
    manager.ReadGIFST("../data/MetroDoPorto","metro"); // Read the GIFST data from a file

    //manager.ReadGIFST("../data/mdb-2027-202504140043","comboio");
    //manager.ReadGIFST("../data/tld-651-202504210112");
    //manager.ReadGIFST("../data/c1","c1");
    manager.ReadGIFST("../data/c2","c2");
    //manager.ReadGIFST("../data/c3","c3");
    //manager.ReadGIFST("../data/c4","c4");
    manager.buildKDTree(); // Build a KD-Tree from the graph data

    //time
    //
    /*
    vector<Point3D> nodes = manager.kdTree.kNearestNeighbors(Coordinates(-8.6190758,41.1570994).toPoint3D(),7);
    for (Point3D point : nodes) {
        Node* node = manager.graph.getNode(point.id);
        cout<<"node "<<node->name<<"id "<<node->id<<endl;
    }*/

    auto timeStart = chrono::high_resolution_clock::now();
    vector<pair<double,vector<Edge*>>> nodes= manager.shortestPath(Coordinates(-8.6190758,41.1570994),Coordinates(-8.5984257,41.1783583),9000000); // Find the shortest path between two coordinates
    auto timeEnd = chrono::high_resolution_clock::now();
    auto deltaTime = chrono::duration_cast<chrono::seconds>(timeEnd - timeStart);

    for (int i = 1; i >=0; i--) {
        manager.printPath(nodes[i].second); // Print the path
    }
    cout<<"Time taken: "<<deltaTime.count()/60<<" Minutes "<<deltaTime.count()%60<<" seconds" << endl;

    //test nearest neighbors

    /*
    auto start = std::chrono::high_resolution_clock::now(); // Start the timer
    Point3D point = manager.kdTree.nearestNeighbor(Coordinates(-8.6190758,41.1570994).toPoint3D()); // Find the nearest neighbor to a given point
    auto end = std::chrono::high_resolution_clock::now(); // End the timer
    std::chrono::duration<double> elapsed = end - start; // Calculate the elapsed time
    cout << "Elapsed time: " << elapsed.count() << " seconds" << endl; // Print the elapsed time
    cout << "Nearest neighbor ID: " << point.id << endl; // Print the ID of the nearest neighbor,

    auto start1 = std::chrono::high_resolution_clock::now(); // Start the timer
    vector<Point3D> points = manager.kdTree.kNearestNeighbors(Coordinates(-8.6190758,41.1570994).toPoint3D(), 5); // Find the k nearest neighbors to a given point
    auto end1 = std::chrono::high_resolution_clock::now(); // End the timer
    std::chrono::duration<double> elapsed1 = end1 - start1; // Calculate the elapsed time
    cout << "Elapsed time: " << elapsed1.count() << " seconds" << endl; // Print the elapsed time
    for (const auto& p : points) {
        cout << "K Nearest neighbor ID: " << p.id << endl; // Print the ID of the k nearest neighbors
    }*/
    return 0;
}