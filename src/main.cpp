#include <iostream>
#include <string>
#include <filesystem>
#include "../include/Manager.h"
#include <chrono>
using namespace std;


struct PathAnalysis {
    int totalTravelTime;       // in seconds
    int totalWaitingTime;      // in seconds
    int numSegments;           // total number of edges
    int numTransfers;          // number of transport type changes
    double totalDistance;      // in meters
    vector<string> transportTypes;  // types of transport used
};

PathAnalysis analyzePath(const vector<Edge*>& path, const Time& startTime) {
    PathAnalysis analysis = {0, 0, 0, 0, 0};
    string currentTransportType = "";
    Time currentTime = startTime;

    for (const Edge* edge : path) {
        // Count segments
        analysis.numSegments++;

        // Calculate waiting time if this is a scheduled transport
        if (edge->time != nullptr) {
            int waitTime = edge->time->difference(currentTime);
            if (waitTime > 0) {
                analysis.totalWaitingTime += waitTime;
            }
            currentTime = edge->time->clone();
        }

        // Add travel time
        analysis.totalTravelTime += edge->travelTime;
        currentTime.add_seconds(edge->travelTime);

        // Count transfers
        if (!edge->type.empty() && edge->type != currentTransportType) {
            if (!currentTransportType.empty()) {
                analysis.numTransfers++;
            }
            currentTransportType = edge->type;
            if (find(analysis.transportTypes.begin(), analysis.transportTypes.end(), edge->type)
                == analysis.transportTypes.end()) {
                analysis.transportTypes.push_back(edge->type);
            }
        }

        // Add distance
        analysis.totalDistance += edge->travelTime; // Using travel time as a proxy for distance
    }

    return analysis;
}

void performPathQualityAnalysis(Manager& manager) {
    // Define test cases (Origin/Destination pairs)
    vector<pair<Coordinates, Coordinates>> testCases = {
        // Short distance
        {Coordinates(-8.6190758,41.1570994), Coordinates(-8.5984257,41.1783583)},
        // Medium distance
        {Coordinates(-8.662980, 41.191962), Coordinates(-8.595707, 41.177225)},
        // Long distance
        {Coordinates(-8.759389, 41.377656), Coordinates(-8.565786, 41.175185)}
    };

    vector<float> multipliers = {1.0f, 1.5f, 2.0f, 2.5f};
    Time startTime(9, 0, 0);

    cout << "\n=== Path Quality Analysis ===\n";

    for (size_t i = 0; i < testCases.size(); i++) {
        cout << "\nTest Case " << i + 1 << " (";
        if (i == 0) cout << "Short Distance";
        else if (i == 1) cout << "Medium Distance";
        else cout << "Long Distance";
        cout << ")\n";
        cout << "----------------------------------------\n";

        for (float multiplier : multipliers) {
            cout << "\nA* Multiplier: " << multiplier << endl;

            vector<pair<double, vector<Edge*>>> paths =
                manager.shortestPath(testCases[i].first, testCases[i].second,
                                   startTime.clone(), 900000000, 1, multiplier);

            if (paths.empty() || paths[0].second.empty()) {
                cout << "No path found!\n";
                continue;
            }

            PathAnalysis analysis = analyzePath(paths[0].second, startTime);

            cout << "Path Characteristics:\n";
            cout << "- Total Travel Time: " << analysis.totalTravelTime / 60
                 << "m " << analysis.totalTravelTime % 60 << "s\n";
            cout << "- Total Waiting Time: " << analysis.totalWaitingTime / 60
                 << "m " << analysis.totalWaitingTime % 60 << "s\n";
            cout << "- Number of Segments: " << analysis.numSegments << "\n";
            cout << "- Number of Transfers: " << analysis.numTransfers << "\n";
            cout << "- Transport Types Used:";
            for (const string& type : analysis.transportTypes) {
                cout << " " << type;
            }
            cout << "\n";
            cout << "- Total Cost (f-score): " << paths[0].first << "\n";
        }
    }
}

void performPathfindingBenchmark(Manager& manager) {
    // Define test cases (Origin/Destination pairs)
    vector<pair<Coordinates, Coordinates>> testCases = {
        // Short distance
        {Coordinates(-8.6190758,41.1570994), Coordinates(-8.5984257,41.1783583)},
        // Medium distance
        {Coordinates(-8.662980, 41.191962), Coordinates(-8.595707, 41.177225)},
        // Long distance
        {Coordinates(-8.759389, 41.377656), Coordinates(-8.565786, 41.175185)}
    };

    // Different A* multiplier values to test
    vector<float> multipliers = {1.0f, 1.5f, 2.0f, 2.5f};

    // Number of times to repeat each test for averaging
    const int numRepetitions = 5;

    // Start time for journey (9:00 AM)
    Time startTime(9, 0, 0);

    cout << "\n=== Pathfinding Performance Benchmark ===\n";

    for (const auto& testCase : testCases) {
        cout << "\nTesting path from ("
             << testCase.first.longitude << "," << testCase.first.latitude
             << ") to ("
             << testCase.second.longitude << "," << testCase.second.latitude
             << ")\n";

        for (float multiplier : multipliers) {
            cout << "\nA* multiplier: " << multiplier << endl;

            vector<double> executionTimes;

            for (int i = 0; i < numRepetitions; i++) {
                Time timeClone = startTime.clone();

                auto startTime = chrono::high_resolution_clock::now();

                // Perform pathfinding
                vector<pair<double, vector<Edge*>>> paths =
                    manager.shortestPath(testCase.first, testCase.second,
                                      timeClone, 900000000, 1, multiplier);

                auto endTime = chrono::high_resolution_clock::now();

                auto duration = chrono::duration_cast<chrono::microseconds>
                              (endTime - startTime).count();

                executionTimes.push_back(duration / 1000.0); // Convert to milliseconds
            }

            // Calculate average execution time
            double avgTime = 0.0;
            for (double time : executionTimes) {
                avgTime += time;
            }
            avgTime /= numRepetitions;

            cout << "Average execution time: " << avgTime << " ms\n";
        }
    }
}

int main(){

    Manager manager; // Create a Manager object
    auto timeStart1 = chrono::high_resolution_clock::now();
    //manager.ReadGTFS("../data/STCP","autocarro"); // Read the GTFS data from a file
    manager.ReadGTFS("../data/STCP","autocarro"); // Read the GTFS data from a file
    manager.ReadGTFS("../data/MetroDoPorto","metro"); // Read the GTFS data from a file
    //manager.ReadGTFS("../data/test","metro");
    //manager.printAllNodes();
    //manager.printAllEdges();

    //manager.ReadGTFS("../data/mdb-2027-202504140043","comboio");
    //manager.ReadGTFS("../data/tld-651-202504210112");
    ////manager.ReadGTFS("../data/c1","c1");
    //manager.ReadGTFS("../data/c2","c2");
    ////manager.ReadGTFS("../data/c3","c3");
    //manager.ReadGTFS("../data/c4","c4");
    //manager.ReadGTFS("../data/germany","germany");
    manager.buildKDTree(); // Build a KD-Tree from the graph data
    auto timeEnd1 = chrono::high_resolution_clock::now();
    auto deltaTime1 = chrono::duration_cast<chrono::seconds>(timeEnd1 - timeStart1);
    cout << "Data loading time: " << deltaTime1.count()/60 << " Minutes "
         << deltaTime1.count()%60 << " seconds" << endl;

    // Run the benchmarks
    //performPathfindingBenchmark(manager);
    //performPathQualityAnalysis(manager);

    //time
    //
    /*
    vector<Point3D> nodes = manager.kdTree.kNearestNeighbors(Coordinates(-8.6190758,41.1570994).toPoint3D(),7);
    for (Point3D point : nodes) {
        Node* node = manager.graph.getNode(point.id);
        cout<<"node "<<node->name<<"id "<<node->id<<endl;
    }*/
    //create a random time

    auto timeStart2 = chrono::high_resolution_clock::now();
    Time timeStart = Time(9, 0, 0); // Create a Time object with the current time
    Time timeClone = timeStart.clone(); // Clone the time object

    vector<pair<double,vector<Edge*>>> nodes= manager.shortestPath(Coordinates(-8.662980, 41.191962), Coordinates(-8.595707, 41.177225),timeClone,900000000, 15, 1); // Find the shortest path between two coordinates
    auto timeEnd = chrono::high_resolution_clock::now();
    auto deltaTime = chrono::duration_cast<chrono::seconds>(timeEnd - timeStart2);
    cout<< "Testing 3";
    timeStart.print();
    for (int i = 1; i >=0; i--) {
        //manager.printPath(nodes[i].second); // Print the path
        manager.newPrintPath(nodes[i].second,timeStart); // Print the path
    }
    cout<<"Time taken: "<<deltaTime.count()/60<<" Minutes "<<deltaTime.count()%60<<" seconds" << endl;

    return 0;
}


