#include "../include/Manager.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
using namespace std;

Manager::Manager(){
}

void Manager::ReadStations(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    string line;
    bool isHeader = true; // To skip the header line
    while (getline(file, line)) {
        if (isHeader) {
            isHeader = false; // Skip the first line (header)
            continue;
        }

        istringstream iss(line);
        string stop_id, stop_code, stop_name, stop_desc, zone_id, stop_url;
        double stop_lat, stop_lon;

        // Parse the CSV line
        getline(iss, stop_id, ','); 
        getline(iss, stop_code, ',');
        getline(iss, stop_name, ',');
        getline(iss, stop_desc, ',');
        iss >> stop_lat;
        iss.ignore(1, ','); 
        iss >> stop_lon;
        iss.ignore(1, ','); 
        getline(iss, zone_id, ',');
        getline(iss, stop_url, ',');
        Node* StartingNode = new Node(stop_id, stop_lat, stop_lon, stop_name, "metro", stop_code, stop_desc, zone_id); // Create a node object
        graph.addNode(StartingNode); // Add the node to the graph
    }

    file.close();
}

void Manager::ReadRoutesStops(const string& filename) {
    ifstream file(filename);
    unordered_map<string, string> nodesMap;

    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    string line;
    bool isHeader = true; // To skip the header line
    while (getline(file, line)) {
        if (isHeader) {
            isHeader = false; // Skip the first line (header)
            continue;
        }

        istringstream iss(line);
        string trip_id,arrival_time,departure_time,stop_id,stop_sequence;
        double stop_lat, stop_lon;

        // Parse the CSV line
        getline(iss, trip_id, ','); 
        getline(iss, arrival_time, ',');
        getline(iss, departure_time, ',');
        getline(iss, stop_id, ',');
        getline(iss, stop_sequence, ',');
        //convert string to int
        int stop_sequence_int = stoi(stop_sequence); // Convert stop_sequence to int
        //check if trip_id is in the map

        if (nodesMap.find(trip_id) != nodesMap.end()){
            string startingStopId = nodesMap[trip_id];
            Node* startingStop = graph.getNode( nodesMap[trip_id]); // Get the starting stop from the map
            Node* destinationStop = graph.getNode(stop_id); // Get the destination stop from the map
            if (startingStop != nullptr && destinationStop != nullptr) {
                //TODO: EXPAND THE ARRIVAL TIME TO A TIME OBJECT
                Time arrival_time_obj = {0, 0, 0}; // Initialize arrival_time object
                Edge* edge = new Edge(startingStop, destinationStop, arrival_time_obj); // Create an edge object
                graph.addEdge(edge); // Add the edge to the graph
            } else {
                cerr << "Error: Node not found for stating_stop_id: " << startingStopId << " or stop_id: " << stop_id <<"Sequence: "<<stop_sequence_int<< endl;
            }
        }
        nodesMap[trip_id] = stop_id; // Add the stop_id to the map with a value of 0
        //Node* StartingNode = new Node(stop_id, stop_lat, stop_lon, stop_name, "metro", stop_code, stop_desc, zone_id); // Create a node object
        
    }
    cout << "Graph has been created with " << graph.getNodeCount() << " nodes and " << graph.getEdgeCount() << " edges." << endl;
    file.close();
}

void Manager::ReadGIFST(const string& filename){
    string stationsFile = filename + "/stops.txt";
    string routesFile = filename + "/stop_times.txt";
    ReadStations(stationsFile);
    ReadRoutesStops(routesFile);
}