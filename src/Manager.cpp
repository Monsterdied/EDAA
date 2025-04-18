#include "Manager.h"

#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

Manager::Manager(){
}

void Manager::ReadMetroStations(const string& filename) {
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