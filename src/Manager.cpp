#include "../include/Manager.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
using namespace std;

Manager::Manager(){
}
void Manager::buildKDTree(){
    // Get all nodes from the graph
    vector<Point3D> points;
    for (const auto& pair : graph.getNodes()) {
        Node* node = pair.second;
        Coordinates cord = Coordinates(node->longitude, node->latitude); // Create a Coordinates object
        points.push_back(node->toPoint3D()); // Convert Node to Point3D and add to the vector
    }
    kdTree = KDTree(points); // Build the KD-Tree with the points
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
    // the map stores for each trip_id the stop_id of the last stop
    unordered_map<string, string> PreviousStopId;
    // the map stores for each trip_id the previous departure time
    unordered_map<string, Time*> PreviousStopDeparture;
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
        string trip_id,arrival_time,departure_time_str,stop_id,stop_sequence;
        double stop_lat, stop_lon;

        // Parse the CSV line
        getline(iss, trip_id, ','); 
        getline(iss, arrival_time, ',');
        getline(iss, departure_time_str, ',');
        getline(iss, stop_id, ',');
        getline(iss, stop_sequence, ',');
        //convert string to int
        int stop_sequence_int = stoi(stop_sequence); // Convert stop_sequence to int
        //check if trip_id is in the map
        
        //parse the time
        std::istringstream iss1(departure_time_str);
        char delim = ':'; // Delimiter for time
        int hours, minutes, seconds;
        iss1 >> hours >> delim >> minutes >> delim >> seconds;
        Time* departure_time = new Time(hours, minutes, seconds); // Create a Time object

        if (PreviousStopId.find(trip_id) != PreviousStopId.end()){
            //get the nodes
            string startingStopId = PreviousStopId[trip_id];
            Node* startingStop = graph.getNode( PreviousStopId[trip_id]); // Get the starting stop from the map
            Node* destinationStop = graph.getNode(stop_id); // Get the destination stop from the map
            //check if the nodes are not null
            if (startingStop != nullptr && destinationStop != nullptr) {
                //create the edge and add it to the graph
                Time* previousDepartureTime = PreviousStopDeparture[trip_id]; // Get the previous departure time from the map
                // Calculate the time difference between the two stops
                int travelTime = departure_time->difference(*previousDepartureTime); // Calculate the time difference
                // Create an edge with the time difference
                //cout << "Edge created from " << startingStop->id << " to " << destinationStop->id <<"travelTime: "<<travelTime<< endl;
                Edge* edge = new Edge(startingStop, destinationStop, departure_time,travelTime); // Create an edge object
                graph.addEdge(edge); // Add the edge to the graph
            } else {
                cerr << "Error: Node not found for stating_stop_id: " << startingStopId << " or stop_id: " << stop_id <<"Sequence: "<<stop_sequence_int<< endl;
            }
        }
        PreviousStopId[trip_id] = stop_id; // Add the stop_id to the map with a value of 0
        PreviousStopDeparture[trip_id] = departure_time; // Add the stop_id to the map with a value of 0
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