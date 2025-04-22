#include "../include/Manager.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <utility>
#include <algorithm>
#include <vector>
using namespace std;

Manager::Manager(){
}
void Manager::buildKDTree(){
    // Get all nodes from the graph
    vector<Point3D> points;
    for (const auto& pair : graph.getNodes()) {
        Node* node = pair.second;
        Coordinates cord = node->coordinates; // Create a Coordinates object
        points.push_back(node->toPoint3D()); // Convert Node to Point3D and add to the vector
    }
    kdTree = KDTree(points); // Build the KD-Tree with the points
}

void Manager::ReadStations(const string& filename,string type) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    string line;
    bool isHeader = true; // To skip the header line
    int stop_name_idx,stop_id_idx,stop_lat_idx,stop_lon_idx;
    int lineCount = 0; // Initialize line count
    while (getline(file, line)) {
        lineCount++; // Increment line count
        //cout << lineCount << flush;
        istringstream iss(line);
        vector<string> vecLine;
        string header;
        while(getline(iss, header, ',')) {
            vecLine.push_back(header); // Store each header in a vector
        }

        if (isHeader) {
            int idx = 0;
            for (const auto& header : vecLine) {
                if (header == "stop_lat") {
                    stop_lat_idx = idx;
                }else if (header == "stop_name") {
                    stop_name_idx = idx;
                }else if (header == "stop_id") {
                    stop_id_idx = idx;
                }else if (header == "stop_lon") {
                    stop_lon_idx = idx;
                }
                idx++;
            }
            isHeader = false; // Skip the first line (header)
            continue;
        }

        string stop_id, stop_name;
        double stop_lat, stop_lon;

        // Parse the CSV line
        //cout <<endl<<""<<endl<<"stop_name:"<< vecLine[stop_name_idx]<<endl<<"stop_id:"<< vecLine[stop_id_idx]<<endl<<"longitude:"<< vecLine[stop_lon_idx]<<endl<<"latitude:"<< vecLine[stop_lat_idx]<<endl;
        stop_id = vecLine[stop_id_idx]; // Get the stop_id from the vector
        stop_lon = stod(vecLine[stop_lon_idx]); // Get the stop_code from the vector
        stop_name = vecLine[stop_name_idx]; // Get the stop_name from the vector
        stop_lat = stod(vecLine[stop_lat_idx]); // Get the stop_desc from the vector
        //cout << "stop_id: " << stop_id << ", stop_lat: " << stop_lat << ", stop_lon: " << stop_lon << ", stop_name: " << stop_name << endl;
        Node* StartingNode = new Node(stop_id, stop_lat, stop_lon, stop_name, type); // Create a node object
        graph.addNode(StartingNode); // Add the node to the graph
    }
    cout << "Graph has been created with " << graph.getNodeCount() << " nodes." << endl;
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
    int trip_id_idx,arrival_time_idx,departure_time_str_idx,stop_id_idx,stop_sequence_idx;
    int lineCount = 0; // Initialize line count
    while (getline(file, line)) {
        lineCount++; // Increment line count
        if(lineCount % 100000 == 0) {
            cout << "Processed " << lineCount << " Edges." << endl; // Print progress every 1000 lines
        }
        istringstream iss(line);
        vector<string> vecLine;
        string header;
        while(getline(iss, header, ',')) {
            vecLine.push_back(header); // Store each header in a vector
        }
        if (isHeader) {
            int idx = 0;
            for (const auto& header : vecLine) {
                if (header == "trip_id") {
                    trip_id_idx = idx;
                }else if (header == "arrival_time") {
                    arrival_time_idx = idx;
                }else if (header == "departure_time") {
                    departure_time_str_idx = idx;
                }else if (header == "stop_id") {
                    stop_id_idx = idx;
                }else if (header == "stop_sequence") {
                    stop_sequence_idx = idx;
                }
                idx++;
            }
            isHeader = false; // Skip the first line (header)
            continue;
        }
        string trip_id,arrival_time,departure_time_str,stop_id,stop_sequence;
        double stop_lat, stop_lon;

        // Parse the CSV line
        trip_id = vecLine[trip_id_idx]; // Get the trip_id from the vector
        stop_id = vecLine[stop_id_idx]; // Get the stop_id from the vector
        stop_sequence = vecLine[stop_sequence_idx]; // Get the stop_sequence from the vector
        departure_time_str = vecLine[departure_time_str_idx]; // Get the departure_time from the vector
        arrival_time = vecLine[arrival_time_idx]; // Get the arrival_time from the vector
        //convert string to int
        //cout << "trip_id: " << trip_id << ", stop_id: " << stop_id << ", stop_sequence: " << stop_sequence << ", departure_time: " << departure_time_str << endl;
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

void Manager::ReadGIFST(const string& filename,string type){
    string stationsFile = filename + "/stops.txt";
    string routesFile = filename + "/stop_times.txt";
    ReadStations(stationsFile,type);
    ReadRoutesStops(routesFile);
}
struct node_cmp
{
   bool operator()( const Node* a, const Node* b ) const 
   {
    return a->bestDistance < b->bestDistance;
   }
};

vector<Edge*> Manager::shortestPath(const Coordinates& start, const Coordinates& goal,double max_tentative) const{
    Point3D startPoint = kdTree.nearestNeighbor(start.toPoint3D()); // Get the closest node to the start coordinates
    Node* startNode = graph.getNode(startPoint.id); // Get the node from the graph using the ID // Set the distance of the start node to 0
    Node* bestPoint =startNode;
    Node* closestNode =startNode;
    startNode->visited = true;
    cout <<"cords1:"<< startNode->coordinates.getCoordinates().first<<" "<< startNode->coordinates.getCoordinates().second<<endl;
    cout <<"cords2:"<< goal.getCoordinates().first<<" "<< goal.getCoordinates().second<<endl;
    double bestDistance = startNode->coordinates.haversineDistance(goal); // Set the best distance to the goal coordinates
    // Priority queue (open set)
    priority_queue<Node*, vector<Node*>, node_cmp> openSet;
    vector<Edge*> path; // Vector to store the path
    bestPoint->bestDistance = bestPoint->coordinates.haversineDistance(goal); // Set the best distance to the goal coordinates
    // Create start node
    openSet.push(startNode);
    double counter = 0;
    // A* main loop
    while (!openSet.empty()) {
        counter++;
        if(counter > max_tentative){
            break;
        }
        Node* current = openSet.top();
        Coordinates currentCords = current->coordinates; // Get the current node
        double distance = currentCords.haversineDistance(goal); // Calculate the distance to the goal
        openSet.pop();
        // Explore neighbors
        vector<Edge*> directions = graph.getAdjacentEdges(current->id);
        for (const auto& dir : directions) {
            Node* neighbor = dir->destinationNode; // Get the neighboring node
            if (neighbor->visited) {
                continue; // Skip if the neighbor has already been visited
            }
            cout<< "Test2"<<endl;
            int distanceToGoal = neighbor->coordinates.haversineDistance(goal); // Calculate the distance to the goal
            // Calculate tentative g-score
            int tentativeG = current->distance + dir->travelTime +distanceToGoal; 
            // If neighbor is new or a better path is found
            if (neighbor->bestDistance > tentativeG) {
                neighbor->distance = current->distance + dir->travelTime;
                neighbor->bestDistance = tentativeG; // Update the best distance
                neighbor->previous = dir; // Set the previous edge
                neighbor->visited = true; // Mark the neighbor as visited Dont use neighboor
                openSet.push(neighbor);
                cout<< "Test3"<<endl;
                if(neighbor->bestDistance < bestDistance){
                    bestDistance = neighbor->bestDistance; // Update the best distance
                    closestNode = neighbor; // Update the closest node
                }
            }
        }
    }
    cout << "No Perfect path found." << endl; // Print if no path is found
    Edge* edge = closestNode->previous; // Get the previous edge
    while (edge != nullptr) {
        cout << "Closest node: " << closestNode->id << endl; // Print the closest node ID
        path.push_back(edge); // Add the current node to the path
        closestNode = edge->startingNode; // Get the starting node of the edge
        edge = closestNode->previous; // Get the previous edge
    }
    reverse(path.begin(), path.end());
    return path;
}