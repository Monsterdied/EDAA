#include "../include/Manager.h"
#include <regex>
#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <utility>
#include <algorithm>
#include <vector>
#include <string>
#include <iomanip>
using namespace std;

struct CompareNodesByBestDistance {
    bool operator()(const Node* a, const Node* b) const {
        // '>' for a min-heap when Node::bestDistance stores f_score
        return a->bestDistance > b->bestDistance;
    }
};


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
        if(lineCount % 10000 == 0) {
            cout << "Processed " << lineCount << " Nodes." << endl; // Print progress every 1000 lines
        }
        //cout << lineCount << flush;
        istringstream iss(line);
        vector<string> vecLine;
        string header;
        regex field_re(R"((?:^|,)(?:\"([^\"]*)\"|([^,]*)))");
        sregex_iterator it(line.begin(), line.end(), field_re);
        sregex_iterator end;
        for (; it != end; ++it) {
            smatch match = *it;
            // Use group 1 if quoted field matched, group 2 otherwise
            string field = match[1].matched ? match[1].str() : match[2].str();
            vecLine.push_back(field);
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
        stop_id = type + vecLine[stop_id_idx]; // Get the stop_id from the vector
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


// Helper to parse time string from GTFS
Time* parseGTFSTime(const string& time_str) {
    if (time_str.empty()) return nullptr;

    std::istringstream iss(time_str);
    char delim = ':';
    int hours, minutes, seconds;
    if (iss >> hours >> delim >> minutes >> delim >> seconds) {
        return new Time(hours, minutes, seconds);
    }
    std::cerr << "Warning: Could not parse time string: " << time_str << std::endl;
    return nullptr;
}

void Manager::ReadRoutesStops(const string& filename, string type) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    struct StopInfo {
        string id;
        Time* departure_time{}; // Departure time from this stop
    };
    unordered_map<string, StopInfo> trip_previous_stop_info;

    string line;
    bool isHeader = true;
    int trip_id_idx = -1, arrival_time_idx = -1, departure_time_idx = -1, stop_id_idx = -1; // Initialize to -1
    int lineCount = 0;

    while (getline(file, line)) {
        lineCount++;
        if (lineCount % 100000 == 0) {
            cout << "Processed " << lineCount << " Edges." << endl;
        }

        istringstream iss_line(line); // Use a different name for the line stream
        vector<string> vecLine;
        string field;
        while (getline(iss_line, field, ',')) {
            vecLine.push_back(field);
        }

        if (isHeader) {
            for (int idx = 0; idx < vecLine.size(); ++idx) {
                if (vecLine[idx] == "trip_id") trip_id_idx = idx;
                else if (vecLine[idx] == "arrival_time") arrival_time_idx = idx;
                else if (vecLine[idx] == "departure_time") departure_time_idx = idx;
                else if (vecLine[idx] == "stop_id") stop_id_idx = idx;
            }
            isHeader = false;
            // Basic check if all required columns were found
            if (trip_id_idx == -1 || arrival_time_idx == -1 || departure_time_idx == -1 || stop_id_idx == -1) {
                cerr << "Error: Missing one or more required columns in stop_times.txt header: " << line << endl;
                file.close();
                // Clean up any Time objects already in trip_previous_stop_info
                for (auto const& pair_info : trip_previous_stop_info) {
                    delete pair_info.second.departure_time;
                }
                return;
            }
            continue;
        }


        // Ensure vector has enough elements for safety
        if (vecLine.size() <= max({trip_id_idx, arrival_time_idx, departure_time_idx, stop_id_idx})) {
            cerr << "Warning: Skipping malformed line (not enough columns): " << line << endl;
            continue;
        }

        string trip_id = vecLine[trip_id_idx];
        string current_stop_id_short = vecLine[stop_id_idx];
        string current_stop_id_full = type + current_stop_id_short;
        string arrival_time_str = vecLine[arrival_time_idx];
        string departure_time_str = vecLine[departure_time_idx];

        Time* current_arrival_time = parseGTFSTime(arrival_time_str);
        Time* current_departure_time = parseGTFSTime(departure_time_str);

        if (!current_arrival_time || !current_departure_time) {
            cerr << "Warning: Failed to parse time for trip " << trip_id << ", stop " << current_stop_id_full << ". Skipping entry." << endl;
            delete current_arrival_time; // delete if one succeeded but other failed or if both were null
            delete current_departure_time;
            continue;
        }

        if (trip_previous_stop_info.count(trip_id)) {
            const StopInfo& prev_stop = trip_previous_stop_info[trip_id];
            Node* startingNode = graph.getNode(prev_stop.id);
            Node* destinationNode = graph.getNode(current_stop_id_full);

            if (startingNode && destinationNode && prev_stop.departure_time) {
                // Edge: startingNode -> destinationNode
                // Scheduled departure for this edge is from startingNode (prev_stop.departure_time)
                Time* edge_scheduled_departure_time = new Time(*(prev_stop.departure_time)); // Crucial: Edge gets its OWN Time object

                // Travel time = arrival at current_stop - departure from prev_stop
                int travelTimeSeconds = current_arrival_time->difference(prev_stop.departure_time);

                if (travelTimeSeconds < 0) {
                    cerr << "Warning: Negative travel time for trip " << trip_id << " from " << startingNode->id << " (depart ";
                    prev_stop.departure_time->print();
                    cerr << ") to " << destinationNode->id << " (arrive ";
                    current_arrival_time->print();
                    cerr << "). Travel time: " << travelTimeSeconds << "s. Skipping edge." << endl;
                    delete edge_scheduled_departure_time; // Clean up the copied time object
                } else {
                    Edge* edge = new Edge(startingNode, destinationNode, edge_scheduled_departure_time, travelTimeSeconds, type);
                    graph.addEdge(edge);
                }
            } else {
                if (!startingNode) cerr << "Error: Previous stop node not found: " << prev_stop.id << endl;
                if (!destinationNode) cerr << "Error: Current stop node not found: " << current_stop_id_full << endl;
                if (!prev_stop.departure_time) cerr << "Error: Missing previous departure time for trip " << trip_id << endl;
            }
            delete prev_stop.departure_time;
        }

        // Update map: current stop becomes the previous stop for the next segment of this trip
        trip_previous_stop_info[trip_id] = {current_stop_id_full, current_departure_time};
        // DO NOT delete current_departure_time here, it's now stored in the map.
        // DO delete current_arrival_time as it's only used for calculation in this iteration.
        delete current_arrival_time;

    } // End of while loop

    // Clean up any remaining Time objects in the map (for the last stop of each trip)
    for (auto const& pair_info : trip_previous_stop_info) {
        delete pair_info.second.departure_time;
    }
    trip_previous_stop_info.clear();

    cout << "Graph processing complete. Nodes: " << graph.getNodeCount() << ", Edges: " << graph.getEdgeCount() << endl;
    file.close();
}

void Manager::ReadGTFS(const string& filename,string type){
    string stationsFile = filename + "/stops.txt";
    string routesFile = filename + "/stop_times.txt";
    ReadStations(stationsFile,type);
    ReadRoutesStops(routesFile,type);
}
void Manager::printPath(const vector<Edge*>& path) const{
    cout << "Shortest path:" <<path.size()<< endl; // Print the shortest path
    int timeCounter = 0;
    Time* prevTime = path[0]->time->clone();
    for (auto& edge : path) {
        string startingNodeId = edge->startingNode != nullptr ? edge->startingNode->id : "start"; // Get the starting node ID
        string destinationNodeId = edge->destinationNode != nullptr ? edge->destinationNode->id : "destination"; // Get the destination node ID
        string startingName = graph.getNode(startingNodeId) != nullptr ? graph.getNode(startingNodeId)->name : "start"; // Get the starting node ID
        string destinationName = graph.getNode(destinationNodeId) != nullptr ? graph.getNode(destinationNodeId)->name : "destination"; // Get the destination node ID
        Node* StartingNode =graph.getNode(startingNodeId);
        Node* DestinationNode = graph.getNode(destinationNodeId);
        if (StartingNode != nullptr && DestinationNode != nullptr) {
            Time* startTime = prevTime;
            Time* destTime = prevTime->clone();
            destTime->add_seconds(edge->travelTime);
            int difference = edge->time->difference(startTime);
            if (difference !=0) {
                cout << "Time arrived at the station ";
                destTime->print();
                cout<<endl;
                cout<< "Time leaving the station";
                edge->time->print();
                cout<<edge->travelTime;
                cout<<endl;
                cout<<"Transfer Wait :"<<difference/60<<" minutes "<<difference%60<<" seconds"<<endl;
            }
            destTime->add_seconds(difference);
            prevTime = destTime;

        }
        timeCounter += edge->travelTime; // Add the travel time to the counter
        int minutes = edge->travelTime/60;
        cout <<left<<std::setfill('-')<<setw(30)<<edge->rideName<<" ";
        cout <<left<<std::setfill('-')<<setw(30)<< startingName << "TravelTime: ";
        string time;
        if (minutes > 0) {
            time = to_string(minutes) + " minutes ";
        }
        time += to_string(edge->travelTime%60) + " seconds"; // Print the travel time
        cout <<left<<std::setfill('-')<<setw(25)<< time;
        cout<<"type: "<< edge->type<<" ---> " << destinationName << endl<<endl;
        if (edge->time != nullptr) {
             edge->time->print();
        }

    }
    int minutes1 = timeCounter/60;
    cout<<"Traveled Time: "<<minutes1 << " minutes " <<timeCounter%60 << " seconds"<<endl; // Print the path

}

vector<Edge*> Manager::getKNearestFootEdges(Node* node, int k,float const a_star_multiplier)const {
    vector<Point3D> footEdges = kdTree.kNearestNeighbors(node->toPoint3D(), k); // Get the k nearest foot edges
    vector<Edge*> footEdgesVector;
    for (const auto& footEdge : footEdges) {
        Node* footNode = graph.getNode(footEdge.id); // Get the foot node from the graph using the ID
        if (footNode->id == node->id) {
            //cout<<"Bugged";
            continue; // Skip if the foot node is the same as the current node
        }
        if (footNode != nullptr) {
            int const distance = node->coordinates.haversineDistance(footNode->coordinates)*(a_star_multiplier+2); // Calculate the distance to the foot node

            Edge* edge = new Edge(node, footNode, nullptr, distance, "foot"); // Create a new edge with the foot node
            edge->rideName = "DEBUG1";
            footEdgesVector.push_back(edge); // Add the edge to the vector
        }else {
            cout<<"Bugged";
        }
    }
    return footEdgesVector; // Return the vector of foot edges
}


double A_star_heuristic(const Node* currNode, const Coordinates goal,const Edge* edge= nullptr,const float a_star_multiplier=2.5,const Time* currTime= nullptr){
    if(edge == nullptr || currTime == nullptr){
        return currNode->coordinates.haversineDistance(goal)*a_star_multiplier + currNode->distance;
    }
    int const timeDiff = edge->time->difference(currTime->clone()); // Calculate the time difference
    //cout<<"Edge diff "<<timeDiff<<"\n"<<endl;
    return edge->destinationNode->coordinates.haversineDistance(goal)*a_star_multiplier + edge->travelTime + currNode->distance + timeDiff;
}
vector<Edge*> Manager::shortestPathAstar(Node* startNode, const Coordinates& goal,double max_tentative,Time* startTime,float const a_star_multiplier) const{
    vector<Edge*> path; // Vector to store the path
    Node* closestNode =startNode;
    if (startNode->name=="Vasco da Gama") {
        cout<<"Vasco da Gama";
    }
    startNode->distance = 0;
    startNode->arrivalTime = startTime;
    startNode->bestDistance = startNode->coordinates.haversineDistance(goal) * a_star_multiplier;

    double overallBestFScore = startNode->bestDistance;

    // Priority queue (open set)
    std::priority_queue<Node*, std::vector<Node*>, CompareNodesByBestDistance> openSet;
    openSet.push(startNode);

    double counter = 0;

    // A* main loop
    while (!openSet.empty()) {
        counter++;
        if(counter > max_tentative){
            cout<< "Breaked\n";
            break;
        }

        Node* current = openSet.top();
        openSet.pop();

        /*if (current->visited) {
            continue;
        }*/
        current->visited = true;

        // Explore neighbors
        vector<Edge*> directions = graph.getAdjacentEdges(current->id);
        vector<Edge*> footDirections = getKNearestFootEdges(current, 5,a_star_multiplier);
        //directions.insert(directions.end(), footDirections.begin(), footDirections.end()); // Add the foot edges to the directions

        for (const auto& dir : directions) {
            Node* neighbor = dir->destinationNode; // Get the neighboring node

            /*if (neighbor->visited) {
                continue; // Skip if the neighbor has already been visited
            }*/

            // Calculate tentative g-score
            int const waitingForStop  =dir->time->difference(current->arrivalTime); // Calculate the waiting time at the stop

            if (waitingForStop < 0) {
                cout<<"Error ALERT "<<waitingForStop;
            }

            double cost_current_to_neighbor = dir->travelTime + waitingForStop;
            double g_score_neighbor = current->distance + cost_current_to_neighbor;


            if (g_score_neighbor < neighbor->distance) {
                neighbor->distance = g_score_neighbor; // Update g-score
                neighbor->previous = dir;
                neighbor->arrivalTime = current->arrivalTime->clone();
                neighbor->arrivalTime->add_seconds(cost_current_to_neighbor); // Update arrival time at neighbor

                double h_score_neighbor = neighbor->coordinates.haversineDistance(goal) * a_star_multiplier;
                neighbor->bestDistance = g_score_neighbor + h_score_neighbor; // Update f-score

                openSet.push(neighbor);

                // Update the overall closest node found to the goal coordinates based on f-score
                if (neighbor->bestDistance < overallBestFScore) {
                    overallBestFScore = neighbor->bestDistance;
                    closestNode = neighbor;
                }

            }

        }
    }
    cout << "A* search finished. Reconstructing path from node: " << closestNode->id << endl;

    if (closestNode->previous == nullptr && closestNode != startNode) {
        cout << "Warning: closestNode has no previous edge and is not the startNode. Path might be incomplete or only contain the start node." << endl;
    }

    Edge* edge = closestNode->previous;

    Node* finalGoalNode = new Node("goal", goal.latitude, goal.longitude);
    finalGoalNode->distance = closestNode->bestDistance;
    cout << "Estimated f-score from start to actual goal coordinates via closest graph node (" << closestNode->id << "): " << finalGoalNode->distance << endl;


    double lastLegCost = closestNode->coordinates.haversineDistance(goal) * a_star_multiplier;
    path.push_back(new Edge(closestNode, finalGoalNode, nullptr, lastLegCost, "foot"));

    while (edge != nullptr) {
        path.push_back(edge);
        if (edge->startingNode == nullptr) {
            cerr << "Error in path reconstruction: edge has null startingNode." << endl;
            break;
        }
        closestNode = edge->startingNode;
        edge = closestNode->previous;
    }

    reverse(path.begin(), path.end());
    return path;
}
bool sortPaths(const pair<double, vector<Edge*>>& a, const pair<double, vector<Edge*>>& b){
    return a.first < b.first;
}
vector<pair<double,vector<Edge*>>> Manager::shortestPath(const Coordinates& start, const Coordinates& goal,Time* startTime,double max_tentative,const int alternatives,const float a_star_multiplier){

    vector<Point3D> nearestNodes= kdTree.kNearestNeighbors(start.toPoint3D(), alternatives); // Get the k nearest neighbors
    //reverse list
    //reverse(nearestNodes.begin(), nearestNodes.end());
    vector<Node*> startNodes;
    vector<float> distances_TMP;
    for (const Point3D node : nearestNodes){
        Node* startNode = graph.getNode(node.id); // Get the node from the graph using the ID
        startNodes.push_back(startNode); // Add the node to the vector
        distances_TMP.push_back(startNode->coordinates.haversineDistance(start));
        cout << "Debug Rodrigo chato: "<<startNode->name <<" "<<startNode->id<< endl;
    }

    vector<pair<double,vector<Edge*>>> result;
    int counter = -1;
    for(Node* station : startNodes){
        counter++;
        graph.reset();
        float distanceWithEuristic = distances_TMP[counter]*a_star_multiplier;
        Time* currTime = startTime->clone();
        currTime->add_seconds(distanceWithEuristic);
        vector<Edge*> path = shortestPathAstar(station,goal,max_tentative,currTime,a_star_multiplier); // Find the shortest path

        cout<<"\nStation : "<<station->name<<endl;
        Time* startTimeTmp = startTime->clone();

        path.insert(path.begin(), new Edge(nullptr,station, startTimeTmp,
            distanceWithEuristic,"foot"));

        // get distance of the last edge
        Node* last = path.back()->startingNode;
        double distance = last->bestDistance + distanceWithEuristic;

        cout << "RESULTS--------------------------------------------------";
        cout << "All distance : " << distance << endl; // Print the distance
        cout<<"Distance to the first station:" <<distanceWithEuristic<<endl;
        cout << "RESULTS--SHOW---------------------------------------------";


        result.push_back(make_pair(distance,path)); // Add the distance and path to the result
        //cout << "Debug ---------------------------------------------------:"<<endl;
        //newPrintPath(path,startTime);

    }
    for (auto i :result) {
        cout << "Debug ---------------------------------------------------:"<<endl;
        cout << "Distance: " << i.first << endl; // Print the distance
        cout <<"First Stop" << i.second.front()->destinationNode->name << endl;
        //newPrintPath(path,startTime);
    }
    std::sort(result.begin(), result.end(),sortPaths);
    return result; // Return the result
}

// Add to Manager.cpp
void Manager::printAllNodes() const {
    cout << "All Nodes in the Graph:" << endl;
    cout << "----------------------" << endl;
    for (const auto& node : graph.getNodes()) {
        cout << "Node ID: " << node.second->id << endl;
        cout << "Coordinates: (";
        auto coords = node.second->coordinates.getCoordinates();
        cout << "Lat: " << coords.first << ", Lon: " << coords.second << ")" << endl;
        cout << "----------------------" << endl;
    }
    cout << "Total number of nodes: " << graph.getNodes().size() << endl;
}

void Manager::printAllEdges() const {
    cout << "All Edges in the Graph:" << endl;
    cout << "----------------------" << endl;
    size_t totalEdges = 0;

    for (const auto& [id, node] : graph.getNodes()) {
        vector<Edge*> edges = graph.getAdjacentEdges(id);
        for (const Edge* edge : edges) {
            cout << "From Node: " << edge->startingNode->id << " -> To Node: " << edge->destinationNode->id << endl;
            edge->time->print();
            cout << "Type: " << edge->type << endl;
            cout << "----------------------" << endl;
            totalEdges++;
        }
    }
    cout << "Total number of edges: " << graph.getEdgeCount() << endl;
}

void Manager::newPrintPath(const vector<Edge*>& path, Time* journeyStartTime) const {
    if (path.empty()) {
        cout << "Path is empty." << endl;
        return;
    }

    cout << "\n==========================================================" << endl;
    cout << "               ROUTE DETAILS" << endl;
    cout << "==========================================================" << endl;
    cout << "Journey initially started at: ";
    journeyStartTime->print(); // Assumes journeyStartTime is the absolute start before any travel.
    cout << "----------------------------------------------------------" << endl;

    Time* currentEffectiveTime = journeyStartTime->clone(); // Tracks the traveller's time progression.
                                             // For the first leg (foot), this is the departure time.
                                             // For transit, this will become the arrival time at the previous stop.

    for (size_t i = 0; i < path.size(); ++i) {
        const Edge* edge = path[i];
        if (!edge) {
            cout << "Error: Null edge in path at index " << i << endl;
            continue;
        }

        Node* legStartNode = edge->startingNode; // Physical station/point where this leg begins
        Node* legDestNode = edge->destinationNode;   // Physical station/point where this leg ends
        //cout<<endl<< "DEBUG "<<legDestNode->id<<endl;
        if (legStartNode !=nullptr) {
            //cout<<endl<< "DEBUG "<<legStartNode->id<<endl;
        }
        string legStartName = "Your Current Location";
        if (legStartNode) {
            legStartName = legStartNode->name;
        }

        string legDestName = "Your Final Destination";
        if (legDestNode) {
            // Check if it's the special "goal" node created in A*
            // You might need a more robust way to identify this (e.g., a specific ID or flag)
            // For now, assuming if its name is empty or "goal" and it's the last edge.
            if (legDestNode->name.empty() || (legDestNode->id == "goal" && i == path.size() - 1) ) {
                 legDestName = "Your Final Destination";
            } else {
                legDestName = legDestNode->name;
            }
        }

        cout << "Leg " << i + 1 << ": Take " << edge->type
             << " from [" << legStartName << "] to [" << legDestName << "]" << endl;

        Time* legDepartureTime;

        if (edge->type == "foot") {
            // For foot travel, departure is immediate from currentEffectiveTime
            legDepartureTime = currentEffectiveTime;
            cout << "  Depart (walking): ";
            legDepartureTime->print();
        } else { // Transit edge
            if (edge->time) { // Scheduled departure time for this transit service
                legDepartureTime = edge->time->clone();
                // Calculate wait time at legStartNode
                // currentEffectiveTime is arrival at legStartNode from previous leg
                if (legStartNode) { // Ensure there's a station we arrived at
                    int waitSeconds = legDepartureTime->difference(currentEffectiveTime);
                    if (waitSeconds < 0) {
                        // This case (scheduled departure is before arrival at stop)
                        // should ideally be prevented by A* logic.
                        // Print a warning or handle as an issue.
                        cout << "  WARNING: Scheduled departure is BEFORE arrival at stop!" << endl;
                        cout << "    Arrival at " << legStartName << ": "; currentEffectiveTime->print();
                        cout << "    Scheduled Depart: "; legDepartureTime->print();
                        // To proceed, we might have to assume we wait for next day or it's an error.
                        // For now, just note it and proceed with scheduled departure.
                    } else if (waitSeconds > 5) { // Only print significant waits (e.g. >5 seconds)
                        cout << "  Wait at [" << legStartName << "]: "
                             << (waitSeconds / 60) << "m " << (waitSeconds % 60) << "s" << endl;
                        cout << "    (Arrived at stop: "; currentEffectiveTime->print();
                        cout << "     Scheduled service departure: "; legDepartureTime->print();
                        cout << "    )" << endl;
                    }
                }
                 cout << "  Depart via " << edge->type << " service: ";
                 legDepartureTime->print();
            } else {
                // Transit edge without a specific schedule? Unlikely with GTFS.
                // Default to immediate departure from currentEffectiveTime.
                Time* legDepartureTime = currentEffectiveTime;
                cout << "  Depart (unscheduled transit): ";
                legDepartureTime->print();
            }
        }

        // Update currentEffectiveTime: it's now the arrival time at legDestNode
        currentEffectiveTime = legDepartureTime; // Start with the departure time of the current leg
        currentEffectiveTime->add_seconds(edge->travelTime); // Add this leg's travel duration

        cout << "  Travel Time for this leg: " << (edge->travelTime / 60) << "m " << (edge->travelTime % 60) << "s" << endl;
        cout << "  Arrive at [" << legDestName << "]: ";
        currentEffectiveTime->print();
        cout << "----------------------------------------------------------" << endl;
    }

    cout << "Total Estimated Journey Time: ";
    int totalSeconds = currentEffectiveTime->difference(journeyStartTime); // journeyStartTime is the absolute start
    int totalHours = totalSeconds / 3600;
    int totalMinutes = (totalSeconds % 3600) / 60;
    int remainingSeconds = totalSeconds % 60;

    if (totalHours > 0) {
        cout << totalHours << "h ";
    }
    cout << totalMinutes << "m " << remainingSeconds << "s" << endl;
    cout << "==========================================================" << endl << endl;
}
