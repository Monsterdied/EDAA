#include <iostream>
#include "../include/Node.h"
#include "../include/Edge.h"

using namespace std;


Edge::Edge(Node* startingNode, Node* destinationNode,Time* time,int travelTime) {
    this->time = time;
    this->startingNode = startingNode;
    this->destinationNode = destinationNode;
    this->travelTime = travelTime; // Set the travel time in seconds
    //cout << "Edge created from " << startingNode->id << " to " << destinationNode->id << endl;
}
