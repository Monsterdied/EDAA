#include <iostream>
#include "../include/Node.h"
#include "../include/Edge.h"
using namespace std;


Edge::Edge(Node* startingNode, Node* destinationNode){
    this->startingNode = startingNode;
    this->destinationNode = destinationNode;
    cout << "Edge created from " << startingNode->id << " to " << destinationNode->id << endl;
}
