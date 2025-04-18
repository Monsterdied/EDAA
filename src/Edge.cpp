#include <iostream>
#include "Node.h"
#include "Edge.h"
using namespace std;


Edge::Edge(Node* startingNode, Node* destinationNode){
    this->startingNode = startingNode;
    this->destinationNode = destinationNode;
    cout << "Edge created from " << startingNode->id << " to " << destinationNode->id << endl;
}
