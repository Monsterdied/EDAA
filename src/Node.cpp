#include "../include/Node.h"
#include <iostream>
#include "../include/Coordinates.h"
using namespace std;


Node::Node(const string& id, double latitude, double longitude, string name, string type, string code) {
    this->id = id;
    this->coordinates = Coordinates(longitude, latitude); // Initialize coordinates
    this->name = name;
    this->type = type;
    this->code = code;
}

Point3D Node::toPoint3D() const{ // Create a Coordinates object
    Point3D point = this->coordinates.toPoint3D(); // Convert Coordinates to Point3D
    point.id = id; // Set the id of the point
    return point; // Assuming z-coordinate is 0 for 2D representation
}