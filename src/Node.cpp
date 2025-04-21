#include "../include/Node.h"
#include <iostream>
#include "../include/Coordinates.h"
using namespace std;


Node::Node(const string& id, double latitude, double longitude, string name, string type, string code, string desc, string zone) {
    this->id = id;
    this->latitude = latitude;
    this->longitude = longitude;
    this->name = name;
    this->type = type;
    this->code = code;
    this->desc = desc;
    this->zone = zone;
}

Point3D Node::toPoint3D() const{
    Coordinates cords =Coordinates(longitude, latitude); // Create a Coordinates object
    Point3D point = cords.toPoint3D(); // Convert Coordinates to Point3D
    point.id = id; // Set the id of the point
    return point; // Assuming z-coordinate is 0 for 2D representation
}