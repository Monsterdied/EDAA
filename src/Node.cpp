#include "Node.h"
#include <iostream>

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
