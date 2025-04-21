#ifndef PROJ_COORDINATES_H
#define PROJ_COORDINATES_H
using namespace std;
#include <string>
// 3D Point (Cartesian coordinates on unit sphere)
struct Point3D {
    double x, y, z;
    string id;  // Explicitly use std::string (better practice in headers)
    
    Point3D(double x = 0, double y = 0, double z = 0, string id = "") 
        : x(x), y(y), z(z), id(id) {}
};

// Convert latitude/longitude (degrees) to 3D Cartesian
class Coordinates {
    private:
    double longitude = 0.0;
    double latitude = 0.0;
    public:
    Coordinates() = default;
    Coordinates(double longitude, double latitude);
    // get the coordinates
    pair<double, double> getCoordinates() const;
    // set the coordinates
    void setCoordinates(double x_val, double y_val);
    // transforms the coordinates to 3D point
    Point3D toPoint3D() const;
    // calculate haversine distance between two coordinates
    double haversineDistance(const Coordinates& other) const;
};



#endif //PROJ_GRAPH_H