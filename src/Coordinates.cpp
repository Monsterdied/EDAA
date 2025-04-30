#include "../include/Coordinates.h"
#include <cmath> // For std::sqrt
#include <utility> // For std::pair, std::make_pair
#include <cmath>
using namespace std;
Coordinates::Coordinates(double longitude_1, double latitude_1) : longitude(longitude_1), latitude(latitude_1) {}


pair<double, double> Coordinates::getCoordinates() const {
    return make_pair(longitude, latitude);
}
//need to be tested WARNING
// Convert latitude and longitude to 3D Cartesian coordinates
// Coordinates::Coordinates(double longitude, double latitude) {
//NEED TO BE TESTED
Coordinates::Coordinates(const Point3D point) {
    const double PI = 3.14159265358979323846; // Pi constant

    // Calculate longitude (λ) in degrees
    double lonRad = atan2(point.y, point.x);
    this->longitude = lonRad * 180.0 / PI;

    // Calculate latitude (φ) in degrees
    double xyProjection = sqrt(point.x * point.x + point.y * point.y);
    double latRad = atan2(point.z, xyProjection);
    this->latitude = latRad * 180.0 / PI;
}
void Coordinates::setCoordinates(double longitude, double latitude) {
    this->latitude = latitude;
    this->longitude = longitude;
}

// Calculate squared Euclidean distance (avoids sqrt for comparison)

 // Calculate actual Euclidean distance
 double Coordinates::haversineDistance(const Coordinates& other) const {
    double lat1 = latitude;
    double lon1 = longitude;
    double lat2 = other.latitude;
    double lon2 = other.longitude;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * 
        M_PI / 180.0;

    // convert to radians
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;

    // apply formulae
    double a = pow(sin(dLat / 2), 2) + 
    pow(sin(dLon / 2), 2) * 
    cos(lat1) * cos(lat2);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    return rad * c*1000; // Convert to meters
}

Point3D Coordinates::toPoint3D() const {
    double latRad = latitude * M_PI / 180.0;
    double lonRad = longitude * M_PI / 180.0;
    return Point3D(cos(latRad) * cos(lonRad), cos(latRad) * sin(lonRad), sin(latRad));
}


