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
    // Convert latitude and longitude from degrees to radians
    double EARTH_RADIUS_KM = 6371.0; // Earth's radius in kilometers
    const double PI = 3.14159265358979323846; // Pi constant
    double DEG_TO_RAD = PI / 180.0;
    double lat1Rad = this->latitude * DEG_TO_RAD;
    double lon1Rad = this->longitude * DEG_TO_RAD;
    double lat2Rad = this->latitude * DEG_TO_RAD;
    double lon2Rad = this->longitude * DEG_TO_RAD;
    
    // Differences in coordinates
    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;
    
    // Haversine formula
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1Rad) * cos(lat2Rad) * 
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    // Calculate distance using the appropriate Earth radius
    double distance = EARTH_RADIUS_KM * c;
    
    return distance;
}

Point3D Coordinates::toPoint3D() const {
    const double PI = 3.14159265358979323846; // Pi constant
    double latRad = latitude * PI / 180.0;
    double lonRad = longitude * PI / 180.0;
    return Point3D(cos(latRad) * cos(lonRad), cos(latRad) * sin(lonRad), sin(latRad));
}


