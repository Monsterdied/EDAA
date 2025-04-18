#include "../include/Coordinates.h"
#include <cmath> // For std::sqrt
#include <utility> // For std::pair, std::make_pair
using namespace std;
Coordinates::Coordinates(double x_val, double y_val) : x(x_val), y(y_val) {}


pair<double, double> Coordinates::getCoordinates() const {
    return make_pair(x, y);
}
void Coordinates::setCoordinates(double x_val, double y_val) {
    x = x_val;
    y = y_val;
}

// Calculate squared Euclidean distance (avoids sqrt for comparison)
double Coordinates::distanceSq(const Coordinates& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    return dx * dx + dy * dy;
}

 // Calculate actual Euclidean distance
double Coordinates::distance(const Coordinates& other) const {
    return std::sqrt(distanceSq(other));
}


