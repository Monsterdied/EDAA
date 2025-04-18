#ifndef PROJ_COORDINATES_H
#define PROJ_COORDINATES_H
using namespace std;
#include "Node.h"
#include "Edge.h"
class Coordinates {
    private:
    double x = 0.0;
    double y = 0.0;
    public:
    Coordinates() = default;
    Coordinates(double x_val, double y_val);
    pair<double, double> getCoordinates() const;
    void setCoordinates(double x_val, double y_val);

    // Calculate squared Euclidean distance (avoids sqrt for comparison)
    double distanceSq(const Coordinates& other) const;

     // Calculate actual Euclidean distance
    double distance(const Coordinates& other) const ;
};



#endif //PROJ_GRAPH_H