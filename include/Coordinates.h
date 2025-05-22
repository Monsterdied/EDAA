#ifndef PROJ_COORDINATES_H
#define PROJ_COORDINATES_H
#include <string>
using namespace std;


/**
 * @struct Point3D
 * @brief Represents a 3D Cartesian point on a unit sphere
 */
struct Point3D {
    /// 3D variables
    double x, y, z;

    ///Identifier
    string id;

    /**
     * @brief Construct a new Point3D object
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param z Z-coordinate
     * @param id Identifier string
     */
    Point3D(double x = 0, double y = 0, double z = 0, string id = "")
        : x(x), y(y), z(z), id(id) {}
};

/**
 * @class Coordinates
 * @brief Handles geographic coordinate conversions between latitude/longitude and 3D Cartesian
 */
class Coordinates {
    public:
        double longitude = 0.0;
        double latitude = 0.0;

        Coordinates() = default;

        /**
         * @brief Construct from explicit longitude/latitude values
         * @param longitude Longitude in degrees
         * @param latitude Latitude in degrees
        */
        Coordinates(double longitude, double latitude);

        /**
         * @brief Construct from a 3D Cartesian point (reverse conversion)
         * @param point 3D point on unit sphere to convert
        */
        Coordinates(const Point3D point);

        /**
        * @brief Get current coordinates as a (longitude, latitude) pair
        * @return std::pair<double, double> where first=longitude, second=latitude
        */
        pair<double, double> getCoordinates() const;

        /**
         * @brief Set new coordinate values
         * @param longitude New longitude value in degrees
         * @param latitude New latitude value in degrees
        */
        void setCoordinates(double x_val, double y_val);

        /**
          * @brief Convert to 3D Cartesian coordinates on a unit sphere
          * @return Point3D with x,y,z coordinates (unit length)
          * @note Uses spherical coordinate conversion formulas
        */
        Point3D toPoint3D() const;

        /**
         * @brief Calculate great-circle distance using Haversine formula
         * @param other Target coordinates for distance calculation
         * @return double Distance in meters
         * @note Earth's radius is approximated as 6371 km
        */
        double haversineDistance(const Coordinates& other) const;
};



#endif //PROJ_GRAPH_H