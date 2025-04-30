#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include "../include/Coordinates.h" // Assuming this is the path to your Coordinates.h


// KD-Tree Node
struct KDNode {
    Point3D point;
    std::unique_ptr<KDNode> left;
    std::unique_ptr<KDNode> right;
    int axis; // 0 (x), 1 (y), or 2 (z)

    KDNode(Point3D p, int axis) : point(p), axis(axis) {}
};
struct PointDistancePair {
    double distance;
    Point3D point;

    bool operator<(const PointDistancePair& other) const {
        return distance < other.distance; // For max-heap
    }

};

// KD-Tree class
class KDTree {
private:
    std::unique_ptr<KDNode> root;
    // Recursive build function
    std::unique_ptr<KDNode> buildTree(std::vector<Point3D>& points, int depth);
    // Recursive nearest neighbor search
    void nearestNeighbor(const KDNode* node, const Point3D& query, Point3D& best_point, double& best_dist) const;
    void nearestKNeighbor(const KDNode* node,const int k ,const Point3D& query,priority_queue<PointDistancePair>& best_points) const;
public:
    KDTree() = default;
    KDTree(std::vector<Point3D> points);

    Point3D nearestNeighbor(const Point3D& query) const;
    vector<Point3D> kNearestNeighbors(const Point3D& query, int k) const;
};
