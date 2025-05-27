#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include "../include/Coordinates.h" // Assuming this is the path to your Coordinates.h


/**
 * @struct KDNode
 * @brief Represents a node in the KD-Tree. Each node stores a 3D point and pointers to the left and right children,
 * with the axis used for splitting at this node.
*/
struct KDNode {
    Point3D point; ///< The 3D point stored in this node.
    std::unique_ptr<KDNode> left; ///< Smart pointer to the left child node.
    std::unique_ptr<KDNode> right; ///< Smart pointer to the right child node.
    int axis; ///< The splitting axis for this node (0 for x, 1 for y, 2 for z).


    /**
     * @brief Constructor.
     * @param p The Point3D to store in this node.
     * @param axis The splitting axis.
    */
    KDNode(Point3D p, int axis) : point(p), axis(axis) {}
};


/**
 * @struct PointDistancePair
 * @brief A utility struct to pair a Point3D with its distance to a query point.
 *
 * Used in k-nearest neighbor searches, particularly with a priority queue.
*/
struct PointDistancePair {
    double distance; ///< The distance of the point from a query point.
    Point3D point;   ///< The Point3D.


    /**
     * @brief Less-than operator for comparing PointDistancePair objects.
     * @param other The other PointDistancePair to compare against.
     * @return True if this pair's distance is less than the other's, false otherwise.
    */
    bool operator<(const PointDistancePair& other) const {
        return distance < other.distance; // For max-heap
    }

};

/**
 * @class KDTree
 * @brief Implements a k-dimensional tree for efficient spatial queries on 3D points.
*/
class KDTree {
private:
    std::unique_ptr<KDNode> root; ///< Pointer to the root node of the KD-Tree.
    /**
     * @brief Recursively builds the KD-Tree.
     * @param points A vector of Point3D objects to build the tree from.
     * @param depth The current depth in the tree.
     * @return A unique_ptr to the root of the constructed (sub)tree.
    */
    std::unique_ptr<KDNode> buildTree(std::vector<Point3D>& points, int depth);

    /**
     * @brief Recursive helper function for nearest neighbor search.
     * @param node The current KDNode being visited.
     * @param query The query Point3D.
     * @param best_point Reference to the current best Point3D found so far.
     * @param best_dist Reference to the distance of the current best_point.
     */
    void nearestNeighbor(const KDNode* node, const Point3D& query, Point3D& best_point, double& best_dist) const;

    /**
     * @brief Recursive helper function for k-nearest neighbors search.
     * @param node The current KDNode being visited.
     * @param k The number of nearest neighbors to find.
     * @param query The query Point3D.
     * @param best_points A max-priority queue storing the k best PointDistancePair found so far.
    */
    void nearestKNeighbor(const KDNode* node,const int k ,const Point3D& query,priority_queue<PointDistancePair>& best_points) const;
public:

    /**
     * @brief Default constructor for KDTree.
    */
    KDTree() = default;

    /**
     * @brief Constructs a KDTree from a vector of Point3D objects.
     * @param points The vector of Point3D objects to build the tree from.
    */
    KDTree(std::vector<Point3D> points);

    /**
     * @brief Finds the single nearest neighbor to a query point.
     * @param query The Point3D to find the nearest neighbor for.
     * @return The Point3D that is nearest to the query point.
    */
    Point3D nearestNeighbor(const Point3D& query) const;

    /**
     * @brief Finds the k nearest neighbors to a query point.
     * @param query The Point3D to find neighbors for.
     * @param k The number of nearest neighbors to find.
     * @return A vector of Point3D objects representing the k nearest neighbors.
    */
    vector<Point3D> kNearestNeighbors(const Point3D& query, int k) const;
};
