#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include "../include/Coordinates.h" // Assuming this is the path to your Coordinates.h
#include "../include/KDtree.h" // Assuming this is the path to your KDtree.h


// Recursive build function
std::unique_ptr<KDNode> KDTree::buildTree(std::vector<Point3D>& points, int depth) {
    if (points.empty()) return nullptr;

    int axis = depth % 3; // Cycle through x, y, z
    size_t median_idx = points.size() / 2;

    // Sort and find median
    std::nth_element(
        points.begin(), 
        points.begin() + median_idx, 
        points.end(),
        [axis](const Point3D& a, const Point3D& b) {
            if (axis == 0) return a.x < b.x;
            if (axis == 1) return a.y < b.y;
            return a.z < b.z;
        }
    );

    // Create node
    auto node = std::make_unique<KDNode>(points[median_idx], axis);
    
    // Recursively build subtrees
    auto left_points = std::vector<Point3D>(points.begin(), points.begin() + median_idx);
    auto right_points = std::vector<Point3D>(points.begin() + median_idx + 1, points.end());

    node->left = buildTree(left_points, depth + 1);
    node->right = buildTree(right_points, depth + 1);

    return node;
}

// Recursive nearest neighbor search
void KDTree::nearestNeighbor(const KDNode* node, const Point3D& query, Point3D& best_point, double& best_dist,int depth) const {
    if (!node) return;

    double dist = std::sqrt(
        std::pow(query.x - node->point.x, 2) +
        std::pow(query.y - node->point.y, 2) +
        std::pow(query.z - node->point.z, 2)
    );

    if (dist < best_dist) {
        best_dist = dist;
        best_point = node->point;
    }

    int axis = depth % 3;
    double diff;
    if (axis == 0) diff = query.x - node->point.x;
    else if (axis == 1) diff = query.y - node->point.y;
    else diff = query.z - node->point.z;

    KDNode* near = diff <= 0 ? node->left.get() : node->right.get();
    KDNode* far = diff <= 0 ? node->right.get() : node->left.get();

    nearestNeighbor(near, query, best_point, best_dist, depth + 1);

    if (std::abs(diff) < best_dist) {
        nearestNeighbor(far, query, best_point, best_dist, depth + 1);
    }
}

KDTree::KDTree(std::vector<Point3D> points) {
    root = buildTree(points, 0);
}

Point3D KDTree::nearestNeighbor(const Point3D& query) const {
    Point3D best_point;
    double best_dist = std::numeric_limits<double>::max();
    nearestNeighbor(root.get(), query, best_point, best_dist, 0);
    return best_point;
}