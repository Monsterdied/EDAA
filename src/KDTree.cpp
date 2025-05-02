#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include "../include/Coordinates.h" // Assuming this is the path to your Coordinates.h
#include "../include/KDtree.h" // Assuming this is the path to your KDtree.h
#include <queue>


double distance(const Point3D& p1, const Point3D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}
// Recursive build function
unique_ptr<KDNode> KDTree::buildTree(vector<Point3D>& points, int depth) {
    if (points.empty()) return nullptr;

    int axis = depth % 3; // Cycle through x, y, z
    size_t median_idx = points.size() / 2;

    // Sort and find median
    nth_element(
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
    auto node = make_unique<KDNode>(points[median_idx], axis);
    
    // Recursively build subtrees
    auto left_points = vector<Point3D>(points.begin(), points.begin() + median_idx);
    auto right_points = vector<Point3D>(points.begin() + median_idx + 1, points.end());

    node->left = buildTree(left_points, depth + 1);
    node->right = buildTree(right_points, depth + 1);

    return node;
}

// Recursive nearest neighbor search
void KDTree::nearestNeighbor(const KDNode* node, const Point3D& query, Point3D& best_point, double& best_dist) const {
    if (!node) return;

    double dist = distance(query, node->point);

    if (dist < best_dist) {
        best_dist = dist;
        best_point = node->point;
    }

    int axis = node->axis;
    double diff;
    if (axis == 0) diff = query.x - node->point.x;
    else if (axis == 1) diff = query.y - node->point.y;
    else diff = query.z - node->point.z;

    KDNode* near = diff <= 0 ? node->left.get() : node->right.get();
    KDNode* far = diff <= 0 ? node->right.get() : node->left.get();

    nearestNeighbor(near, query, best_point, best_dist);

    if (abs(diff) < best_dist) {
        nearestNeighbor(far, query, best_point, best_dist);
    }
}

KDTree::KDTree(vector<Point3D> points) {
    root = buildTree(points, 0);
}

Point3D KDTree::nearestNeighbor(const Point3D& query) const {
    Point3D best_point;
    double best_dist = numeric_limits<double>::max();
    nearestNeighbor(root.get(), query, best_point, best_dist);
    return best_point;
}

// K Nearest Neighbors

struct node_cmp
{
    bool operator()( const pair<double,Point3D> a, const pair<double,Point3D> b ) const
    {
        return a.first < b.first;
    }
};
// Recursive nearest neighbor search
void KDTree::nearestKNeighbor(const KDNode* node,const int k ,const Point3D& query,priority_queue<PointDistancePair>& best_points) const {
    if (!node) return;

    double dist = distance(query, node->point);
    //update priority list

    if (size(best_points)< k) {
        best_points.push(PointDistancePair{dist, node->point});
    }else if (dist < best_points.top().distance) {
        best_points.pop();
        best_points.push(PointDistancePair{dist, node->point});
    }

    int axis = node->axis;
    double diff;
    if (axis == 0) diff = query.x - node->point.x;
    else if (axis == 1) diff = query.y - node->point.y;
    else diff = query.z - node->point.z;

    KDNode* near = diff <= 0 ? node->left.get() : node->right.get();
    KDNode* far = diff <= 0 ? node->right.get() : node->left.get();

    nearestKNeighbor(near, k,query, best_points);

    if (best_points.size() < k || std::abs(diff) < best_points.top().distance) {
        // The hyperplane could contain a point closer than the current k-th nearest,
        // so search the "far" child.
        nearestKNeighbor(far, k, query,best_points);
    }
}
vector<Point3D> KDTree::kNearestNeighbors(const Point3D& query, int k) const{
    priority_queue<PointDistancePair> best_points;
    nearestKNeighbor(root.get(),k, query, best_points);
    vector<Point3D> points;
    while(!best_points.empty()){
        points.push_back(best_points.top().point);
        best_points.pop();
    }
    return points;

}
