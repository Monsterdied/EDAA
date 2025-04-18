#include "quadtree_locator.h"
#include <stdexcept> // For invalid_argument
#include <algorithm> // For std::reverse
#include <vector>    // Include vector for sorting children pointers

// --- QuadtreeNode Implementation ---

QuadtreeNode::QuadtreeNode(const Rectangle& boundary, int capacity)
    : boundary_(boundary), capacity_(capacity > 0 ? capacity : 1), divided_(false)
{
    // Reserve some space anticipating points, minor optimization
    stations_.reserve(capacity_);
}

void QuadtreeNode::subdivide() {
    double cx = boundary_.centerX;
    double cy = boundary_.centerY;
    double hw = boundary_.halfWidth / 2.0; // New half width
    double hh = boundary_.halfHeight / 2.0; // New half height

    // Create the four children with their respective boundaries
    northWest_ = std::make_unique<QuadtreeNode>(Rectangle(cx - hw, cy + hh, hw, hh), capacity_);
    northEast_ = std::make_unique<QuadtreeNode>(Rectangle(cx + hw, cy + hh, hw, hh), capacity_);
    southWest_ = std::make_unique<QuadtreeNode>(Rectangle(cx - hw, cy - hh, hw, hh), capacity_);
    southEast_ = std::make_unique<QuadtreeNode>(Rectangle(cx + hw, cy - hh, hw, hh), capacity_);

    divided_ = true;

    // Redistribute points from this node to the appropriate children
    // Important: Create a copy before clearing, otherwise iterators invalidate
    std::vector<Station> pointsToRedistribute = std::move(stations_);
    stations_.clear(); // This node no longer holds points directly (only children do)
    stations_.shrink_to_fit(); // Optional: release memory

    for (const auto& station : pointsToRedistribute) {
        // Try inserting into each child - only one should succeed
        if (northWest_->insert(station)) continue;
        if (northEast_->insert(station)) continue;
        if (southWest_->insert(station)) continue;
        if (southEast_->insert(station)) continue;
        // Should ideally not happen if boundary checks are correct, but handle defensively
        // Maybe log an error or store in the parent if subdivision fails unexpectedly?
        // For simplicity here, we assume one child will always accept it.
    }
}

bool QuadtreeNode::insert(const Station& station) {
    // Ignore points outside this node's boundary
    if (!boundary_.contains(station.coords)) {
        return false;
    }

    // If this node is a leaf node
    if (!divided_) {
        // If there's space, add the point here
        if (stations_.size() < static_cast<size_t>(capacity_)) {
            stations_.push_back(station);
            return true;
        }
        // Otherwise, subdivide the node
        else {
            subdivide();
            // After subdividing, fall through to insert into the correct child (below)
        }
    }

    // If this node is already divided, insert into the appropriate child
    // Note: After subdivision in the block above, 'divided_' is now true
    if (northWest_->insert(station)) return true;
    if (northEast_->insert(station)) return true;
    if (southWest_->insert(station)) return true;
    if (southEast_->insert(station)) return true;

    // Should not happen if logic is correct, point is within boundary and node is divided
    // Indicates an issue with contains() or subdivision logic.
    return false; // Indicate insertion failed somehow
}


// Recursive helper for finding K nearest neighbors
void QuadtreeNode::queryNearestRecursive(
    const Coordinates& location,
    int k,
    std::priority_queue<std::pair<double, int>>& nearestStationsHeap,
    double& maxDistanceSq // Farthest distance in the heap (or infinity if not full)
) const
{
    // Pruning Rule 1: If the query point is farther away from this node's
    // boundary than the K-th nearest point found so far, we don't need
    // to explore this node or its children.
    if (boundary_.minDistanceSq(location) > maxDistanceSq) {
        return;
    }

    // Check points within this node (only applies to leaf nodes after subdivision,
    // or nodes that haven't been subdivided yet)
    for (const auto& station : stations_) {
        double distSq = location.distanceSq(station.coords);

        if (nearestStationsHeap.size() < static_cast<size_t>(k)) {
            nearestStationsHeap.push({distSq, station.id});
            // Update maxDistanceSq if the heap just became full
            if (nearestStationsHeap.size() == static_cast<size_t>(k)) {
                 maxDistanceSq = nearestStationsHeap.top().first;
            }
             // If not full yet, max distance is effectively infinite for pruning purposes
             // but we can track the current max for slightly better pruning below
             else if (distSq > maxDistanceSq) {
                  // this comparison only matters if heap is not full
                  // use a large value if empty for maxDistanceSq initially
                  maxDistanceSq = distSq; // Update max seen so far (though not the Kth yet)
             }
        } else if (distSq < maxDistanceSq) { // Found a closer point
            nearestStationsHeap.pop(); // Remove the farthest
            nearestStationsHeap.push({distSq, station.id});
            maxDistanceSq = nearestStationsHeap.top().first; // Update the Kth distance
        }
    }

    // Recursively search children if this node is divided
    if (divided_) {
        // Optimization: Search children potentially closer to the point first.
        // Create pairs of {min_distance_to_child_boundary_sq, child_pointer}
        std::vector<std::pair<double, const QuadtreeNode*>> childrenDistances;
        childrenDistances.reserve(CHILD_COUNT);
        if(northWest_) childrenDistances.push_back({northWest_->boundary_.minDistanceSq(location), northWest_.get()});
        if(northEast_) childrenDistances.push_back({northEast_->boundary_.minDistanceSq(location), northEast_.get()});
        if(southWest_) childrenDistances.push_back({southWest_->boundary_.minDistanceSq(location), southWest_.get()});
        if(southEast_) childrenDistances.push_back({southEast_->boundary_.minDistanceSq(location), southEast_.get()});

        // Sort children by distance to their boundary (ascending)
        std::sort(childrenDistances.begin(), childrenDistances.end(),
                  [](const auto& a, const auto& b) {
                      return a.first < b.first;
                  });

        // Recursively query children in order of proximity
        for (const auto& pair : childrenDistances) {
            // Important: Re-check pruning rule before recursing into each child,
            // because maxDistanceSq might have been updated by other children/points.
             if (pair.first <= maxDistanceSq) { // Use <= because boundary distance could be 0
                 pair.second->queryNearestRecursive(location, k, nearestStationsHeap, maxDistanceSq);
             }
        }

        // --- Simpler alternative (without sorting children): ---
        // northWest_->queryNearestRecursive(location, k, nearestStationsHeap, maxDistanceSq);
        // northEast_->queryNearestRecursive(location, k, nearestStationsHeap, maxDistanceSq);
        // southWest_->queryNearestRecursive(location, k, nearestStationsHeap, maxDistanceSq);
        // southEast_->queryNearestRecursive(location, k, nearestStationsHeap, maxDistanceSq);
    }
}


// --- QuadtreeLocator Implementation ---

QuadtreeLocator::QuadtreeLocator(const Rectangle& boundary, int nodeCapacity)
    : boundary_(boundary), nodeCapacity_(nodeCapacity), stationCount_(0)
{
    root_ = std::make_unique<QuadtreeNode>(boundary_, nodeCapacity_);
}

bool QuadtreeLocator::insert(int id, double x, double y) {
    return insert(Station(id, x, y));
}

bool QuadtreeLocator::insert(const Station& station) {
     // Ensure the station is within the root boundary before attempting insertion
    if (!boundary_.contains(station.coords)) {
       // Option 1: Reject the point
       // return false;

       // Option 2: Expand the root boundary (more complex) - not implemented here

       // Option 3: Allow insertion but log warning (simplest if points are expected near boundary)
        // std::cerr << "Warning: Station " << station.id << " at (" << station.coords.x << "," << station.coords.y
        //           << ") is outside the initial quadtree boundary." << std::endl;
        // If we allow it, the root->insert check will handle it, but it might fail if root can't contain it.
        // Let's stick to rejecting points outside the initial boundary for clarity.
         return false;
    }

    if (root_->insert(station)) {
        stationCount_++;
        return true;
    }
    return false; // Insertion failed (e.g., duplicate coords at leaf capacity, though unlikely with subdivision)
}

std::vector<int> QuadtreeLocator::findNearestStations(const Coordinates& location, int k) const {
    if (k <= 0 || stationCount_ == 0) {
        return {};
    }

    // Max-heap to store {squared_distance, station_id}
    std::priority_queue<std::pair<double, int>> nearestStationsHeap;

    // Initialize maxDistanceSq to infinity. This represents the distance to the
    // K-th element. If the heap has less than K elements, any distance is smaller.
    double maxDistanceSq = std::numeric_limits<double>::infinity();

    // Start the recursive query from the root
    if (root_) {
        root_->queryNearestRecursive(location, k, nearestStationsHeap, maxDistanceSq);
    }

    // Extract results from the heap (they come out farthest first)
    std::vector<int> result_ids;
    result_ids.reserve(nearestStationsHeap.size());
    while (!nearestStationsHeap.empty()) {
        result_ids.push_back(nearestStationsHeap.top().second);
        nearestStationsHeap.pop();
    }

    // Reverse to get nearest first
    std::reverse(result_ids.begin(), result_ids.end());

    return result_ids;
}

void QuadtreeLocator::clear() {
    // Replace the root with a new empty one, letting unique_ptr handle deletion
    root_ = std::make_unique<QuadtreeNode>(boundary_, nodeCapacity_);
    stationCount_ = 0;
}