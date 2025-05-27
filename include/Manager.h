#ifndef PROJ_MANAGER_H
#define PROJ_MANAGER_H

#pragma once
#include "Graph.h"
#include "KDTree.h"
using namespace std;

/**
 * @class Manager
 * @brief Manages graph data, KD-Tree, and provides high-level operations.
*/
class Manager {

    private:
        /**
         * @brief Reads station data from a specified file.
         * @param filename The path to the file containing station data.
         * @param type A string indicating the type of the transportation.
        */
        void ReadStations(const string& filename,string type);

        /**
         * @brief Reads route and stop data from a specified file.
         * @param filename The path to the file containing route and stop data.
         * @param type A string indicating the type of the transportation.
        */
        void ReadRoutesStops(const string& filename,string type);

        /**
         * @brief Retrieves k-nearest foot path edges for a given node.
         * @param node Pointer to the source Node.
         * @param k The number of nearest foot path edges to find.
         * @return A vector of Edge pointers representing the k nearest foot path edges.
        */
         vector<Edge*> getKNearestFootEdges(Node* node, int k,float a_star_multiplier)const;

    public:
        Graph graph; ///< The Graph object managing the network structure.
        KDTree kdTree; ///< The KD-Tree object for spatial indexing and nearest neighbor search.

        /**
         * @brief Reads GTFS (General Transit Feed Specification) data from a file.
         * @param filename The path to the GTFS data file.
         * @param type Type of the Transportation.
        */
        void ReadGTFS(const string& filename,string type);

        /**
         * @brief Builds the KD-Tree from the nodes present in the graph.
        */
        void buildKDTree();

        /**
         * @brief Finds the shortest path using the A* algorithm.
         * @param startNode Pointer to the starting Node for the path.
         * @param goal The Coordinates of the destination.
         * @param max_tentative A maximum tentative distance or cost for the search.
         * @param StartTime The time at which the journey starts (relevant for time-dependent graphs).
         * @param a_star_multiplier A multiplier for the A* heuristic function.
         * @return A vector of Edge pointers representing the shortest path found. Returns an empty vector if no path is found.
        */
        vector<Edge*> shortestPathAstar(Node* startNode, const Coordinates& goal,double max_tentative,Time* StartTime,float a_star_multiplier) const;

        /**
        * @brief Finds the shortest path(s) between start and goal coordinates.
        * @param start The Coordinates of the starting point.
        * @param goal The Coordinates of the destination point.
        * @param startTime The time at which the journey begins.
        * @param max_tentative Maximum tentative cost/distance for path search.
        * @param alternatives The number of alternative paths to find.
        * @param a_star_multiplier Multiplier for the A* heuristic.
        * @return A vector of pairs, where each pair consists of the path cost and the path itself.
        */
        vector<pair<double,vector<Edge*>>> shortestPath(const Coordinates& start, const Coordinates& goal,Time* startTime,double max_tentative=1000000000,
                                                        const int alternatives=15,const float a_star_multiplier=2); // Function to find the shortest path between two coordinates

       /**
       * @brief Constructor for the Manager class.
       */
        Manager();

       /**
        * @brief Prints all nodes in the graph to standard output.
        */
        void printAllNodes() const;

       /**
        * @brief Prints all edges in the graph to standard output.
        */
        void printAllEdges() const;

        /**
        * @brief Prints a given path with details considering the journey start time.
        * @param path A vector of Edge pointers representing the path.
        * @param journeyStartTime The Time at which the journey along this path begins.
        */
        void newPrintPath(const vector<Edge*>& path, Time* journeyStartTime) const;

         void printPath(const vector<Edge*>& path) const;

    
};

#endif //PROJ_MANAGER_H
