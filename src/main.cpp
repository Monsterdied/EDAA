#include <iostream>
#include <string>
#include <filesystem>
#include "Manager.h"



using namespace std;

int main(){


    Manager manager; // Create a Manager object
    manager.ReadMetroStations("../../data/MetroDoPorto/stops.txt"); // Read metro stations from a file
    
    return 0;
}