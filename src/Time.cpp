#include <iostream>
#include <chrono>
#include <iomanip> // for std::setfill and std::setw
#include <ctime>
#include "../include/Time.h"

// Constructor: Set time using hours, minutes, seconds
Time::Time(int hours, int minutes, int seconds) {
    total_seconds = hours *3600 + minutes *60 + seconds;
}
Time::Time(int totalSeconds) {
    total_seconds = totalSeconds;
}

//default constructor has the current time


// Get hours, minutes, seconds separately
int Time::get_hours() const {
    return (total_seconds/(60*60))%24;
}

int Time::get_minutes() const {
    return (total_seconds/60)%60;
}

int Time::get_seconds() const {
    return total_seconds%60;
}
void Time::add_seconds(int seconds) {
    total_seconds += seconds;
    total_seconds = total_seconds % (60*24*60);
}

// Print time in HH:MM:SS format
void Time::print() const {
    std::cout 
        << std::setfill('0') << std::setw(2) << get_hours() << ":"
        << std::setfill('0') << std::setw(2) << get_minutes() << ":"
        << std::setfill('0') << std::setw(2) << get_seconds() 
        << std::endl;
}


// Difference between two times (returns seconds)
int Time::difference(const Time* other) const {
    int differenceTmp =total_seconds - other->total_seconds;
    if (differenceTmp < 0) {
        differenceTmp = (60*60*24 -other->total_seconds) +total_seconds;
    }
    return differenceTmp;
}
Time* Time::clone() const {
    return new Time(total_seconds);
}
