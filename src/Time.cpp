#include <iostream>
#include <chrono>
#include <iomanip> // for std::setfill and std::setw
#include "../include/Time.h"

// Constructor: Set time using hours, minutes, seconds
Time::Time(int hours, int minutes, int seconds) {
    total_seconds = 
        std::chrono::hours(hours) + 
        std::chrono::minutes(minutes) + 
        std::chrono::seconds(seconds);
}

// Get hours, minutes, seconds separately
int Time::get_hours() const {
    return std::chrono::duration_cast<std::chrono::hours>(total_seconds).count() % 24;
}

int Time::get_minutes() const {
    return std::chrono::duration_cast<std::chrono::minutes>(total_seconds).count() % 60;
}

int Time::get_seconds() const {
    return total_seconds.count() % 60;
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
int Time::difference(const Time& other) const {
    return (total_seconds - other.total_seconds).count();
}
    