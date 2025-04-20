// Time.h
#ifndef TIME_H  // Include guard
#define TIME_H


#include <iostream>
#include <chrono>
#include <iomanip> // for std::setfill and std::setw
class Time {
    private:
        std::chrono::seconds total_seconds; // Stores time as total seconds since 00:00:00
    
    public:
        // Constructor: Set time using hours, minutes, seconds
        Time(int hours, int minutes, int seconds);
    
        // Get hours, minutes, seconds separately
        int get_hours() const;
    
        int get_minutes() const;
    
        int get_seconds() const;
    
        // Print time in HH:MM:SS format
        void print() const;
    
        // Difference between two times (returns seconds)
        int difference(const Time& other) const;
    };

#endif // TIME_H