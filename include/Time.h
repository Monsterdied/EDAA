// Time.h
#ifndef TIME_H  // Include guard
#define TIME_H

#include <iostream>
#include <chrono>

/**
 * @class Time
 * @brief Represents a time of day with hour, minute, and second components.
 *
 * The Time class stores time internally as total seconds since 00:00:00
 * and provides various operations for time manipulation and comparison.
*/
class Time {
    private:
        std::chrono::seconds total_seconds; ///< Stores time as total seconds since 00:00:00
    
    public:

        /**
        * @brief Default constructor.
        * @details Initializes time to 00:00:00.
        */
        Time();

        /**
        * @brief Parameterized constructor.
        * @param hours The hour component
        * @param minutes The minute component
        * @param seconds The second component
        */
        Time(int hours, int minutes, int seconds);

        /**
         * @brief Adds seconds to the current time.
         * @param seconds Number of seconds to add
        */
        void add_seconds(int seconds);
    
        /**
         * @brief Gets the hour component.
         * @return Hour
        */
        int get_hours() const;

        /**
         * @brief Gets the minute component.
         * @return Minute
        */
        int get_minutes() const;

        /**
         * @brief Gets the second component.
         * @return Second
        */
        int get_seconds() const;

        /**
         * @brief Prints the time in HH:MM:SS format to standard output.
        */
        void print() const;

        /**
         * @brief Calculates the difference between two times.
         * @param other The other Time object to compare with
         * @return Difference in seconds (positive if this time is later than other)
        */
        int difference(const Time& other) const;

        /**
         * @brief Compares if this time is earlier than another time.
         * @param other The other Time object to compare with
         * @return true if this time is earlier than other, false otherwise
         */
        bool isEarlierThan(const Time& other) const;

        /**
         * @brief Creates a copy of this Time object.
         * @return A new Time object with identical time values
         */
        Time clone() const;
    };

#endif // TIME_H