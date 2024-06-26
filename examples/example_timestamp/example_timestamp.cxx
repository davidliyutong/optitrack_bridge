#include <iostream>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#else
#include <ctime>
#endif

#include "time_utils.hpp"

int main() {
    // Get current Unix timestamp in microseconds
    std::uint64_t unix_timestamp_microseconds = TimeUtils::GetUnixTimestampMicroseconds();
    std::cout << "Current Unix Timestamp in Microseconds: " << unix_timestamp_microseconds << std::endl;

    // Get performance counter
    uint64_t counter = TimeUtils::GetPerformanceCounter();
    std::cout << "Performance Counter: " << counter << std::endl;

    // Get performance frequency
    uint64_t frequency = TimeUtils::GetPerformanceFrequency();
    std::cout << "Performance Frequency: " << frequency << " counts per second" << std::endl;


    return 0;
}
