#include <iostream>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#else
#include <ctime>
#endif

class TimeUtils {
public:
    // Function to get the current Unix timestamp in microseconds
    static std::uint64_t GetUnixTimestampMicroseconds() {
        auto now = std::chrono::system_clock::now();
        auto epoch = now.time_since_epoch();
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(epoch).count();
        return (uint64_t) microseconds;
    }

#ifdef _WIN32
    // Function to get the performance counter on Windows
    static std::uint64_t GetPerformanceCounter() {
        LARGE_INTEGER counter;
        if (!QueryPerformanceCounter(&counter)) {
            std::cerr << "QueryPerformanceCounter failed!\n";
            counter.QuadPart = -1; // Indicate failure
        }
        return std::uint64_t(counter.QuadPart);
    }

    // Function to get the performance frequency on Windows
    static std::uint64_t GetPerformanceFrequency() {
        LARGE_INTEGER frequency;
        if (!QueryPerformanceFrequency(&frequency)) {
            std::cerr << "QueryPerformanceFrequency failed!\n";
            frequency.QuadPart = -1; // Indicate failure
        }
        return std::uint64_t(frequency.QuadPart);
    }
#else
    // Function to get the performance counter on Linux
    static std::uint64_t GetPerformanceCounter() {
        struct timespec ts;
        if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
            std::cerr << "clock_gettime failed!\n";
            return -1; // Indicate failure
        }
        return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    }

    // Function to get the performance frequency on Linux
    static std::uint64_t GetPerformanceFrequency() {
        return (uint64_t) 1000000000LL; // CLOCK_MONOTONIC is in nanoseconds
    }
#endif
};