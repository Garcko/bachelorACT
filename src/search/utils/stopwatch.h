#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <chrono>
#include <ostream>

// Measure amount of time elapsed since construction/reset
class Stopwatch {
public:
    void reset();
    // Returns the elapsed time since start
    double operator()() const;

private:
    std::chrono::steady_clock::time_point startTime;
};

// Convenience operator to stream elapsed time with seconds as unit
std::ostream& operator<<(std::ostream& os, const Stopwatch& stopwatch);

#endif
