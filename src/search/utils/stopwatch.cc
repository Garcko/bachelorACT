#include "stopwatch.h"

using namespace std::chrono;

void Stopwatch::reset() {
    startTime = steady_clock::now();
    stopTime = steady_clock::now();
}
void Stopwatch::saveTime() {
    stopTime = steady_clock::now();
}
void Stopwatch::continueTime() {
    auto time_span2 = steady_clock::now() - stopTime;
    startTime.operator+=(time_span2);
}

double Stopwatch::operator()() const {
    duration<double> time_span = steady_clock::now() - startTime;
    return time_span.count();
}

std::ostream& operator<<(std::ostream& os, Stopwatch const& stopwatch) {
    os << stopwatch() << "s";
    return os;
}
