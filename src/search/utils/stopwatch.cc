#include "stopwatch.h"

using namespace std::chrono;

void Stopwatch::reset() {
    startTime = steady_clock::now();
    clocktime = std::chrono::steady_clock::time_point();

}
void Stopwatch::saveTime() {
    //stopTime = steady_clock::now();
   clocktime = steady_clock::time_point(steady_clock::now() - startTime);

}
void Stopwatch::continueTime() {
    startTime =steady_clock::time_point( steady_clock::now() - clocktime);
   // startTime.operator+=(time_span2);
}

double Stopwatch::operator()() const {
    duration<double> time_span = steady_clock::now() - startTime;
    return time_span.count();
}

std::ostream& operator<<(std::ostream& os, Stopwatch const& stopwatch) {
    os << stopwatch() << "s";
    return os;
}
