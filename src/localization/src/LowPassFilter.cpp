#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(double alpha)
    : alpha(alpha), initialized(false), prev(0.0) {}

double LowPassFilter::filter(double x) {
    if (!initialized) {
        prev = x;
        initialized = true;
    }
    prev = alpha * x + (1.0 - alpha) * prev;
    return prev;
}
