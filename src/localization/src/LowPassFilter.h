#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

class LowPassFilter {
public:
    LowPassFilter(double alpha);
    double filter(double x);

private:
    double alpha;
    bool initialized;
    double prev;
};

#endif // LOWPASSFILTER_H
