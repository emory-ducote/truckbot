#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

namespace localization {

class LowPassFilter {
public:
    LowPassFilter(double alpha);
    double filter(double x);

private:
    double alpha;
    bool initialized;
    double prev;
};

} // namespace localization

#endif // LOWPASSFILTER_H
