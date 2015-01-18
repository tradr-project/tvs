#include <boost/locale.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sstream>
#include <string>
#include "utils.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

std::string getDateTimeString(const char *format) {
    const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet * const f = new boost::posix_time::time_facet(format);
    std::stringstream ss;
    ss.imbue(std::locale(ss.getloc(),f));
    ss << now;
    return ss.str();
}

void LinVelProfInt::step(double timeStep) {
    x += double(sgn(tx - x)) * vmax * timeStep;
}

double LinVelProfInt::get() {
    return x;
}

void LinVelProfInt::reset() {
    tx = 0;
    x = 0;
}

void LinVelProfInt::set(double target) {
    tx = target;
}

void LinVelProfInt::setSlope(double slope) {
    vmax = slope;
}