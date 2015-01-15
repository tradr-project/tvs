#include <boost/locale.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sstream>
#include <string>

std::string getDateTimeString(const char *format) {
    const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet * const f = new boost::posix_time::time_facet(format);
    std::stringstream ss;
    ss.imbue(std::locale(ss.getloc(),f));
    ss << now;
    return ss.str();
}
