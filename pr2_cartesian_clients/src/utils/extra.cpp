#include <utils/extra.hpp>

namespace pr2_cartesian_clients{
  /*
    Returns true if the given string is in the given vector
  */
  bool stringInVector(std::string s, std::vector<std::string> v)
  {
    return std::find(v.begin(), v.end(), s) != v.end();
  }
}
