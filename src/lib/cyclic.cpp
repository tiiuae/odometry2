// clang: MatousFormat

/* from https://github.com/ctu-mrs/mrs_lib */

#include <lib/geometry/cyclic.h>

namespace geometry
{

  // to ensure these classes are generated
  template struct cyclic<double, radians>;
  template struct cyclic<double, sradians>;
  template struct cyclic<double, degrees>;
  template struct cyclic<double, sdegrees>;

}  // namespace geometry

