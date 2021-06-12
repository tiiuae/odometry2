#pragma once
#ifndef ODOMETRY2_ODOMETRY_UTILS_H
#define ODOMETRY2_ODOMETRY_UTILS_H

#include <geometry_msgs/msg/quaternion.hpp>

namespace odometry2
{
namespace odometry_utils
{

/* struct GetHeadingException //{*/
struct GetHeadingException : public std::exception
{
  const char* what() throw() {
    return "Cannot calculate heading. The rotated x-axis is parallel to the world's z-axis.";
  }
};
/*//}*/

/* getHeading() //{*/
double getHeading(const geometry_msgs::msg::Quaternion& q) {

  tf2::Quaternion tf2_q;
  tf2_q.setX(q.x);
  tf2_q.setY(q.y);
  tf2_q.setZ(q.z);
  tf2_q.setW(q.w);

  tf2::Vector3 b1 = tf2::Vector3(1, 0, 0);

  tf2::Vector3 x_new = tf2::Transform(tf2_q) * b1;

  if (fabs(x_new[0]) <= 1e-3 && fabs(x_new[1]) <= 1e-3) {
    throw GetHeadingException();
  }

  return atan2(x_new[1], x_new[0]);
}
/*//}*/

/* TODO: IMPLEMENT!!! getAltitudeFromPressure() //{*/
  double getAltitudeFromPressure(const double pressure, const double temperature) {
    
    std::cerr << "getAltitudeFromPressure() is not implemented! Returns 0!" << std::endl;
    return 0.0;
  }
/*//}*/

}  // namespace odometry_utils
}  // namespace odometry2

#endif
