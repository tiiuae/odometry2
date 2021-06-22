#ifndef MEDIAN_FILTER
#define MEDIAN_FILTER

/* author: Daniel Hert */

/* from https://github.com/ctu-mrs/mrs_lib */

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

class MedianFilter : public rclcpp::Node {

public:
  MedianFilter(int buffer_size, double max_valid_value, double min_valid_value, double max_difference, rclcpp::NodeOptions options);
  bool   isValid(double input);
  bool   isFilled();
  double getMedian();

private:
  std::vector<double> buffer;
  int                 buffer_size;
  int                 next;
  double              max_valid_value;
  double              min_valid_value;
  double              max_difference;
  bool                is_filled;
  double              m_median;
  std::mutex          mutex_median;
};

#endif
