#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <mutex>
#include <rclcpp/subscription.hpp>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <fog_msgs/srv/get_origin.hpp>
#include <fog_msgs/srv/get_bool.hpp>
#include <fog_msgs/srv/vec4.hpp>
#include <fog_msgs/srv/change_estimator.hpp>
#include <fog_msgs/msg/estimator_type.hpp>
#include <nav_msgs/msg/path.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/sensor_baro.hpp>

#include <mavsdk/geometry.h>

#include <eigen3/Eigen/Core>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mrs_lib/median_filter.h>
#include <mrs_lib/geometry/cyclic.h>

#include "types.h"
#include "altitude_estimator.h"
#include "heading_estimator.h"
#include "lateral_estimator.h"

/* defines //{ */

#define NAME_OF(v) #v

//}

using namespace std::placeholders;

namespace odometry2
{

/* class Odometry2 //{ */
class Odometry2 : public rclcpp::Node {
public:
  Odometry2(rclcpp::NodeOptions options);

private:
  bool is_initialized_    = false;
  bool callbacks_enabled_ = false;

  std::atomic_bool getting_garmin_       = false;
  std::atomic_bool getting_pixhawk_odom_ = false;
  std::atomic_bool getting_hector_       = false;
  std::atomic_bool getting_baro_         = false;
  std::atomic_bool odom_ready_           = false;
  std::atomic_bool getting_gps_          = false;

  double odometry_loop_rate_;

  std::string uav_name_            = "";
  std::string world_frame_         = "";
  std::string ned_origin_frame_    = "";
  std::string ned_fcu_frame_       = "";
  std::string fcu_frame_           = "";
  std::string hector_origin_frame_ = "";
  std::string hector_frame_        = "";

  // use takeoff lat and long to initialize local frame
  mavsdk::geometry::CoordinateTransformation::GlobalCoordinate ref;

  unsigned int px4_system_id_;
  unsigned int px4_component_id_ = 1;
  std::string  target_ip_addr_;
  unsigned int target_udp_port_;

  std::shared_ptr<tf2_ros::Buffer>                     tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // GPS
  std::atomic<float> pos_gps_[3];
  std::atomic<float> ori_gps_[4];
  /* std::mutex mutex_gps_; */

  // Hector
  std::atomic<float> pos_hector_[3];
  std::atomic<float> ori_hector_[4];
  /* std::mutex mutex_hector_; */
  int c_hector_init_msgs_ = 0;
  double hector_hdg_previous_ = 0.0;

  // Vehicle local position
  std::atomic<float> pos_local_[3];
  std::atomic<float> ori_local_[4];
  /* std::mutex mutex_local_; */

  // Altitude estimation
  std::shared_ptr<AltitudeEstimator> garmin_alt_estimator_;
  std::vector<alt_R_t> R_alt_vec_;

  std::unique_ptr<MedianFilter> alt_mf_garmin_;
  std::atomic<double> garmin_alt_correction_;
  std::atomic_bool got_garmin_alt_correction_ = false;

  std::atomic<double> baro_alt_correction_;
  std::atomic<double> baro_alt_correction_prev_;
  std::atomic_bool got_baro_alt_correction_ = false;
  rclcpp::Time time_baro_prev_;

  // Heading estimation
  std::shared_ptr<HeadingEstimator> hector_hdg_estimator_;
  std::vector<hdg_R_t> R_hdg_vec_;

  std::atomic<double> hector_hdg_correction_;
  std::atomic_bool got_hector_hdg_correction_ = false;

  // Lateral estimation
  std::shared_ptr<LateralEstimator> hector_lat_estimator_;
  std::vector<lat_R_t> R_lat_vec_;

  std::atomic<double> hector_lat_correction_[2];
  std::atomic_bool got_hector_lat_correction_ = false;;

  // Odometry switch
  fog_msgs::msg::EstimatorType    estimator_source_;
  std::string                     _estimator_source_param_;
  std::mutex                      mutex_estimator_source_;
  std::vector<std::string>        estimator_type_names_;
  std::atomic<unsigned long long> timestamp_;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom_publisher_;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr              timesync_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr       pixhawk_odom_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr            baro_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr      hector_pose_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr              garmin_subscriber_;

  // subscriber callbacks
  void timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg);
  void gpsCallback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg);
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void hectorPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg);
  void baroCallback(const px4_msgs::msg::SensorBaro::UniquePtr msg);
  void garminCallback(const sensor_msgs::msg::Range::UniquePtr msg);

  // services provided
  rclcpp::Service<fog_msgs::srv::ChangeEstimator>::SharedPtr change_odometry_source_;
  rclcpp::Service<fog_msgs::srv::GetBool>::SharedPtr         getting_odom_service_;
  rclcpp::Service<fog_msgs::srv::GetOrigin>::SharedPtr       get_origin_service_;

  // service callbacks
  bool callbackChangeEstimator(const std::shared_ptr<fog_msgs::srv::ChangeEstimator::Request> req,
                               std::shared_ptr<fog_msgs::srv::ChangeEstimator::Response>      res);
  bool getOdomCallback(const std::shared_ptr<fog_msgs::srv::GetBool::Request> request, std::shared_ptr<fog_msgs::srv::GetBool::Response> response);
  bool getOriginCallback(const std::shared_ptr<fog_msgs::srv::GetOrigin::Request> request, std::shared_ptr<fog_msgs::srv::GetOrigin::Response> response);


  // internal functions
  bool        isValidType(const fog_msgs::msg::EstimatorType &type);
  bool        changeCurrentEstimator(const fog_msgs::msg::EstimatorType &desired_estimator);
  void        setupEstimator(const std::string &type);
  std::string printOdometryDiag();
  std::string toUppercase(const std::string &type);

  void publishTF();
  void publishStaticTF();
  void publishLocalOdom();
  void updateEstimators();

  geometry_msgs::msg::PoseStamped transformBetween(std::string frame_from, std::string frame_to);
  std_msgs::msg::ColorRGBA        generateColor(const double r, const double g, const double b, const double a);

  // timers
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr     odometry_timer_;
  void                             odometryRoutine(void);
  rclcpp::Time                        time_odometry_timer_prev_;

  // utils
  template <class T>
  bool parse_param(std::string param_name, T &param_dest);
};
//}

/* constructor //{ */
Odometry2::Odometry2(rclcpp::NodeOptions options) : Node("odometry2", options) {

  RCLCPP_INFO(this->get_logger(), "[%s]: Initializing...", this->get_name());
  ref.latitude_deg = 0.0;
  ref.latitude_deg = 0.0;

  // Getting
  try {
    uav_name_ = std::string(std::getenv("DRONE_DEVICE_ID"));
  }
  catch (...) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Environment variable DRONE_DEVICE_ID was not defined!", this->get_name());
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: UAV name is: '%s'", this->get_name(), uav_name_.c_str());

  /* parse params from config file //{ */

  bool callbacks_enabled_ = false;
  parse_param("odometry_source", _estimator_source_param_);
  parse_param("odometry_loop_rate", odometry_loop_rate_);

  /* frame definition */
  world_frame_         = "world";
  fcu_frame_           = uav_name_ + "/fcu";
  ned_fcu_frame_       = uav_name_ + "/ned_fcu";
  ned_origin_frame_    = uav_name_ + "/ned_origin";
  hector_origin_frame_ = uav_name_ + "/hector_origin";
  hector_frame_        = uav_name_ + "/hector_fcu";

  //}

/* initialize altitude estimation //{*/
  RCLCPP_INFO(this->get_logger(), "[Odometry]: Loading altitude estimation parameters");

  /* altitude median filters //{ */

  double buffer_size, max_valid, min_valid, max_diff;

  // We want to gate the measurements before median filtering to prevent the median becoming an invalid value
  min_valid = -1000.0;
  max_valid = 1000.0;

  // Garmin
  param_loader.loadParam("altitude/median_filter/garmin/buffer_size", buffer_size);
  param_loader.loadParam("altitude/median_filter/garmin/max_diff", max_diff);
  alt_mf_garmin_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  //}

  /* altitude measurement min and max value gates //{ */

  param_loader.loadParam("altitude/gate/garmin/min", _garmin_min_valid_alt_);
  param_loader.loadParam("altitude/gate/garmin/max", _garmin_max_valid_alt_);

  //}

  /* excessive tilt //{ */

  double excessive_tilt_tmp;
  param_loader.loadParam("altitude/excessive_tilt", excessive_tilt_tmp);
  _excessive_tilt_sq_ = std::pow(excessive_tilt_tmp, 2);

  //}

  /* altitude measurement covariances (R matrices) //{ */

    alt_R_t temp_matrix, R_alt;
    param_loader.loadMatrixStatic("altitude/R/garmin" temp_matrix);
    R_alt = R_alt.Identity() * temp_matrix(0);
    R_alt_vec_.push_back(R_alt);
    param_loader.loadMatrixStatic("altitude/R/baro" temp_matrix);
    R_alt = R_alt.Identity() * temp_matrix(0);
    R_alt_vec_.push_back(R_alt);

  //}

  /* altitude process covariance (Q matrix) //{ */

  alt_Q_t Q_alt;
  param_loader.loadMatrixStatic("altitude/Q", Q_alt);

  //}

  // initialize altitude estimator
  garmin_alt_estimator_ = std::make_shared<AltitudeEstimator>("garmin", Q_alt, R_alt_vec_);
/*//}*/

/* initialize heading estimation //{*/
  RCLCPP_INFO(this->get_logger(), "[Odometry]: Loading heading estimation parameters");

  /* heading measurement covariances (R matrices) //{ */

    hdg_R_t temp_matrix, R_hdg;
    param_loader.loadMatrixStatic("heading/R/hector" temp_matrix);
    R_hdg = R_hdg.Identity() * temp_matrix(0);
    R_hdg_vec_.push_back(R_hdg);
    param_loader.loadMatrixStatic("heading/R/gyro" temp_matrix);
    R_hdg = R_hdg.Identity() * temp_matrix(0);
    R_hdg_vec_.push_back(R_hdg);

  //}

  /* heading process covariance (Q matrix) //{ */

  hdg_Q_t Q_hdg;
  param_loader.loadMatrixStatic("heading/Q", Q_hdg);

  //}

  // initialize heading estimator
  hector_hdg_estimator_ = std::make_shared<HeadingEstimator>("hector", Q_hdg, R_hdg_vec_);

/*//}*/

  /* initialize lateral estimation //{*/
  RCLCPP_INFO(this->get_logger(), "[Odometry]: Loading lateral estimation parameters");

  /* lateral measurement covariances (R matrices) //{ */

  lat_R_t temp_matrix, R_lat;
  param_loader.loadMatrixStatic("lateral/R/hector" temp_matrix);
  R_lat = R_lat.Identity() * temp_matrix(0);
  R_lat_vec_.push_back(R_lat);

  //}

  /* lateral process covariance (Q matrix) //{ */

  lat_Q_t Q_lat;
  param_loader.loadMatrixStatic("lateral/Q", Q_lat);

  //}
  
  hector_lat_estimator_ = std::make_shared<LateralEstimator>("hector", Q_lat, R_lat_vec_);
  /*//}*/

  estimator_type_names_.push_back(NAME_OF(fog_msgs::msg::EstimatorType::HECTOR));
  estimator_type_names_.push_back(NAME_OF(fog_msgs::msg::EstimatorType::GPS));

  setupEstimator(_estimator_source_param_);

  // publishers
  local_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", 10);

  // subscribers
  timesync_subscriber_ = this->create_subscription<px4_msgs::msg::Timesync>("~/timesync_in", 10, std::bind(&Odometry2::timesyncCallback, this, _1));
  gps_subscriber_      = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>("~/gps_in", 10, std::bind(&Odometry2::gpsCallback, this, _1));
  pixhawk_odom_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", 10, std::bind(&Odometry2::pixhawkOdomCallback, this, _1));
  hector_pose_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>("~/hector_pose_in", 10, std::bind(&Odometry2::hectorPoseCallback, this, _1));
  garmin_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>("~/garmin_in", 10, std::bind(&Odometry2::garminCallback, this, _1));
  baro_subscriber_   = this->create_subscription<px4_msgs::msg::SensorBaro>("~/baro_in", 10, std::bind(&Odometry2::baroCallback, this, _1));

  // service handlers
  change_odometry_source_ =
      this->create_service<fog_msgs::srv::ChangeEstimator>("~/change_odometry_source", std::bind(&Odometry2::callbackChangeEstimator, this, _1, _2));
  getting_odom_service_ = this->create_service<fog_msgs::srv::GetBool>("~/odom_available", std::bind(&Odometry2::getOdomCallback, this, _1, _2));
  get_origin_service_   = this->create_service<fog_msgs::srv::GetOrigin>("~/get_origin", std::bind(&Odometry2::getOriginCallback, this, _1, _2));

  odometry_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / odometry_loop_rate_), std::bind(&Odometry2::odometryRoutine, this), callback_group_);


  tf_broadcaster_        = nullptr;
  static_tf_broadcaster_ = nullptr;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: Initialized", this->get_name());
}
//}

/* timesyncCallback //{ */
void Odometry2::timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }
  timestamp_.store(msg->timestamp);
}
//}

/* gpsCallback //{ */
void Odometry2::gpsCallback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  /* Initialize the global frame position */
  if (!getting_gps_) {
    ref.latitude_deg  = msg->lat;
    ref.longitude_deg = msg->lon;
    getting_gps_      = true;
    RCLCPP_INFO(this->get_logger(), "[%s] Getting GPS!", this->get_name());
  }
}
//}

/* pixhawkOdomCallback //{ */
void Odometry2::pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  {
    /* std::scoped_lock lock(mutex_gps_); */
    pos_gps_[0] = msg->x;
    pos_gps_[1] = msg->y;
    pos_gps_[2] = msg->z;
    ori_gps_[0] = msg->q[0];
    ori_gps_[1] = msg->q[1];
    ori_gps_[2] = msg->q[2];
    ori_gps_[3] = msg->q[3];
  }

  getting_pixhawk_odom_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting pixhawk odometry!", this->get_name());

}
//}

/* hectorOdomCallback //{ */
void Odometry2::hectorPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  {
    /* std::scoped_lock lock(mutex_hector_); */
    pos_hector_[0] = msg->pose.position.x;
    pos_hector_[1] = msg->pose.position.y;
    pos_hector_[2] = 0;  // TODO:: Hector does not provide z. We have to combine the information from pixhawk z fusion of garmin and barometer
    ori_hector_[0] = msg->pose.orientation.w;
    ori_hector_[1] = msg->pose.orientation.x;
    ori_hector_[2] = msg->pose.orientation.y;
    ori_hector_[3] = msg->pose.orientation.z;
  }

  /* TODO: Hector state estimator correction step */

  getting_hector_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting hector poses!", this->get_name());
  
  if (!std::isfinite(msg->pose.position.x)) {
    auto &clk       = *this->get_clock();
    RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 1000, "[Odometry]: not finite value detected in variable \"pose.position.x\" (hectorCallback) !!!");
    return;
  }

  if (!std::isfinite(msg->pose.position.y)) {
    auto &clk       = *this->get_clock();
    RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 1000, "[Odometry]: not finite value detected in variable \"pose.position.y\" (hectorCallback) !!!");
    return;
  }

  // wait for hector convergence to initial position
  if (c_hector_init_msgs_++ < 10) {
    RCLCPP_INFO(this->get_logger(), "[Odometry]: Hector pose #%d - x: %f y: %f", c_hector_init_msgs_, msg->pose.position.x, msg->pose.position.y);
    return;
  }

  /* get heading from hector orientation quaternion //{*/

  double hdg_hector;
  try {
  /* TODO: implement getHeading() */
    hdg_hector = odometry_utils::getHeading(msg->pose.orientation);
    
  // unwrap heading to prevent discrete jumps
  hdg_hector = mrs_lib::geometry::radians::unwrap(hdg_hector, hector_hdg_previous_);
  hector_hdg_previous_ = hdg_hector;
  got_hector_hdg_correction_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[Odometry]: Getting hector heading corrections");

  }
  catch (...) {
    RCLCPP_WARN(this->get_logger(), "[Odometry]: failed to getHeading() from hector orientation, dropping this correction");
  }

  hector_lat_correction_[0] = msg->pose.position.x
  hector_lat_correction_[1] = msg->pose.position.y
  got_hector_lat_correction_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[Odometry]: Getting hector lateral corrections");

/*//}*/

}
//}

/* getOdomCallback //{ */
bool Odometry2::getOdomCallback(const std::shared_ptr<fog_msgs::srv::GetBool::Request> request, std::shared_ptr<fog_msgs::srv::GetBool::Response> response) {
  if (!is_initialized_) {
    return false;
  }
  if (odom_ready_) {
    response->value = true;
    auto &clk       = *this->get_clock();
    RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 1000, "[%s]: getOdometry service responded!", this->get_name());

  } else {
    response->value = false;
    auto &clk       = *this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "[%s]: Waiting for Odometry availability!", this->get_name());
    return false;
  }
  return true;
}
//}

/* getOriginCallback //{ */
bool Odometry2::getOriginCallback(const std::shared_ptr<fog_msgs::srv::GetOrigin::Request> request,
                                  std::shared_ptr<fog_msgs::srv::GetOrigin::Response>      response) {
  if (!is_initialized_) {
    return false;
  }
  if (getting_gps_) {
    response->latitude  = ref.latitude_deg;
    response->longitude = ref.longitude_deg;
    response->success   = true;
    RCLCPP_INFO(this->get_logger(), "[%s]: GPS origin service responded!", this->get_name());
  } else {
    auto &clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "[%s]: GPS origin not set yet!", this->get_name());
    response->success = false;
    return false;
  }
  return true;
}
//}

/* callbackChangeEstimator //{ */
bool Odometry2::callbackChangeEstimator(const std::shared_ptr<fog_msgs::srv::ChangeEstimator::Request>  req,
                                        const std::shared_ptr<fog_msgs::srv::ChangeEstimator::Response> res) {

  if (!is_initialized_) {
    return false;
  }

  if (!callbacks_enabled_) {
    res->success = false;
    res->message = ("Service callbacks are disabled");
    RCLCPP_WARN(this->get_logger(), "[%s]: Ignoring service all. Callbacks are disabled", this->get_name());
    return false;
  }

  // Check whether a valid type was requested
  if (!isValidType(req->estimator_type)) {
    RCLCPP_INFO(this->get_logger(), "test 1");
    RCLCPP_ERROR(this->get_logger(), "[%s]: %d is not a valid odometry type", this->get_name(), req->estimator_type.type);
    res->success = false;
    res->message = ("Not a valid odometry type");
    {
      std::scoped_lock lock(mutex_estimator_source_);
      res->estimator_type.type = estimator_source_.type;
    }
    return true;
  }

  RCLCPP_INFO(this->get_logger(), "test 2");
  // Change the estimator
  bool                         success = false;
  fog_msgs::msg::EstimatorType desired_estimator;
  RCLCPP_INFO(this->get_logger(), "test 3");
  desired_estimator.type = req->estimator_type.type;
  desired_estimator.name = estimator_type_names_[desired_estimator.type];
  success                = changeCurrentEstimator(desired_estimator);
  RCLCPP_INFO(this->get_logger(), "test 4");

  RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->get_name(), printOdometryDiag().c_str());

  res->success = success;
  res->message = (printOdometryDiag().c_str());
  {
    std::scoped_lock lock(mutex_estimator_source_);

    res->estimator_type.type = estimator_source_.type;
  }

  return true;
}
//}

/* baroCallback //{ */
void Odometry2::baroCallback(const px4_msgs::msg::SensorBaro::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }
  getting_baro_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "Getting baro!");

  /* TODO: get AGL altitude from pressure, temperature and takeoff ASL altitude */
  double measurement = getAltitudeFromPressure(msg->pressure, msg->temperature);

  if (!std::isfinite(measurement)) {
    auto &clk       = *this->get_clock();
    RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 1000, "[Odometry]: not finite value detected in variable \"measurement\" (baroCallback) !!!");
    return;
  }

  if (!got_baro_alt_correction_) {
    baro_alt_correction_prev_ = baro_alt_correction_;
    time_baro_prev_ = rclcpp::Time::now();
    got_baro_alt_correction_ = true;
    return;
  }
    // calculate time since last estimators update
    double    dt;
    rclcpp::Time time_now    = rclcpp::Time::now();
    dt                    = (time_now - time_baro_prev_).toSec();
    time_baro_prev_ = time_now;

    if (dt <= 0.0) {
      auto &clk       = *this->get_clock();
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), clk, 1000, "[Odometry]: baro callback dt=%f, discarding correction.", dt);
      return;
    }


  baro_alt_correction_ = (measurement - baro_alt_correction_prev_) / dt;
  baro_alt_correction_prev_ = measurement;

  RCLCPP_INFO_ONCE(this->get_logger(), "[Odometry]: Getting barometric altitude corrections");
}
//}

/* garminCallback //{ */
void Odometry2::garminCallback(const sensor_msgs::msg::Range::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }
  getting_garmin_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting garmin!", this->get_name());

  /* TODO: Correction step for altitude estimator */

  double measurement = msg->range;
  if (!std::isfinite(measurement)) {
      auto &clk       = *this->get_clock();
    RCLCPP_ERROR_THROTTLE(this->get_logger(), clk, 1000, "[Odometry]: not finite value detected in variable \"measurement\" (garminCallback) !!!");
    return;
  }

  // do not fuse garmin measurements when a height jump is detected - most likely the UAV is flying above an obstacle
  if (isUavFlying()) {
    if (!alt_mf_garmin_->isValid(measurement)) {
      auto &clk       = *this->get_clock();
      RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "[Odometry]: Garmin measurement %f declined by median filter.", measurement);
      return;
    }
  }

  garmin_alt_correction_ = measurement;
  got_garmin_alt_correction_ = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "[Odometry]: Getting Garmin altitude corrections");
}
//}

/* odometryRoutine //{ */
void Odometry2::odometryRoutine(void) {
  if (is_initialized_ && getting_pixhawk_odom_ && getting_gps_) {  // && getting_hector_) {
    callbacks_enabled_ = true;
    odom_ready_        = true;
    if (static_tf_broadcaster_ == nullptr) {
      static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
      publishStaticTF();
      return;
    }

    updateEstimators();

    publishTF();
    publishLocalOdom();
  }
}
//}

/* publishStaticTF //{ */
void Odometry2::publishStaticTF() {

  geometry_msgs::msg::TransformStamped tf_stamped;
  tf2::Quaternion                      q;

  /* q.setRPY(-M_PI, 0, 0); */
  /* tf_stamped.header.frame_id         = "local_odom"; */
  /* tf_stamped.child_frame_id          = "fcu"; */
  /* tf_stamped.transform.translation.x = 0.0; */
  /* tf_stamped.transform.translation.y = 0.0; */
  /* tf_stamped.transform.translation.z = 0.0; */
  /* tf_stamped.transform.rotation.x    = q.getX(); */
  /* tf_stamped.transform.rotation.y    = q.getY(); */
  /* tf_stamped.transform.rotation.z    = q.getZ(); */
  /* tf_stamped.transform.rotation.w    = q.getW(); */
  /* static_tf_broadcaster_->sendTransform(tf_stamped); */

  q.setRPY(M_PI, 0, M_PI / 2);
  q                                  = q.inverse();
  tf_stamped.header.frame_id         = world_frame_;
  tf_stamped.child_frame_id          = ned_origin_frame_;
  tf_stamped.transform.translation.x = 0.0;
  tf_stamped.transform.translation.y = 0.0;
  tf_stamped.transform.translation.z = 0.0;
  tf_stamped.transform.rotation.x    = q.getX();
  tf_stamped.transform.rotation.y    = q.getY();
  tf_stamped.transform.rotation.z    = q.getZ();
  tf_stamped.transform.rotation.w    = q.getW();
  static_tf_broadcaster_->sendTransform(tf_stamped);
}
//}

/* publishTF //{ */
void Odometry2::publishTF() {
  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }
  geometry_msgs::msg::TransformStamped tf1;
  tf2::Quaternion                      q;

  {
    std::scoped_lock lock(mutex_estimator_source_);

    /* GPS //{ */
    if (estimator_source_.type == fog_msgs::msg::EstimatorType::GPS) {
      /* std::scoped_lock lock(mutex_gps_); */

      tf1.header.stamp            = this->get_clock()->now();
      tf1.header.frame_id         = ned_origin_frame_;
      tf1.child_frame_id          = ned_fcu_frame_;
      tf1.transform.translation.x = pos_gps_[0];
      tf1.transform.translation.y = pos_gps_[1];
      tf1.transform.translation.z = pos_gps_[2];
      tf1.transform.rotation.w    = ori_gps_[0];
      tf1.transform.rotation.x    = ori_gps_[1];
      tf1.transform.rotation.y    = ori_gps_[2];
      tf1.transform.rotation.z    = ori_gps_[3];
      tf_broadcaster_->sendTransform(tf1);

      tf1.header.stamp            = this->get_clock()->now();
      tf1.header.frame_id         = ned_fcu_frame_;
      tf1.child_frame_id          = fcu_frame_;
      tf1.transform.translation.x = 0;
      tf1.transform.translation.y = 0;
      tf1.transform.translation.z = 0;
      q.setRPY(-M_PI, 0, 0);
      tf1.transform.rotation.w = q.getW();
      tf1.transform.rotation.x = q.getX();
      tf1.transform.rotation.y = q.getY();
      tf1.transform.rotation.z = q.getZ();
      tf_broadcaster_->sendTransform(tf1);
      return;
    }
    //}

    /* HECTOR //{ */
    if (estimator_source_.type == fog_msgs::msg::EstimatorType::HECTOR) {
      /* std::scoped_lock lock(mutex_hector_); */

      geometry_msgs::msg::TransformStamped tf1;
      tf1.header.stamp            = this->get_clock()->now();
      tf1.header.frame_id         = hector_origin_frame_;
      tf1.child_frame_id          = hector_frame_;
      tf1.transform.translation.x = pos_hector_[0];
      tf1.transform.translation.y = pos_hector_[1];
      tf1.transform.translation.z = pos_hector_[2];
      tf1.transform.rotation.w    = ori_hector_[0];
      tf1.transform.rotation.x    = ori_hector_[1];
      tf1.transform.rotation.y    = ori_hector_[2];
      tf1.transform.rotation.z    = ori_hector_[3];
      tf_broadcaster_->sendTransform(tf1);

      tf1.header.stamp            = this->get_clock()->now();
      tf1.header.frame_id         = hector_frame_;
      tf1.child_frame_id          = fcu_frame_;
      tf1.transform.translation.x = 0;
      tf1.transform.translation.y = 0;
      tf1.transform.translation.z = 0;
      q.setRPY(0, 0, 0);
      tf1.transform.rotation.w = q.getW();
      tf1.transform.rotation.x = q.getX();
      tf1.transform.rotation.y = q.getY();
      tf1.transform.rotation.z = q.getZ();
      tf_broadcaster_->sendTransform(tf1);
      return;
    }
    //}

    RCLCPP_ERROR(this->get_logger(), "[%s]: '%s' is not a valid source of odometry", this->get_name(), estimator_source_.name.c_str());
  }
}
//}

/* publishLocalOdom //{ */
void Odometry2::publishLocalOdom() {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp    = this->get_clock()->now();
  msg.header.frame_id = world_frame_;
  msg.child_frame_id  = fcu_frame_;
  auto tf             = transformBetween(fcu_frame_, world_frame_);
  /* auto tf                     = transformBetween(world_frame_, fcu_frame_); */
  msg.pose.pose.position.x    = tf.pose.position.x;
  msg.pose.pose.position.y    = tf.pose.position.y;
  msg.pose.pose.position.z    = tf.pose.position.z;
  msg.pose.pose.orientation.w = tf.pose.orientation.w;
  msg.pose.pose.orientation.x = tf.pose.orientation.x;
  msg.pose.pose.orientation.y = tf.pose.orientation.y;
  msg.pose.pose.orientation.z = tf.pose.orientation.z;
  local_odom_publisher_->publish(msg);
}
//}

/* transformBetween //{ */
geometry_msgs::msg::PoseStamped Odometry2::transformBetween(std::string frame_from, std::string frame_to) {
  geometry_msgs::msg::PoseStamped pose_out;
  try {
    auto transform_stamped      = tf_buffer_->lookupTransform(frame_to, frame_from, rclcpp::Time(0));
    pose_out.pose.position.x    = transform_stamped.transform.translation.x;
    pose_out.pose.position.y    = transform_stamped.transform.translation.y;
    pose_out.pose.position.z    = transform_stamped.transform.translation.z;
    pose_out.pose.orientation.w = transform_stamped.transform.rotation.w;
    pose_out.pose.orientation.x = transform_stamped.transform.rotation.x;
    pose_out.pose.orientation.y = transform_stamped.transform.rotation.y;
    pose_out.pose.orientation.z = transform_stamped.transform.rotation.z;
  }
  catch (...) {
  }
  return pose_out;
}
//}

/* generateColor//{ */
std_msgs::msg::ColorRGBA Odometry2::generateColor(const double r, const double g, const double b, const double a) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}
//}

/* //{ isValidType */
bool Odometry2::isValidType(const fog_msgs::msg::EstimatorType &type) {

  if (type.type == fog_msgs::msg::EstimatorType::GPS || type.type == fog_msgs::msg::EstimatorType::HECTOR) {
    return true;
  }

  return false;
}

//}

/* //{ changeCurrentEstimator */
bool Odometry2::changeCurrentEstimator(const fog_msgs::msg::EstimatorType &desired_estimator) {

  fog_msgs::msg::EstimatorType target_estimator = desired_estimator;
  target_estimator.name                         = estimator_type_names_[target_estimator.type];

  RCLCPP_INFO(this->get_logger(), "test 5");
  // Return if already active
  {
    std::scoped_lock lock(mutex_estimator_source_);
    if (toUppercase(estimator_source_.name) == toUppercase(target_estimator.name)) {
      RCLCPP_INFO(this->get_logger(), "[%s]: Desired estimator '%s' already active. Not switching.", this->get_name(), target_estimator.name.c_str());
      return true;
    }
  }
  RCLCPP_INFO(this->get_logger(), "test 6");
  // Check odometry source availability
  if (target_estimator.type == fog_msgs::msg::EstimatorType::GPS) {

    if (!getting_pixhawk_odom_) {
      RCLCPP_ERROR(this->get_logger(), "[%s]: Cannot transition to GPS type. GPS source is not active.", this->get_name());
      return false;
    }
  } else if (target_estimator.type == fog_msgs::msg::EstimatorType::HECTOR) {

    if (!getting_hector_) {
      RCLCPP_ERROR(this->get_logger(), "[%s]: Cannot transition to HECTOR type. HECTOR source is not active.", this->get_name());
      return false;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Rejected transition to invalid types %s", this->get_name(), target_estimator.name.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "test 7");

  {
    std::scoped_lock lock(mutex_estimator_source_);
    estimator_source_      = target_estimator;
    estimator_source_.name = _estimator_source_param_[estimator_source_.type];
    RCLCPP_WARN(this->get_logger(), "[%s]: Transition to %s state estimator successful", this->get_name(), estimator_source_.name.c_str());
  }
  return true;
}


//}

/* //{ setupEstimator */
void Odometry2::setupEstimator(const std::string &type) {

  std::scoped_lock lock(mutex_estimator_source_);
  if (type == "GPS") {
    RCLCPP_INFO(this->get_logger(), "[%s]: Settting up odometry source '%s'", this->get_name(), type.c_str());
    estimator_source_.type = fog_msgs::msg::EstimatorType::GPS;
    estimator_source_.name = estimator_type_names_[estimator_source_.type];

  } else if (type == "HECTOR") {
    RCLCPP_INFO(this->get_logger(), "[%s]: Settting up odometry source '%s'", this->get_name(), type.c_str());
    estimator_source_.type = fog_msgs::msg::EstimatorType::HECTOR;
    estimator_source_.name = estimator_type_names_[estimator_source_.type];

  } else {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Could not find a odometry source: '%s'", this->get_name(), type);
  }
  return;
}

//}

/* updateEstimators //{*/
void Odometry2::updateEstimators() {

    // calculate time since last estimators update
    double    dt;
    rclcpp::Time time_now    = rclcpp::Time::now();
    dt                    = (time_now - time_odometry_timer_prev_).toSec();
    time_odometry_timer_prev_ = time_now;

    if (dt <= 0.0) {
      auto &clk       = *this->get_clock();
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), clk, 1000, "[Odometry]: odometry timer dt=%f, skipping estimator update.", dt);
      return;
    }

    /* altitude estimator update and predict //{*/

    if (got_garmin_alt_correction_) {
      garmin_alt_estimator_->doCorrection(garmin_alt_correction_, ALT_GARMIN);
    }

    if (got_baro_alt_correction_) {
      garmin_alt_estimator_->doCorrection(baro_alt_correction_, ALT_BARO);
    }

    garmin_alt_estimator_->doPrediction(0, dt);

    /*//}*/

    /* heading estimator update and predict //{*/

    if (got_hector_hdg_correction_) {
      hector_hdg_estimator_->doCorrection(hector_hdg_correction_, HDG_HECTOR);
    }

    if (got_gyro_hdg_correction_) {
      hector_hdg_estimator_->doCorrection(hector_hdg_correction_, HDG_GYRO);
    }

    hector_hdg_estimator_->doPrediction(0, dt);

    if (got_hector_lat_correction_) {
      hector_lat_estimator_->doCorrection(hector_lat_correction_[0], hector_lat_correction_[1], LAT_HECTOR);
    }

    hector_lat_estimator_->doPrediction(0, dt);

    /*//}*/

}

/*//}*/

/* //{ printOdometryDiag */
std::string Odometry2::printOdometryDiag() {

  std::string s_diag;

  fog_msgs::msg::EstimatorType type;

  {
    std::scoped_lock lock(mutex_estimator_source_);

    type.type = estimator_source_.type;
  }

  s_diag += "Current estimator type: ";
  s_diag += std::to_string(type.type);

  if (type.type == fog_msgs::msg::EstimatorType::GPS) {
    s_diag += "GPS";
  } else if (type.type == fog_msgs::msg::EstimatorType::HECTOR) {
    s_diag += "HECTOR";
  } else {
    s_diag += "UNKNOWN";
  }

  return s_diag;
}

//}

/* toUppercase //{ */

std::string Odometry2::toUppercase(const std::string &str_in) {
  std::string str_out = str_in;
  std::transform(str_out.begin(), str_out.end(), str_out.begin(), ::toupper);
  return str_out;
}

//}

/* parse_param //{ */
template <class T>
bool Odometry2::parse_param(std::string param_name, T &param_dest) {
  const std::string param_path = "param_namespace." + param_name;
  this->declare_parameter(param_path);
  if (!this->get_parameter(param_path, param_dest)) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Could not load param '%s'", this->get_name(), param_name.c_str());
    return false;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << this->get_name() << "]: Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}
//}

/* parse_param impl //{ */
template bool Odometry2::parse_param<int>(std::string param_name, int &param_dest);
template bool Odometry2::parse_param<double>(std::string param_name, double &param_dest);
template bool Odometry2::parse_param<float>(std::string param_name, float &param_dest);
template bool Odometry2::parse_param<std::string>(std::string param_name, std::string &param_dest);
template bool Odometry2::parse_param<unsigned int>(std::string param_name, unsigned int &param_dest);
//}

}  // namespace odometry2


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(odometry2::Odometry2)
