#include <atomic>
#include <chrono>
#include <cstdint>
#include <tuple>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <mutex>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/subscription.hpp>
#include <std_srvs/srv/detail/set_bool__struct.hpp>
#include <std_srvs/srv/detail/trigger__struct.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fog_msgs/srv/get_origin.hpp>
#include <fog_msgs/srv/get_bool.hpp>
#include <fog_msgs/srv/vec4.hpp>
#include <fog_msgs/srv/get_px4_param_int.hpp>
#include <fog_msgs/srv/set_px4_param_int.hpp>
#include <fog_msgs/srv/get_px4_param_float.hpp>
#include <fog_msgs/srv/set_px4_param_float.hpp>
#include <nav_msgs/msg/path.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/sensor_baro.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

#include "geodesy/utm.h"
#include "geodesy/wgs84.h"

#include <mavsdk/geometry.h>

#include <eigen3/Eigen/Core>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lib/median_filter.h>
#include <lib/geometry/cyclic.h>

#include "types.h"
#include "odometry_utils.h"
#include "altitude_estimator.h"
#include "heading_estimator.h"
#include "lateral_estimator.h"

/* defines //{ */

#define NAME_OF(v) #v

//}

typedef std::tuple<std::string, int>   px4_int;
typedef std::tuple<std::string, float> px4_float;

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
  std::atomic_bool publishing_static_tf_ = false;
  std::atomic_bool publishing_odometry_  = false;

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

  // PX4 PARAMS
  std::vector<px4_int>   px4_params_int;
  std::vector<px4_float> px4_params_float;
  std::atomic_bool       set_initial_px4_params_ = false;

  // GPS
  bool             gps_use_ = false;
  float            pos_gps_[3];
  float            ori_gps_[4];
  int              c_gps_init_msgs_ = 0;
  float            gps_eph_max_     = 0;
  int              gps_msg_good_    = 0;
  int              gps_msg_err_     = 0;
  int              c_gps_eph_err_   = 0;
  int              c_gps_eph_good_  = 0;
  std::atomic_bool gps_reliable_    = false;

  // HECTOR
  bool             hector_use_ = false;
  float            pos_hector_[3];
  std::mutex       mutex_hector_raw_;
  float            pos_hector_raw_prev_[2];
  float            pos_hector_raw_[2];
  float            ori_hector_[4];
  float            pos_orig_hector_[3];
  float            ori_orig_hector_[4];
  int              c_hector_init_msgs_       = 0;
  int              hector_num_init_msgs_     = 10;
  float            hector_msg_interval_max_  = 0.0;
  float            hector_msg_interval_warn_ = 0.0;
  double           hector_hdg_previous_      = 0.0;
  float            hector_reset_wait_        = 0.0;
  float            hector_fusion_wait_       = 0.0;
  float            hector_max_position_jump_ = 0.0;
  float            hector_max_velocity_      = 0.0;
  std::atomic_bool hector_reliable_          = true;
  std::atomic_bool hector_tf_setup_          = false;
  std::atomic_bool hector_reset_called_      = false;

  std::chrono::time_point<std::chrono::system_clock> time_hector_last_msg_;
  std::chrono::time_point<std::chrono::system_clock> hector_reset_called_time_ = std::chrono::system_clock::now();

  // VISION SENSOR
  px4_msgs::msg::VehicleVisualOdometry               visual_odometry_;
  std::atomic<unsigned long long>                    timestamp_;
  std::atomic<std::int64_t>                          timestamp_raw_;
  std::chrono::time_point<std::chrono::system_clock> time_sync_time_;


  // Vehicle local position
  std::atomic<float> pos_local_[3];
  std::atomic<float> ori_local_[4];
  /* std::mutex mutex_local_; */
  tf2::Quaternion mavros_orientation_;
  std::mutex      mutex_mavros_orientation_;

  // Altitude estimation
  std::shared_ptr<AltitudeEstimator> garmin_alt_estimator_;
  std::vector<alt_R_t>               R_alt_vec_;

  std::unique_ptr<MedianFilter> alt_mf_garmin_;
  double                        _garmin_min_valid_alt_;
  double                        _garmin_max_valid_alt_;
  double                        _excessive_tilt_sq_;
  std::atomic<double>           garmin_alt_correction_;
  std::atomic_bool              got_garmin_alt_correction_ = false;

  std::atomic<double> baro_alt_correction_;
  std::atomic<double> baro_alt_measurement_prev_;
  std::atomic_bool    got_baro_alt_correction_ = false;
  rclcpp::Time        time_baro_prev_;

  std::vector<float> garmin_buffer_z_;
  std::vector<float> garmin_buffer_vz_;
  int                garmin_buffer_size_;


  // Heading estimation
  std::shared_ptr<HeadingEstimator> hector_hdg_estimator_;
  std::vector<hdg_R_t>              R_hdg_vec_;

  std::atomic<double> hector_hdg_correction_;
  std::atomic_bool    got_hector_hdg_correction_ = false;

  std::atomic_bool got_gyro_hdg_correction_ = false;

  // Lateral estimation
  std::shared_ptr<LateralEstimator> hector_lat_estimator_;
  std::vector<lat_R_t>              R_lat_vec_;

  std::atomic<double> hector_lat_correction_[2];
  std::atomic_bool    got_hector_lat_correction_ = false;

  // Odometry switch
  std::atomic_int  current_ekf_bitmask_    = 0;
  std::atomic_bool get_ekf_bitmask_called_ = false;
  std::atomic_bool set_ekf_bitmask_called_ = false;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_hector_publisher_;
  /* rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr             pixhawk_odom_publisher_; */
  rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr visual_odom_publisher_;
  rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr             gps_sensor_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                reset_hector_publisher_;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr           timesync_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr gps_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr    pixhawk_odom_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr         baro_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   hector_pose_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr           garmin_subscriber_;

  // subscriber callbacks
  void timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg);
  void gpsCallback(const px4_msgs::msg::VehicleGpsPosition::UniquePtr msg);
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void hectorPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg);
  void baroCallback(const px4_msgs::msg::SensorBaro::UniquePtr msg);
  void garminCallback(const sensor_msgs::msg::Range::UniquePtr msg);

  // services provided
  rclcpp::Service<fog_msgs::srv::GetBool>::SharedPtr   getting_odom_service_;
  rclcpp::Service<fog_msgs::srv::GetOrigin>::SharedPtr get_origin_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr   reset_hector_service_;

  // service callbacks
  bool getOdomCallback(const std::shared_ptr<fog_msgs::srv::GetBool::Request> request, std::shared_ptr<fog_msgs::srv::GetBool::Response> response);
  bool getOriginCallback(const std::shared_ptr<fog_msgs::srv::GetOrigin::Request> request, std::shared_ptr<fog_msgs::srv::GetOrigin::Response> response);
  bool resetHectorCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // service clients
  bool getPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::GetPx4ParamInt>::SharedFuture future);
  bool setPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedFuture future);
  bool setPx4FloatParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedFuture future);
  bool resetHectorClientCallback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  // service clients
  rclcpp::Client<fog_msgs::srv::GetPx4ParamInt>::SharedPtr   get_px4_param_int_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr          land_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr          reset_hector_client_;

  // internal functions
  bool        isValidGate(const double value, const double min_value, const double max_value, const std::string &value_name);
  bool        setInitialPx4Params();
  std::string printOdometryDiag();
  std::string toUppercase(const std::string &type);

  void publishTF();
  void publishStaticTF();
  void publishLocalOdom();
  void updateEstimators();
  void publishOdometry();
  void checkHectorReliability();
  void resetHector();
  void pixhawkEkfUpdate();

  geometry_msgs::msg::PoseStamped transformBetween(std::string frame_from, std::string frame_to);
  std_msgs::msg::ColorRGBA        generateColor(const double r, const double g, const double b, const double a);

  // timers
  rclcpp::CallbackGroup::SharedPtr                   callback_group_;
  rclcpp::TimerBase::SharedPtr                       odometry_timer_;
  void                                               odometryRoutine(void);
  std::chrono::time_point<std::chrono::system_clock> time_odometry_timer_prev_;
  std::atomic_bool                                   time_odometry_timer_set_ = false;

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

  /* parse general params from config file //{ */

  bool callbacks_enabled_ = false;
  parse_param("odometry_loop_rate", odometry_loop_rate_);
  parse_param("gps.use_gps", gps_use_);
  parse_param("gps.eph_max", gps_eph_max_);
  parse_param("gps.msg_good", gps_msg_good_);
  parse_param("gps.msg_err", gps_msg_err_);
  parse_param("hector.use_hector", hector_use_);
  parse_param("hector.num_init_msgs", hector_num_init_msgs_);
  parse_param("hector.msg_interval_max", hector_msg_interval_max_);
  parse_param("hector.msg_interval_warn", hector_msg_interval_warn_);
  parse_param("hector.reset_wait", hector_reset_wait_);
  parse_param("hector.fusion_wait", hector_fusion_wait_);
  parse_param("hector.max_velocity", hector_max_velocity_);
  parse_param("hector.max_position_jump", hector_max_position_jump_);

  parse_param("hector.garmin_msg_delay", garmin_buffer_size_);

  int   param_int;
  float param_float;
  parse_param("px4.EKF2_AID_MASK", param_int);
  px4_params_int.push_back(px4_int("EKF2_AID_MASK", param_int));
  parse_param("px4.EKF2_EV_NOISE_MD", param_int);
  px4_params_int.push_back(px4_int("EKF2_EV_NOISE_MD", param_int));
  parse_param("px4.EKF2_RNG_AID", param_int);
  px4_params_int.push_back(px4_int("EKF2_RNG_AID", param_int));
  parse_param("px4.EKF2_HGT_MODE", param_int);
  px4_params_int.push_back(px4_int("EKF2_HGT_MODE", param_int));

  parse_param("px4.EKF2_EV_DELAY", param_float);
  px4_params_float.push_back(px4_float("EKF2_EV_DELAY", param_float));
  parse_param("px4.EKF2_EVP_NOISE", param_float);
  px4_params_float.push_back(px4_float("EKF2_EVP_NOISE", param_float));
  parse_param("px4.EKF2_EVV_NOISE", param_float);
  px4_params_float.push_back(px4_float("EKF2_EVV_NOISE", param_float));
  parse_param("px4.EKF2_RNG_A_HMAX", param_float);
  px4_params_float.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));
  parse_param("px4.MPC_XY_CRUISE", param_float);
  px4_params_float.push_back(px4_float("MPC_XY_CRUISE", param_float));
  parse_param("px4.MC_YAWRATE_MAX", param_float);
  px4_params_float.push_back(px4_float("MC_YAWRATE_MAX", param_float));
  parse_param("px4.MPC_ACC_HOR", param_float);
  px4_params_float.push_back(px4_float("MPC_ACC_HOR", param_float));
  parse_param("px4.MPC_ACC_DOWN_MAX", param_float);
  px4_params_float.push_back(px4_float("MPC_ACC_DOWN_MAX", param_float));
  parse_param("px4.MPC_ACC_UP_MAX", param_float);
  px4_params_float.push_back(px4_float("MPC_ACC_UP_MAX", param_float));

  /* frame definition */
  world_frame_         = "world";
  fcu_frame_           = uav_name_ + "/fcu";
  ned_fcu_frame_       = uav_name_ + "/ned_fcu";
  ned_origin_frame_    = uav_name_ + "/ned_origin";
  hector_origin_frame_ = uav_name_ + "/hector_origin";
  hector_frame_        = uav_name_ + "/hector_fcu";

  //}

  /* initialize altitude estimation //{*/
  RCLCPP_INFO(this->get_logger(), "[%s]: Loading altitude estimation parameters", this->get_name());

  /* altitude median filters //{ */

  double buffer_size, max_valid, min_valid, max_diff;

  // We want to gate the measurements before median filtering to prevent the median becoming an invalid value
  min_valid = -1000.0;
  max_valid = 1000.0;

  // Garmin
  parse_param("altitude.median_filter.garmin.buffer_size", buffer_size);
  parse_param("altitude.median_filter.garmin.max_diff", max_diff);
  /* alt_mf_garmin_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff,
   * rclcpp::NodeOptions(rclcpp::contexts::get_global_default_context)); */

  //}

  /* altitude measurement min and max value gates //{ */

  parse_param("altitude.gate.garmin.min", _garmin_min_valid_alt_);
  parse_param("altitude.gate.garmin.max", _garmin_max_valid_alt_);

  //}

  /* excessive tilt //{ */

  /* TODO: not used yet */
  double excessive_tilt_tmp;
  parse_param("altitude.excessive_tilt", excessive_tilt_tmp);
  _excessive_tilt_sq_ = std::pow(excessive_tilt_tmp, 2);

  //}

  /* altitude measurement covariances (R matrices) //{ */

  alt_R_t R_alt;
  double  R_alt_tmp;
  parse_param("altitude.measurement_covariance.garmin", R_alt_tmp);
  R_alt = R_alt.Identity() * R_alt_tmp;
  R_alt_vec_.push_back(R_alt);
  parse_param("altitude.measurement_covariance.baro", R_alt_tmp);
  R_alt = R_alt.Identity() * R_alt_tmp;
  R_alt_vec_.push_back(R_alt);

  //}

  /* altitude process covariance (Q matrix) //{ */

  alt_Q_t Q_alt = alt_Q_t::Identity();
  double  alt_pos, alt_vel, alt_acc;
  parse_param("altitude.process_covariance.pos", alt_pos);
  Q_alt(STATE_POS, STATE_POS) *= alt_pos;
  parse_param("altitude.process_covariance.vel", alt_vel);
  Q_alt(STATE_VEL, STATE_VEL) *= alt_vel;
  parse_param("altitude.process_covariance.acc", alt_acc);
  Q_alt(STATE_ACC, STATE_ACC) *= alt_acc;

  //}

  // initialize altitude estimator
  garmin_alt_estimator_ = std::make_shared<AltitudeEstimator>("garmin", Q_alt, R_alt_vec_);
  /*//}*/

  /* initialize heading estimation //{*/
  RCLCPP_INFO(this->get_logger(), "[%s]: Loading heading estimation parameters", this->get_name());

  /* heading measurement covariances (R matrices) //{ */

  hdg_R_t R_hdg;
  double  R_hdg_tmp;
  parse_param("heading.measurement_covariance.hector", R_hdg_tmp);
  R_hdg = R_hdg.Identity() * R_hdg_tmp;
  R_hdg_vec_.push_back(R_hdg);
  parse_param("heading.measurement_covariance.gyro", R_hdg_tmp);
  R_hdg = R_hdg.Identity() * R_hdg_tmp;
  R_hdg_vec_.push_back(R_hdg);

  //}

  /* heading process covariance (Q matrix) //{ */

  hdg_Q_t Q_hdg = hdg_Q_t::Identity();
  double  hdg_pos, hdg_vel, hdg_acc;
  parse_param("heading.process_covariance.pos", hdg_pos);
  Q_hdg(STATE_POS, STATE_POS) *= hdg_pos;
  parse_param("heading.process_covariance.vel", hdg_vel);
  Q_hdg(STATE_VEL, STATE_VEL) *= hdg_vel;
  parse_param("heading.process_covariance.acc", hdg_acc);
  Q_hdg(STATE_ACC, STATE_ACC) *= hdg_acc;

  //}

  // initialize heading estimator
  hector_hdg_estimator_ = std::make_shared<HeadingEstimator>("hector", Q_hdg, R_hdg_vec_);

  /*//}*/

  /* initialize lateral estimation //{*/
  RCLCPP_INFO(this->get_logger(), "[%s]: Loading lateral estimation parameters", this->get_name());

  /* lateral measurement covariances (R matrices) //{ */

  lat_R_t R_lat;
  double  R_lat_tmp;
  parse_param("lateral.measurement_covariance.hector", R_lat_tmp);
  R_lat = R_lat.Identity() * R_lat_tmp;
  R_lat_vec_.push_back(R_lat);

  //}

  /* lateral process covariance (Q matrix) //{ */

  lat_Q_t Q_lat = lat_Q_t::Identity();
  double  lat_pos, lat_vel, lat_acc;
  parse_param("lateral.process_covariance.pos", lat_pos);
  Q_lat(LAT_POS_X, LAT_POS_X) *= lat_pos;
  Q_lat(LAT_POS_Y, LAT_POS_Y) *= lat_pos;
  parse_param("lateral.process_covariance.vel", lat_vel);
  Q_lat(LAT_VEL_X, LAT_VEL_X) *= lat_vel;
  Q_lat(LAT_VEL_Y, LAT_VEL_Y) *= lat_vel;
  parse_param("lateral.process_covariance.acc", lat_acc);
  Q_lat(LAT_ACC_X, LAT_ACC_X) *= lat_acc;
  Q_lat(LAT_ACC_Y, LAT_ACC_Y) *= lat_acc;

  //}

  hector_lat_estimator_ = std::make_shared<LateralEstimator>("hector", Q_lat, R_lat_vec_);
  /*//}*/

  // publishers
  local_odom_publisher_   = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", 10);
  local_hector_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("~/local_hector_out", 10);
  /* pixhawk_odom_publisher_ = this->create_publisher<px4_msgs::msg::SensorGps>("~/pixhawk_odom_out", 10); */
  visual_odom_publisher_  = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("~/visual_odom_out", 10);
  gps_sensor_publisher_   = this->create_publisher<px4_msgs::msg::SensorGps>("~/sensor_gps_out", 10);
  reset_hector_publisher_ = this->create_publisher<std_msgs::msg::String>("~/reset_hector_out", 10);

  // subscribers
  timesync_subscriber_ =
      this->create_subscription<px4_msgs::msg::Timesync>("~/timesync_in", rclcpp::SystemDefaultsQoS(), std::bind(&Odometry2::timesyncCallback, this, _1));
  gps_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleGpsPosition>("~/gps_in", rclcpp::SystemDefaultsQoS(), std::bind(&Odometry2::gpsCallback, this, _1));
  pixhawk_odom_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&Odometry2::pixhawkOdomCallback, this, _1));
  hector_pose_subscriber_  = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/hector_pose_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&Odometry2::hectorPoseCallback, this, _1));
  garmin_subscriber_ =
      this->create_subscription<sensor_msgs::msg::Range>("~/garmin_in", rclcpp::SystemDefaultsQoS(), std::bind(&Odometry2::garminCallback, this, _1));
  baro_subscriber_ =
      this->create_subscription<px4_msgs::msg::SensorBaro>("~/baro_in", rclcpp::SystemDefaultsQoS(), std::bind(&Odometry2::baroCallback, this, _1));

  // service handlers
  getting_odom_service_ = this->create_service<fog_msgs::srv::GetBool>("~/getting_odom", std::bind(&Odometry2::getOdomCallback, this, _1, _2));
  get_origin_service_   = this->create_service<fog_msgs::srv::GetOrigin>("~/get_origin", std::bind(&Odometry2::getOriginCallback, this, _1, _2));
  reset_hector_service_ = this->create_service<std_srvs::srv::Trigger>("~/reset_hector_service_in", std::bind(&Odometry2::resetHectorCallback, this, _1, _2));

  // service clients
  get_px4_param_int_   = this->create_client<fog_msgs::srv::GetPx4ParamInt>("~/get_px4_param_int");
  set_px4_param_int_   = this->create_client<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int");
  set_px4_param_float_ = this->create_client<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float");
  land_service_        = this->create_client<std_srvs::srv::SetBool>("~/land_out");
  reset_hector_client_ = this->create_client<std_srvs::srv::Trigger>("~/reset_hector_service_out");

  callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
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
  if (msg->sys_id == 1) {
    timestamp_.store(msg->timestamp);
    timestamp_raw_.store(msg->tc1);
    time_sync_time_ = std::chrono::system_clock::now();
  }
}
//}

/* gpsCallback //{ */
void Odometry2::gpsCallback(const px4_msgs::msg::VehicleGpsPosition::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  // wait for gps convergence after bootup
  if (c_gps_init_msgs_++ < 10) {
    RCLCPP_INFO(this->get_logger(), "[%s]: GPS pose #%d - lat: %f lon: %f", this->get_name(), c_gps_init_msgs_, msg->lat, msg->lon);
    c_gps_init_msgs_++;
    return;
  }

  /* Initialize the global frame position */
  if (!getting_gps_) {
    ref.latitude_deg  = msg->lat * 1e-7;
    ref.longitude_deg = msg->lon * 1e-7;

    getting_gps_ = true;
    RCLCPP_INFO(this->get_logger(), "[%s] Getting GPS!", this->get_name());
    RCLCPP_INFO(this->get_logger(), "[%s] GPS origin - lat: %f ; lon: %f", this->get_name(), ref.latitude_deg, ref.longitude_deg);
  }

  // GPS quality is getting lower
  if (msg->eph > gps_eph_max_) {

    if (gps_reliable_) {

      RCLCPP_WARN(this->get_logger(), "[%s] GPS quality is too low! EPH value: %f, msg num#%d", this->get_name(), msg->eph, c_gps_eph_err_);

      if (c_gps_eph_err_ >= gps_msg_err_) {

        RCLCPP_WARN(this->get_logger(), "[%s] Turning off GPS usage!", this->get_name(), msg->eph);
        gps_reliable_  = false;
        c_gps_eph_err_ = 0;

      } else {
        c_gps_eph_err_++;
      }
    }
    // GPS is getting better
  } else if (!gps_reliable_) {

    RCLCPP_WARN(this->get_logger(), "[%s] GPS quality is improving! EPH value: %f, msg num #%d", this->get_name(), msg->eph, c_gps_eph_good_);

    if (c_gps_eph_good_ >= gps_msg_good_) {

      RCLCPP_WARN(this->get_logger(), "[%s] Turning on GPS usage!", this->get_name(), msg->eph);
      gps_reliable_   = true;
      c_gps_eph_good_ = 0;

    } else {
      c_gps_eph_good_++;
    }
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

    std::scoped_lock lock(mutex_mavros_orientation_);

    mavros_orientation_.setW(msg->q[0]);
    mavros_orientation_.setX(msg->q[1]);
    mavros_orientation_.setY(msg->q[2]);
    mavros_orientation_.setZ(msg->q[3]);
  }

  getting_pixhawk_odom_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting pixhawk odometry!", this->get_name());
}
//}

/* hectorOdomCallback //{ */
void Odometry2::hectorPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg) {

  // If not initialized, or hector not used
  if (!is_initialized_) {  //|| !hector_use_) {
    return;
  }

  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting hector poses!", this->get_name());

  time_hector_last_msg_ = std::chrono::system_clock::now();

  if (!std::isfinite(msg->pose.position.x)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: not finite value detected in variable \"pose.position.x\" (hectorcallback) !!!",
                          this->get_name());
    return;
  }

  if (!std::isfinite(msg->pose.position.y)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: not finite value detected in variable \"pose.position.y\" (hectorCallback) !!!",
                          this->get_name());
    return;
  }

  /* RCLCPP_INFO(this->get_logger(), "[%s]: Hector pose - x: %f y: %f", this->get_name(), msg->pose.position.x, msg->pose.position.y); */

  /*When receive the 0,0 pose, reset all requires states//{*/
  if (hector_reset_called_ && msg->pose.position.x == 0 && msg->pose.position.x == 0) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Hector reinitalizied!", this->get_name());

    // Reset HECTOR heading
    hector_hdg_estimator_->setState(0, 0);

    // Reset HECTOR position
    hector_lat_estimator_->setState(0, 0);
    hector_lat_estimator_->setState(1, 0);
    hector_lat_estimator_->setState(2, 0);
    hector_lat_estimator_->setState(3, 0);
    hector_lat_estimator_->setState(4, 0);
    hector_lat_estimator_->setState(5, 0);

    RCLCPP_WARN(this->get_logger(), "[Odometry2]: Hector estimators reset");

    {
      std::scoped_lock lock(mutex_hector_raw_);
      pos_hector_raw_[0]      = 0;
      pos_hector_raw_[1]      = 0;
      pos_hector_raw_prev_[0] = 0;
      pos_hector_raw_prev_[1] = 0;
    }

    hector_lat_correction_[0] = 0;
    hector_lat_correction_[1] = 0;

    c_hector_init_msgs_       = 0;
    hector_reset_called_      = false;
    hector_tf_setup_          = false;
    hector_reset_called_time_ = std::chrono::system_clock::now();
    hector_reliable_          = true;
    getting_hector_           = false;

    // Reset estimator time update
    time_odometry_timer_set_ = false;
  }
  /*//}*/

  // wait for hector convergence to initial position
  if (c_hector_init_msgs_++ < hector_num_init_msgs_) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Hector pose #%d - x: %f y: %f", this->get_name(), c_hector_init_msgs_, msg->pose.position.x, msg->pose.position.y);
    std::scoped_lock lock(mutex_hector_raw_);
    pos_hector_raw_[0] = msg->pose.position.x;
    pos_hector_raw_[1] = msg->pose.position.y;
    return;
  }

  // Wait to obtain pixhawk odom to attach hector tf accordingly
  if (!getting_pixhawk_odom_) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Waiting for pixhawk odom to initialize tf", this->get_name());
    return;
  }

  {
    // Update od the position for the reliability check
    std::scoped_lock lock(mutex_hector_raw_);
    pos_hector_raw_prev_[0] = pos_hector_raw_[0];
    pos_hector_raw_prev_[1] = pos_hector_raw_[1];

    pos_hector_raw_[0] = msg->pose.position.x;
    pos_hector_raw_[1] = msg->pose.position.y;
  }

  /* Setup variables for tfs */ /*//{*/
  if (!hector_tf_setup_ & publishing_static_tf_) {
    auto tf             = transformBetween(fcu_frame_, world_frame_);
    pos_orig_hector_[0] = tf.pose.position.x;
    pos_orig_hector_[1] = tf.pose.position.y;
    pos_orig_hector_[2] = 0;
    ori_orig_hector_[0] = tf.pose.orientation.w;
    ori_orig_hector_[1] = tf.pose.orientation.x;
    ori_orig_hector_[2] = tf.pose.orientation.y;
    ori_orig_hector_[3] = tf.pose.orientation.z;
    RCLCPP_INFO(this->get_logger(), "[%s]: Hector origin coordinates set - x: %f y: %f z: %f, w: %f, x: %f, y: %f, z: %f", this->get_name(),
                pos_orig_hector_[0], pos_orig_hector_[1], pos_orig_hector_[2], ori_orig_hector_[0], ori_orig_hector_[1], ori_orig_hector_[2],
                ori_orig_hector_[3]);
    hector_tf_setup_ = true;
  } /*//}*/

  /* get heading from hector orientation quaternion //{*/

  double hdg_hector;
  try {
    hdg_hector = odometry_utils::getHeading(msg->pose.orientation);

    // unwrap heading to prevent discrete jumps
    hdg_hector                 = geometry::radians::unwrap(hdg_hector, hector_hdg_previous_);
    hector_hdg_previous_       = hdg_hector;
    hector_hdg_correction_     = hdg_hector;
    got_hector_hdg_correction_ = true;
    RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting hector heading corrections", this->get_name());
    /* RCLCPP_INFO(this->get_logger(), "[Odometry2]: hector hdg: %f", hdg_hector); */
  }
  catch (...) {
    RCLCPP_WARN(this->get_logger(), "[%s]: failed to getHeading() from hector orientation, dropping this correction", this->get_name());
  }

  hector_lat_correction_[0]  = msg->pose.position.x;
  hector_lat_correction_[1]  = msg->pose.position.y;
  got_hector_lat_correction_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting hector lateral corrections", this->get_name());
  /*//}*/

  getting_hector_ = true;
}
//}

/* getOdomCallback //{ */
bool Odometry2::getOdomCallback(const std::shared_ptr<fog_msgs::srv::GetBool::Request> request, std::shared_ptr<fog_msgs::srv::GetBool::Response> response) {
  if (!is_initialized_) {
    return false;
  }
  if (odom_ready_) {
    response->value = true;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: getOdometry service responded!", this->get_name());

  } else {
    response->value = false;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Waiting for Odometry availability!", this->get_name());
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
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: GPS origin not set yet!", this->get_name());
    response->success = false;
    return false;
  }
  return true;
}
//}

/* resetHectorCallback //{ */
bool Odometry2::resetHectorCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_initialized_) {
    response->message = "ROS not initialized";
    return false;
  }

  if (hector_reset_called_) {
    response->message = "Hector in reset mode";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "[Odometry2]: Hector is still resetting");
    return false;
  }

  std::chrono::duration<double> dt = std::chrono::system_clock::now() - hector_reset_called_time_;
  if (dt.count() < hector_reset_wait_) {
    response->message = "Waiting for next hector reset availability";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "[Odometry2]: Waiting for next hector reset availability. Call again in dt: %f",
                         hector_reset_wait_ - dt.count());
    return false;
  }

  hector_reset_called_      = true;
  hector_reset_called_time_ = std::chrono::system_clock::now();
  hector_reliable_          = false;

  RCLCPP_INFO(this->get_logger(), "[Odometry2]: Calling hector map reset");
  std_msgs::msg::String reset_msg;
  reset_msg.data = "reset";
  try {
    reset_hector_publisher_->publish(reset_msg);
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "[Odometry2]: Exception caught during publishing hector reset topic");
    response->message = "Exception caught during publishing hector reset topic";
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[Odometry2]: Hector map reset called");

  return true;
}
//}

/* getPx4ParamIntCallback //{ */
bool Odometry2::getPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::GetPx4ParamInt>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::GetPx4ParamInt::Response> result = future.get();
  if (result->success) {
    current_ekf_bitmask_ = result->value;
    RCLCPP_INFO(this->get_logger(), "[%s]: Parameter %s has value: %d", this->get_name(), result->param_name.c_str(), result->value);
    get_ekf_bitmask_called_ = false;
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Cannot get EKF2 parameter %s with message: %s", this->get_name(), result->param_name.c_str(),
                 result->message.c_str());
    get_ekf_bitmask_called_ = false;
    return false;
  }
}
//}

/* setPx4ParamIntCallback //{ */
bool Odometry2::setPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Response> result = future.get();
  if (result->success) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Parameter %s has been set to value: %d", this->get_name(), result->param_name.c_str(), result->value);
    if (result->param_name == "EKF2_AID_MASK") {
      current_ekf_bitmask_    = result->value;
      set_ekf_bitmask_called_ = false;
    }
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Cannot set the parameter %s with message: %s", this->get_name(), result->param_name.c_str(),
                 result->message.c_str());
    if (result->param_name == "EKF2_AID_MASK") {
      set_ekf_bitmask_called_ = false;
    }
    return false;
  }
}
//}

/* setPx4ParamFloatCallback //{ */
bool Odometry2::setPx4FloatParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::SetPx4ParamFloat::Response> result = future.get();
  if (result->success) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Parameter %s has been set to value: %f", this->get_name(), result->param_name.c_str(), result->value);
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Cannot set the parameter %s with message: %s", this->get_name(), result->param_name.c_str(),
                 result->message.c_str());
    return false;
  }
}
//}

/* resetHectorClientCallback //{ */
bool Odometry2::resetHectorClientCallback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
  std::shared_ptr<std_srvs::srv::Trigger::Response> result = future.get();
  if (result->success) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Hector successfully reset", this->get_name());
    return true;
  } else {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Hector was not reset, message: %s", this->get_name(), result->message.c_str());
    return false;
  }
}
//}

/* baroCallback //{ */
void Odometry2::baroCallback(const px4_msgs::msg::SensorBaro::UniquePtr msg) {
  if (!is_initialized_) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Not getting baro!");
    return;
  }
  getting_baro_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting baro!", this->get_name());

  /* TODO: get AGL altitude from pressure, temperature and takeoff ASL altitude */
  /* double measurement = odometry_utils::getAltitudeFromPressure(msg->pressure, msg->temperature); */
  double measurement = 0.0;
  /* RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: getAltitudeFromPressure() not implemented. Using 0 for testing !!!", */
  /* this->get_name()); */

  if (!std::isfinite(measurement)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: not finite value detected in variable \"measurement\" (baroCallback) !!!",
                          this->get_name());
    return;
  }

  if (!got_baro_alt_correction_) {
    baro_alt_measurement_prev_ = measurement;
    baro_alt_correction_       = 0.0;
    time_baro_prev_            = this->get_clock()->now();
    got_baro_alt_correction_   = true;
    return;
  }
  // calculate time since last estimators update
  double       dt;
  rclcpp::Time time_now = this->get_clock()->now();
  dt                    = (time_now - time_baro_prev_).seconds();
  time_baro_prev_       = time_now;

  if (dt <= 0.0) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: baro callback dt=%f, discarding correction.", this->get_name(), dt);
    return;
  }


  baro_alt_correction_       = (measurement - baro_alt_measurement_prev_) / dt;
  baro_alt_measurement_prev_ = measurement;

  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting barometric altitude corrections", this->get_name());
}
//}

/* garminCallback //{ */
void Odometry2::garminCallback(const sensor_msgs::msg::Range::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }
  getting_garmin_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting garmin!", this->get_name());

  double measurement = msg->range;
  if (!std::isfinite(measurement)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: not finite value detected in variable \"measurement\" (garminCallback) !!!",
                          this->get_name());
    return;
  }

  if (!isValidGate(measurement, _garmin_min_valid_alt_, _garmin_max_valid_alt_, "garmin range")) {
    return;
  }

  // do not fuse garmin measurements when a height jump is detected - most likely the UAV is flying above an obstacle
  /* if (!alt_mf_garmin_->isValid(measurement)) { */
  /*   RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Garmin measurement %f declined by median filter.", this->get_name(),
   * measurement); */
  /*   return; */
  /* } */

  garmin_alt_correction_     = measurement;
  got_garmin_alt_correction_ = true;

  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting Garmin altitude corrections", this->get_name());
}
//}

/* odometryRoutine //{ */
void Odometry2::odometryRoutine(void) {
  if (is_initialized_ && getting_pixhawk_odom_ && getting_gps_) {

    if (!set_initial_px4_params_) {
      setInitialPx4Params();
      set_initial_px4_params_ = true;
    }
    callbacks_enabled_ = true;
    odom_ready_        = true;
    if (static_tf_broadcaster_ == nullptr) {
      static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
      publishStaticTF();
      return;
    }

    pixhawkEkfUpdate();

    publishTF();
    publishLocalOdom();


    /* if (hector_use_) { */
    if (hector_reliable_) {
      updateEstimators();
      publishOdometry();
      checkHectorReliability();
    } else {
      resetHector();
    }
    /* } */
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

  publishing_static_tf_ = true;
}
//}

/* publishTF //{ */
void Odometry2::publishTF() {
  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }
  geometry_msgs::msg::TransformStamped tf1;
  tf2::Quaternion                      q;

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

  if (hector_tf_setup_) {
    tf1.header.stamp            = this->get_clock()->now();
    tf1.header.frame_id         = world_frame_;
    tf1.child_frame_id          = hector_origin_frame_;
    tf1.transform.translation.x = pos_orig_hector_[0];
    tf1.transform.translation.y = pos_orig_hector_[1];
    tf1.transform.translation.z = pos_orig_hector_[2];
    /* q.setRPY(0, 0, 0); */
    /* tf1.transform.rotation.w = q.getW(); */
    /* tf1.transform.rotation.x = q.getX(); */
    /* tf1.transform.rotation.y = q.getY(); */
    /* tf1.transform.rotation.z = q.getZ(); */
    tf1.transform.rotation.w = ori_orig_hector_[0];
    tf1.transform.rotation.x = ori_orig_hector_[1];
    tf1.transform.rotation.y = ori_orig_hector_[2];
    tf1.transform.rotation.z = ori_orig_hector_[3];
    tf_broadcaster_->sendTransform(tf1);

    if (publishing_odometry_) {
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
    }
  }
}
//}

/* publishLocalOdom //{ */
void Odometry2::publishLocalOdom() {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp            = this->get_clock()->now();
  msg.header.frame_id         = world_frame_;
  msg.child_frame_id          = fcu_frame_;
  auto tf                     = transformBetween(fcu_frame_, world_frame_);
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

/*isValidGate() //{*/
bool Odometry2::isValidGate(const double value, const double min_value, const double max_value, const std::string &value_name) {

  // Min value check
  if (value < min_value) {
    if (value_name != "") {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: %s value %f < %f is not valid.", this->get_name(), value_name.c_str(), value,
                           min_value);
    }
    return false;
  }

  // Max value check
  if (value > max_value) {
    if (value_name != "") {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: %s value %f > %f is not valid.", this->get_name(), value_name.c_str(), value,
                           max_value);
    }
    return false;
  }

  return true;
}
/*//}*/

/*setInitialPx4Params//{*/
bool Odometry2::setInitialPx4Params() {
  for (px4_int item : px4_params_int) {
    auto request        = std::make_shared<fog_msgs::srv::SetPx4ParamInt::Request>();
    request->param_name = std::get<0>(item);
    request->value      = std::get<1>(item);
    RCLCPP_INFO(this->get_logger(), "[%s]: Setting %s, value: %d", this->get_name(), std::get<0>(item).c_str(), std::get<1>(item));
    auto call_result = set_px4_param_int_->async_send_request(request, std::bind(&Odometry2::setPx4IntParamCallback, this, std::placeholders::_1));
  }

  for (px4_float item : px4_params_float) {
    auto request        = std::make_shared<fog_msgs::srv::SetPx4ParamFloat::Request>();
    request->param_name = std::get<0>(item);
    request->value      = std::get<1>(item);
    RCLCPP_INFO(this->get_logger(), "[%s]: Setting %s, value: %f", this->get_name(), std::get<0>(item).c_str(), std::get<1>(item));
    auto call_result = set_px4_param_float_->async_send_request(request, std::bind(&Odometry2::setPx4FloatParamCallback, this, std::placeholders::_1));
  }

  return true;
}

/*//}*/
/* pixhawkEkfUpdate //{*/
void Odometry2::pixhawkEkfUpdate() {

  std::chrono::duration<double> dt = std::chrono::system_clock::now() - hector_reset_called_time_;

  // TODO:: If everything to false, it will keep the previous settings without any change.

  /* // Initialize the bitmask value */
  /* if (current_ekf_bitmask_ == 0 && !get_ekf_bitmask_called_) { */
  /*   get_ekf_bitmask_called_ = true; */
  /*   auto request            = std::make_shared<fog_msgs::srv::GetPx4ParamInt::Request>(); */
  /*   request->param_name     = "EKF2_AID_MASK"; */
  /*   auto call_result        = get_px4_param_int_->async_send_request(request, std::bind(&Odometry2::getPx4IntParamCallback, this, std::placeholders::_1)); */
  /*   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Get EKF2 aid mask called", this->get_name()); */
  /* } else { */
  auto request        = std::make_shared<fog_msgs::srv::SetPx4ParamInt::Request>();
  request->param_name = "EKF2_AID_MASK";

  if (gps_reliable_ && hector_reliable_ && current_ekf_bitmask_ != 321 && !set_ekf_bitmask_called_ && dt.count() > hector_fusion_wait_ && gps_use_ &&
      hector_use_) {
    set_ekf_bitmask_called_ = true;
    request->value          = 321;
    RCLCPP_INFO(this->get_logger(), "[%s]: Setting EKF2 aid mask, value: %d", this->get_name(), request->value);
    auto call_result = set_px4_param_int_->async_send_request(request, std::bind(&Odometry2::setPx4IntParamCallback, this, std::placeholders::_1));
  } else if (!gps_reliable_ && hector_reliable_ && current_ekf_bitmask_ != 320 && !set_ekf_bitmask_called_ && dt.count() > hector_fusion_wait_ && hector_use_) {
    set_ekf_bitmask_called_ = true;
    request->value          = 320;
    RCLCPP_INFO(this->get_logger(), "[%s]: Setting EKF2 aid mask, value: %d", this->get_name(), request->value);
    auto call_result = set_px4_param_int_->async_send_request(request, std::bind(&Odometry2::setPx4IntParamCallback, this, std::placeholders::_1));
  } else if (gps_reliable_ && (!hector_use_ || !hector_reliable_) && current_ekf_bitmask_ != 1 && !set_ekf_bitmask_called_ && gps_use_) {
    set_ekf_bitmask_called_ = true;
    request->value          = 1;
    RCLCPP_INFO(this->get_logger(), "[%s]: Setting EKF2 aid mask, value: %d", this->get_name(), request->value);
    auto call_result = set_px4_param_int_->async_send_request(request, std::bind(&Odometry2::setPx4IntParamCallback, this, std::placeholders::_1));
  } else if (!gps_reliable_ && !hector_reliable_ && (gps_use_ || hector_use_)) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    RCLCPP_ERROR(this->get_logger(), "[%s]: No reliable odometry, landing", this->get_name());
    auto call_result = land_service_->async_send_request(request);
    gps_use_         = false;
    hector_use_      = false;
  }
  /* } */
}
//}

/*resetHector//{*/
void Odometry2::resetHector() {
  if (!hector_reset_called_) {
    auto request     = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto call_result = reset_hector_client_->async_send_request(request, std::bind(&Odometry2::resetHectorClientCallback, this, std::placeholders::_1));
  }
}  //}

/* updateEstimators //{*/
void Odometry2::updateEstimators() {

  if (!time_odometry_timer_set_) {
    time_odometry_timer_prev_ = std::chrono::system_clock::now();
    time_odometry_timer_set_  = true;
  }

  // calculate time since last estimators update
  std::chrono::time_point       time_now = std::chrono::system_clock::now();
  std::chrono::duration<double> dt       = time_now - time_odometry_timer_prev_;

  if (dt.count() <= 0) {
    RCLCPP_WARN(this->get_logger(), "[%s]: odometry timer dt=%f, skipping estimator update.", this->get_name(), dt.count());
    return;
  }

  time_odometry_timer_prev_ = time_now;

  /* altitude estimator update and predict //{ */

  if (got_garmin_alt_correction_) {
    garmin_alt_estimator_->doCorrection(garmin_alt_correction_, ALT_GARMIN);
  }

  if (got_baro_alt_correction_) {
    /* garmin_alt_estimator_->doCorrection(baro_alt_correction_, ALT_BARO); TODO:: Missing baro estimator */
  }

  /* TODO: add control input to prediction? */
  garmin_alt_estimator_->doPrediction(0.0, dt.count());

  //}

  /* heading estimator update and predict //{ */

  if (got_hector_hdg_correction_) {
    hector_hdg_estimator_->doCorrection(hector_hdg_correction_, HDG_HECTOR);
  }

  /* if (got_gyro_hdg_correction_) { */
  /*   hector_hdg_estimator_->doCorrection(hector_hdg_correction_, HDG_GYRO); */
  /* } */

  /* TODO: add control input to prediction? */
  hector_hdg_estimator_->doPrediction(0.0, dt.count());

  if (got_hector_lat_correction_) {
    hector_lat_estimator_->doCorrection(hector_lat_correction_[0], hector_lat_correction_[1], LAT_HECTOR);
  }

  /* TODO: add control input to prediction? */
  hector_lat_estimator_->doPrediction(0.0, 0.0, dt.count());

  //}
}

/*//}*/

/*publishOdometry//{*/
void Odometry2::publishOdometry() {

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = this->get_clock()->now();
  /* msg.header.frame_id = world_frame_; */
  /* msg.child_frame_id  = fcu_frame_; */
  msg.header.frame_id = hector_origin_frame_;
  msg.child_frame_id  = hector_frame_;

  // altitude
  alt_x_t alt_x = alt_x.Zero();

  if (!garmin_alt_estimator_->getStates(alt_x)) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Altitude estimator not initialized.", this->get_name());
    return;
  }

  // heading
  double                         hdg, mavros_hdg = 0;
  Eigen::Matrix3d                mat;
  Eigen::Quaterniond             quaternion;
  geometry_msgs::msg::Quaternion geom_quaternion;
  tf2::Quaternion                tf2_quaternion;

  hector_hdg_estimator_->getState(0, hdg);

  // Obtain mavros orientation
  try {
    mavros_hdg = odometry_utils::getHeading(tf2::toMsg(mavros_orientation_));
  }
  catch (...) {
    RCLCPP_WARN(this->get_logger(), "[%s]: failed to getHeading() from mavros_orientation", this->get_name());
  }

  // Build rotation matrix from difference between new heading and mavros heading
  mat = Eigen::AngleAxisd(hdg - mavros_hdg, Eigen::Vector3d::UnitZ());

  quaternion = Eigen::Quaterniond(mat);

  tf2_quaternion.setX(quaternion.x());
  tf2_quaternion.setY(quaternion.y());
  tf2_quaternion.setZ(quaternion.z());
  tf2_quaternion.setW(quaternion.w());

  // Transform the mavros orientation by the rotation matrix
  quaternion = Eigen::Quaterniond(tf2::Transform(tf2::Matrix3x3(tf2_quaternion)) * mavros_orientation_);

  /* geom_quaternion.x = quaternion.x(); */
  /* geom_quaternion.y = quaternion.y(); */
  /* geom_quaternion.z = quaternion.z(); */
  /* geom_quaternion.w = quaternion.w(); */
  /* ori_hector_[0] = quaternion.w(); */
  /* ori_hector_[1] = quaternion.x(); */
  /* ori_hector_[2] = quaternion.y(); */
  /* ori_hector_[3] = quaternion.z(); */
  // TODO: This orientation cannot be used, try to orient it corretly. Using mavros magnetometer orientation instead
  ori_hector_[0] = mavros_orientation_.getW();
  ori_hector_[1] = mavros_orientation_.getX();
  ori_hector_[2] = mavros_orientation_.getY();
  ori_hector_[3] = mavros_orientation_.getZ();

  // TODO:: apply inverse rotation

  // lateral
  double pos_x, pos_y, pos_z, vel_x, vel_y, acc_x, acc_y;

  hector_lat_estimator_->getState(0, pos_x);
  hector_lat_estimator_->getState(1, pos_y);
  hector_lat_estimator_->getState(2, vel_x);
  hector_lat_estimator_->getState(3, vel_y);
  hector_lat_estimator_->getState(4, acc_x);
  hector_lat_estimator_->getState(5, acc_y);

  pos_hector_[0] = pos_x;
  pos_hector_[1] = pos_y;

  // altitude
  double vel_z;

  garmin_alt_estimator_->getState(0, pos_z);
  garmin_alt_estimator_->getState(1, vel_z);

  pos_hector_[2] = pos_z;

  // TODO:: Change of getting variables into updateEstimators
  publishing_odometry_ = true;

  // publish odometry//{
  msg.pose.pose.position.x = pos_hector_[0];
  msg.pose.pose.position.y = pos_hector_[1];
  msg.pose.pose.position.z = pos_hector_[2];
  /* msg.twist.twist.linear.x    = vel_x; */
  /* msg.twist.twist.linear.y    = vel_y; */
  /* msg.twist.twist.linear.z    = vel_z; */
  msg.pose.pose.orientation.w = ori_hector_[0];
  msg.pose.pose.orientation.x = ori_hector_[1];
  msg.pose.pose.orientation.y = ori_hector_[2];
  msg.pose.pose.orientation.z = ori_hector_[3];
  /* msg.pose.pose.position.x    = pos_hector_[1]; */
  /* msg.pose.pose.position.y    = pos_hector_[0]; */
  /* msg.pose.pose.position.z    = -pos_hector_[2]; */
  /* msg.twist.twist.linear.x    = vel_y; */
  /* msg.twist.twist.linear.y    = vel_x; */
  /* msg.twist.twist.linear.z    = -vel_z; */
  /* msg.pose.pose.orientation.w = ori_hector_[0]; */
  /* msg.pose.pose.orientation.x = ori_hector_[1]; */
  /* msg.pose.pose.orientation.y = ori_hector_[2]; */
  /* msg.pose.pose.orientation.z = ori_hector_[3]; */

  /* local_hector_publisher_->publish(msg); */
  /*//}*/

  // publish visual odometry//{

  try {
    auto tf = transformBetween(hector_frame_, ned_origin_frame_);

    std::chrono::duration<long int, std::nano> diff = std::chrono::system_clock::now() - time_sync_time_;

    visual_odometry_.timestamp        = timestamp_ + diff.count() / 1000;
    visual_odometry_.timestamp_sample = timestamp_raw_ / 1000 + diff.count() / 1000;

    visual_odometry_.x = tf.pose.position.x;
    visual_odometry_.y = tf.pose.position.y;
    /* visual_odometry_.z = tf.pose.position.z; */

    garmin_buffer_z_.insert(garmin_buffer_z_.begin(), tf.pose.position.z);
    if (garmin_buffer_z_.size() > garmin_buffer_size_) {
      visual_odometry_.z = garmin_buffer_z_.back();
      garmin_buffer_z_.pop_back();
    } else {
      visual_odometry_.z = 0.0;
    }

    tf2::Quaternion q_orig, q_new, q_rot;

    tf2::convert(tf.pose.orientation, q_orig);
    double r = M_PI, p = 0, y = M_PI;
    q_rot.setRPY(r, p, y);
    q_new = q_rot * q_orig;  // Calculate the new orientation
    q_new.normalize();

    /* visual_odometry_.q[0] = q_new.w(); */
    /* visual_odometry_.q[1] = q_new.x(); */
    /* visual_odometry_.q[2] = q_new.y(); */
    /* visual_odometry_.q[3] = q_new.z(); */

    /* visual_odometry_.q[0] = tf.pose.orientation.w; */
    /* visual_odometry_.q[1] = tf.pose.orientation.x; */
    /* visual_odometry_.q[2] = tf.pose.orientation.y; */
    /* visual_odometry_.q[3] = tf.pose.orientation.z; */

    visual_odometry_.q[0] = mavros_orientation_.getW();
    visual_odometry_.q[1] = mavros_orientation_.getX();
    visual_odometry_.q[2] = mavros_orientation_.getY();
    visual_odometry_.q[3] = mavros_orientation_.getZ();
  }
  catch (...) {
    std::fill(visual_odometry_.q_offset.begin(), visual_odometry_.q_offset.end(), NAN);

    std::fill(visual_odometry_.pose_covariance.begin(), visual_odometry_.pose_covariance.end(), NAN);
  }

  try {
    auto transform_stamped = tf_buffer_->lookupTransform(ned_origin_frame_, hector_origin_frame_, rclcpp::Time(0));

    geometry_msgs::msg::Vector3Stamped hector_vel, global_vel;
    hector_vel.header.frame_id = hector_frame_;
    hector_vel.header.stamp    = rclcpp::Time(0);
    hector_vel.vector.x        = vel_x;
    hector_vel.vector.y        = vel_y;
    hector_vel.vector.z        = vel_z;

    tf2::doTransform(hector_vel, global_vel, transform_stamped);

    visual_odometry_.vx = global_vel.vector.x;
    visual_odometry_.vy = global_vel.vector.y;
    /* visual_odometry_.vz = global_vel.vector.z; */

    garmin_buffer_vz_.insert(garmin_buffer_vz_.begin(), global_vel.vector.z);
    if (garmin_buffer_vz_.size() > garmin_buffer_size_) {
      visual_odometry_.vz = garmin_buffer_vz_.back();
      garmin_buffer_vz_.pop_back();
    } else {
      visual_odometry_.vz = 0.0;
    }

    msg.twist.twist.linear.x = global_vel.vector.x;
    msg.twist.twist.linear.y = global_vel.vector.y;
    msg.twist.twist.linear.z = global_vel.vector.z;
  }
  catch (...) {
    return;
  }
  /* visual_odometry_.vx = vel_y; */
  /* visual_odometry_.vy = vel_x; */
  /* visual_odometry_.vz = -vel_z; */

  visual_odometry_.rollspeed  = NAN;
  visual_odometry_.pitchspeed = NAN;
  visual_odometry_.yawspeed   = NAN;

  std::fill(visual_odometry_.velocity_covariance.begin(), visual_odometry_.velocity_covariance.end(), NAN);

  /* std::chrono::duration<long int, std::nano> diff = std::chrono::system_clock::now() - time_sync_time_; */

  /* visual_odometry_.timestamp        = timestamp_ + diff.count() / 1000; */
  /* visual_odometry_.timestamp_sample = timestamp_raw_ / 1000 + diff.count() / 1000; */

  /* visual_odometry_.x = pos_hector_[1]; */
  /* visual_odometry_.y = pos_hector_[0]; */
  /* visual_odometry_.z = -pos_hector_[2]; */

  /* visual_odometry_.q[0] = mavros_orientation_.getW(); */
  /* visual_odometry_.q[1] = mavros_orientation_.getX(); */
  /* visual_odometry_.q[2] = mavros_orientation_.getY(); */
  /* visual_odometry_.q[3] = mavros_orientation_.getZ(); */

  /* std::fill(visual_odometry_.q_offset.begin(), visual_odometry_.q_offset.end(), NAN); */

  /* std::fill(visual_odometry_.pose_covariance.begin(), visual_odometry_.pose_covariance.end(), NAN); */

  /* visual_odometry_.vx = vel_y; */
  /* visual_odometry_.vy = vel_x; */
  /* visual_odometry_.vz = -vel_z; */

  /* visual_odometry_.rollspeed  = NAN; */
  /* visual_odometry_.pitchspeed = NAN; */
  /* visual_odometry_.yawspeed   = NAN; */

  /* std::fill(visual_odometry_.velocity_covariance.begin(), visual_odometry_.velocity_covariance.end(), NAN); */

  local_hector_publisher_->publish(msg);

  visual_odom_publisher_->publish(visual_odometry_);

  /*//}*/

} /*//}*/

/* checkHectorReliability //{*/
void Odometry2::checkHectorReliability() {

  if (getting_hector_) {
    // Check hector message interval//{
    std::chrono::duration<double> dt = std::chrono::system_clock::now() - time_hector_last_msg_;

    if (dt.count() > hector_msg_interval_warn_) {
      RCLCPP_WARN(this->get_logger(), "[Odometry2]: Hector pose not received for %f seconds.", dt.count());
      if (dt.count() > hector_msg_interval_max_) {
        RCLCPP_WARN(this->get_logger(), "[Odometry2]: Hector pose not received for %f seconds. Not reliable.", dt.count());
        hector_reliable_ = false;
        return;
      }
    }
    /*//}*/

    // Detect jump since previous pose//{

    {
      std::scoped_lock lock(mutex_hector_raw_);
      if (std::pow(pos_hector_raw_[0] - pos_hector_raw_prev_[0], 2) > hector_max_position_jump_ ||
          std::pow(pos_hector_raw_[1] - pos_hector_raw_prev_[1], 2) > hector_max_position_jump_) {
        RCLCPP_WARN(this->get_logger(), "[Odometry2]: Jump detected in Hector Slam pose. orig_x: %f, orig_y: %f, x: %f, y: %f .Not reliable",
                    pos_hector_raw_prev_[0], pos_hector_raw_prev_[1], pos_hector_raw_[0], pos_hector_raw_[1]);
        hector_reliable_ = false;
        return;
      }
    }
    /*//}*/

    // Check if hector speed reliable//{
    double vel_x, vel_y;

    hector_lat_estimator_->getState(2, vel_x);
    hector_lat_estimator_->getState(3, vel_y);

    if (vel_x > hector_max_velocity_ || vel_y > hector_max_velocity_) {
      hector_reliable_ = false;
      RCLCPP_WARN(this->get_logger(), "[Odometry2]: Hector velocity too large - x: %f, y: %f. Not reliable.", vel_x, vel_y);
      return;
    } /*//}*/

    // Hector is reliable then
    hector_reliable_ = true;
  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: No hector reliability check. Waiting for full hector initialization",
                         this->get_name());
  }
} /*//}*/

/* //{ printOdometryDiag */
std::string Odometry2::printOdometryDiag() {

  std::string s_diag;

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
  const std::string param_path = "odometry2." + param_name;
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
