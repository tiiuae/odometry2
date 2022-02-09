#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <fog_msgs/srv/get_px4_param_int.hpp>
#include <fog_msgs/srv/set_px4_param_int.hpp>
#include <fog_msgs/srv/get_px4_param_float.hpp>
#include <fog_msgs/srv/set_px4_param_float.hpp>

#include <fog_msgs/msg/control_interface_diagnostics.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/home_position.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'
#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/string.hpp>

#include <fog_lib/params.h>

#include "gps_conversions.h"

typedef std::tuple<std::string, int>   px4_int;
typedef std::tuple<std::string, float> px4_float;

using namespace std::placeholders;
using namespace fog_lib;

namespace odometry2
{

/* class Odometry2 //{ */
class Odometry2 : public rclcpp::Node {
public:
  Odometry2(rclcpp::NodeOptions options);

private:
  std::atomic_bool is_initialized_ = false;

  // | ------------------------ TF frames ----------------------- |
  std::string uav_name_           = "";
  std::string utm_origin_frame_   = "";
  std::string local_origin_frame_ = "";
  std::string ned_origin_frame_   = "";
  std::string frd_fcu_frame_      = "";
  std::string fcu_frame_          = "";

  // | ---------------------- TF variables ---------------------- |
  std::shared_ptr<tf2_ros::Buffer>                     tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  geometry_msgs::msg::TransformStamped                 tf_world_to_ned_origin_frame_;

  // | ---------------------- PX parameters --------------------- |
  std::vector<px4_int>   px4_params_int_;
  std::vector<px4_float> px4_params_float_;
  std::atomic_bool       set_initial_px4_params_                = false;
  std::atomic_bool       getting_pixhawk_odom_                  = false;
  std::atomic_bool       getting_px4_utm_position_              = false;
  std::atomic_bool       getting_control_interface_diagnostics_ = false;

  float      px4_position_[3];
  float      px4_orientation_[4];
  std::mutex px4_pose_mutex_;

  float            px4_utm_position_[2];
  std::atomic_bool republish_static_tf_ = false;
  std::mutex       px4_utm_position_mutex_;

  px4_msgs::msg::HomePosition  px4_home_position_;
  rclcpp::TimerBase::SharedPtr home_position_timer_;
  std::mutex                   px4_home_position_mutex_;

  // | ----------------------- Publishers ----------------------- |
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr     local_odom_publisher_;
  rclcpp::Publisher<px4_msgs::msg::HomePosition>::SharedPtr home_position_publisher_;

  // | ----------------------- Subscribers ---------------------- |
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr             pixhawk_odom_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr                home_position_subscriber_;
  rclcpp::Subscription<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr control_interface_diagnostics_subscriber_;

  // | --------------------- Service clients -------------------- |
  rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_;

  // | ------------------ Subscriber callbacks ------------------ |
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg);
  void ControlInterfaceDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg);

  // | ---------------- Service clients handlers ---------------- |
  bool setPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedFuture future);
  bool setPx4FloatParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedFuture future);

  // | ------------------- Internal functions ------------------- |
  bool setInitialPx4Params();

  void publishStaticTF();
  void publishLocalOdomAndTF();

  // | -------------------- Routine handling -------------------- |
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr     odometry_timer_;
  double                           odometry_loop_rate_;
  double                           home_position_publish_rate_;

  void odometryRoutine(void);
  void homePositionPublisher(void);

};
//}

/* constructor //{ */
Odometry2::Odometry2(rclcpp::NodeOptions options) : Node("odometry2", options) {

  RCLCPP_INFO(this->get_logger(), "[%s]: Initializing...", this->get_name());

  // Getting
  try {
    uav_name_ = std::string(std::getenv("DRONE_DEVICE_ID"));
  }
  catch (...) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Environment variable DRONE_DEVICE_ID was not defined!", this->get_name());
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: UAV name is: '%s'", this->get_name(), uav_name_.c_str());

  /* parse general params from config file //{ */
  RCLCPP_INFO(this->get_logger(), "-------------- Loading parameters --------------");
  bool loaded_successfully = true;

  loaded_successfully &= parse_param("odometry_loop_rate", odometry_loop_rate_, *this);
  loaded_successfully &= parse_param("home_position_publish_rate", home_position_publish_rate_, *this);

  int   param_int;
  float param_float;

  loaded_successfully &= parse_param("px4.EKF2_AID_MASK", param_int, *this);
  px4_params_int_.push_back(px4_int("EKF2_AID_MASK", param_int));

  loaded_successfully &= parse_param("px4.EKF2_RNG_AID", param_int, *this);
  px4_params_int_.push_back(px4_int("EKF2_RNG_AID", param_int));

  loaded_successfully &= parse_param("px4.EKF2_HGT_MODE", param_int, *this);
  px4_params_int_.push_back(px4_int("EKF2_HGT_MODE", param_int));

  loaded_successfully &= parse_param("px4.EKF2_RNG_A_HMAX", param_float, *this);
  px4_params_float_.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));

  loaded_successfully &= parse_param("world_frame", local_origin_frame_, *this);
  
  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(this->get_logger(), "%s", str.c_str());
    rclcpp::shutdown();
    return;
  }
  //}

  /* frame definition */
  utm_origin_frame_   = uav_name_ + "/utm_origin";    // FLU frame (Front-Left-Up) match also ENU frame (East-North-Up)
  local_origin_frame_ = uav_name_ + "/local_origin";  // FLU frame (Front-Left-Up) match also ENU frame (East-North-Up)
  fcu_frame_          = uav_name_ + "/fcu";           // FLU frame (Front-Left-Up) match also ENU frame (East-North-Up)
  frd_fcu_frame_      = uav_name_ + "/frd_fcu";       // FRD frame (Front-Right-Down)
  ned_origin_frame_   = uav_name_ + "/ned_origin";    // NED frame (North-East-Down)

  // publishers
  local_odom_publisher_    = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", 10);
  home_position_publisher_ = this->create_publisher<px4_msgs::msg::HomePosition>("~/home_position_out", 10);

  // subscribers
  pixhawk_odom_subscriber_  = this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&Odometry2::pixhawkOdomCallback, this, _1));
  home_position_subscriber_ = this->create_subscription<px4_msgs::msg::HomePosition>("~/home_position_in", rclcpp::SystemDefaultsQoS(),
                                                                                     std::bind(&Odometry2::homePositionCallback, this, _1));
  control_interface_diagnostics_subscriber_ = this->create_subscription<fog_msgs::msg::ControlInterfaceDiagnostics>(
      "~/control_interface_diagnostics_in", rclcpp::SystemDefaultsQoS(), std::bind(&Odometry2::ControlInterfaceDiagnosticsCallback, this, _1));

  // service clients
  set_px4_param_int_   = this->create_client<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int");
  set_px4_param_float_ = this->create_client<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float");

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odometry_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / odometry_loop_rate_), std::bind(&Odometry2::odometryRoutine, this), callback_group_);
  home_position_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / home_position_publish_rate_),
                                                 std::bind(&Odometry2::homePositionPublisher, this), callback_group_);

  tf_broadcaster_        = nullptr;
  static_tf_broadcaster_ = nullptr;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: Initialized", this->get_name());
}
//}

/* pixhawkOdomCallback //{ */
void Odometry2::pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
  std::scoped_lock lock(px4_pose_mutex_);

  if (!is_initialized_) {
    return;
  }

  px4_position_[0]    = msg->x;
  px4_position_[1]    = msg->y;
  px4_position_[2]    = msg->z;
  px4_orientation_[0] = msg->q[0];  // q.w
  px4_orientation_[1] = msg->q[1];  // q.x
  px4_orientation_[2] = msg->q[2];  // q.y
  px4_orientation_[3] = msg->q[3];  // q.z

  getting_pixhawk_odom_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting pixhawk odometry!", this->get_name());
}
//}

/* homePositionCallback //{ */
void Odometry2::homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg) {
  if (!is_initialized_.load()) {
    return;
  }

  {
    std::scoped_lock lock(px4_home_position_mutex_);

    px4_home_position_.timestamp =
        msg->timestamp;  // TODO:: Timestamp will be still the same but is it important? At least we know the last published message time
    px4_home_position_.lat = msg->lat;
    px4_home_position_.lon = msg->lon;
    px4_home_position_.alt = msg->alt;

    px4_home_position_.x = msg->x;
    px4_home_position_.y = msg->y;
    px4_home_position_.z = msg->z;

    px4_home_position_.yaw = msg->yaw;

    px4_home_position_.valid_alt  = msg->valid_alt;
    px4_home_position_.valid_hpos = msg->valid_hpos;
    px4_home_position_.valid_lpos = msg->valid_lpos;

    px4_home_position_.manual_home = msg->manual_home;
  }

  double out_x, out_y;

  UTM(msg->lat, msg->lon, &out_x, &out_y);

  if (!std::isfinite(out_x)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: NaN detected in UTM variable \"out_x\"!!!", this->get_name());
    return;
  }

  if (!std::isfinite(out_y)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: NaN detected in UTM variable \"out_y\"!!!", this->get_name());
    return;
  }

  {
    std::scoped_lock lock(px4_utm_position_mutex_);

    px4_utm_position_[0] = out_x;
    px4_utm_position_[1] = out_y;
    republish_static_tf_ = true;
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: GPS origin set! UTM x: %.2f, y: %.2f", this->get_name(), out_x, out_y);

  getting_px4_utm_position_ = true;
}
//}

/* ControlInterfaceDiagnosticsCallback //{ */
void Odometry2::ControlInterfaceDiagnosticsCallback([[maybe_unused]] const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg) {

  if (!is_initialized_ || (msg->vehicle_state.state == fog_msgs::msg::ControlInterfaceVehicleState::NOT_CONNECTED)){ 
    return;
  }

  getting_control_interface_diagnostics_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting control diagnostics!", this->get_name());

  // TODO: there is no proper way how to unsubscribe from a topic yet. This will at least delete subscriber.
  // However DDS layer is still receiving msgs until new timer/subsriber/service is registered.
  // See: https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/
  control_interface_diagnostics_subscriber_.reset();
}
//}

/* setPx4ParamIntCallback //{ */
bool Odometry2::setPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Response> result = future.get();
  if (result->success) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Parameter %s has been set to value: %ld", this->get_name(), result->param_name.c_str(), result->value);
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Cannot set the parameter %s with message: %s", this->get_name(), result->param_name.c_str(),
                 result->message.c_str());
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

/* odometryRoutine //{ */
void Odometry2::odometryRoutine(void) {

  if (is_initialized_ && getting_pixhawk_odom_ && getting_control_interface_diagnostics_) {
    RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Everything ready -> Publishing odometry", this->get_name());
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[%s]: Publishing odometry", this->get_name());

    if (!set_initial_px4_params_) {
      setInitialPx4Params();
      set_initial_px4_params_ = true;
    }

    if (static_tf_broadcaster_ == nullptr) {
      static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
      publishStaticTF();
    }

    if (republish_static_tf_) {
      republish_static_tf_ = false;
      publishStaticTF();
    }

    publishLocalOdomAndTF();

  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[%s]: Getting PX4 odometry: %s, Getting control_interface diagnostics: %s",
                         this->get_name(), getting_pixhawk_odom_.load() ? "TRUE" : "FALSE", getting_control_interface_diagnostics_ ? "TRUE" : "FALSE");
  }
}
//}

/* homePositionPublisher //{ */
void Odometry2::homePositionPublisher(void) {
  std::scoped_lock lock(px4_home_position_mutex_);
  home_position_publisher_->publish(px4_home_position_);
}
//}

/* publishStaticTF //{ */
void Odometry2::publishStaticTF() {

  std::vector<geometry_msgs::msg::TransformStamped> v_transforms;

  std::scoped_lock lock(px4_utm_position_mutex_);

  geometry_msgs::msg::TransformStamped tf;
  tf2::Quaternion                      q;

  tf.header.stamp = this->get_clock()->now();

  tf.header.frame_id         = frd_fcu_frame_;
  tf.child_frame_id          = fcu_frame_;
  tf.transform.translation.x = 0;
  tf.transform.translation.y = 0;
  tf.transform.translation.z = 0;
  q.setRPY(-M_PI, 0, 0);
  tf.transform.rotation.x = q.getX();
  tf.transform.rotation.y = q.getY();
  tf.transform.rotation.z = q.getZ();
  tf.transform.rotation.w = q.getW();
  v_transforms.push_back(tf);

  tf.header.frame_id         = local_origin_frame_;
  tf.child_frame_id          = ned_origin_frame_;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  q.setRPY(M_PI, 0, M_PI_2);
  q                       = q.inverse();
  tf.transform.rotation.x = q.getX();
  tf.transform.rotation.y = q.getY();
  tf.transform.rotation.z = q.getZ();
  tf.transform.rotation.w = q.getW();
  v_transforms.push_back(tf);

  tf_world_to_ned_origin_frame_ = tf;

  tf.header.frame_id         = utm_origin_frame_;
  tf.child_frame_id          = local_origin_frame_;
  tf.transform.translation.x = px4_utm_position_[0];
  tf.transform.translation.y = px4_utm_position_[1];
  tf.transform.translation.z = 0;
  tf.transform.rotation.w    = 1;
  tf.transform.rotation.x    = 0;
  tf.transform.rotation.y    = 0;
  tf.transform.rotation.z    = 0;
  v_transforms.push_back(tf);

  static_tf_broadcaster_->sendTransform(v_transforms);
}
//}

/* publishLocalOdomAndTF //{ */
void Odometry2::publishLocalOdomAndTF() {
  std::scoped_lock lock(px4_pose_mutex_);

  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }

  std::vector<geometry_msgs::msg::TransformStamped> v_transforms;

  geometry_msgs::msg::TransformStamped tf;

  tf.header.stamp = this->get_clock()->now();

  tf.header.frame_id         = ned_origin_frame_;
  tf.child_frame_id          = frd_fcu_frame_;
  tf.transform.translation.x = px4_position_[0];
  tf.transform.translation.y = px4_position_[1];
  tf.transform.translation.z = px4_position_[2];
  tf.transform.rotation.w    = px4_orientation_[0];
  tf.transform.rotation.x    = px4_orientation_[1];
  tf.transform.rotation.y    = px4_orientation_[2];
  tf.transform.rotation.z    = px4_orientation_[3];
  v_transforms.push_back(tf);

  tf_broadcaster_->sendTransform(v_transforms);

  // frd -> flu (enu) is rotation 180 degrees around x
  tf2::Quaternion q_orig, q_rot, q_new;

  q_orig.setW(px4_orientation_[0]);
  q_orig.setX(px4_orientation_[1]);
  q_orig.setY(px4_orientation_[2]);
  q_orig.setZ(px4_orientation_[3]);

  q_rot.setRPY(M_PI, 0, 0);
  q_new = q_orig * q_rot;
  q_new.normalize();

  geometry_msgs::msg::PoseStamped pose_ned, pose_enu;

  pose_ned.pose.position.x = px4_position_[0];
  pose_ned.pose.position.y = px4_position_[1];
  pose_ned.pose.position.z = px4_position_[2];
  tf2::convert(q_new, pose_ned.pose.orientation);

  tf2::doTransform(pose_ned, pose_enu, tf_world_to_ned_origin_frame_);

  nav_msgs::msg::Odometry msg;

  msg.header.stamp    = this->get_clock()->now();
  msg.header.frame_id = local_origin_frame_;
  msg.child_frame_id  = frd_fcu_frame_;
  msg.pose.pose       = pose_enu.pose;

  local_odom_publisher_->publish(msg);
}
//}

/*setInitialPx4Params//{*/
bool Odometry2::setInitialPx4Params() {
  for (const px4_int item : px4_params_int_) {
    auto request        = std::make_shared<fog_msgs::srv::SetPx4ParamInt::Request>();
    request->param_name = std::get<0>(item);
    request->value      = std::get<1>(item);
    RCLCPP_INFO(this->get_logger(), "[%s]: Setting %s, value: %d", this->get_name(), std::get<0>(item).c_str(), std::get<1>(item));
    auto call_result = set_px4_param_int_->async_send_request(request, std::bind(&Odometry2::setPx4IntParamCallback, this, std::placeholders::_1));
  }

  for (const px4_float item : px4_params_float_) {
    auto request        = std::make_shared<fog_msgs::srv::SetPx4ParamFloat::Request>();
    request->param_name = std::get<0>(item);
    request->value      = std::get<1>(item);
    RCLCPP_INFO(this->get_logger(), "[%s]: Setting %s, value: %f", this->get_name(), std::get<0>(item).c_str(), std::get<1>(item));
    auto call_result = set_px4_param_float_->async_send_request(request, std::bind(&Odometry2::setPx4FloatParamCallback, this, std::placeholders::_1));
  }

  return true;
}

/*//}*/

}  // namespace odometry2


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(odometry2::Odometry2)
