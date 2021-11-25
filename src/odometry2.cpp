#include <rclcpp/callback_group.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include <fog_msgs/srv/get_px4_param_int.hpp>
#include <fog_msgs/srv/set_px4_param_int.hpp>
#include <fog_msgs/srv/get_px4_param_float.hpp>
#include <fog_msgs/srv/set_px4_param_float.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'

#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/string.hpp>

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
  std::atomic_bool is_initialized_      = false;

  // | ------------------------ TF frames ----------------------- |
  std::string uav_name_         = "";
  std::string world_frame_      = "";
  std::string ned_origin_frame_ = "";
  std::string ned_fcu_frame_    = "";
  std::string fcu_frame_        = "";

  // | ---------------------- TF variables ---------------------- |
  std::shared_ptr<tf2_ros::Buffer>                     tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::atomic_bool                                     publishing_static_tf_ = false;

  // | ---------------------- PX parameters --------------------- |
  std::vector<px4_int>   px4_params_int;
  std::vector<px4_float> px4_params_float;
  std::atomic_bool       set_initial_px4_params_ = false;
  std::atomic_bool       getting_pixhawk_odom_   = false;

  // | --------------------- GPS parameters --------------------- |
  std::atomic_bool getting_gps_ = false;
  float            pos_gps_[3];
  float            ori_gps_[4];

  // | ----------------------- Publishers ----------------------- |
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            local_odom_publisher_;

  // | ----------------------- Subscribers ---------------------- |
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr    pixhawk_odom_subscriber_;

  // | ------------------ Subscriber callbacks ------------------ |
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);

  // | ---------------- Service clients handlers ---------------- |
  bool setPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedFuture future);
  bool setPx4FloatParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedFuture future);

  // | --------------------- Service clients -------------------- |
  rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_;

  // | ------------------- Internal functions ------------------- |
  bool setInitialPx4Params();

  geometry_msgs::msg::PoseStamped transformBetween(std::string frame_from, std::string frame_to);

  void publishTF();
  void publishStaticTF();
  void publishLocalOdom();

  // | -------------------- Routine handling -------------------- |
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr     odometry_timer_;
  double                           odometry_loop_rate_;

  void odometryRoutine(void);

  // utils
  template <class T>
  bool parse_param(std::string param_name, T &param_dest);
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

  parse_param("odometry_loop_rate", odometry_loop_rate_);

  int   param_int;
  float param_float;
  parse_param("px4.EKF2_AID_MASK", param_int);
  px4_params_int.push_back(px4_int("EKF2_AID_MASK", param_int));
  parse_param("px4.EKF2_RNG_AID", param_int);
  px4_params_int.push_back(px4_int("EKF2_RNG_AID", param_int));
  parse_param("px4.EKF2_HGT_MODE", param_int);
  px4_params_int.push_back(px4_int("EKF2_HGT_MODE", param_int));

  parse_param("px4.EKF2_RNG_A_HMAX", param_float);
  px4_params_float.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));
  parse_param("px4.MPC_XY_CRUISE", param_float);
  px4_params_float.push_back(px4_float("MPC_XY_CRUISE", param_float));
  parse_param("px4.MC_YAWRATE_MAX", param_float);
  px4_params_float.push_back(px4_float("MC_YAWRATE_MAX", param_float));
  parse_param("px4.MPC_ACC_HOR", param_float);
  px4_params_float.push_back(px4_float("MPC_ACC_HOR", param_float));
  parse_param("px4.MPC_ACC_HOR_MAX", param_float);
  px4_params_float.push_back(px4_float("MPC_ACC_HOR_MAX", param_float));
  parse_param("px4.MPC_JERK_AUTO", param_float);
  px4_params_float.push_back(px4_float("MPC_JERK_AUTO", param_float));
  parse_param("px4.MPC_JERK_MAX", param_float);
  px4_params_float.push_back(px4_float("MPC_JERK_MAX", param_float));
  parse_param("px4.MPC_ACC_DOWN_MAX", param_float);
  px4_params_float.push_back(px4_float("MPC_ACC_DOWN_MAX", param_float));
  parse_param("px4.MPC_ACC_UP_MAX", param_float);
  px4_params_float.push_back(px4_float("MPC_ACC_UP_MAX", param_float));

  /* frame definition */
  world_frame_      = "world";
  fcu_frame_        = uav_name_ + "/fcu";
  ned_fcu_frame_    = uav_name_ + "/ned_fcu";
  ned_origin_frame_ = uav_name_ + "/ned_origin";
  //}

  // publishers
  local_odom_publisher_  = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", 10);

  // subscribers
  pixhawk_odom_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&Odometry2::pixhawkOdomCallback, this, _1));
  // service clients
  set_px4_param_int_   = this->create_client<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int");
  set_px4_param_float_ = this->create_client<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float");

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

/* pixhawkOdomCallback //{ */
void Odometry2::pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  pos_gps_[0] = msg->x;
  pos_gps_[1] = msg->y;
  pos_gps_[2] = msg->z;
  ori_gps_[0] = msg->q[0];
  ori_gps_[1] = msg->q[1];
  ori_gps_[2] = msg->q[2];
  ori_gps_[3] = msg->q[3];

  getting_pixhawk_odom_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting pixhawk odometry!", this->get_name());
}
//}

/* setPx4ParamIntCallback //{ */
bool Odometry2::setPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Response> result = future.get();
  if (result->success) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Parameter %s has been set to value: %d", this->get_name(), result->param_name.c_str(), result->value);
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

  if (is_initialized_ && getting_pixhawk_odom_) {

    std::chrono::time_point<std::chrono::high_resolution_clock> time_now = std::chrono::high_resolution_clock::now();

    if (!set_initial_px4_params_) {
      setInitialPx4Params();
      set_initial_px4_params_ = true;
    }

    if (static_tf_broadcaster_ == nullptr) {
      static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
      publishStaticTF();
      return;
    }

    publishTF();
    publishLocalOdom();

    std::chrono::duration<double, std::milli> dt = (std::chrono::high_resolution_clock::now() - time_now) / 1000;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Dt: %f milliseconds", dt.count() * 1000);

  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[%s]: Waiting for sensor initalization GPS_READY: %d", this->get_name(),
                         getting_gps_.load());
  }
}
//}

/* publishStaticTF //{ */
void Odometry2::publishStaticTF() {

  geometry_msgs::msg::TransformStamped tfs1, tfs2;
  tf2::Quaternion                      q;

  tfs1.header.stamp            = this->get_clock()->now();
  tfs1.header.frame_id         = ned_fcu_frame_;
  tfs1.child_frame_id          = fcu_frame_;
  tfs1.transform.translation.x = 0;
  tfs1.transform.translation.y = 0;
  tfs1.transform.translation.z = 0;
  q.setRPY(-M_PI, 0, 0);
  tfs1.transform.rotation.w = q.getW();
  tfs1.transform.rotation.x = q.getX();
  tfs1.transform.rotation.y = q.getY();
  tfs1.transform.rotation.z = q.getZ();
  static_tf_broadcaster_->sendTransform(tfs1);

  q.setRPY(M_PI, 0, M_PI / 2);
  q                            = q.inverse();
  tfs2.header.frame_id         = world_frame_;
  tfs2.child_frame_id          = ned_origin_frame_;
  tfs2.transform.translation.x = 0.0;
  tfs2.transform.translation.y = 0.0;
  tfs2.transform.translation.z = 0.0;
  tfs2.transform.rotation.x    = q.getX();
  tfs2.transform.rotation.y    = q.getY();
  tfs2.transform.rotation.z    = q.getZ();
  tfs2.transform.rotation.w    = q.getW();
  static_tf_broadcaster_->sendTransform(tfs2);

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

}  // namespace odometry2


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(odometry2::Odometry2)
