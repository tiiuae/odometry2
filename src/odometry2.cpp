#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <fog_msgs/srv/get_px4_param_int.hpp>
#include <fog_msgs/srv/set_px4_param_int.hpp>
#include <fog_msgs/srv/get_px4_param_float.hpp>
#include <fog_msgs/srv/set_px4_param_float.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

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
  std::atomic_bool is_initialized_ = false;

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

  // | ---------------------- PX parameters --------------------- |
  std::vector<px4_int>   px4_params_int_;
  std::vector<px4_float> px4_params_float_;
  std::atomic_bool       set_initial_px4_params_ = false;
  std::atomic_bool       getting_pixhawk_odom_   = false;

  float      px4_position_[3];
  float      px4_orientation_[4];
  std::mutex px4_pose_mutex_;

  // | ----------------------- Publishers ----------------------- |
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom_publisher_;

  // | ----------------------- Subscribers ---------------------- |
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr pixhawk_odom_subscriber_;

  // | --------------------- Service clients -------------------- |
  rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_;

  // | ------------------ Subscriber callbacks ------------------ |
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);

  // | ---------------- Service clients handlers ---------------- |
  bool setPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedFuture future);
  bool setPx4FloatParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedFuture future);

  // | ------------------- Internal functions ------------------- |
  bool setInitialPx4Params();

  geometry_msgs::msg::PoseStamped transformBetween(const std::string &frame_from, const std::string &frame_to);

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
  bool parse_param(const std::string &param_name, T &param_dest);
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

  loaded_successfully &= parse_param("odometry_loop_rate", odometry_loop_rate_);

  int   param_int;
  float param_float;

  loaded_successfully &= parse_param("px4.EKF2_AID_MASK", param_int);
  px4_params_int_.push_back(px4_int("EKF2_AID_MASK", param_int));

  loaded_successfully &= parse_param("px4.EKF2_RNG_AID", param_int);
  px4_params_int_.push_back(px4_int("EKF2_RNG_AID", param_int));

  loaded_successfully &= parse_param("px4.EKF2_HGT_MODE", param_int);
  px4_params_int_.push_back(px4_int("EKF2_HGT_MODE", param_int));

  loaded_successfully &= parse_param("px4.EKF2_RNG_A_HMAX", param_float);
  px4_params_float_.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));

  loaded_successfully &= parse_param("world_frame", world_frame_);

  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(this->get_logger(), str);
    rclcpp::shutdown();
    return;
  }
  //}

  /* frame definition */
  fcu_frame_        = uav_name_ + "/fcu";
  ned_fcu_frame_    = uav_name_ + "/ned_fcu";
  ned_origin_frame_ = uav_name_ + "/ned_origin";

  // publishers
  local_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", 10);

  // subscribers
  pixhawk_odom_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&Odometry2::pixhawkOdomCallback, this, _1));
  // service clients
  set_px4_param_int_   = this->create_client<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int");
  set_px4_param_float_ = this->create_client<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float");

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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

    publishTF();
    publishLocalOdom();

  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[%s]: Waiting for PX4 odometry.", this->get_name());
  }
}
//}

/* publishStaticTF //{ */
void Odometry2::publishStaticTF() {
  std::vector<geometry_msgs::msg::TransformStamped> v_transforms;

  geometry_msgs::msg::TransformStamped tf;
  tf2::Quaternion                      q;

  tf.header.stamp            = this->get_clock()->now();
  tf.header.frame_id         = ned_fcu_frame_;
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

  tf.header.frame_id         = world_frame_;
  tf.child_frame_id          = ned_origin_frame_;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  q.setRPY(M_PI, 0, M_PI / 2);
  q                       = q.inverse();
  tf.transform.rotation.x = q.getX();
  tf.transform.rotation.y = q.getY();
  tf.transform.rotation.z = q.getZ();
  tf.transform.rotation.w = q.getW();
  v_transforms.push_back(tf);

  static_tf_broadcaster_->sendTransform(v_transforms);
}
//}

/* publishTF //{ */
void Odometry2::publishTF() {
  std::scoped_lock lock(px4_pose_mutex_);

  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }

  geometry_msgs::msg::TransformStamped tf;

  tf.header.stamp            = this->get_clock()->now();
  tf.header.frame_id         = ned_origin_frame_;
  tf.child_frame_id          = ned_fcu_frame_;
  tf.transform.translation.x = px4_position_[0];
  tf.transform.translation.y = px4_position_[1];
  tf.transform.translation.z = px4_position_[2];
  tf.transform.rotation.w    = px4_orientation_[0];
  tf.transform.rotation.x    = px4_orientation_[1];
  tf.transform.rotation.y    = px4_orientation_[2];
  tf.transform.rotation.z    = px4_orientation_[3];
  /* RCLCPP_INFO_STREAM(this->get_logger(), px4_position_[0] << ", "<< px4_position_[1] << ", "<< px4_position_[2]); */
  tf_broadcaster_->sendTransform(tf);
}
//}

/* publishLocalOdom //{ */
void Odometry2::publishLocalOdom() {
  const geometry_msgs::msg::PoseStamped tf = transformBetween(fcu_frame_, world_frame_);

  nav_msgs::msg::Odometry msg;

  msg.header.stamp            = this->get_clock()->now();
  msg.header.frame_id         = world_frame_;
  msg.child_frame_id          = fcu_frame_;
  msg.pose.pose.position.x    = tf.pose.position.x;
  msg.pose.pose.position.y    = tf.pose.position.y;
  msg.pose.pose.position.z    = tf.pose.position.z;
  msg.pose.pose.orientation.x = tf.pose.orientation.x;
  msg.pose.pose.orientation.y = tf.pose.orientation.y;
  msg.pose.pose.orientation.z = tf.pose.orientation.z;
  msg.pose.pose.orientation.w = tf.pose.orientation.w;

  /* RCLCPP_INFO_STREAM(this->get_logger(), tf.pose.position.x << ", "<< tf.pose.position.y << ", "<< tf.pose.position.z); */
  /* RCLCPP_INFO_STREAM(this->get_logger(), "-------------------------------"); */
  local_odom_publisher_->publish(msg);
}
//}

/* transformBetween //{ */
geometry_msgs::msg::PoseStamped Odometry2::transformBetween(const std::string &frame_from, const std::string &frame_to) {
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

/* parse_param //{ */
template <class T>
bool Odometry2::parse_param(const std::string &param_name, T &param_dest) {
  this->declare_parameter(param_name);
  if (!this->get_parameter(param_name, param_dest)) {
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
