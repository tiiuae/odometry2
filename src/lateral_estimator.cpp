#include "lateral_estimator.h"

namespace odometry2
{

/*  //{ LateralEstimator() */

// clang-format off
LateralEstimator::LateralEstimator(
    const std::string &estimator_name,
    const lat_Q_t &Q,
    const std::vector<lat_R_t> &R_multi)
    :
    m_estimator_name(estimator_name),
    m_Q(Q),
    m_R_multi(R_multi)
  {

  // clang-format on

  // Number of states
  m_n_states = m_sc.x.size();

  // Number of measurement types
  m_n_measurement_types = 1;

  lat_H_t pos_H, vel_H, acc_H;
  pos_H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;
  m_H_multi = {pos_H};

  /*  //{ sanity checks */

  // Check size of m_Q
  if (m_Q.rows() != m_n_states) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".LateralEstimator()"
              << "): wrong size of \"Q.rows()\". Should be: " << m_n_states << " is:" << m_Q.rows() << std::endl;
    return;
  }

  if (m_Q.cols() != m_n_states) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".LateralEstimator()"
              << "): wrong size of \"Q.cols()\". Should be: " << m_n_states << " is:" << m_Q.cols() << std::endl;
    return;
  }

  // Check size of m_R_multi
  if (m_R_multi.size() != m_n_measurement_types) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".LateralEstimator()"
              << "): wrong size of \"m_R_multi\". Should be: " << m_n_measurement_types << " is:" << m_R_multi.size() << std::endl;
    return;
  }

  // Check size of m_R_multi elements
  for (size_t i = 0; i < m_R_multi.size(); i++) {
    if (m_R_multi[i].rows() != 2 || m_R_multi[i].cols() != 2) {
      std::cerr << "[LateralEstimator]: " << m_estimator_name << ".LateralEstimator()"
                << "): wrong size of \"m_R_multi[" << i << "]\". Should be: (2, " << 2 << ") is: (" << m_R_multi[i].rows() << ", " << m_R_multi[i].cols() << ")"
                << std::endl;
      return;
    }
  }


  //}

  // clang-format off
  m_A << 1, 0, m_dt, 0, m_dt_sq, 0,
         0, 1, 0, m_dt, 0, m_dt_sq,
         0, 0, 1, 0, m_dt, 0,
         0, 0, 0, 1, 0, m_dt,
         0, 0, 0, 0, 1-m_b, 0,
         0, 0, 0, 0, 0, 1-m_b;

  m_B << 0, 0, 
         0, 0, 
         0, 0, 
         0, 0, 
         m_b, 0,
         0, m_b;

  // clang-format on

  // Initialize all states to 0
  const lat_x_t        x0    = lat_x_t::Zero();
  lat_P_t              P_tmp = lat_P_t::Identity();
  const lat_P_t        P0    = 1000.0 * P_tmp;
  const lat_statecov_t sc0({x0, P0});
  m_sc                  = sc0;
  const lat_u_t      u0 = lat_u_t::Zero();
  const rclcpp::Time t0 = rclcpp::Time(0);

  // Initialize a single LKF
  mp_lkf = std::make_unique<lkf_lat_t>(m_A, m_B, pos_H);

  std::cout << "[LateralEstimator]: New LateralEstimator initialized " << std::endl;
  std::cout << "name: " << m_estimator_name << std::endl;

  std::cout << std::endl << " R_multi: " << std::endl;
  for (size_t i = 0; i < m_R_multi.size(); i++) {
    std::cout << m_R_multi[i] << std::endl;
  }
  std::cout << std::endl << " H_multi: " << std::endl;
  for (size_t i = 0; i < m_H_multi.size(); i++) {
    std::cout << m_H_multi[i] << std::endl;
  }
  std::cout << std::endl << " Q: " << std::endl << m_Q << std::endl;

  m_is_initialized = true;
}

//}

/*  //{ doPrediction() */

bool LateralEstimator::doPrediction(const double x, const double y, double dt) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(x)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".doPrediction(const double &x=" << x << ", double dt=" << dt
              << "): NaN detected in variable \"x\"." << std::endl;
    return false;
  }

  if (!std::isfinite(y)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".doPrediction(const double &y=" << y << ", double dt=" << dt
              << "): NaN detected in variable \"y\"." << std::endl;
    return false;
  }

  if (!std::isfinite(dt)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".doPrediction(): NaN detected in variable \"dt\"." << std::endl;
    return false;
  }

  // Check for non-positive dt
  if (dt <= 0) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".doPrediction(): \"dt\" should be > 0." << std::endl;
    return false;
  }

  //}

  lat_u_t u;
  u << x, y;

  /* lat_A_t newA = m_A; */
  /* newA(0, 2)  = dt; */
  /* newA(1, 3)  = dt; */
  /* newA(2, 4)  = dt; */
  /* newA(3, 5)  = dt; */
  /* newA(0, 4)  = std::pow(dt, 2); */
  /* newA(1, 5)  = std::pow(dt, 2); */

  {
    std::scoped_lock lock(mutex_lkf);

    try {
      // Apply the prediction step
      //
      /* std::cout << "before prediction: " << m_sc.x(0) << std::endl; */

      m_sc = mp_lkf->predict(m_sc, u, m_Q, dt);

      /* std::cout << "after prediction: " << m_sc.x(0) << std::endl; */
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      std::cerr << "[Odometry]: LKF prediction step failed: " << e.what();
    }
  }

  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[LateralEstimator]: prediction step" << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[LateralEstimator]: input  x:" << u_x << std::endl << "y: " << u_y << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[LateralEstimator]: m_Q:" << m_Q << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[LateralEstimator]: dt:" << dt << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[LateralEstimator]: state  x:" << sc_x.x << std::endl << "y: " << sc_y.x << std::endl << std::endl); */

  return true;
}

//}

/*  //{ doCorrection() */

bool LateralEstimator::doCorrection(const double x, const double y, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(x)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".doCorrection(const double x=" << x << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"x\"." << std::endl;
    return false;
  }

  if (!std::isfinite(y)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".doCorrection(const double y=" << y << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement_type)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".doCorrection(const double x=" << x << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"measurement_type\"." << std::endl;
    return false;
  }

  // Check for valid value of measurement
  if (measurement_type < 0) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".doCorrection(const double x=" << x << ", int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  lat_z_t z;
  z << x, y;
  lat_R_t R;
  R << m_R_multi[measurement_type];

  {
    std::scoped_lock lock(mutex_lkf);

    try {
      /* std::cout << "before correction: " << m_sc.x(0) << std::endl; */
      /* std::cout << "measurement: " << x << std::endl; */
      m_sc = mp_lkf->correct(m_sc, z, R);
      /* std::cout << "after correction: " << m_sc.x(0) << std::endl; */
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      std::cerr << "[LateralEstimator]: LKF correction step failed: " << e.what();
    }
  }

  /* if (measurement_type==1) { */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[LateralEstimator]: correction type: " << measurement_type << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[LateralEstimator]: corr  x:" << z_x << std::endl << "y: " << z_y << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[LateralEstimator]: R: " << R << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[LateralEstimator]: state  x:" << sc_x.x << std::endl << "y: " << sc_y.x << std::endl << std::endl); */
  /* } */

  return true;
}

//}

/*  //{ getStates() */

bool LateralEstimator::getStates(lat_x_t &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  std::scoped_lock lock(mutex_lkf);

  states = m_sc.x;

  std::cout << m_sc.P << std::endl;

  return true;
}

//}

/*  //{ getState() */

bool LateralEstimator::getState(int state_id, double &state) {

  /*  //{ sanity checks */

  if (!m_is_initialized) {
    std::cerr << "[LateralEstimator]: Not initialized" << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(state_id)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << "): NaN detected in variable \"state_id\"."
              << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    /* std::cout << "[LateralEstimator]: " << m_estimator_name << " getting value: " << mp_lkf_x->getState(state_id) << " of state: " << state_id << std::endl;
     */
    state = m_sc.x(state_id);
  }

  return true;
}

//}

/*  //{ getName() */

std::string LateralEstimator::getName(void) {
  return m_estimator_name;
}

//}

/*  //{ setState() */

bool LateralEstimator::setState(int state_id, const double &state) {

  /*  //{ sanity checks */

  if (!m_is_initialized) {
    /* ROS_WARN("[%s]: Lateral estimator %s method setState() called before init.", ros::this_node::getName().c_str(), m_estimator_name.c_str()); */
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(state)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state
              << "): NaN detected in variable \"state\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state_id)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x(state_id) = state;
  }
  /* ROS_INFO("[%s]: Set state %d of %s to x: %f y: %f. State after x: %f y: %f", ros::this_node::getName().c_str(), state_id, m_estimator_name.c_str(),
   * state(0), state(1), sc_x.x(state_id), sc_y.x(state_id)); */

  return true;
}

//}

/*  //{ setStates() */

bool LateralEstimator::setStates(const lat_x_t &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  {
    std::scoped_lock lock(mutex_lkf);
    m_sc.x = states;
  }

  return true;
}

//}

/*  //{ setR() */

bool LateralEstimator::setR(double cov, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > m_n_measurement_types || measurement_type < 0) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  /* double old_cov = m_R_multi[measurement_type](0, 0); */

  {
    std::scoped_lock lock(mutex_lkf);

    m_R_multi[measurement_type](0, 0) = cov;
  }

  /* std::cout << "[LateralEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type << ")" */
  /* << " Changed covariance from: " << old_cov << " to: " << m_R_multi[measurement_type](0, 0) << std::endl; */

  return true;
}

//}

/*  //{ getR() */

bool LateralEstimator::getR(double &cov, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement_type)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): NaN detected in variable \"measurement_type\"." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > m_n_measurement_types || measurement_type < 0) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    cov = m_R_multi[measurement_type](0, 0);
  }

  return true;
}

//}

/*  //{ getQ() */

bool LateralEstimator::getQ(double &cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"idx\" should be < " << m_n_states << "." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);
    
    cov = m_Q(idx(0), idx(1));
  }

  return true;
}

//}

/*  //{ setQ() */

bool LateralEstimator::setQ(double cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"idx\" should be < " << m_n_states << "." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_Q(idx(0), idx(1)) = cov;
  }

  /* std::cout << "[LateralEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type << ")" */
  /* << " Changed covariance from: " << old_cov << " to: " << m_Q_arr[measurement_type](0, 0) << std::endl; */

  return true;
}

//}

/*  //{ reset() */

bool LateralEstimator::reset(const lat_x_t &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;
  // Check size of states
  if ((int)states.rows() != m_n_states) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.rows()\". Should be: " << m_n_states << " is:" << states.rows() << std::endl;
    return false;
  }

  if (states.cols() != 2) {
    std::cerr << "[LateralEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.cols()\". Should be: " << 2 << " is:" << states.cols() << std::endl;
    return false;
  }

  // Check for NaNs
  for (int i = 0; i < states.rows(); i++) {
    for (int j = 0; j < states.cols(); j++) {
      if (!std::isfinite(states(i, j))) {
        std::cerr << "[LateralEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
                  << "): NaN detected in variable \"states(" << i << ", " << j << ")\"." << std::endl;
        return false;
      }
    }
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x = states;
  }

  return true;
}

//}

}  // namespace odometry2
