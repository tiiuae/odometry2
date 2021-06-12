#ifndef ODOMETRY2_LATERAL_ESTIMATOR_H
#define ODOMETRY2_LATERAL_ESTIMATOR_H

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>
#include <vector>

#include <mrs_lib/lkf.h>

#include <types.h>

#define LAT_DT 0.01
/* #define LAT_INPUT_COEFF 0.10 */
#define LAT_INPUT_COEFF 0.0

namespace odometry2
{

using lat_statecov_t = lkf_lat_t::statecov_t;
using lat_A_t        = lkf_lat_t::A_t;
using lat_B_t        = lkf_lat_t::B_t;
using lat_x_t        = lkf_lat_t::x_t;
using lat_P_t        = lkf_lat_t::P_t;
using lat_u_t        = lkf_lat_t::u_t;
using lat_z_t        = lkf_lat_t::z_t;
using lat_Q_t        = lkf_lat_t::Q_t;
using lat_R_t        = lkf_lat_t::R_t;
using lat_H_t        = lkf_lat_t::H_t;

class LateralEstimator {

public:
  LateralEstimator(const std::string &estimator_name, const lat_Q_t &Q,
                 const std::vector<lat_R_t> &R_multi);


  bool doPrediction(const double x, const double y, const double dt);
  bool doCorrection(const double x, const double y, int measurement_type);
  bool getStates(lat_x_t &states);
  bool getState(int state_id, double &state);
  std::string getName(void);
  bool        setState(int state_id, const double &state);
  bool        setStates(const lat_x_t &states);
  bool        setR(double cov, int measurement_type);
  bool        getR(double &cov, int measurement_type);
  bool        setQ(double cov, const Eigen::Vector2i &idx);
  bool        getQ(double &cov, const Eigen::Vector2i &idx);
  bool        getQ(double &cov, int diag);
  bool        setQ(double cov, int diag, const std::vector<int> &except);
  bool        reset(const lat_x_t &states);

private:
  std::string                m_estimator_name;
  std::vector<bool>          m_fusing_measurement;

  // State transition matrix
  lat_A_t m_A;

  // Input matrix
  lat_B_t m_B;

  // Input coefficient
  double m_b = LAT_INPUT_COEFF;

  std::vector<lat_H_t> m_H_multi;

  lat_Q_t                     m_Q;

  std::vector<lat_R_t>          m_R_multi;

  // Default dt
  double m_dt    = LAT_DT;
  double m_dt_sq = m_dt * m_dt / 2;

  int    m_n_states;
  size_t m_n_measurement_types;

  std::unique_ptr<lkf_lat_t> mp_lkf;

  lat_statecov_t m_sc;

  std::mutex mutex_lkf;

  bool m_is_initialized = false;
};

}  // namespace odometry2

#endif
