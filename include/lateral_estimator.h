#ifndef ODOMETRY2_LATERAL_ESTIMATOR_H
#define ODOMETRY2_LATERAL_ESTIMATOR_H

#include <ros/ros.h>

#include <mutex>
#include <string>
#include <vector>

#include <mrs_lib/lkf.h>

#include <types.h>

namespace odometry2
{

using statecov_t = lkf_lat_t::statecov_t;
using x_t        = lkf_lat_t::x_t;
using P_t        = lkf_lat_t::P_t;
using u_t        = lkf_lat_t::u_t;
using z_t        = lkf_lat_t::z_t;
using Q_t        = lkf_lat_t::Q_t;
using R_t        = lkf_lat_t::R_t;
using H_t        = lkf_lat_t::H_t;

class LateralEstimator {

public:
  LateralEstimator(const std::string &estimator_name, const Q_t &Q,
                 const std::vector<R_t> &R_multi);


  bool doPrediction(const double x, const double y, const double dt);
  bool doCorrection(const double x, const double y, int measurement_type);
  bool getStates(x_t &states);
  bool getState(int state_id, double &state);
  std::string getName(void);
  bool        setState(int state_id, const double &state);
  bool        setStates(const x_t &states);
  bool        setR(double cov, int measurement_type);
  bool        getR(double &cov, int measurement_type);
  bool        setQ(double cov, const Eigen::Vector2i &idx);
  bool        getQ(double &cov, const Eigen::Vector2i &idx);
  bool        getQ(double &cov, int diag);
  bool        setQ(double cov, int diag, const std::vector<int> &except);
  bool        reset(const x_t &states);

private:
  std::string                m_estimator_name;
  std::vector<bool>          m_fusing_measurement;
  Q_t                     m_Q;
  std::vector<H_t> m_H;
  std::vector<R_t>          m_R_arr;

  int    m_n_states;
  size_t m_n_measurement_types;

  std::unique_ptr<lkf_lat_t> mp_lkf;

  statecov_t m_sc;

  std::mutex mutex_lkf;

  bool m_is_initialized = false;
};

}  // namespace odometry2

#endif
