#pragma once
#ifndef ODOMETRY2_TYPES_H
#define ODOMETRY2_TYPES_H

#include <Eigen/Eigen>

#include <lib/lkf.h>

/* defines //{ */

// trackers

#define NULL_TRACKER "NullTracker"
#define LANDOFF_TRACKER "LandoffTracker"

// lateral
#define LAT_N_STATES 6
#define LAT_M_INPUTS 2
#define LAT_P_MEASUREMENTS 2

// altitude
#define ALT_N_STATES 3
#define ALT_M_INPUTS 1
#define ALT_P_MEASUREMENTS 1

// heading
#define HDG_N_STATES 3
#define HDG_M_INPUTS 2
#define HDG_P_MEASUREMENTS 1

// model states
#define STATE_POS 0
#define STATE_VEL 1
#define STATE_ACC 2

// lateral model states
#define LAT_POS_X 0
#define LAT_POS_Y 1
#define LAT_VEL_X 2
#define LAT_VEL_Y 3
#define LAT_ACC_X 4
#define LAT_ACC_Y 5

// altitude state corrections
#define ALT_GARMIN 0
#define ALT_BARO 1
#define ALT_IMU 2

// heading state corrections
#define HDG_HECTOR 0
#define HDG_GYRO 1

// lateral state corrections
#define LAT_HECTOR 0

/*//}*/

/* typedefs //{ */

typedef Eigen::Matrix<double, 2, 1> Vec2;

// lateral
typedef LKF<LAT_N_STATES, LAT_M_INPUTS, LAT_P_MEASUREMENTS> lkf_lat_t;
typedef lkf_lat_t::statecov_t                                        lat_statecov_t;
typedef lkf_lat_t::x_t                                               lat_x_t;
typedef lkf_lat_t::u_t                                               lat_u_t;
typedef lkf_lat_t::z_t                                               lat_z_t;
typedef lkf_lat_t::A_t                                               lat_A_t;
typedef lkf_lat_t::B_t                                               lat_B_t;
typedef lkf_lat_t::H_t                                               lat_H_t;
typedef lkf_lat_t::R_t                                               lat_R_t;
typedef lkf_lat_t::Q_t                                               lat_Q_t;
typedef lkf_lat_t::P_t                                               lat_P_t;

// altitude
typedef LKF<ALT_N_STATES, ALT_M_INPUTS, ALT_P_MEASUREMENTS> lkf_alt_t;
typedef lkf_alt_t::statecov_t                                        alt_statecov_t;
typedef lkf_alt_t::x_t                                               alt_x_t;
typedef lkf_alt_t::u_t                                               alt_u_t;
typedef lkf_alt_t::z_t                                               alt_z_t;
typedef lkf_alt_t::A_t                                               alt_A_t;
typedef lkf_alt_t::B_t                                               alt_B_t;
typedef lkf_alt_t::H_t                                               alt_H_t;
typedef lkf_alt_t::R_t                                               alt_R_t;
typedef lkf_alt_t::Q_t                                               alt_Q_t;
typedef lkf_alt_t::P_t                                               alt_P_t;

// heading
typedef LKF<HDG_N_STATES, HDG_M_INPUTS, HDG_P_MEASUREMENTS> lkf_hdg_t;
typedef lkf_hdg_t::statecov_t                                        hdg_statecov_t;
typedef lkf_hdg_t::x_t                                               hdg_x_t;
typedef lkf_hdg_t::u_t                                               hdg_u_t;
typedef lkf_hdg_t::z_t                                               hdg_z_t;
typedef lkf_hdg_t::A_t                                               hdg_A_t;
typedef lkf_hdg_t::B_t                                               hdg_B_t;
typedef lkf_hdg_t::H_t                                               hdg_H_t;
typedef lkf_hdg_t::R_t                                               hdg_R_t;
typedef lkf_hdg_t::Q_t                                               hdg_Q_t;
typedef lkf_hdg_t::P_t                                               hdg_P_t;

/*//}*/

#endif

