#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "copilot_core_types.h"
#include "copilot_core.h"

static float stream_time_cpy;
static float stream_throttle_cpy;
static float stream_dz_cpy;
static bool stream_requestedTakeoff_cpy;
static float stream_roll_cpy;
static float stream_dy_cpy;
static float stream_phi_cpy;
static float stream_dt_cpy;
static float stream_pitch_cpy;
static float stream_dx_cpy;
static float stream_theta_cpy;
static float stream_yaw_cpy;
static float stream_dpsi_cpy;
static float stream_dphi_cpy;
static float stream_dtheta_cpy;
static float s0[(1)] = {((float)(0.0f))};
static float s1[(1)] = {((float)(0.0f))};
static float s2[(1)] = {((float)(0.0f))};
static float s3[(1)] = {((float)(0.0f))};
static size_t s0_idx = (0);
static size_t s1_idx = (0);
static size_t s2_idx = (0);
static size_t s3_idx = (0);

static float s0_get(size_t x) {
  return (s0)[((s0_idx) + (x)) % ((size_t)(1))];
}

static float s1_get(size_t x) {
  return (s1)[((s1_idx) + (x)) % ((size_t)(1))];
}

static float s2_get(size_t x) {
  return (s2)[((s2_idx) + (x)) % ((size_t)(1))];
}

static float s3_get(size_t x) {
  return (s3)[((s3_idx) + (x)) % ((size_t)(1))];
}

static float s0_gen(void) {
  return ((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * (stream_dt_cpy)))) < ((float)(-25.0f))) ? (float)(-25.0f) : (((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * (stream_dt_cpy)))) > ((float)(25.0f))) ? (float)(25.0f) : (((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * (stream_dt_cpy)))));
}

static float s1_gen(void) {
  return ((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * (stream_dt_cpy)))) < ((float)(-25.0f))) ? (float)(-25.0f) : (((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * (stream_dt_cpy)))) > ((float)(25.0f))) ? (float)(25.0f) : (((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * (stream_dt_cpy)))));
}

static float s2_gen(void) {
  return ((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s2_get)((0))) + (((stream_yaw_cpy) - (stream_dpsi_cpy)) * (stream_dt_cpy)))) < ((float)(-25.0f))) ? (float)(-25.0f) : (((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s2_get)((0))) + (((stream_yaw_cpy) - (stream_dpsi_cpy)) * (stream_dt_cpy)))) > ((float)(25.0f))) ? (float)(25.0f) : (((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s2_get)((0))) + (((stream_yaw_cpy) - (stream_dpsi_cpy)) * (stream_dt_cpy)))));
}

static float s3_gen(void) {
  return (stream_yaw_cpy) - (stream_dpsi_cpy);
}

static bool setDemands_guard(void) {
  return true;
}

static float setDemands_arg0(void) {
  return ((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)));
}

static float setDemands_arg1(void) {
  return ((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy))) + (((float)(3.0e-3f)) * (((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * (stream_dt_cpy)))) < ((float)(-25.0f))) ? (float)(-25.0f) : (((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * (stream_dt_cpy)))) > ((float)(25.0f))) ? (float)(25.0f) : (((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * (stream_dt_cpy)))))))) - (((float)(5.0e-4f)) * (stream_dphi_cpy)));
}

static float setDemands_arg2(void) {
  return ((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy))) + (((float)(3.0e-3f)) * (((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * (stream_dt_cpy)))) < ((float)(-25.0f))) ? (float)(-25.0f) : (((((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * (stream_dt_cpy)))) > ((float)(25.0f))) ? (float)(25.0f) : (((((stream_time_cpy) > ((float)(1.0f))) ? ((float)(74.565f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(75.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * (stream_dt_cpy)))))))) - (((float)(5.0e-4f)) * (stream_dtheta_cpy)));
}

static float setDemands_arg3(void) {
  return ((((float)(3.0e-3f)) * ((stream_yaw_cpy) - (stream_dpsi_cpy))) + (((float)(5.0e-4f)) * ((s2_get)((0))))) - (((float)(1.5e-6f)) * ((((stream_yaw_cpy) - (stream_dpsi_cpy)) - ((s3_get)((0)))) / (stream_dt_cpy)));
}

void copilot_step_core(void) {
  float s0_tmp;
  float s1_tmp;
  float s2_tmp;
  float s3_tmp;
  float setDemands_arg_temp0;
  float setDemands_arg_temp1;
  float setDemands_arg_temp2;
  float setDemands_arg_temp3;
  (stream_time_cpy) = (stream_time);
  (stream_throttle_cpy) = (stream_throttle);
  (stream_dz_cpy) = (stream_dz);
  (stream_requestedTakeoff_cpy) = (stream_requestedTakeoff);
  (stream_roll_cpy) = (stream_roll);
  (stream_dy_cpy) = (stream_dy);
  (stream_phi_cpy) = (stream_phi);
  (stream_dt_cpy) = (stream_dt);
  (stream_pitch_cpy) = (stream_pitch);
  (stream_dx_cpy) = (stream_dx);
  (stream_theta_cpy) = (stream_theta);
  (stream_yaw_cpy) = (stream_yaw);
  (stream_dpsi_cpy) = (stream_dpsi);
  (stream_dphi_cpy) = (stream_dphi);
  (stream_dtheta_cpy) = (stream_dtheta);
  if ((setDemands_guard)()) {
    {(setDemands_arg_temp0) = ((setDemands_arg0)());
     (setDemands_arg_temp1) = ((setDemands_arg1)());
     (setDemands_arg_temp2) = ((setDemands_arg2)());
     (setDemands_arg_temp3) = ((setDemands_arg3)());
     (setDemands)((setDemands_arg_temp0), (setDemands_arg_temp1), (setDemands_arg_temp2), (setDemands_arg_temp3));}
  };
  (s0_tmp) = ((s0_gen)());
  (s1_tmp) = ((s1_gen)());
  (s2_tmp) = ((s2_gen)());
  (s3_tmp) = ((s3_gen)());
  ((s0)[s0_idx]) = (s0_tmp);
  ((s1)[s1_idx]) = (s1_tmp);
  ((s2)[s2_idx]) = (s2_tmp);
  ((s3)[s3_idx]) = (s3_tmp);
  (s0_idx) = (((s0_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s1_idx) = (((s1_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s2_idx) = (((s2_idx) + ((size_t)(1))) % ((size_t)(1)));
  (s3_idx) = (((s3_idx) + ((size_t)(1))) % ((size_t)(1)));
}
