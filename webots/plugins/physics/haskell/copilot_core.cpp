#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "copilot_core_types.h"
#include "copilot_core.h"

static bool stream_completedTakeoff_cpy;
static float stream_throttle_cpy;
static float stream_dz_cpy;
static bool stream_requestedTakeoff_cpy;
static float stream_roll_cpy;
static float stream_dy_cpy;
static float stream_phi_cpy;
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
  return (((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))));
}

static float s1_gen(void) {
  return (((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))));
}

static float s2_gen(void) {
  return (((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s2_get)((0))) + (((stream_yaw_cpy) - (stream_dpsi_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s2_get)((0))) + (((stream_yaw_cpy) - (stream_dpsi_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s2_get)((0))) + (((stream_yaw_cpy) - (stream_dpsi_cpy)) * ((float)(1.0e-2f))))));
}

static float s3_gen(void) {
  return (stream_yaw_cpy) - (stream_dpsi_cpy);
}

static bool setMotors_guard(void) {
  return true;
}

static float setMotors_arg0(void) {
  return ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) - (((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy))) + (((float)(3.0e-3f)) * ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))))))) - (((float)(5.0e-4f)) * (stream_dphi_cpy))))) + (((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy))) + (((float)(3.0e-3f)) * ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))))))) - (((float)(5.0e-4f)) * (stream_dtheta_cpy))))) - (((((float)(3.0e-3f)) * ((stream_yaw_cpy) - (stream_dpsi_cpy))) + (((float)(5.0e-4f)) * ((s2_get)((0))))) - (((float)(1.5e-6f)) * ((((stream_yaw_cpy) - (stream_dpsi_cpy)) - ((s3_get)((0)))) / ((float)(1.0e-2f)))));
}

static float setMotors_arg1(void) {
  return ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) - (((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy))) + (((float)(3.0e-3f)) * ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))))))) - (((float)(5.0e-4f)) * (stream_dphi_cpy))))) - (((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy))) + (((float)(3.0e-3f)) * ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))))))) - (((float)(5.0e-4f)) * (stream_dtheta_cpy))))) + (((((float)(3.0e-3f)) * ((stream_yaw_cpy) - (stream_dpsi_cpy))) + (((float)(5.0e-4f)) * ((s2_get)((0))))) - (((float)(1.5e-6f)) * ((((stream_yaw_cpy) - (stream_dpsi_cpy)) - ((s3_get)((0)))) / ((float)(1.0e-2f)))));
}

static float setMotors_arg2(void) {
  return ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) + (((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy))) + (((float)(3.0e-3f)) * ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))))))) - (((float)(5.0e-4f)) * (stream_dphi_cpy))))) + (((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy))) + (((float)(3.0e-3f)) * ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))))))) - (((float)(5.0e-4f)) * (stream_dtheta_cpy))))) + (((((float)(3.0e-3f)) * ((stream_yaw_cpy) - (stream_dpsi_cpy))) + (((float)(5.0e-4f)) * ((s2_get)((0))))) - (((float)(1.5e-6f)) * ((((stream_yaw_cpy) - (stream_dpsi_cpy)) - ((s3_get)((0)))) / ((float)(1.0e-2f)))));
}

static float setMotors_arg3(void) {
  return ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) + (((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy))) + (((float)(3.0e-3f)) * ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s0_get)((0))) + (((((float)(10.0f)) * ((stream_roll_cpy) - (stream_dy_cpy))) - (stream_phi_cpy)) * ((float)(1.0e-2f))))))))) - (((float)(5.0e-4f)) * (stream_dphi_cpy))))) - (((float)(50.0f)) * (((((float)(2.0e-3f)) * ((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy))) + (((float)(3.0e-3f)) * ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) < ((float)(-25.0f))) ? (float)(-25.0f) : ((((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))) > ((float)(25.0f))) ? (float)(25.0f) : ((((stream_completedTakeoff_cpy) ? ((float)(55.385f)) + (((float)(2.0f)) * ((stream_throttle_cpy) - (stream_dz_cpy))) : ((stream_requestedTakeoff_cpy) ? (float)(56.0f) : ((float)(0.0f)))) < ((float)(6.0e-2f))) ? (float)(0.0f) : (((s1_get)((0))) + (((((float)(10.0f)) * ((stream_pitch_cpy) - (stream_dx_cpy))) - (stream_theta_cpy)) * ((float)(1.0e-2f))))))))) - (((float)(5.0e-4f)) * (stream_dtheta_cpy))))) - (((((float)(3.0e-3f)) * ((stream_yaw_cpy) - (stream_dpsi_cpy))) + (((float)(5.0e-4f)) * ((s2_get)((0))))) - (((float)(1.5e-6f)) * ((((stream_yaw_cpy) - (stream_dpsi_cpy)) - ((s3_get)((0)))) / ((float)(1.0e-2f)))));
}

void copilot_step_core(void) {
  float s0_tmp;
  float s1_tmp;
  float s2_tmp;
  float s3_tmp;
  float setMotors_arg_temp0;
  float setMotors_arg_temp1;
  float setMotors_arg_temp2;
  float setMotors_arg_temp3;
  (stream_completedTakeoff_cpy) = (stream_completedTakeoff);
  (stream_throttle_cpy) = (stream_throttle);
  (stream_dz_cpy) = (stream_dz);
  (stream_requestedTakeoff_cpy) = (stream_requestedTakeoff);
  (stream_roll_cpy) = (stream_roll);
  (stream_dy_cpy) = (stream_dy);
  (stream_phi_cpy) = (stream_phi);
  (stream_pitch_cpy) = (stream_pitch);
  (stream_dx_cpy) = (stream_dx);
  (stream_theta_cpy) = (stream_theta);
  (stream_yaw_cpy) = (stream_yaw);
  (stream_dpsi_cpy) = (stream_dpsi);
  (stream_dphi_cpy) = (stream_dphi);
  (stream_dtheta_cpy) = (stream_dtheta);
  if ((setMotors_guard)()) {
    {(setMotors_arg_temp0) = ((setMotors_arg0)());
     (setMotors_arg_temp1) = ((setMotors_arg1)());
     (setMotors_arg_temp2) = ((setMotors_arg2)());
     (setMotors_arg_temp3) = ((setMotors_arg3)());
     (setMotors)((setMotors_arg_temp0), (setMotors_arg_temp1), (setMotors_arg_temp2), (setMotors_arg_temp3));}
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
