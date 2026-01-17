/*
 * rover_sw_pwrtrain_24b.cpp
 *
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * Code generation for model "rover_sw_pwrtrain_24b".
 *
 * Model version              : 1.108
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C++ source code generated on : Fri Jan 16 23:21:35 2026
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Apple->ARM64
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "rover_sw_pwrtrain_24b.h"
#include "rtwtypes.h"
#include "rover_sw_pwrtrain_24b_types.h"
#include <string.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include <math.h>
#include "rover_sw_pwrtrain_24b_private.h"
#include "rmw/qos_profiles.h"
#include <stddef.h>
#include "rt_defines.h"

/* Named constants for Chart: '<S35>/adaptive_throttle_contller' */
const uint8_T rove_IN_COLLISION_FLT_FAST_STOP = 1U;
const uint8_T rover_sw_IN_LOST_COMM_OPERATION = 3U;
const uint8_T rover_sw_p_IN_LOSS_COMM_MONITOR = 2U;
const uint8_T rover_sw_pw_IN_NORMAL_OPERATION = 4U;
const uint8_T rover_sw_pwr_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T rover_sw_pwrtrain_24b_IN_MILD = 1U;
const uint8_T rover_sw_pwrtrain_24b_IN_SEVERE = 2U;
const uint8_T rover_sw_pwrtrain_IN_FAST_DECEL = 1U;
const uint8_T rover_sw_pwrtrain_IN_SLOW_ACCEL = 2U;
const uint8_T rover_sw_pwrtrain_IN_SLOW_DECEL = 2U;

/* user code (top of source file) */
int vesc_open_noargs(void);
void vesc_close(void);
int vesc_send_duty(double duty);
int vesc_poll(float* v_in, float* t_mos, float* t_motor);

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
void rover_sw_pwrtrain_24b::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  rover_sw_pwrtrain_24b_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  rover_sw_pwrtrain_24b_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  rover_sw_pwrtrain_24b_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * System initialize for enable system:
 *    '<S14>/Enabled Subsystem'
 *    '<S15>/Enabled Subsystem'
 *    '<S16>/Enabled Subsystem'
 */
void rover_sw_pwrtrain_24b::rover_sw__EnabledSubsystem_Init
  (B_EnabledSubsystem_rover_sw_p_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S18>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Float32));
}

/*
 * Output and update for enable system:
 *    '<S14>/Enabled Subsystem'
 *    '<S15>/Enabled Subsystem'
 *    '<S16>/Enabled Subsystem'
 */
void rover_sw_pwrtrain_24b::rover_sw_pwrtr_EnabledSubsystem(boolean_T rtu_Enable,
  const SL_Bus_std_msgs_Float32 *rtu_In1, B_EnabledSubsystem_rover_sw_p_T
  *localB)
{
  /* Outputs for Enabled SubSystem: '<S14>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S18>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S18>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S14>/Enabled Subsystem' */
}

void rover_sw_pwrtrain_24b::rover_sw_p_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[18] = "/ps4/left_stick_y";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S14>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 18; i++) {
    /* Start for MATLABSystem: '<S14>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_me[i] = b_zeroDelimTopic[i];
  }

  Sub_rover_sw_pwrtrain_24b_390.createSubscriber
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_me[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rover_sw_Subscriber_setupImpl_m(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[19] = "/ps4/right_stick_x";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S15>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 19; i++) {
    /* Start for MATLABSystem: '<S15>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_g[i] = b_zeroDelimTopic[i];
  }

  Sub_rover_sw_pwrtrain_24b_391.createSubscriber
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_g[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rover_sw_Publisher_setupImpl_my(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[11];
  static const char_T b_zeroDelimTopic_0[11] = "/led_state";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S12>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline_p.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_p.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_p, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 11; i++) {
    /* Start for MATLABSystem: '<S12>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_246.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rover_sw__Publisher_setupImpl_m(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "vesc/telemetry/status";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S11>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 22; i++) {
    /* Start for MATLABSystem: '<S11>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_cv[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_826.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_cv[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rover_s_Subscriber_setupImpl_my(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[16] = "/front_distance";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S16>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 16; i++) {
    /* Start for MATLABSystem: '<S16>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_n[i] = b_zeroDelimTopic[i];
  }

  Sub_rover_sw_pwrtrain_24b_392.createSubscriber
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_n[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rover__Subscriber_setupImpl_myg(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[18] = "/auto_mode_enable";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S17>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 18; i++) {
    /* Start for MATLABSystem: '<S17>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_g1[i] = b_zeroDelimTopic[i];
  }

  Sub_rover_sw_pwrtrain_24b_521.createSubscriber
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_g1[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rover_sw_pw_Publisher_setupImpl(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[12];
  static const char_T b_zeroDelimTopic_0[12] = "/decay_mode";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S10>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<S10>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_907.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rover_s_Publisher_setupImpl_myg(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[20] = "/throttel_regulated";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S13>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 20; i++) {
    /* Start for MATLABSystem: '<S13>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_f[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_903.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_f[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rover__Publisher_setupImpl_mygo(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[27] = "/vesc/last_cmd_status_left";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S43>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 27; i++) {
    /* Start for MATLABSystem: '<S43>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_b[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_138.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_b[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rover_Publisher_setupImpl_mygou(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[28] = "/vesc/last_cmd_status_right";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S44>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 28; i++) {
    /* Start for MATLABSystem: '<S44>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_c[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_139.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_c[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rove_Publisher_setupImpl_mygou5(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[27] = "/vesc/last_cmd_status_left";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S39>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 27; i++) {
    /* Start for MATLABSystem: '<S39>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_cx[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_542.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_cx[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rov_Publisher_setupImpl_mygou5b(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[28] = "/vesc/last_cmd_status_right";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S40>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 28; i++) {
    /* Start for MATLABSystem: '<S40>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_m[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_543.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_m[0], qos_profile);
}

void rover_sw_pwrtrain_24b::DifferentialDriveKinematics_set
  (robotics_slmobile_internal_bl_T *obj)
{
  static const char_T b[23] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p', 'e',
    'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e' };

  for (int32_T i = 0; i < 23; i++) {
    /* Start for MATLABSystem: '<S47>/MATLAB System' */
    obj->KinModel.VehicleInputsInternal[i] = b[i];
  }

  /* Start for MATLABSystem: '<S47>/MATLAB System' */
  obj->KinModel.TrackWidth = obj->TrackWidth;
  obj->KinModel.WheelRadius = obj->WheelRadius;
  obj->KinModel.WheelSpeedRange[0] = obj->WheelSpeedRange[0];
  obj->KinModel.WheelSpeedRange[1] = obj->WheelSpeedRange[1];
}

void rover_sw_pwrtrain_24b::ro_Publisher_setupImpl_mygou5bo(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[31] = "vesc/telemetry/voltage/average";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S55>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 31; i++) {
    /* Start for MATLABSystem: '<S55>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_818.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic[0], qos_profile);
}

void rover_sw_pwrtrain_24b::r_Publisher_setupImpl_mygou5bow(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[24] = "vesc/telemetry/voltages";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S56>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 24; i++) {
    /* Start for MATLABSystem: '<S56>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_p[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_822.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_p[0], qos_profile);
}

void rover_sw_pwrtrain_24b::Publisher_setupImpl_mygou5bowb(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[27] = "vesc/telemetry/voltage_one";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S57>/SinkBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 27; i++) {
    /* Start for MATLABSystem: '<S57>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_k[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_835.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_k[0], qos_profile);
}

real_T rover_sw_pwrtrain_24b::rover_sw_pwrtrain_24b_norm(const real_T x[2])
{
  real_T y;
  rover_sw_pwrtrain_24b_B.scale = 3.3121686421112381E-170;

  /* Start for MATLABSystem: '<S25>/Pure Pursuit' */
  rover_sw_pwrtrain_24b_B.absxk = fabs(x[0]);
  if (rover_sw_pwrtrain_24b_B.absxk > 3.3121686421112381E-170) {
    y = 1.0;
    rover_sw_pwrtrain_24b_B.scale = rover_sw_pwrtrain_24b_B.absxk;
  } else {
    rover_sw_pwrtrain_24b_B.t = rover_sw_pwrtrain_24b_B.absxk /
      3.3121686421112381E-170;
    y = rover_sw_pwrtrain_24b_B.t * rover_sw_pwrtrain_24b_B.t;
  }

  /* Start for MATLABSystem: '<S25>/Pure Pursuit' */
  rover_sw_pwrtrain_24b_B.absxk = fabs(x[1]);
  if (rover_sw_pwrtrain_24b_B.absxk > rover_sw_pwrtrain_24b_B.scale) {
    rover_sw_pwrtrain_24b_B.t = rover_sw_pwrtrain_24b_B.scale /
      rover_sw_pwrtrain_24b_B.absxk;
    y = y * rover_sw_pwrtrain_24b_B.t * rover_sw_pwrtrain_24b_B.t + 1.0;
    rover_sw_pwrtrain_24b_B.scale = rover_sw_pwrtrain_24b_B.absxk;
  } else {
    rover_sw_pwrtrain_24b_B.t = rover_sw_pwrtrain_24b_B.absxk /
      rover_sw_pwrtrain_24b_B.scale;
    y += rover_sw_pwrtrain_24b_B.t * rover_sw_pwrtrain_24b_B.t;
  }

  y = rover_sw_pwrtrain_24b_B.scale * sqrt(y);

  /* Start for MATLABSystem: '<S25>/Pure Pursuit' */
  if (rtIsNaN(y)) {
    int32_T c_k;
    c_k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (c_k < 2) {
        if (rtIsNaN(x[c_k])) {
          exitg1 = 1;
        } else {
          c_k++;
        }
      } else {
        y = (rtInf);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return y;
}

real_T rover_sw_pwrtrain_24b::rover_sw_pwr_closestPointOnLine(const real_T pt1[2],
  real_T pt2[2], const real_T refPt[2])
{
  real_T distance;
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;

  /* Start for MATLABSystem: '<S25>/Pure Pursuit' */
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(pt1[b_k] == pt2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  if (p) {
    pt2[0] = pt1[0];
    rover_sw_pwrtrain_24b_B.refPt[0] = refPt[0] - pt1[0];
    pt2[1] = pt1[1];
    rover_sw_pwrtrain_24b_B.refPt[1] = refPt[1] - pt1[1];
    distance = rover_sw_pwrtrain_24b_norm(rover_sw_pwrtrain_24b_B.refPt);
  } else {
    rover_sw_pwrtrain_24b_B.alpha = pt2[0] - pt1[0];
    rover_sw_pwrtrain_24b_B.v12 = (pt2[0] - refPt[0]) *
      rover_sw_pwrtrain_24b_B.alpha;
    rover_sw_pwrtrain_24b_B.v12_d = rover_sw_pwrtrain_24b_B.alpha *
      rover_sw_pwrtrain_24b_B.alpha;
    rover_sw_pwrtrain_24b_B.alpha = pt2[1] - pt1[1];
    rover_sw_pwrtrain_24b_B.alpha = ((pt2[1] - refPt[1]) *
      rover_sw_pwrtrain_24b_B.alpha + rover_sw_pwrtrain_24b_B.v12) /
      (rover_sw_pwrtrain_24b_B.alpha * rover_sw_pwrtrain_24b_B.alpha +
       rover_sw_pwrtrain_24b_B.v12_d);
    if (rover_sw_pwrtrain_24b_B.alpha > 1.0) {
      pt2[0] = pt1[0];
      pt2[1] = pt1[1];
    } else if (!(rover_sw_pwrtrain_24b_B.alpha < 0.0)) {
      pt2[0] = (1.0 - rover_sw_pwrtrain_24b_B.alpha) * pt2[0] +
        rover_sw_pwrtrain_24b_B.alpha * pt1[0];
      pt2[1] = (1.0 - rover_sw_pwrtrain_24b_B.alpha) * pt2[1] +
        rover_sw_pwrtrain_24b_B.alpha * pt1[1];
    }

    rover_sw_pwrtrain_24b_B.refPt[0] = refPt[0] - pt2[0];
    rover_sw_pwrtrain_24b_B.refPt[1] = refPt[1] - pt2[1];
    distance = rover_sw_pwrtrain_24b_norm(rover_sw_pwrtrain_24b_B.refPt);
  }

  /* End of Start for MATLABSystem: '<S25>/Pure Pursuit' */
  return distance;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void rover_sw_pwrtrain_24b::step()
{
  /* local block i/o variables */
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_p;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_m;
  SL_Bus_std_msgs_Bool rtb_SourceBlock_o2_l_0;
  boolean_T searchFlag;
  static const char_T a[23] = { 'W', 'h', 'e', 'e', 'l', 'S', 'p', 'e', 'e', 'd',
    's', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-' };

  static const char_T a_0[23] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e' };

  SL_Bus_std_msgs_Float32 rtb_BusAssignment1_a5;
  SL_Bus_std_msgs_Float32 rtb_BusAssignment2_h;
  SL_Bus_std_msgs_Int32 rtb_BusAssignment1;
  SL_Bus_std_msgs_Int32 rtb_BusAssignment1_j;
  SL_Bus_std_msgs_Int32 rtb_BusAssignment2_f;
  SL_Bus_std_msgs_Int32 rtb_BusAssignment5;
  int32_T tmp_0;
  int32_T tmp_size_idx_0;
  int32_T trueCount;
  int8_T rtAction;
  boolean_T exitg1;
  boolean_T rtb_OR;
  boolean_T tmp;
  if (rtmIsMajorTimeStep((&rover_sw_pwrtrain_24b_M))) {
    /* set solver stop time */
    if (!((&rover_sw_pwrtrain_24b_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
                            (((&rover_sw_pwrtrain_24b_M)->Timing.clockTickH0 + 1)
        * (&rover_sw_pwrtrain_24b_M)->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
                            (((&rover_sw_pwrtrain_24b_M)->Timing.clockTick0 + 1)
        * (&rover_sw_pwrtrain_24b_M)->Timing.stepSize0 +
        (&rover_sw_pwrtrain_24b_M)->Timing.clockTickH0 *
        (&rover_sw_pwrtrain_24b_M)->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&rover_sw_pwrtrain_24b_M))) {
    (&rover_sw_pwrtrain_24b_M)->Timing.t[0] = rtsiGetT
      (&(&rover_sw_pwrtrain_24b_M)->solverInfo);
  }

  tmp = rtmIsMajorTimeStep((&rover_sw_pwrtrain_24b_M));
  if (tmp) {
    /* MATLABSystem: '<S14>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_mk =
      Sub_rover_sw_pwrtrain_24b_390.getLatestMessage(&rtb_SourceBlock_o2_m);

    /* Outputs for Enabled SubSystem: '<S14>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_mk,
      &rtb_SourceBlock_o2_m, &rover_sw_pwrtrain_24b_B.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S14>/Enabled Subsystem' */

    /* MATLABSystem: '<S15>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_n =
      Sub_rover_sw_pwrtrain_24b_391.getLatestMessage(&rtb_SourceBlock_o2_p);

    /* Outputs for Enabled SubSystem: '<S15>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_n,
      &rtb_SourceBlock_o2_p, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_o);

    /* End of Outputs for SubSystem: '<S15>/Enabled Subsystem' */

    /* Outputs for Atomic SubSystem: '<S22>/status_indicator_lamp' */
    /* Switch: '<S29>/Switch' incorporates:
     *  Logic: '<S29>/OR'
     */
    if ((rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data != 0.0F) ||
        (rover_sw_pwrtrain_24b_B.EnabledSubsystem_o.In1.data != 0.0F)) {
      /* BusAssignment: '<S3>/Bus Assignment3' incorporates:
       *  Constant: '<S29>/Constant'
       *  DataTypeConversion: '<S29>/Data Type Conversion1'
       */
      rover_sw_pwrtrain_24b_B.BusAssignment3.data = 1300;
    } else {
      /* BusAssignment: '<S3>/Bus Assignment3' incorporates:
       *  Constant: '<S29>/Constant1'
       *  DataTypeConversion: '<S29>/Data Type Conversion1'
       */
      rover_sw_pwrtrain_24b_B.BusAssignment3.data = 1500;
    }

    /* End of Switch: '<S29>/Switch' */
    /* End of Outputs for SubSystem: '<S22>/status_indicator_lamp' */

    /* MATLABSystem: '<S12>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_246.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment3);

    /* MATLAB Function: '<S27>/MATLAB Function' */
    rover_sw_pwrtrain_24b_B.voltages[0] = 0.0F;
    rover_sw_pwrtrain_24b_B.current_motor[0] = 0.0F;
    rover_sw_pwrtrain_24b_B.current_battery[0] = 0.0F;
    rover_sw_pwrtrain_24b_B.rpm[0] = 0.0F;
    rover_sw_pwrtrain_24b_B.fault_codes[0] = 0U;
    rover_sw_pwrtrain_24b_B.voltages[1] = 0.0F;
    rover_sw_pwrtrain_24b_B.current_motor[1] = 0.0F;
    rover_sw_pwrtrain_24b_B.current_battery[1] = 0.0F;
    rover_sw_pwrtrain_24b_B.rpm[1] = 0.0F;
    rover_sw_pwrtrain_24b_B.fault_codes[1] = 0U;
    rover_sw_pwrtrain_24b_B.voltages[2] = 0.0F;
    rover_sw_pwrtrain_24b_B.current_motor[2] = 0.0F;
    rover_sw_pwrtrain_24b_B.current_battery[2] = 0.0F;
    rover_sw_pwrtrain_24b_B.rpm[2] = 0.0F;
    rover_sw_pwrtrain_24b_B.fault_codes[2] = 0U;
    rover_sw_pwrtrain_24b_B.voltages[3] = 0.0F;
    rover_sw_pwrtrain_24b_B.current_motor[3] = 0.0F;
    rover_sw_pwrtrain_24b_B.current_battery[3] = 0.0F;
    rover_sw_pwrtrain_24b_B.rpm[3] = 0.0F;
    rover_sw_pwrtrain_24b_B.fault_codes[3] = 0U;

    /* BusAssignment: '<S3>/Bus Assignment2' incorporates:
     *  MATLAB Function: '<S27>/MATLAB Function'
     */
    rover_sw_pwrtrain_24b_B.BusAssignment2.data = 0;
    get_telemetry_wrapper(&rover_sw_pwrtrain_24b_B.voltages[0],
                          &rover_sw_pwrtrain_24b_B.current_motor[0],
                          &rover_sw_pwrtrain_24b_B.current_battery[0],
                          &rover_sw_pwrtrain_24b_B.rpm[0],
                          &rover_sw_pwrtrain_24b_B.fault_codes[0],
                          &rover_sw_pwrtrain_24b_B.BusAssignment2.data);

    /* MATLABSystem: '<S11>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_826.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment2);

    /* MATLABSystem: '<S16>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_m =
      Sub_rover_sw_pwrtrain_24b_392.getLatestMessage(&rtb_SourceBlock_o2);

    /* Outputs for Enabled SubSystem: '<S16>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_m,
      &rtb_SourceBlock_o2, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_p);

    /* End of Outputs for SubSystem: '<S16>/Enabled Subsystem' */

    /* Switch: '<S4>/Switch' */
    if (rover_sw_pwrtrain_24b_B.SourceBlock_o1_m) {
      /* Switch: '<S4>/Switch' incorporates:
       *  Abs: '<S4>/Abs'
       */
      rover_sw_pwrtrain_24b_B.Switch = static_cast<real32_T>(fabs
        (static_cast<real_T>(rover_sw_pwrtrain_24b_B.EnabledSubsystem_p.In1.data)));
    } else {
      /* Switch: '<S4>/Switch' incorporates:
       *  Constant: '<S4>/Constant'
       */
      rover_sw_pwrtrain_24b_B.Switch = 20000.0;
    }

    /* End of Switch: '<S4>/Switch' */

    /* Sum: '<S30>/Add' incorporates:
     *  Gain: '<S30>/1-alpha'
     *  Gain: '<S30>/alpha'
     *  UnitDelay: '<S30>/Unit Delay'
     */
    rover_sw_pwrtrain_24b_B.Add = 0.4 * rover_sw_pwrtrain_24b_B.Switch + 0.6 *
      rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE;

    /* Switch: '<S30>/Switch' incorporates:
     *  Constant: '<S30>/Constant'
     *  Constant: '<S30>/Constant2'
     *  Constant: '<S31>/Constant'
     *  Constant: '<S32>/Constant'
     *  RelationalOperator: '<S31>/Compare'
     *  RelationalOperator: '<S32>/Compare'
     *  Switch: '<S30>/Switch1'
     */
    if (rover_sw_pwrtrain_24b_B.Add <= 50.0) {
      rover_sw_pwrtrain_24b_B.Switch_b = MILD;
    } else if (rover_sw_pwrtrain_24b_B.Add <= 80.0) {
      /* Switch: '<S30>/Switch1' incorporates:
       *  Constant: '<S30>/Constant1'
       */
      rover_sw_pwrtrain_24b_B.Switch_b = SEVERE;
    } else {
      rover_sw_pwrtrain_24b_B.Switch_b = NONE;
    }

    /* End of Switch: '<S30>/Switch' */

    /* MATLABSystem: '<S17>/SourceBlock' */
    searchFlag = Sub_rover_sw_pwrtrain_24b_521.getLatestMessage
      (&rtb_SourceBlock_o2_l_0);

    /* Logic: '<S4>/OR' incorporates:
     *  MATLABSystem: '<S17>/SourceBlock'
     * */
    rtb_OR = (rover_sw_pwrtrain_24b_B.SourceBlock_o1_mk ||
              rover_sw_pwrtrain_24b_B.SourceBlock_o1_n || searchFlag);

    /* Chart: '<S35>/adaptive_throttle_contller' incorporates:
     *  Constant: '<S35>/Constant'
     */
    if (rover_sw_pwrtrain_24b_DW.temporalCounter_i1 < 1023) {
      rover_sw_pwrtrain_24b_DW.temporalCounter_i1 = static_cast<uint16_T>
        (rover_sw_pwrtrain_24b_DW.temporalCounter_i1 + 1);
    }

    if (rover_sw_pwrtrain_24b_DW.is_active_c3_rover_sw_pwrtrain_ == 0) {
      rover_sw_pwrtrain_24b_DW.is_active_c3_rover_sw_pwrtrain_ = 1U;
      if (rover_sw_pwrtrain_24b_B.Switch_b != NONE) {
        rover_sw_pwrtrain_24b_DW.temporalCounter_i1 = 0U;
        rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
          rove_IN_COLLISION_FLT_FAST_STOP;
        rover_sw_pwrtrain_24b_DW.collision_time = 0.0;
        rover_sw_pwrtrain_24b_DW.comm_loss_time = 0.0;
        rover_sw_pwrtrain_24b_DW.phase2_time = 0.0;
        rover_sw_pwrtrain_24b_B.decay_mode = 1.0;
        rover_sw_pwrtrain_24b_DW.is_COLLISION_FLT_FAST_STOP =
          rover_sw_pwrtrain_24b_IN_MILD;
      } else if (!rtb_OR) {
        rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
          rover_sw_p_IN_LOSS_COMM_MONITOR;
        rover_sw_pwrtrain_24b_DW.loss_tmr = 0.0;
        rover_sw_pwrtrain_24b_DW.comm_ref_throttle =
          rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i;
      } else {
        rover_sw_pwrtrain_24b_DW.temporalCounter_i1 = 0U;
        rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
          rover_sw_pw_IN_NORMAL_OPERATION;
        rover_sw_pwrtrain_24b_B.decay_mode = 0.0;
        rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION =
          rover_sw_pwrtrain_IN_SLOW_ACCEL;

        /*  Slower acceleration */
        rover_sw_pwrtrain_24b_DW.max_increase = 0.005;
      }
    } else {
      switch (rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b) {
       case rove_IN_COLLISION_FLT_FAST_STOP:
        if (((rover_sw_pwrtrain_24b_DW.temporalCounter_i1 >= 1000) &&
             (rover_sw_pwrtrain_24b_B.Switch_b == NONE)) ||
            (rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data <= -0.1)) {
          rover_sw_pwrtrain_24b_DW.is_COLLISION_FLT_FAST_STOP =
            rover_sw_pwr_IN_NO_ACTIVE_CHILD;
          rover_sw_pwrtrain_24b_DW.temporalCounter_i1 = 0U;
          rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
            rover_sw_pw_IN_NORMAL_OPERATION;
          rover_sw_pwrtrain_24b_B.decay_mode = 0.0;
          rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION =
            rover_sw_pwrtrain_IN_SLOW_ACCEL;

          /*  Slower acceleration */
          rover_sw_pwrtrain_24b_DW.max_increase = 0.005;
        } else {
          rover_sw_pwrtrain_24b_DW.collision_time =
            rover_sw_pwrtrain_24b_DW.collision_time + 0.01 <= 2.5 ?
            rover_sw_pwrtrain_24b_DW.collision_time + 0.01 : 2.5;
          if (rover_sw_pwrtrain_24b_DW.is_COLLISION_FLT_FAST_STOP ==
              rover_sw_pwrtrain_24b_IN_MILD) {
            rover_sw_pwrtrain_24b_DW.is_COLLISION_FLT_FAST_STOP =
              rover_sw_pwrtrain_24b_IN_SEVERE;
            rover_sw_pwrtrain_24b_DW.severe_time = 0.0;
          } else {
            /* case IN_SEVERE: */
            rover_sw_pwrtrain_24b_DW.severe_time += 0.01;
            rover_sw_pwrtrain_24b_B.throttle_out = exp(-1.2 *
              rover_sw_pwrtrain_24b_DW.severe_time) *
              rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i <= 0.0 ? 0.0 : exp
              (-1.2 * rover_sw_pwrtrain_24b_DW.severe_time) *
              rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i;
          }
        }
        break;

       case rover_sw_p_IN_LOSS_COMM_MONITOR:
        if (rover_sw_pwrtrain_24b_DW.loss_tmr > 0.5) {
          rover_sw_pwrtrain_24b_DW.temporalCounter_i1 = 0U;
          rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
            rover_sw_IN_LOST_COMM_OPERATION;
          rover_sw_pwrtrain_24b_DW.comm_loss_time = 0.0;
          rover_sw_pwrtrain_24b_DW.is_LOST_COMM_OPERATION =
            rover_sw_pwrtrain_IN_SLOW_DECEL;
        } else if (rtb_OR) {
          rover_sw_pwrtrain_24b_DW.temporalCounter_i1 = 0U;
          rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
            rover_sw_pw_IN_NORMAL_OPERATION;
          rover_sw_pwrtrain_24b_B.decay_mode = 0.0;
          rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION =
            rover_sw_pwrtrain_IN_SLOW_ACCEL;

          /*  Slower acceleration */
          rover_sw_pwrtrain_24b_DW.max_increase = 0.005;
        } else {
          rover_sw_pwrtrain_24b_DW.loss_tmr += 0.01;
          rover_sw_pwrtrain_24b_B.throttle_out =
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i;
        }
        break;

       case rover_sw_IN_LOST_COMM_OPERATION:
        if ((rover_sw_pwrtrain_24b_B.Switch_b != NONE) && rtb_OR) {
          rover_sw_pwrtrain_24b_DW.is_LOST_COMM_OPERATION =
            rover_sw_pwr_IN_NO_ACTIVE_CHILD;
          rover_sw_pwrtrain_24b_DW.temporalCounter_i1 = 0U;
          rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
            rove_IN_COLLISION_FLT_FAST_STOP;
          rover_sw_pwrtrain_24b_DW.collision_time = 0.0;
          rover_sw_pwrtrain_24b_DW.comm_loss_time = 0.0;
          rover_sw_pwrtrain_24b_DW.phase2_time = 0.0;
          rover_sw_pwrtrain_24b_B.decay_mode = 1.0;
          rover_sw_pwrtrain_24b_DW.is_COLLISION_FLT_FAST_STOP =
            rover_sw_pwrtrain_24b_IN_MILD;
        } else if ((rover_sw_pwrtrain_24b_DW.temporalCounter_i1 >= 1000) &&
                   (!rtb_OR)) {
          rover_sw_pwrtrain_24b_DW.is_LOST_COMM_OPERATION =
            rover_sw_pwr_IN_NO_ACTIVE_CHILD;
          rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
            rover_sw_p_IN_LOSS_COMM_MONITOR;
          rover_sw_pwrtrain_24b_DW.loss_tmr = 0.0;
          rover_sw_pwrtrain_24b_DW.comm_ref_throttle =
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i;
        } else {
          rover_sw_pwrtrain_24b_B.decay_mode = 2.0;
          if (rover_sw_pwrtrain_24b_DW.is_LOST_COMM_OPERATION ==
              rover_sw_pwrtrain_IN_FAST_DECEL) {
            rover_sw_pwrtrain_24b_DW.phase2_time += 0.01;
            rover_sw_pwrtrain_24b_B.throttle_out = exp(-0.8 *
              rover_sw_pwrtrain_24b_DW.phase2_time) *
              rover_sw_pwrtrain_24b_DW.comm_ref_throttle;

            /* case IN_SLOW_DECEL: */
          } else if (rover_sw_pwrtrain_24b_DW.comm_loss_time > 20.0) {
            rover_sw_pwrtrain_24b_DW.is_LOST_COMM_OPERATION =
              rover_sw_pwrtrain_IN_FAST_DECEL;
            rover_sw_pwrtrain_24b_DW.phase2_time = 0.0;
          } else {
            rover_sw_pwrtrain_24b_DW.comm_loss_time += 0.01;
            rover_sw_pwrtrain_24b_B.throttle_out = (1.0 - 0.15 *
              rover_sw_pwrtrain_24b_DW.comm_loss_time) *
              rover_sw_pwrtrain_24b_DW.comm_ref_throttle <= 0.0 ? 0.0 : (1.0 -
              0.15 * rover_sw_pwrtrain_24b_DW.comm_loss_time) *
              rover_sw_pwrtrain_24b_DW.comm_ref_throttle;
          }
        }
        break;

       default:
        /* case IN_NORMAL_OPERATION: */
        if (rover_sw_pwrtrain_24b_B.Switch_b != NONE) {
          rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION =
            rover_sw_pwr_IN_NO_ACTIVE_CHILD;
          rover_sw_pwrtrain_24b_DW.temporalCounter_i1 = 0U;
          rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
            rove_IN_COLLISION_FLT_FAST_STOP;
          rover_sw_pwrtrain_24b_DW.collision_time = 0.0;
          rover_sw_pwrtrain_24b_DW.comm_loss_time = 0.0;
          rover_sw_pwrtrain_24b_DW.phase2_time = 0.0;
          rover_sw_pwrtrain_24b_B.decay_mode = 1.0;
          rover_sw_pwrtrain_24b_DW.is_COLLISION_FLT_FAST_STOP =
            rover_sw_pwrtrain_24b_IN_MILD;
        } else if ((rover_sw_pwrtrain_24b_DW.temporalCounter_i1 >= 1000) &&
                   (!rtb_OR)) {
          rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION =
            rover_sw_pwr_IN_NO_ACTIVE_CHILD;
          rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
            rover_sw_p_IN_LOSS_COMM_MONITOR;
          rover_sw_pwrtrain_24b_DW.loss_tmr = 0.0;
          rover_sw_pwrtrain_24b_DW.comm_ref_throttle =
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i;
        } else if (rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION ==
                   rover_sw_pwrtrain_IN_FAST_DECEL) {
          /*  20% per cycle */
          rover_sw_pwrtrain_24b_B.throttle_out =
            rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data >=
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i -
            rover_sw_pwrtrain_24b_DW.max_decrease ? static_cast<real_T>
            (rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data) :
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i -
              rover_sw_pwrtrain_24b_DW.max_decrease;

          /* case IN_SLOW_ACCEL: */
        } else if (rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data <=
                   rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i) {
          rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION =
            rover_sw_pwrtrain_IN_FAST_DECEL;

          /*  Faster deceleration allowed */
          rover_sw_pwrtrain_24b_DW.max_decrease = 0.05;
        } else {
          /*  1% per cycle */
          rover_sw_pwrtrain_24b_B.throttle_out =
            rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data <=
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i +
            rover_sw_pwrtrain_24b_DW.max_increase ? static_cast<real_T>
            (rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data) :
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i +
              rover_sw_pwrtrain_24b_DW.max_increase;
        }
        break;
      }
    }

    /* End of Chart: '<S35>/adaptive_throttle_contller' */

    /* DataTypeConversion: '<S3>/Data Type Conversion1' */
    rover_sw_pwrtrain_24b_B.Add = floor(rover_sw_pwrtrain_24b_B.decay_mode);
    if (rtIsNaN(rover_sw_pwrtrain_24b_B.Add) || rtIsInf
        (rover_sw_pwrtrain_24b_B.Add)) {
      rover_sw_pwrtrain_24b_B.Add = 0.0;
    } else {
      rover_sw_pwrtrain_24b_B.Add = fmod(rover_sw_pwrtrain_24b_B.Add,
        4.294967296E+9);
    }

    /* BusAssignment: '<S3>/Bus Assignment1' incorporates:
     *  DataTypeConversion: '<S3>/Data Type Conversion1'
     */
    rtb_BusAssignment1.data = rover_sw_pwrtrain_24b_B.Add < 0.0 ? -static_cast<
      int32_T>(static_cast<uint32_T>(-rover_sw_pwrtrain_24b_B.Add)) :
      static_cast<int32_T>(static_cast<uint32_T>(rover_sw_pwrtrain_24b_B.Add));

    /* MATLABSystem: '<S10>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_907.publish(&rtb_BusAssignment1);

    /* DataTypeConversion: '<S3>/Data Type Conversion5' */
    rover_sw_pwrtrain_24b_B.Add = floor(rover_sw_pwrtrain_24b_B.throttle_out);
    if (rtIsNaN(rover_sw_pwrtrain_24b_B.Add) || rtIsInf
        (rover_sw_pwrtrain_24b_B.Add)) {
      rover_sw_pwrtrain_24b_B.Add = 0.0;
    } else {
      rover_sw_pwrtrain_24b_B.Add = fmod(rover_sw_pwrtrain_24b_B.Add,
        4.294967296E+9);
    }

    /* BusAssignment: '<S3>/Bus Assignment5' incorporates:
     *  DataTypeConversion: '<S3>/Data Type Conversion5'
     */
    rtb_BusAssignment5.data = rover_sw_pwrtrain_24b_B.Add < 0.0 ? -static_cast<
      int32_T>(static_cast<uint32_T>(-rover_sw_pwrtrain_24b_B.Add)) :
      static_cast<int32_T>(static_cast<uint32_T>(rover_sw_pwrtrain_24b_B.Add));

    /* MATLABSystem: '<S13>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_903.publish(&rtb_BusAssignment5);

    /* Outputs for Enabled SubSystem: '<S17>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S21>/Enable'
     */
    /* Start for MATLABSystem: '<S17>/SourceBlock' */
    if (searchFlag) {
      /* SignalConversion generated from: '<S21>/In1' */
      rover_sw_pwrtrain_24b_B.In1 = rtb_SourceBlock_o2_l_0;
    }

    /* End of Outputs for SubSystem: '<S17>/Enabled Subsystem' */
  }

  /* Integrator: '<S47>/Integrator' */
  rover_sw_pwrtrain_24b_B.Integrator[0] =
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[0];
  rover_sw_pwrtrain_24b_B.Integrator[1] =
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[1];
  rover_sw_pwrtrain_24b_B.Integrator[2] =
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[2];
  if (tmp) {
    /* MATLABSystem: '<S25>/Pure Pursuit' incorporates:
     *  Constant: '<S26>/Constant4'
     */
    if (rover_sw_pwrtrain_24b_DW.obj.DesiredLinearVelocity != 0.5) {
      rover_sw_pwrtrain_24b_DW.obj.DesiredLinearVelocity = 0.5;
    }

    if (rover_sw_pwrtrain_24b_DW.obj.MaxAngularVelocity != 1.0) {
      rover_sw_pwrtrain_24b_DW.obj.MaxAngularVelocity = 1.0;
    }

    if (rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance != 0.1) {
      rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance = 0.1;
    }

    searchFlag = false;
    rtb_OR = true;
    rover_sw_pwrtrain_24b_B.ret = 0;
    exitg1 = false;
    while ((!exitg1) && (rover_sw_pwrtrain_24b_B.ret < 1756)) {
      if ((rover_sw_pwrtrain_24b_DW.obj.WaypointsInternal[rover_sw_pwrtrain_24b_B.ret]
           ==
           rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret])
          || (rtIsNaN
              (rover_sw_pwrtrain_24b_DW.obj.WaypointsInternal[rover_sw_pwrtrain_24b_B.ret])
              && rtIsNaN
              (rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret])))
      {
        rover_sw_pwrtrain_24b_B.ret++;
      } else {
        rtb_OR = false;
        exitg1 = true;
      }
    }

    if (rtb_OR) {
      searchFlag = true;
    }

    if (!searchFlag) {
      memcpy(&rover_sw_pwrtrain_24b_DW.obj.WaypointsInternal[0],
             &rover_sw_pwrtrain_24b_ConstP.Constant4_Value[0], 1756U * sizeof
             (real_T));
      rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex = 0.0;
    }

    for (rover_sw_pwrtrain_24b_B.ret = 0; rover_sw_pwrtrain_24b_B.ret < 1756;
         rover_sw_pwrtrain_24b_B.ret++) {
      rover_sw_pwrtrain_24b_B.b[rover_sw_pwrtrain_24b_B.ret] = !rtIsNaN
        (rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret]);
    }

    trueCount = 0;

    /* MATLABSystem: '<S25>/Pure Pursuit' */
    for (rover_sw_pwrtrain_24b_B.ret = 0; rover_sw_pwrtrain_24b_B.ret < 878;
         rover_sw_pwrtrain_24b_B.ret++) {
      searchFlag = (rover_sw_pwrtrain_24b_B.b[rover_sw_pwrtrain_24b_B.ret] &&
                    rover_sw_pwrtrain_24b_B.b[rover_sw_pwrtrain_24b_B.ret + 878]);
      rover_sw_pwrtrain_24b_B.bv[rover_sw_pwrtrain_24b_B.ret] = searchFlag;
      if (searchFlag) {
        trueCount++;
      }
    }

    tmp_size_idx_0 = trueCount;
    trueCount = 0;
    for (rover_sw_pwrtrain_24b_B.ret = 0; rover_sw_pwrtrain_24b_B.ret < 878;
         rover_sw_pwrtrain_24b_B.ret++) {
      /* MATLABSystem: '<S25>/Pure Pursuit' */
      if (rover_sw_pwrtrain_24b_B.bv[rover_sw_pwrtrain_24b_B.ret]) {
        /* Start for MATLABSystem: '<S25>/Pure Pursuit' */
        rover_sw_pwrtrain_24b_B.tmp_data[trueCount] = static_cast<int16_T>
          (rover_sw_pwrtrain_24b_B.ret);
        trueCount++;
      }
    }

    /* MATLABSystem: '<S25>/Pure Pursuit' incorporates:
     *  Constant: '<S26>/Constant4'
     *  ZeroOrderHold: '<S25>/Zero-Order Hold2'
     */
    if (tmp_size_idx_0 == 0) {
      /* Product: '<S48>/Product' */
      rover_sw_pwrtrain_24b_B.Product = 0.0;
      rover_sw_pwrtrain_24b_B.Product1 = 0.0;
    } else {
      searchFlag = false;
      if (rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex == 0.0) {
        searchFlag = true;
        rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [0]];
        rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [0] + 878];
        rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex = 1.0;
      }

      if (tmp_size_idx_0 == 1) {
        rover_sw_pwrtrain_24b_B.minDistance =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [0]];
        rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] =
          rover_sw_pwrtrain_24b_B.minDistance;
        rover_sw_pwrtrain_24b_B.Add =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [0] + 878];
        rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] =
          rover_sw_pwrtrain_24b_B.Add;
        rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] =
          rover_sw_pwrtrain_24b_B.minDistance;
        rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] =
          rover_sw_pwrtrain_24b_B.Add;
      } else {
        rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1]];
        rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1] + 878];
        rover_sw_pwrtrain_24b_B.minDistance = rover_sw_pwr_closestPointOnLine
          (rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint,
           rover_sw_pwrtrain_24b_B.lookaheadStartPt,
           &rover_sw_pwrtrain_24b_B.Integrator[0]);
        rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[0];
        rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[0] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] -
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1]];
        rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[1];
        rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[1] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] -
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1] + 878];
        rover_sw_pwrtrain_24b_B.Add = rover_sw_pwrtrain_24b_norm
          (rover_sw_pwrtrain_24b_B.lookaheadStartPt_l);
        rover_sw_pwrtrain_24b_B.lookaheadIdx =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex + 1.0;
        trueCount = static_cast<int32_T>((1.0 -
          (rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex + 1.0)) + (
          static_cast<real_T>(tmp_size_idx_0) - 1.0)) - 1;
        rover_sw_pwrtrain_24b_B.b_i = 0;
        exitg1 = false;
        while ((!exitg1) && (rover_sw_pwrtrain_24b_B.b_i <= trueCount)) {
          rover_sw_pwrtrain_24b_B.i = rover_sw_pwrtrain_24b_B.lookaheadIdx +
            static_cast<real_T>(rover_sw_pwrtrain_24b_B.b_i);
          if ((!searchFlag) && (rover_sw_pwrtrain_24b_B.Add >
                                rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance))
          {
            exitg1 = true;
          } else {
            rover_sw_pwrtrain_24b_B.ret = rover_sw_pwrtrain_24b_B.tmp_data[
              static_cast<int32_T>(rover_sw_pwrtrain_24b_B.i + 1.0) - 1];
            tmp_0 = rover_sw_pwrtrain_24b_B.tmp_data[static_cast<int32_T>
              (rover_sw_pwrtrain_24b_B.i) - 1];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[0] =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[tmp_0] -
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret];
            rover_sw_pwrtrain_24b_B.dv[0] =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[tmp_0];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret
              + 878];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp_j =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[tmp_0 + 878];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[1] =
              rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp_j -
              rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp;
            rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] =
              rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp;
            rover_sw_pwrtrain_24b_B.dv[1] =
              rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp_j;
            rover_sw_pwrtrain_24b_B.Add += rover_sw_pwrtrain_24b_norm
              (rover_sw_pwrtrain_24b_B.lookaheadStartPt_l);
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp =
              rover_sw_pwr_closestPointOnLine(rover_sw_pwrtrain_24b_B.dv,
              rover_sw_pwrtrain_24b_B.lookaheadStartPt,
              &rover_sw_pwrtrain_24b_B.Integrator[0]);
            if (rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp <
                rover_sw_pwrtrain_24b_B.minDistance) {
              rover_sw_pwrtrain_24b_B.minDistance =
                rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp;
              rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] =
                rover_sw_pwrtrain_24b_B.lookaheadStartPt[0];
              rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] =
                rover_sw_pwrtrain_24b_B.lookaheadStartPt[1];
              rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex =
                rover_sw_pwrtrain_24b_B.i;
            }

            rover_sw_pwrtrain_24b_B.b_i++;
          }
        }

        rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[0] =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] -
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1]];
        rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[1] =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] -
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1] + 878];
        rover_sw_pwrtrain_24b_B.Add = rover_sw_pwrtrain_24b_norm
          (rover_sw_pwrtrain_24b_B.lookaheadStartPt_l);
        rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0];
        rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1]];
        rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1];
        rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1] + 878];
        rover_sw_pwrtrain_24b_B.minDistance = rover_sw_pwrtrain_24b_B.Add -
          rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance;
        rover_sw_pwrtrain_24b_B.lookaheadIdx =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex;
        while ((rover_sw_pwrtrain_24b_B.minDistance < 0.0) &&
               (rover_sw_pwrtrain_24b_B.lookaheadIdx < static_cast<real_T>
                (tmp_size_idx_0) - 1.0)) {
          rover_sw_pwrtrain_24b_B.lookaheadIdx++;
          trueCount = rover_sw_pwrtrain_24b_B.tmp_data[static_cast<int32_T>
            (rover_sw_pwrtrain_24b_B.lookaheadIdx) - 1];
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[trueCount];
          rover_sw_pwrtrain_24b_B.ret = rover_sw_pwrtrain_24b_B.tmp_data[
            static_cast<int32_T>(rover_sw_pwrtrain_24b_B.lookaheadIdx + 1.0) - 1];
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret];
          rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[0] =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[trueCount] -
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret];
          rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[trueCount + 878];
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] =
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp;
          rover_sw_pwrtrain_24b_B.minDistance =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret
            + 878];
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] =
            rover_sw_pwrtrain_24b_B.minDistance;
          rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[1] =
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp -
            rover_sw_pwrtrain_24b_B.minDistance;
          rover_sw_pwrtrain_24b_B.Add += rover_sw_pwrtrain_24b_norm
            (rover_sw_pwrtrain_24b_B.lookaheadStartPt_l);
          rover_sw_pwrtrain_24b_B.minDistance = rover_sw_pwrtrain_24b_B.Add -
            rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance;
        }

        rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[0] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] -
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0];
        rover_sw_pwrtrain_24b_B.lookaheadStartPt_l[1] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] -
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1];
        rover_sw_pwrtrain_24b_B.Add = rover_sw_pwrtrain_24b_B.minDistance /
          rover_sw_pwrtrain_24b_norm(rover_sw_pwrtrain_24b_B.lookaheadStartPt_l);
        if (rover_sw_pwrtrain_24b_B.Add > 0.0) {
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] = (1.0 -
            rover_sw_pwrtrain_24b_B.Add) *
            rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] +
            rover_sw_pwrtrain_24b_B.Add *
            rover_sw_pwrtrain_24b_B.lookaheadStartPt[0];
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] = (1.0 -
            rover_sw_pwrtrain_24b_B.Add) *
            rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] +
            rover_sw_pwrtrain_24b_B.Add *
            rover_sw_pwrtrain_24b_B.lookaheadStartPt[1];
        }
      }

      rover_sw_pwrtrain_24b_B.Add = rt_atan2d_snf
        (rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] -
         rover_sw_pwrtrain_24b_B.Integrator[1],
         rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] -
         rover_sw_pwrtrain_24b_B.Integrator[0]) -
        rover_sw_pwrtrain_24b_B.Integrator[2];
      if (fabs(rover_sw_pwrtrain_24b_B.Add) > 3.1415926535897931) {
        if (rtIsNaN(rover_sw_pwrtrain_24b_B.Add + 3.1415926535897931) || rtIsInf
            (rover_sw_pwrtrain_24b_B.Add + 3.1415926535897931)) {
          rover_sw_pwrtrain_24b_B.minDistance = (rtNaN);
        } else if (rover_sw_pwrtrain_24b_B.Add + 3.1415926535897931 == 0.0) {
          rover_sw_pwrtrain_24b_B.minDistance = 0.0;
        } else {
          rover_sw_pwrtrain_24b_B.minDistance = fmod(rover_sw_pwrtrain_24b_B.Add
            + 3.1415926535897931, 6.2831853071795862);
          searchFlag = (rover_sw_pwrtrain_24b_B.minDistance == 0.0);
          if (!searchFlag) {
            rover_sw_pwrtrain_24b_B.lookaheadIdx = fabs
              ((rover_sw_pwrtrain_24b_B.Add + 3.1415926535897931) /
               6.2831853071795862);
            searchFlag = !(fabs(rover_sw_pwrtrain_24b_B.lookaheadIdx - floor
                                (rover_sw_pwrtrain_24b_B.lookaheadIdx + 0.5)) >
                           2.2204460492503131E-16 *
                           rover_sw_pwrtrain_24b_B.lookaheadIdx);
          }

          if (searchFlag) {
            rover_sw_pwrtrain_24b_B.minDistance = 0.0;
          } else if (rover_sw_pwrtrain_24b_B.minDistance < 0.0) {
            rover_sw_pwrtrain_24b_B.minDistance += 6.2831853071795862;
          }
        }

        if ((rover_sw_pwrtrain_24b_B.minDistance == 0.0) &&
            (rover_sw_pwrtrain_24b_B.Add + 3.1415926535897931 > 0.0)) {
          rover_sw_pwrtrain_24b_B.minDistance = 6.2831853071795862;
        }

        rover_sw_pwrtrain_24b_B.Add = rover_sw_pwrtrain_24b_B.minDistance -
          3.1415926535897931;
      }

      rover_sw_pwrtrain_24b_B.Product1 = 2.0 * sin(rover_sw_pwrtrain_24b_B.Add) /
        rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance;
      if (rtIsNaN(rover_sw_pwrtrain_24b_B.Product1)) {
        rover_sw_pwrtrain_24b_B.Product1 = 0.0;
      }

      if (fabs(fabs(rover_sw_pwrtrain_24b_B.Add) - 3.1415926535897931) <
          1.4901161193847656E-8) {
        if (rtIsNaN(rover_sw_pwrtrain_24b_B.Product1)) {
          rover_sw_pwrtrain_24b_B.Product1 = (rtNaN);
        } else if (rover_sw_pwrtrain_24b_B.Product1 < 0.0) {
          rover_sw_pwrtrain_24b_B.Product1 = -1.0;
        } else {
          rover_sw_pwrtrain_24b_B.Product1 = (rover_sw_pwrtrain_24b_B.Product1 >
            0.0);
        }
      }

      if (fabs(rover_sw_pwrtrain_24b_B.Product1) >
          rover_sw_pwrtrain_24b_DW.obj.MaxAngularVelocity) {
        if (rtIsNaN(rover_sw_pwrtrain_24b_B.Product1)) {
          rover_sw_pwrtrain_24b_B.Add = (rtNaN);
        } else if (rover_sw_pwrtrain_24b_B.Product1 < 0.0) {
          rover_sw_pwrtrain_24b_B.Add = -1.0;
        } else {
          rover_sw_pwrtrain_24b_B.Add = (rover_sw_pwrtrain_24b_B.Product1 > 0.0);
        }

        rover_sw_pwrtrain_24b_B.Product1 = rover_sw_pwrtrain_24b_B.Add *
          rover_sw_pwrtrain_24b_DW.obj.MaxAngularVelocity;
      }

      /* Product: '<S48>/Product' incorporates:
       *  Constant: '<S26>/Constant4'
       *  ZeroOrderHold: '<S25>/Zero-Order Hold2'
       */
      rover_sw_pwrtrain_24b_B.Product =
        rover_sw_pwrtrain_24b_DW.obj.DesiredLinearVelocity;
      rover_sw_pwrtrain_24b_DW.obj.LastPose[0] =
        rover_sw_pwrtrain_24b_B.Integrator[0];
      rover_sw_pwrtrain_24b_DW.obj.LastPose[1] =
        rover_sw_pwrtrain_24b_B.Integrator[1];
      rover_sw_pwrtrain_24b_DW.obj.LastPose[2] =
        rover_sw_pwrtrain_24b_B.Integrator[2];
    }

    /* If: '<S24>/If' */
    if (rtsiIsModeUpdateTimeStep(&(&rover_sw_pwrtrain_24b_M)->solverInfo)) {
      rtAction = static_cast<int8_T>(rover_sw_pwrtrain_24b_B.In1.data);
      rover_sw_pwrtrain_24b_DW.If_ActiveSubsystem = static_cast<int8_T>
        (rover_sw_pwrtrain_24b_B.In1.data);
    } else {
      rtAction = rover_sw_pwrtrain_24b_DW.If_ActiveSubsystem;
    }

    if (rtAction == 0) {
      /* Outputs for IfAction SubSystem: '<S24>/Subsystem1' incorporates:
       *  ActionPort: '<S34>/Action Port'
       */
      /* BusAssignment: '<S34>/Bus Assignment1' incorporates:
       *  CCaller: '<S45>/vesc_send_duty_left'
       *  Product: '<S45>/Product'
       *  Sum: '<S45>/Plus'
       */
      rtb_BusAssignment1_j.data = vesc_send_duty_left
        (rover_sw_pwrtrain_24b_B.throttle_out *
         rover_sw_pwrtrain_24b_B.EnabledSubsystem_o.In1.data +
         rover_sw_pwrtrain_24b_B.throttle_out);

      /* MATLABSystem: '<S43>/SinkBlock' */
      Pub_rover_sw_pwrtrain_24b_138.publish(&rtb_BusAssignment1_j);

      /* BusAssignment: '<S34>/Bus Assignment2' incorporates:
       *  CCaller: '<S45>/vesc_send_duty_right'
       *  Product: '<S45>/Product1'
       *  Sum: '<S45>/Plus2'
       */
      rtb_BusAssignment2_f.data = vesc_send_duty_right
        (rover_sw_pwrtrain_24b_B.throttle_out -
         rover_sw_pwrtrain_24b_B.throttle_out *
         rover_sw_pwrtrain_24b_B.EnabledSubsystem_o.In1.data);

      /* MATLABSystem: '<S44>/SinkBlock' */
      Pub_rover_sw_pwrtrain_24b_139.publish(&rtb_BusAssignment2_f);

      /* End of Outputs for SubSystem: '<S24>/Subsystem1' */
    } else {
      /* Outputs for IfAction SubSystem: '<S24>/Subsystem' incorporates:
       *  ActionPort: '<S33>/Action Port'
       */
      /* BusAssignment: '<S33>/Bus Assignment1' incorporates:
       *  CCaller: '<S33>/vesc_send_duty_left'
       *  Switch: '<S33>/Switch'
       */
      rtb_BusAssignment1_j.data = vesc_send_duty_left
        (rover_sw_pwrtrain_24b_B.throttle_out);

      /* MATLABSystem: '<S39>/SinkBlock' */
      Pub_rover_sw_pwrtrain_24b_542.publish(&rtb_BusAssignment1_j);

      /* BusAssignment: '<S33>/Bus Assignment2' incorporates:
       *  CCaller: '<S33>/vesc_send_duty_right'
       *  Switch: '<S33>/Switch1'
       */
      rtb_BusAssignment2_f.data = vesc_send_duty_right
        (rover_sw_pwrtrain_24b_B.throttle_out);

      /* MATLABSystem: '<S40>/SinkBlock' */
      Pub_rover_sw_pwrtrain_24b_543.publish(&rtb_BusAssignment2_f);

      /* End of Outputs for SubSystem: '<S24>/Subsystem' */
    }

    /* End of If: '<S24>/If' */
  }

  /* MATLABSystem: '<S47>/MATLAB System' */
  if (rover_sw_pwrtrain_24b_DW.obj_j.TrackWidth != 0.5) {
    if (rover_sw_pwrtrain_24b_DW.obj_j.isInitialized == 1) {
      rover_sw_pwrtrain_24b_DW.obj_j.TunablePropsChanged = true;
      rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[0] = true;
    }

    rover_sw_pwrtrain_24b_DW.obj_j.TrackWidth = 0.5;
  }

  if (rover_sw_pwrtrain_24b_DW.obj_j.WheelRadius != 0.095) {
    if (rover_sw_pwrtrain_24b_DW.obj_j.isInitialized == 1) {
      rover_sw_pwrtrain_24b_DW.obj_j.TunablePropsChanged = true;
      rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[1] = true;
    }

    rover_sw_pwrtrain_24b_DW.obj_j.WheelRadius = 0.095;
  }

  searchFlag = false;
  rtb_OR = true;
  rover_sw_pwrtrain_24b_B.ret = 0;
  exitg1 = false;
  while ((!exitg1) && (rover_sw_pwrtrain_24b_B.ret < 2)) {
    if (!(rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[rover_sw_pwrtrain_24b_B.ret]
          ==
          rover_sw_pwrtrain_24_ConstInitP.MATLABSystem_WheelSpeedRange[rover_sw_pwrtrain_24b_B.ret]))
    {
      rtb_OR = false;
      exitg1 = true;
    } else {
      rover_sw_pwrtrain_24b_B.ret++;
    }
  }

  if (rtb_OR) {
    searchFlag = true;
  }

  if (!searchFlag) {
    if (rover_sw_pwrtrain_24b_DW.obj_j.isInitialized == 1) {
      rover_sw_pwrtrain_24b_DW.obj_j.TunablePropsChanged = true;
      rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[2] = true;
    }

    rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[0] = (rtMinusInf);
    rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[1] = (rtInf);
  }

  if (rover_sw_pwrtrain_24b_DW.obj_j.TunablePropsChanged) {
    rover_sw_pwrtrain_24b_DW.obj_j.TunablePropsChanged = false;
    if (rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[1]) {
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius =
        rover_sw_pwrtrain_24b_DW.obj_j.WheelRadius;
    }

    if (rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[0]) {
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.TrackWidth =
        rover_sw_pwrtrain_24b_DW.obj_j.TrackWidth;
    }

    if (rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[2]) {
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0] =
        rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[0];
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1] =
        rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[1];
    }

    rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[0] = false;
    rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[1] = false;
    rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[2] = false;
  }

  rover_sw_pwrtrain_24b_B.Add = 0.0;

  /* MATLABSystem: '<S47>/MATLAB System' */
  rover_sw_pwrtrain_24b_B.MATLABSystem[2] = 0.0;

  /* MATLABSystem: '<S47>/MATLAB System' */
  rover_sw_pwrtrain_24b_B.ret = std::memcmp(&a[0],
    &rover_sw_pwrtrain_24b_DW.obj_j.KinModel.VehicleInputsInternal[0], 23);
  if (rover_sw_pwrtrain_24b_B.ret == 0) {
    rover_sw_pwrtrain_24b_B.ret = 0;
  } else {
    rover_sw_pwrtrain_24b_B.ret = std::memcmp(&a_0[0],
      &rover_sw_pwrtrain_24b_DW.obj_j.KinModel.VehicleInputsInternal[0], 23);
    if (rover_sw_pwrtrain_24b_B.ret == 0) {
      rover_sw_pwrtrain_24b_B.ret = 1;
    } else {
      rover_sw_pwrtrain_24b_B.ret = -1;
    }
  }

  switch (rover_sw_pwrtrain_24b_B.ret) {
   case 0:
    if ((rover_sw_pwrtrain_24b_B.Product >=
         rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0]) || rtIsNaN
        (rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0])) {
      rover_sw_pwrtrain_24b_B.minDistance = rover_sw_pwrtrain_24b_B.Product;
    } else {
      rover_sw_pwrtrain_24b_B.minDistance =
        rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0];
    }

    searchFlag = !rtIsNaN
      (rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1]);
    if ((!(rover_sw_pwrtrain_24b_B.minDistance <=
           rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1])) &&
        searchFlag) {
      rover_sw_pwrtrain_24b_B.minDistance =
        rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1];
    }

    if ((rover_sw_pwrtrain_24b_B.Product1 >=
         rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0]) || rtIsNaN
        (rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0])) {
      rover_sw_pwrtrain_24b_B.lookaheadIdx = rover_sw_pwrtrain_24b_B.Product1;
    } else {
      rover_sw_pwrtrain_24b_B.lookaheadIdx =
        rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0];
    }

    if ((!(rover_sw_pwrtrain_24b_B.lookaheadIdx <=
           rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1])) &&
        searchFlag) {
      rover_sw_pwrtrain_24b_B.lookaheadIdx =
        rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1];
    }

    rover_sw_pwrtrain_24b_B.Add = (rover_sw_pwrtrain_24b_B.lookaheadIdx +
      rover_sw_pwrtrain_24b_B.minDistance) *
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius / 2.0;

    /* MATLABSystem: '<S47>/MATLAB System' */
    rover_sw_pwrtrain_24b_B.MATLABSystem[2] =
      (rover_sw_pwrtrain_24b_B.lookaheadIdx -
       rover_sw_pwrtrain_24b_B.minDistance) *
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius /
      (rover_sw_pwrtrain_24b_DW.obj_j.KinModel.TrackWidth / 2.0 * 2.0);
    break;

   case 1:
    rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp =
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.TrackWidth / 2.0 *
      rover_sw_pwrtrain_24b_B.Product1;
    rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] =
      (rover_sw_pwrtrain_24b_B.Product -
       rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp) /
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius;
    rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] =
      (rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp +
       rover_sw_pwrtrain_24b_B.Product) /
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius;
    searchFlag = rtIsNaN
      (rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0]);
    if ((rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] >=
         rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0]) ||
        searchFlag) {
      rover_sw_pwrtrain_24b_B.minDistance =
        rover_sw_pwrtrain_24b_B.lookaheadStartPt[0];
    } else {
      rover_sw_pwrtrain_24b_B.minDistance =
        rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0];
    }

    rtb_OR = !rtIsNaN(rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1]);
    if ((!(rover_sw_pwrtrain_24b_B.minDistance <=
           rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1])) &&
        rtb_OR) {
      rover_sw_pwrtrain_24b_B.minDistance =
        rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1];
    }

    if ((rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] >=
         rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0]) ||
        searchFlag) {
      rover_sw_pwrtrain_24b_B.lookaheadIdx =
        rover_sw_pwrtrain_24b_B.lookaheadStartPt[1];
    } else {
      rover_sw_pwrtrain_24b_B.lookaheadIdx =
        rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0];
    }

    if ((!(rover_sw_pwrtrain_24b_B.lookaheadIdx <=
           rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1])) &&
        rtb_OR) {
      rover_sw_pwrtrain_24b_B.lookaheadIdx =
        rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1];
    }

    rover_sw_pwrtrain_24b_B.Add = (rover_sw_pwrtrain_24b_B.lookaheadIdx +
      rover_sw_pwrtrain_24b_B.minDistance) *
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius / 2.0;

    /* MATLABSystem: '<S47>/MATLAB System' */
    rover_sw_pwrtrain_24b_B.MATLABSystem[2] =
      (rover_sw_pwrtrain_24b_B.lookaheadIdx -
       rover_sw_pwrtrain_24b_B.minDistance) *
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius /
      (rover_sw_pwrtrain_24b_DW.obj_j.KinModel.TrackWidth / 2.0 * 2.0);
    break;
  }

  /* MATLABSystem: '<S47>/MATLAB System' */
  rover_sw_pwrtrain_24b_B.MATLABSystem[0] = cos
    (rover_sw_pwrtrain_24b_B.Integrator[2]) * rover_sw_pwrtrain_24b_B.Add;
  rover_sw_pwrtrain_24b_B.MATLABSystem[1] = sin
    (rover_sw_pwrtrain_24b_B.Integrator[2]) * rover_sw_pwrtrain_24b_B.Add;
  if (tmp) {
    /* BusAssignment: '<S27>/Bus Assignment' */
    memset(&rover_sw_pwrtrain_24b_B.BusAssignment, 0, sizeof
           (SL_Bus_std_msgs_Float32MultiArray));

    /* BusAssignment: '<S27>/Bus Assignment1' incorporates:
     *  MATLAB Function: '<S27>/MATLAB Function1'
     */
    rtb_BusAssignment1_a5.data = (((rover_sw_pwrtrain_24b_B.voltages[0] +
      rover_sw_pwrtrain_24b_B.voltages[1]) + rover_sw_pwrtrain_24b_B.voltages[2])
      + rover_sw_pwrtrain_24b_B.voltages[3]) / 4.0F;

    /* MATLABSystem: '<S55>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_818.publish(&rtb_BusAssignment1_a5);

    /* MATLAB Function: '<S27>/MATLAB Function2' incorporates:
     *  Math: '<S27>/Transpose'
     */
    memset(&rover_sw_pwrtrain_24b_B.data[0], 0, sizeof(real32_T) << 7U);
    rover_sw_pwrtrain_24b_B.data[0] = rover_sw_pwrtrain_24b_B.voltages[0];
    rover_sw_pwrtrain_24b_B.data[1] = rover_sw_pwrtrain_24b_B.voltages[1];
    rover_sw_pwrtrain_24b_B.data[2] = rover_sw_pwrtrain_24b_B.voltages[2];
    rover_sw_pwrtrain_24b_B.data[3] = rover_sw_pwrtrain_24b_B.voltages[3];

    /* BusAssignment: '<S27>/Bus Assignment' */
    memcpy(&rover_sw_pwrtrain_24b_B.BusAssignment.data[0],
           &rover_sw_pwrtrain_24b_B.data[0], sizeof(real32_T) << 7U);

    /* MATLABSystem: '<S56>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_822.publish(&rover_sw_pwrtrain_24b_B.BusAssignment);

    /* BusAssignment: '<S27>/Bus Assignment2' incorporates:
     *  Math: '<S27>/Transpose'
     */
    rtb_BusAssignment2_h.data = rover_sw_pwrtrain_24b_B.voltages[0];

    /* MATLABSystem: '<S57>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_835.publish(&rtb_BusAssignment2_h);
  }

  if (rtmIsMajorTimeStep((&rover_sw_pwrtrain_24b_M))) {
    if (rtmIsMajorTimeStep((&rover_sw_pwrtrain_24b_M))) {
      /* Update for UnitDelay: '<S30>/Unit Delay' */
      rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE = rover_sw_pwrtrain_24b_B.Switch;

      /* Update for UnitDelay: '<S35>/Unit Delay' */
      rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i =
        rover_sw_pwrtrain_24b_B.throttle_out;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&rover_sw_pwrtrain_24b_M))) {
    rt_ertODEUpdateContinuousStates(&(&rover_sw_pwrtrain_24b_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&rover_sw_pwrtrain_24b_M)->Timing.clockTick0)) {
      ++(&rover_sw_pwrtrain_24b_M)->Timing.clockTickH0;
    }

    (&rover_sw_pwrtrain_24b_M)->Timing.t[0] = rtsiGetSolverStopTime
      (&(&rover_sw_pwrtrain_24b_M)->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&rover_sw_pwrtrain_24b_M)->Timing.clockTick1++;
      if (!(&rover_sw_pwrtrain_24b_M)->Timing.clockTick1) {
        (&rover_sw_pwrtrain_24b_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void rover_sw_pwrtrain_24b::rover_sw_pwrtrain_24b_derivatives()
{
  XDot_rover_sw_pwrtrain_24b_T *_rtXdot;
  _rtXdot = ((XDot_rover_sw_pwrtrain_24b_T *) (&rover_sw_pwrtrain_24b_M)->derivs);

  /* Derivatives for Integrator: '<S47>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = rover_sw_pwrtrain_24b_B.MATLABSystem[0];
  _rtXdot->Integrator_CSTATE[1] = rover_sw_pwrtrain_24b_B.MATLABSystem[1];
  _rtXdot->Integrator_CSTATE[2] = rover_sw_pwrtrain_24b_B.MATLABSystem[2];
}

/* Model initialize function */
void rover_sw_pwrtrain_24b::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  rover_sw_pwrtrain_24_ConstInitP.MATLABSystem_WheelSpeedRange[0] = rtMinusInf;
  rover_sw_pwrtrain_24_ConstInitP.MATLABSystem_WheelSpeedRange[1] = rtInf;

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
                          &(&rover_sw_pwrtrain_24b_M)->Timing.simTimeStep);
    rtsiSetTPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo, &rtmGetTPtr
                ((&rover_sw_pwrtrain_24b_M)));
    rtsiSetStepSizePtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
                       &(&rover_sw_pwrtrain_24b_M)->Timing.stepSize0);
    rtsiSetdXPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
                 &(&rover_sw_pwrtrain_24b_M)->derivs);
    rtsiSetContStatesPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo, (real_T **) &(
      &rover_sw_pwrtrain_24b_M)->contStates);
    rtsiSetNumContStatesPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
      &(&rover_sw_pwrtrain_24b_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
      &(&rover_sw_pwrtrain_24b_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
      &(&rover_sw_pwrtrain_24b_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
      &(&rover_sw_pwrtrain_24b_M)->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
      (boolean_T**) &(&rover_sw_pwrtrain_24b_M)->contStateDisabled);
    rtsiSetErrorStatusPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
                          (&rtmGetErrorStatus((&rover_sw_pwrtrain_24b_M))));
    rtsiSetRTModelPtr(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
                      (&rover_sw_pwrtrain_24b_M));
  }

  rtsiSetSimTimeStep(&(&rover_sw_pwrtrain_24b_M)->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&(&rover_sw_pwrtrain_24b_M)->solverInfo,
    false);
  rtsiSetIsContModeFrozen(&(&rover_sw_pwrtrain_24b_M)->solverInfo, false);
  (&rover_sw_pwrtrain_24b_M)->intgData.y = (&rover_sw_pwrtrain_24b_M)->odeY;
  (&rover_sw_pwrtrain_24b_M)->intgData.f[0] = (&rover_sw_pwrtrain_24b_M)->odeF[0];
  (&rover_sw_pwrtrain_24b_M)->intgData.f[1] = (&rover_sw_pwrtrain_24b_M)->odeF[1];
  (&rover_sw_pwrtrain_24b_M)->intgData.f[2] = (&rover_sw_pwrtrain_24b_M)->odeF[2];
  (&rover_sw_pwrtrain_24b_M)->contStates = ((X_rover_sw_pwrtrain_24b_T *)
    &rover_sw_pwrtrain_24b_X);
  (&rover_sw_pwrtrain_24b_M)->contStateDisabled = ((XDis_rover_sw_pwrtrain_24b_T
    *) &rover_sw_pwrtrain_24b_XDis);
  (&rover_sw_pwrtrain_24b_M)->Timing.tStart = (0.0);
  rtsiSetSolverData(&(&rover_sw_pwrtrain_24b_M)->solverInfo, static_cast<void *>
                    (&(&rover_sw_pwrtrain_24b_M)->intgData));
  rtsiSetSolverName(&(&rover_sw_pwrtrain_24b_M)->solverInfo,"ode3");
  rtmSetTPtr((&rover_sw_pwrtrain_24b_M), &(&rover_sw_pwrtrain_24b_M)
             ->Timing.tArray[0]);
  (&rover_sw_pwrtrain_24b_M)->Timing.stepSize0 = 0.01;

  {
    int32_T i;

    /* Start for MATLABSystem: '<S14>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_i.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_i.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_c = true;
    rover_sw_pwrtrain_24b_DW.obj_i.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_i.isInitialized = 1;
    rover_sw_p_Subscriber_setupImpl(&rover_sw_pwrtrain_24b_DW.obj_i);
    rover_sw_pwrtrain_24b_DW.obj_i.isSetupComplete = true;

    /* Start for MATLABSystem: '<S15>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_ng.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_ng.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_ea = true;
    rover_sw_pwrtrain_24b_DW.obj_ng.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_ng.isInitialized = 1;
    rover_sw_Subscriber_setupImpl_m(&rover_sw_pwrtrain_24b_DW.obj_ng);
    rover_sw_pwrtrain_24b_DW.obj_ng.isSetupComplete = true;

    /* Start for MATLABSystem: '<S12>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_g.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_g.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_k = true;
    rover_sw_pwrtrain_24b_DW.obj_g.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_g.isInitialized = 1;
    rover_sw_Publisher_setupImpl_my(&rover_sw_pwrtrain_24b_DW.obj_g);
    rover_sw_pwrtrain_24b_DW.obj_g.isSetupComplete = true;

    /* Start for MATLABSystem: '<S11>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_l.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_l.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_f = true;
    rover_sw_pwrtrain_24b_DW.obj_l.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_l.isInitialized = 1;
    rover_sw__Publisher_setupImpl_m(&rover_sw_pwrtrain_24b_DW.obj_l);
    rover_sw_pwrtrain_24b_DW.obj_l.isSetupComplete = true;

    /* Start for MATLABSystem: '<S16>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_hm.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_hm.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_er = true;
    rover_sw_pwrtrain_24b_DW.obj_hm.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_hm.isInitialized = 1;
    rover_s_Subscriber_setupImpl_my(&rover_sw_pwrtrain_24b_DW.obj_hm);
    rover_sw_pwrtrain_24b_DW.obj_hm.isSetupComplete = true;

    /* Start for MATLABSystem: '<S17>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_m.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_m.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_l = true;
    rover_sw_pwrtrain_24b_DW.obj_m.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_m.isInitialized = 1;
    rover__Subscriber_setupImpl_myg(&rover_sw_pwrtrain_24b_DW.obj_m);
    rover_sw_pwrtrain_24b_DW.obj_m.isSetupComplete = true;

    /* Start for MATLABSystem: '<S10>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_o.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_o.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_b = true;
    rover_sw_pwrtrain_24b_DW.obj_o.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_o.isInitialized = 1;
    rover_sw_pw_Publisher_setupImpl(&rover_sw_pwrtrain_24b_DW.obj_o);
    rover_sw_pwrtrain_24b_DW.obj_o.isSetupComplete = true;

    /* Start for MATLABSystem: '<S13>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_h.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_h.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_eh = true;
    rover_sw_pwrtrain_24b_DW.obj_h.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_h.isInitialized = 1;
    rover_s_Publisher_setupImpl_myg(&rover_sw_pwrtrain_24b_DW.obj_h);
    rover_sw_pwrtrain_24b_DW.obj_h.isSetupComplete = true;

    /* Start for MATLABSystem: '<S25>/Pure Pursuit' */
    rover_sw_pwrtrain_24b_DW.objisempty_e = true;
    rover_sw_pwrtrain_24b_DW.obj.DesiredLinearVelocity = 0.5;
    rover_sw_pwrtrain_24b_DW.obj.MaxAngularVelocity = 1.0;
    rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance = 0.1;
    rover_sw_pwrtrain_24b_DW.obj.isInitialized = 1;
    for (i = 0; i < 1756; i++) {
      rover_sw_pwrtrain_24b_DW.obj.WaypointsInternal[i] = (rtNaN);
    }

    rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] = 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] = 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LastPose[0] = 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LastPose[1] = 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LastPose[2] = 0.0;
    rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] = (rtNaN);
    rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] = (rtNaN);
    rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex = 0.0;

    /* End of Start for MATLABSystem: '<S25>/Pure Pursuit' */

    /* Start for If: '<S24>/If' */
    rover_sw_pwrtrain_24b_DW.If_ActiveSubsystem = -1;

    /* Start for IfAction SubSystem: '<S24>/Subsystem1' */
    /* Start for MATLABSystem: '<S43>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_a.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_a.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_n = true;
    rover_sw_pwrtrain_24b_DW.obj_a.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_a.isInitialized = 1;
    rover__Publisher_setupImpl_mygo(&rover_sw_pwrtrain_24b_DW.obj_a);
    rover_sw_pwrtrain_24b_DW.obj_a.isSetupComplete = true;

    /* Start for MATLABSystem: '<S44>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_p.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_p.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_d = true;
    rover_sw_pwrtrain_24b_DW.obj_p.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_p.isInitialized = 1;
    rover_Publisher_setupImpl_mygou(&rover_sw_pwrtrain_24b_DW.obj_p);
    rover_sw_pwrtrain_24b_DW.obj_p.isSetupComplete = true;

    /* End of Start for SubSystem: '<S24>/Subsystem1' */

    /* Start for IfAction SubSystem: '<S24>/Subsystem' */
    /* Start for MATLABSystem: '<S39>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_c.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_c.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_jb = true;
    rover_sw_pwrtrain_24b_DW.obj_c.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_c.isInitialized = 1;
    rove_Publisher_setupImpl_mygou5(&rover_sw_pwrtrain_24b_DW.obj_c);
    rover_sw_pwrtrain_24b_DW.obj_c.isSetupComplete = true;

    /* Start for MATLABSystem: '<S40>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_n.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_n.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_j = true;
    rover_sw_pwrtrain_24b_DW.obj_n.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_n.isInitialized = 1;
    rov_Publisher_setupImpl_mygou5b(&rover_sw_pwrtrain_24b_DW.obj_n);
    rover_sw_pwrtrain_24b_DW.obj_n.isSetupComplete = true;

    /* End of Start for SubSystem: '<S24>/Subsystem' */

    /* Start for MATLABSystem: '<S47>/MATLAB System' */
    rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[0] = false;
    rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[1] = false;
    rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[2] = false;
    rover_sw_pwrtrain_24b_DW.objisempty_ok = true;
    rover_sw_pwrtrain_24b_DW.obj_j.TrackWidth = 0.5;
    rover_sw_pwrtrain_24b_DW.obj_j.WheelRadius = 0.095;
    rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[0] = (rtMinusInf);
    rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[1] = (rtInf);
    rover_sw_pwrtrain_24b_DW.obj_j.isInitialized = 1;
    DifferentialDriveKinematics_set(&rover_sw_pwrtrain_24b_DW.obj_j);
    rover_sw_pwrtrain_24b_DW.obj_j.TunablePropsChanged = false;

    /* Start for MATLABSystem: '<S55>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_ju.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_ju.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_o = true;
    rover_sw_pwrtrain_24b_DW.obj_ju.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_ju.isInitialized = 1;
    ro_Publisher_setupImpl_mygou5bo(&rover_sw_pwrtrain_24b_DW.obj_ju);
    rover_sw_pwrtrain_24b_DW.obj_ju.isSetupComplete = true;

    /* Start for MATLABSystem: '<S56>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_k.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_k.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_p = true;
    rover_sw_pwrtrain_24b_DW.obj_k.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_k.isInitialized = 1;
    r_Publisher_setupImpl_mygou5bow(&rover_sw_pwrtrain_24b_DW.obj_k);
    rover_sw_pwrtrain_24b_DW.obj_k.isSetupComplete = true;

    /* Start for MATLABSystem: '<S57>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_d.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_d.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty = true;
    rover_sw_pwrtrain_24b_DW.obj_d.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_d.isInitialized = 1;
    Publisher_setupImpl_mygou5bowb(&rover_sw_pwrtrain_24b_DW.obj_d);
    rover_sw_pwrtrain_24b_DW.obj_d.isSetupComplete = true;
  }

  {
    int32_T i;
    static const char_T tmp[23] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
      'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e' };

    /* InitializeConditions for Integrator: '<S47>/Integrator' */
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[0] = 0.0;
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[1] = 0.0;
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[2] = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S14>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem);

    /* End of SystemInitialize for SubSystem: '<S14>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S15>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_o);

    /* End of SystemInitialize for SubSystem: '<S15>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S16>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_p);

    /* End of SystemInitialize for SubSystem: '<S16>/Enabled Subsystem' */

    /* SystemInitialize for SignalConversion generated from: '<S21>/In1' */
    memset(&rover_sw_pwrtrain_24b_B.In1, 0, sizeof(SL_Bus_std_msgs_Bool));

    /* InitializeConditions for MATLABSystem: '<S25>/Pure Pursuit' */
    rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LastPose[0] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LastPose[1] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LastPose[2] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] = (rtNaN);
    rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] = (rtNaN);
    rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex *= 0.0;

    /* InitializeConditions for MATLABSystem: '<S47>/MATLAB System' */
    for (i = 0; i < 23; i++) {
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.VehicleInputsInternal[i] = tmp[i];
    }

    rover_sw_pwrtrain_24b_DW.obj_j.KinModel.TrackWidth =
      rover_sw_pwrtrain_24b_DW.obj_j.TrackWidth;
    rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius =
      rover_sw_pwrtrain_24b_DW.obj_j.WheelRadius;
    rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[0] =
      rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[0];
    rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1] =
      rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[1];

    /* End of InitializeConditions for MATLABSystem: '<S47>/MATLAB System' */

    /* Outputs for Atomic SubSystem: '<Root>/Initialize Function' */
    /* CCaller: '<S1>/C Caller' */
    vesc_open_noargs();

    /* End of Outputs for SubSystem: '<Root>/Initialize Function' */
  }
}

/* Model terminate function */
void rover_sw_pwrtrain_24b::terminate()
{
  /* Outputs for Atomic SubSystem: '<Root>/Terminate Function' */
  /* CCaller: '<S2>/C Caller1' */
  vesc_close();

  /* End of Outputs for SubSystem: '<Root>/Terminate Function' */

  /* Terminate for MATLABSystem: '<S14>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_i.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_i.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_i.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_i.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_390.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S14>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S15>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_ng.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_ng.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_ng.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_ng.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_391.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S15>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S12>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_g.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_g.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_g.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_g.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_246.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S12>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S11>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_l.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_l.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_l.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_l.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_826.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S11>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S16>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_hm.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_hm.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_hm.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_hm.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_392.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S16>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S17>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_m.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_m.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_m.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_m.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_521.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S17>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S10>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_o.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_o.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_o.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_o.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_907.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S10>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S13>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_h.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_h.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_h.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_903.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S13>/SinkBlock' */

  /* Terminate for IfAction SubSystem: '<S24>/Subsystem1' */
  /* Terminate for MATLABSystem: '<S43>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_a.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_a.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_a.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_a.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_138.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S43>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S44>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_p.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_p.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_p.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_p.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_139.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S44>/SinkBlock' */
  /* End of Terminate for SubSystem: '<S24>/Subsystem1' */

  /* Terminate for IfAction SubSystem: '<S24>/Subsystem' */
  /* Terminate for MATLABSystem: '<S39>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_c.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_c.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_c.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_c.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_542.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S39>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S40>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_n.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_n.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_n.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_n.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_543.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S40>/SinkBlock' */
  /* End of Terminate for SubSystem: '<S24>/Subsystem' */

  /* Terminate for MATLABSystem: '<S55>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_ju.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_ju.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_ju.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_ju.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_818.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S55>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S56>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_k.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_k.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_k.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_k.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_822.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S56>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S57>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_d.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_d.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_d.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_d.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_835.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S57>/SinkBlock' */
}

/* Constructor */
rover_sw_pwrtrain_24b::rover_sw_pwrtrain_24b() :
  rover_sw_pwrtrain_24b_B(),
  rover_sw_pwrtrain_24b_DW(),
  rover_sw_pwrtrain_24b_X(),
  rover_sw_pwrtrain_24b_XDis(),
  rover_sw_pwrtrain_24b_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
rover_sw_pwrtrain_24b::~rover_sw_pwrtrain_24b()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_rover_sw_pwrtrain_24_T * rover_sw_pwrtrain_24b::getRTM()
{
  return (&rover_sw_pwrtrain_24b_M);
}
