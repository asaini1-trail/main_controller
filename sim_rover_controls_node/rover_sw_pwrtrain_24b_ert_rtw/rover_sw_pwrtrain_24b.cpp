/*
 * rover_sw_pwrtrain_24b.cpp
 *
 * Home License - for personal use only.  Not for government, academic,
 * research, commercial, or other organizational use.
 *
 * Code generation for model "rover_sw_pwrtrain_24b".
 *
 * Model version              : 1.145
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C++ source code generated on : Sat Jan 31 01:10:01 2026
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

/* Named constants for Chart: '<S53>/Chart' */
const uint8_T rover_IN_TRAILHEAD_READY_TO_ARM = 3U;
const uint8_T rover__IN_TRAILHEAD_ERROR_STATE = 2U;
const uint8_T rover_sw_p_IN_TRAILHEAD_UNARMED = 4U;
const uint8_T rover_sw_pwr_IN_TRAILHEAD_ARMED = 1U;
const uint8_T rover_sw_pwrtrain_24_IN_mil_off = 1U;
const uint8_T rover_sw_pwrtrain_24b_IN_mil_on = 2U;
const uint8_T rover_sw_pwrtrain_IN_FAST_DECEL = 1U;
const uint8_T rover_sw_pwrtrain_IN_SLOW_ACCEL = 2U;

/* Named constants for Chart: '<S69>/adaptive_throttle_contller' */
const uint8_T rove_IN_COLLISION_FLT_FAST_STOP = 1U;
const uint8_T rover_sw_IN_LOST_COMM_OPERATION = 3U;
const uint8_T rover_sw_p_IN_LOSS_COMM_MONITOR = 2U;
const uint8_T rover_sw_pw_IN_NORMAL_OPERATION = 4U;
const uint8_T rover_sw_pwrtrain_24b_IN_MILD = 1U;
const uint8_T rover_sw_pwrtrain_24b_IN_SEVERE = 2U;
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
 *    '<S25>/Enabled Subsystem'
 *    '<S26>/Enabled Subsystem'
 *    '<S27>/Enabled Subsystem'
 *    '<S28>/Enabled Subsystem'
 *    '<S29>/Enabled Subsystem'
 *    '<S30>/Enabled Subsystem'
 *    '<S31>/Enabled Subsystem'
 *    '<S32>/Enabled Subsystem'
 *    '<S34>/Enabled Subsystem'
 *    '<S35>/Enabled Subsystem'
 *    ...
 */
void rover_sw_pwrtrain_24b::rover_sw__EnabledSubsystem_Init
  (B_EnabledSubsystem_rover_sw_p_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S39>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Float32));
}

/*
 * Output and update for enable system:
 *    '<S25>/Enabled Subsystem'
 *    '<S26>/Enabled Subsystem'
 *    '<S27>/Enabled Subsystem'
 *    '<S28>/Enabled Subsystem'
 *    '<S29>/Enabled Subsystem'
 *    '<S30>/Enabled Subsystem'
 *    '<S31>/Enabled Subsystem'
 *    '<S32>/Enabled Subsystem'
 *    '<S34>/Enabled Subsystem'
 *    '<S35>/Enabled Subsystem'
 *    ...
 */
void rover_sw_pwrtrain_24b::rover_sw_pwrtr_EnabledSubsystem(boolean_T rtu_Enable,
  const SL_Bus_std_msgs_Float32 *rtu_In1, B_EnabledSubsystem_rover_sw_p_T
  *localB)
{
  /* Outputs for Enabled SubSystem: '<S25>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S39>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S39>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S25>/Enabled Subsystem' */
}

/*
 * System initialize for enable system:
 *    '<S33>/Enabled Subsystem'
 *    '<S38>/Enabled Subsystem'
 */
void rover_sw_pwrtrain_24b::rover_s_EnabledSubsystem_p_Init
  (B_EnabledSubsystem_rover_sw_e_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S47>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Bool));
}

/*
 * Output and update for enable system:
 *    '<S33>/Enabled Subsystem'
 *    '<S38>/Enabled Subsystem'
 */
void rover_sw_pwrtrain_24b::rover_sw_pwr_EnabledSubsystem_g(boolean_T rtu_Enable,
  const SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_rover_sw_e_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S33>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S47>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S47>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S33>/Enabled Subsystem' */
}

void rover_sw_pwrtrain_24b::rover_s_Publisher_setupImpl_myg(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[11];
  static const char_T b_zeroDelimTopic_0[11] = "/led_state";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S17>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline_j.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_j.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_j, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 11; i++) {
    /* Start for MATLABSystem: '<S17>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_246.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rover_sw_Publisher_setupImpl_my(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "vesc/telemetry/status";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S16>/SinkBlock' */
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
    /* Start for MATLABSystem: '<S16>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_826.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic[0], qos_profile);
}

void rover_sw_pwrtrain_24b::DifferentialDriveKinematics_set
  (robotics_slmobile_internal_bl_T *obj)
{
  static const char_T b[23] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p', 'e',
    'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e' };

  for (int32_T i = 0; i < 23; i++) {
    /* Start for MATLABSystem: '<S78>/MATLAB System' */
    obj->KinModel.VehicleInputsInternal[i] = b[i];
  }

  /* Start for MATLABSystem: '<S78>/MATLAB System' */
  obj->KinModel.TrackWidth = obj->TrackWidth;
  obj->KinModel.WheelRadius = obj->WheelRadius;
  obj->KinModel.WheelSpeedRange[0] = obj->WheelSpeedRange[0];
  obj->KinModel.WheelSpeedRange[1] = obj->WheelSpeedRange[1];
}

void rover_sw_pwrtrain_24b::rover_sw_Subscriber_setupImpl_m(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[8];
  static const char_T b_zeroDelimTopic_0[8] = "/rc/ch2";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S26>/SourceBlock' */
  rover_sw_pwrtrain_24b_B.deadline_ln.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_ln.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_ln, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S26>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_rover_sw_pwrtrain_24b_391.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rover_sw_p_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[8];
  static const char_T b_zeroDelimTopic_0[8] = "/rc/ch1";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S25>/SourceBlock' */
  rover_sw_pwrtrain_24b_B.deadline_bs.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_bs.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_bs, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S25>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_rover_sw_pwrtrain_24b_390.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::r_Subscriber_setupImpl_mygou5bo(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[8];
  static const char_T b_zeroDelimTopic_0[8] = "/rc/ch3";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S33>/SourceBlock' */
  rover_sw_pwrtrain_24b_B.deadline_nu.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_nu.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_nu, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S33>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_rover_sw_pwrtrain_24b_521.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::ro_Subscriber_setupImpl_mygou5b(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/vesc/voltage/lf";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S32>/SourceBlock' */
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
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S32>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_p[i] = b_zeroDelimTopic[i];
  }

  Sub_rover_sw_pwrtrain_24b_986.createSubscriber
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_p[0], qos_profile);
}

void rover_sw_pwrtrain_24b::Subscriber_setupImpl_mygou5bow(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/vesc/voltage/lr";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S34>/SourceBlock' */
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
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S34>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_b[i] = b_zeroDelimTopic[i];
  }

  Sub_rover_sw_pwrtrain_24b_987.createSubscriber
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_b[0], qos_profile);
}

void rover_sw_pwrtrain_24b::Subscriber_setupImpl_mygou5bowb(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/vesc/voltage/rf";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S35>/SourceBlock' */
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
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S35>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_cx[i] = b_zeroDelimTopic[i];
  }

  Sub_rover_sw_pwrtrain_24b_988.createSubscriber
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_cx[0], qos_profile);
}

void rover_sw_pwrtrain_24b::Subscriber_setupImp_mygou5bowbh(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/vesc/voltage/rr";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S36>/SourceBlock' */
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
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S36>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_k[i] = b_zeroDelimTopic[i];
  }

  Sub_rover_sw_pwrtrain_24b_989.createSubscriber
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_k[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rover__Publisher_setupImpl_mygo(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[9];
  static const char_T b_zeroDelimTopic_0[9] = "/pwm/mil";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S18>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline_lx.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_lx.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_lx, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S18>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_1006.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::Publisher_setupImpl_mygou5bowb(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[9];
  static const char_T b_zeroDelimTopic_0[9] = "/duty/lf";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S72>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline_dy.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_dy.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_dy, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S72>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_1057.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::Publisher_setupImp_mygou5bowbhv(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[9];
  static const char_T b_zeroDelimTopic_0[9] = "/duty/lr";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S74>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline_dh.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_dh.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_dh, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S74>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_1059.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::Publisher_setupImpl_mygou5bowbh(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[9];
  static const char_T b_zeroDelimTopic_0[9] = "/duty/rf";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S73>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline_ld.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_ld.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_ld, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S73>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_1058.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::Publisher_setupIm_mygou5bowbhvx(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[9];
  static const char_T b_zeroDelimTopic_0[9] = "/duty/rr";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S75>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline_gu.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_gu.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_gu, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S75>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_1060.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rov_Subscriber_setupImpl_mygou5(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[16] = "/front_distance";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S31>/SourceBlock' */
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
    /* Start for MATLABSystem: '<S31>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_g[i] = b_zeroDelimTopic[i];
  }

  Sub_rover_sw_pwrtrain_24b_392.createSubscriber
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_g[0], qos_profile);
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

  /* Start for MATLABSystem: '<S14>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline_l.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_l.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_l, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<S14>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_907.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rover_Publisher_setupImpl_mygou(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[20] = "/throttel_regulated";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S19>/SinkBlock' */
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
    /* Start for MATLABSystem: '<S19>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_m[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_903.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_m[0], qos_profile);
}

void rover_sw_pwrtrain_24b::rov_Publisher_setupImpl_mygou5b(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[10];
  static const char_T b_zeroDelimTopic_0[10] = "v_batt_av";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S21>/SinkBlock' */
  rover_sw_pwrtrain_24b_B.deadline_d.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_d.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_d, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 10; i++) {
    /* Start for MATLABSystem: '<S21>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_1039.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::Subscriber_setupIm_mygou5bowbhv(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[8];
  static const char_T b_zeroDelimTopic_0[8] = "/rc/ch4";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S37>/SourceBlock' */
  rover_sw_pwrtrain_24b_B.deadline_b.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_b.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_b, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S37>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_rover_sw_pwrtrain_24b_1018.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rove_Publisher_setupImpl_mygou5(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[14];
  static const char_T b_zeroDelimTopic_0[14] = "/pwm/headlamp";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S20>/SinkBlock' */
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
  for (int32_T i = 0; i < 14; i++) {
    /* Start for MATLABSystem: '<S20>/SinkBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Pub_rover_sw_pwrtrain_24b_1009.createPublisher(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rover_s_Subscriber_setupImpl_my(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[13];
  static const char_T b_zeroDelimTopic_0[13] = "/vesc/rpm/rf";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S27>/SourceBlock' */
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
  for (int32_T i = 0; i < 13; i++) {
    /* Start for MATLABSystem: '<S27>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_rover_sw_pwrtrain_24b_1130.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rover__Subscriber_setupImpl_myg(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[13];
  static const char_T b_zeroDelimTopic_0[13] = "/vesc/rpm/lr";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S28>/SourceBlock' */
  rover_sw_pwrtrain_24b_B.deadline_n.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_n.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_n, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 13; i++) {
    /* Start for MATLABSystem: '<S28>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_rover_sw_pwrtrain_24b_1131.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rover_Subscriber_setupImpl_mygo(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[13];
  static const char_T b_zeroDelimTopic_0[13] = "/vesc/rpm/rf";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S29>/SourceBlock' */
  rover_sw_pwrtrain_24b_B.deadline_m.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_m.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_m, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 13; i++) {
    /* Start for MATLABSystem: '<S29>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_rover_sw_pwrtrain_24b_1132.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rove_Subscriber_setupImpl_mygou(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[13];
  static const char_T b_zeroDelimTopic_0[13] = "/vesc/rpm/rr";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S30>/SourceBlock' */
  rover_sw_pwrtrain_24b_B.deadline_g.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_g.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_g, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 13; i++) {
    /* Start for MATLABSystem: '<S30>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_rover_sw_pwrtrain_24b_1133.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

void rover_sw_pwrtrain_24b::rover_sw__Publisher_setupImpl_m(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[16] = "/stats/distance";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S15>/SinkBlock' */
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
    /* Start for MATLABSystem: '<S15>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_f[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_1122.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_f[0], qos_profile);
}

void rover_sw_pwrtrain_24b::ro_Publisher_setupImpl_mygou5bo(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[16] = "/stats/op_hours";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S22>/SinkBlock' */
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
    /* Start for MATLABSystem: '<S22>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_cv[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_1087.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_cv[0], qos_profile);
}

void rover_sw_pwrtrain_24b::r_Publisher_setupImpl_mygou5bow(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/stats/avg_speed";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S23>/SinkBlock' */
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
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S23>/SinkBlock' */
    rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_c[i] = b_zeroDelimTopic[i];
  }

  Pub_rover_sw_pwrtrain_24b_1115.createPublisher
    (&rover_sw_pwrtrain_24b_B.b_zeroDelimTopic_c[0], qos_profile);
}

void rover_sw_pwrtrain_24b::Subscriber_setupI_mygou5bowbhvx(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[8];
  static const char_T b_zeroDelimTopic_0[8] = "/rc/ch7";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S38>/SourceBlock' */
  rover_sw_pwrtrain_24b_B.deadline_o.sec = 0.0;
  rover_sw_pwrtrain_24b_B.deadline_o.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 rover_sw_pwrtrain_24b_B.deadline_o, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S38>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_rover_sw_pwrtrain_24b_1042.createSubscriber(&b_zeroDelimTopic[0],
    qos_profile);
}

real_T rover_sw_pwrtrain_24b::rover_sw_pwrtrain_24b_norm(const real_T x[2])
{
  real_T y;
  rover_sw_pwrtrain_24b_B.scale = 3.3121686421112381E-170;

  /* Start for MATLABSystem: '<S57>/Pure Pursuit' */
  rover_sw_pwrtrain_24b_B.absxk = fabs(x[0]);
  if (rover_sw_pwrtrain_24b_B.absxk > 3.3121686421112381E-170) {
    y = 1.0;
    rover_sw_pwrtrain_24b_B.scale = rover_sw_pwrtrain_24b_B.absxk;
  } else {
    rover_sw_pwrtrain_24b_B.t = rover_sw_pwrtrain_24b_B.absxk /
      3.3121686421112381E-170;
    y = rover_sw_pwrtrain_24b_B.t * rover_sw_pwrtrain_24b_B.t;
  }

  /* Start for MATLABSystem: '<S57>/Pure Pursuit' */
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

  /* Start for MATLABSystem: '<S57>/Pure Pursuit' */
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

  /* Start for MATLABSystem: '<S57>/Pure Pursuit' */
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

  /* End of Start for MATLABSystem: '<S57>/Pure Pursuit' */
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
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_b;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_m;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_c;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_l;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_lk;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_cs;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_e;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_lm;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_j;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_p;
  SL_Bus_std_msgs_Float32 rtb_SourceBlock_o2_m1;
  SL_Bus_std_msgs_Bool rtb_SourceBlock_o2_bb;
  SL_Bus_std_msgs_Bool rtb_SourceBlock_o2_ln;
  boolean_T p;
  boolean_T searchFlag;
  static const char_T a[23] = { 'W', 'h', 'e', 'e', 'l', 'S', 'p', 'e', 'e', 'd',
    's', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-' };

  static const char_T a_0[23] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
    'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e' };

  boolean_T exitg1;
  boolean_T rtb_OR;
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

  /* Integrator: '<S78>/Integrator' */
  rover_sw_pwrtrain_24b_B.Integrator[0] =
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[0];
  rover_sw_pwrtrain_24b_B.Integrator[1] =
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[1];
  rover_sw_pwrtrain_24b_B.Integrator[2] =
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[2];
  rtb_OR = rtmIsMajorTimeStep((&rover_sw_pwrtrain_24b_M));
  if (rtb_OR) {
    /* MATLABSystem: '<S57>/Pure Pursuit' incorporates:
     *  Constant: '<S58>/Constant4'
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
    p = true;
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
        p = false;
        exitg1 = true;
      }
    }

    if (p) {
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

    rover_sw_pwrtrain_24b_B.trueCount = 0;

    /* MATLABSystem: '<S57>/Pure Pursuit' */
    for (rover_sw_pwrtrain_24b_B.ret = 0; rover_sw_pwrtrain_24b_B.ret < 878;
         rover_sw_pwrtrain_24b_B.ret++) {
      searchFlag = (rover_sw_pwrtrain_24b_B.b[rover_sw_pwrtrain_24b_B.ret] &&
                    rover_sw_pwrtrain_24b_B.b[rover_sw_pwrtrain_24b_B.ret + 878]);
      rover_sw_pwrtrain_24b_B.bv[rover_sw_pwrtrain_24b_B.ret] = searchFlag;
      if (searchFlag) {
        rover_sw_pwrtrain_24b_B.trueCount++;
      }
    }

    rover_sw_pwrtrain_24b_B.tmp_size_idx_0 = rover_sw_pwrtrain_24b_B.trueCount;
    rover_sw_pwrtrain_24b_B.trueCount = 0;
    for (rover_sw_pwrtrain_24b_B.ret = 0; rover_sw_pwrtrain_24b_B.ret < 878;
         rover_sw_pwrtrain_24b_B.ret++) {
      /* MATLABSystem: '<S57>/Pure Pursuit' */
      if (rover_sw_pwrtrain_24b_B.bv[rover_sw_pwrtrain_24b_B.ret]) {
        /* Start for MATLABSystem: '<S57>/Pure Pursuit' */
        rover_sw_pwrtrain_24b_B.tmp_data[rover_sw_pwrtrain_24b_B.trueCount] =
          static_cast<int16_T>(rover_sw_pwrtrain_24b_B.ret);
        rover_sw_pwrtrain_24b_B.trueCount++;
      }
    }

    /* MATLABSystem: '<S57>/Pure Pursuit' incorporates:
     *  Constant: '<S58>/Constant4'
     *  ZeroOrderHold: '<S57>/Zero-Order Hold2'
     */
    if (rover_sw_pwrtrain_24b_B.tmp_size_idx_0 == 0) {
      /* Product: '<S79>/Product' */
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

      if (rover_sw_pwrtrain_24b_B.tmp_size_idx_0 == 1) {
        rover_sw_pwrtrain_24b_B.minDistance =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [0]];
        rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] =
          rover_sw_pwrtrain_24b_B.minDistance;
        rover_sw_pwrtrain_24b_B.dist =
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [0] + 878];
        rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] =
          rover_sw_pwrtrain_24b_B.dist;
        rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] =
          rover_sw_pwrtrain_24b_B.minDistance;
        rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] =
          rover_sw_pwrtrain_24b_B.dist;
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
        rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[0] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] -
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1]];
        rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[1];
        rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[1] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] -
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1] + 878];
        rover_sw_pwrtrain_24b_B.dist = rover_sw_pwrtrain_24b_norm
          (rover_sw_pwrtrain_24b_B.lookaheadStartPt_h);
        rover_sw_pwrtrain_24b_B.lookaheadIdx =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex + 1.0;
        rover_sw_pwrtrain_24b_B.trueCount = static_cast<int32_T>((1.0 -
          (rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex + 1.0)) + (
          static_cast<real_T>(rover_sw_pwrtrain_24b_B.tmp_size_idx_0) - 1.0)) -
          1;
        rover_sw_pwrtrain_24b_B.b_i = 0;
        exitg1 = false;
        while ((!exitg1) && (rover_sw_pwrtrain_24b_B.b_i <=
                             rover_sw_pwrtrain_24b_B.trueCount)) {
          rover_sw_pwrtrain_24b_B.i = rover_sw_pwrtrain_24b_B.lookaheadIdx +
            static_cast<real_T>(rover_sw_pwrtrain_24b_B.b_i);
          if ((!searchFlag) && (rover_sw_pwrtrain_24b_B.dist >
                                rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance))
          {
            exitg1 = true;
          } else {
            rover_sw_pwrtrain_24b_B.ret = rover_sw_pwrtrain_24b_B.tmp_data[
              static_cast<int32_T>(rover_sw_pwrtrain_24b_B.i + 1.0) - 1];
            rover_sw_pwrtrain_24b_B.i_e = rover_sw_pwrtrain_24b_B.tmp_data[
              static_cast<int32_T>(rover_sw_pwrtrain_24b_B.i) - 1];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[0] =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.i_e]
              - rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret];
            rover_sw_pwrtrain_24b_B.dv[0] =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.i_e];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret
              + 878];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp_b =
              rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.i_e
              + 878];
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[1] =
              rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp_b -
              rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp;
            rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] =
              rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp;
            rover_sw_pwrtrain_24b_B.dv[1] =
              rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp_b;
            rover_sw_pwrtrain_24b_B.dist += rover_sw_pwrtrain_24b_norm
              (rover_sw_pwrtrain_24b_B.lookaheadStartPt_h);
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

        rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[0] =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] -
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1]];
        rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[1] =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] -
          rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.tmp_data
          [static_cast<int32_T>(rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex
          + 1.0) - 1] + 878];
        rover_sw_pwrtrain_24b_B.dist = rover_sw_pwrtrain_24b_norm
          (rover_sw_pwrtrain_24b_B.lookaheadStartPt_h);
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
        rover_sw_pwrtrain_24b_B.minDistance = rover_sw_pwrtrain_24b_B.dist -
          rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance;
        rover_sw_pwrtrain_24b_B.lookaheadIdx =
          rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex;
        while ((rover_sw_pwrtrain_24b_B.minDistance < 0.0) &&
               (rover_sw_pwrtrain_24b_B.lookaheadIdx < static_cast<real_T>
                (rover_sw_pwrtrain_24b_B.tmp_size_idx_0) - 1.0)) {
          rover_sw_pwrtrain_24b_B.lookaheadIdx++;
          rover_sw_pwrtrain_24b_B.trueCount = rover_sw_pwrtrain_24b_B.tmp_data[
            static_cast<int32_T>(rover_sw_pwrtrain_24b_B.lookaheadIdx) - 1];
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.trueCount];
          rover_sw_pwrtrain_24b_B.ret = rover_sw_pwrtrain_24b_B.tmp_data[
            static_cast<int32_T>(rover_sw_pwrtrain_24b_B.lookaheadIdx + 1.0) - 1];
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret];
          rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[0] =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.trueCount]
            - rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret];
          rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.trueCount
            + 878];
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] =
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp;
          rover_sw_pwrtrain_24b_B.minDistance =
            rover_sw_pwrtrain_24b_ConstP.Constant4_Value[rover_sw_pwrtrain_24b_B.ret
            + 878];
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] =
            rover_sw_pwrtrain_24b_B.minDistance;
          rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[1] =
            rover_sw_pwrtrain_24b_B.lookaheadStartPt_tmp -
            rover_sw_pwrtrain_24b_B.minDistance;
          rover_sw_pwrtrain_24b_B.dist += rover_sw_pwrtrain_24b_norm
            (rover_sw_pwrtrain_24b_B.lookaheadStartPt_h);
          rover_sw_pwrtrain_24b_B.minDistance = rover_sw_pwrtrain_24b_B.dist -
            rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance;
        }

        rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[0] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[0] -
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0];
        rover_sw_pwrtrain_24b_B.lookaheadStartPt_h[1] =
          rover_sw_pwrtrain_24b_B.lookaheadStartPt[1] -
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1];
        rover_sw_pwrtrain_24b_B.dist = rover_sw_pwrtrain_24b_B.minDistance /
          rover_sw_pwrtrain_24b_norm(rover_sw_pwrtrain_24b_B.lookaheadStartPt_h);
        if (rover_sw_pwrtrain_24b_B.dist > 0.0) {
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] = (1.0 -
            rover_sw_pwrtrain_24b_B.dist) *
            rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] +
            rover_sw_pwrtrain_24b_B.dist *
            rover_sw_pwrtrain_24b_B.lookaheadStartPt[0];
          rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] = (1.0 -
            rover_sw_pwrtrain_24b_B.dist) *
            rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] +
            rover_sw_pwrtrain_24b_B.dist *
            rover_sw_pwrtrain_24b_B.lookaheadStartPt[1];
        }
      }

      rover_sw_pwrtrain_24b_B.dist = rt_atan2d_snf
        (rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] -
         rover_sw_pwrtrain_24b_B.Integrator[1],
         rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] -
         rover_sw_pwrtrain_24b_B.Integrator[0]) -
        rover_sw_pwrtrain_24b_B.Integrator[2];
      if (fabs(rover_sw_pwrtrain_24b_B.dist) > 3.1415926535897931) {
        if (rtIsNaN(rover_sw_pwrtrain_24b_B.dist + 3.1415926535897931) ||
            rtIsInf(rover_sw_pwrtrain_24b_B.dist + 3.1415926535897931)) {
          rover_sw_pwrtrain_24b_B.minDistance = (rtNaN);
        } else if (rover_sw_pwrtrain_24b_B.dist + 3.1415926535897931 == 0.0) {
          rover_sw_pwrtrain_24b_B.minDistance = 0.0;
        } else {
          rover_sw_pwrtrain_24b_B.minDistance = fmod
            (rover_sw_pwrtrain_24b_B.dist + 3.1415926535897931,
             6.2831853071795862);
          searchFlag = (rover_sw_pwrtrain_24b_B.minDistance == 0.0);
          if (!searchFlag) {
            rover_sw_pwrtrain_24b_B.lookaheadIdx = fabs
              ((rover_sw_pwrtrain_24b_B.dist + 3.1415926535897931) /
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
            (rover_sw_pwrtrain_24b_B.dist + 3.1415926535897931 > 0.0)) {
          rover_sw_pwrtrain_24b_B.minDistance = 6.2831853071795862;
        }

        rover_sw_pwrtrain_24b_B.dist = rover_sw_pwrtrain_24b_B.minDistance -
          3.1415926535897931;
      }

      rover_sw_pwrtrain_24b_B.Product1 = 2.0 * sin(rover_sw_pwrtrain_24b_B.dist)
        / rover_sw_pwrtrain_24b_DW.obj.LookaheadDistance;
      if (rtIsNaN(rover_sw_pwrtrain_24b_B.Product1)) {
        rover_sw_pwrtrain_24b_B.Product1 = 0.0;
      }

      if (fabs(fabs(rover_sw_pwrtrain_24b_B.dist) - 3.1415926535897931) <
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
          rover_sw_pwrtrain_24b_B.minDistance = (rtNaN);
        } else if (rover_sw_pwrtrain_24b_B.Product1 < 0.0) {
          rover_sw_pwrtrain_24b_B.minDistance = -1.0;
        } else {
          rover_sw_pwrtrain_24b_B.minDistance =
            (rover_sw_pwrtrain_24b_B.Product1 > 0.0);
        }

        rover_sw_pwrtrain_24b_B.Product1 = rover_sw_pwrtrain_24b_B.minDistance *
          rover_sw_pwrtrain_24b_DW.obj.MaxAngularVelocity;
      }

      /* Product: '<S79>/Product' incorporates:
       *  Constant: '<S58>/Constant4'
       *  ZeroOrderHold: '<S57>/Zero-Order Hold2'
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
  }

  /* MATLABSystem: '<S78>/MATLAB System' */
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
  p = true;
  rover_sw_pwrtrain_24b_B.ret = 0;
  exitg1 = false;
  while ((!exitg1) && (rover_sw_pwrtrain_24b_B.ret < 2)) {
    if (!(rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[rover_sw_pwrtrain_24b_B.ret]
          ==
          rover_sw_pwrtrain_24_ConstInitP.MATLABSystem_WheelSpeedRange[rover_sw_pwrtrain_24b_B.ret]))
    {
      p = false;
      exitg1 = true;
    } else {
      rover_sw_pwrtrain_24b_B.ret++;
    }
  }

  if (p) {
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
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius = 0.095;
    }

    if (rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[0]) {
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.TrackWidth = 0.5;
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

  rover_sw_pwrtrain_24b_B.dist = 0.0;

  /* MATLABSystem: '<S78>/MATLAB System' */
  rover_sw_pwrtrain_24b_B.MATLABSystem[2] = 0.0;

  /* MATLABSystem: '<S78>/MATLAB System' */
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

    rover_sw_pwrtrain_24b_B.dist = (rover_sw_pwrtrain_24b_B.lookaheadIdx +
      rover_sw_pwrtrain_24b_B.minDistance) *
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius / 2.0;

    /* MATLABSystem: '<S78>/MATLAB System' */
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

    p = !rtIsNaN(rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1]);
    if ((!(rover_sw_pwrtrain_24b_B.minDistance <=
           rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1])) && p) {
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
           rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1])) && p) {
      rover_sw_pwrtrain_24b_B.lookaheadIdx =
        rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelSpeedRange[1];
    }

    rover_sw_pwrtrain_24b_B.dist = (rover_sw_pwrtrain_24b_B.lookaheadIdx +
      rover_sw_pwrtrain_24b_B.minDistance) *
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius / 2.0;

    /* MATLABSystem: '<S78>/MATLAB System' */
    rover_sw_pwrtrain_24b_B.MATLABSystem[2] =
      (rover_sw_pwrtrain_24b_B.lookaheadIdx -
       rover_sw_pwrtrain_24b_B.minDistance) *
      rover_sw_pwrtrain_24b_DW.obj_j.KinModel.WheelRadius /
      (rover_sw_pwrtrain_24b_DW.obj_j.KinModel.TrackWidth / 2.0 * 2.0);
    break;
  }

  /* MATLABSystem: '<S78>/MATLAB System' */
  rover_sw_pwrtrain_24b_B.MATLABSystem[0] = cos
    (rover_sw_pwrtrain_24b_B.Integrator[2]) * rover_sw_pwrtrain_24b_B.dist;
  rover_sw_pwrtrain_24b_B.MATLABSystem[1] = sin
    (rover_sw_pwrtrain_24b_B.Integrator[2]) * rover_sw_pwrtrain_24b_B.dist;
  if (rtb_OR) {
    /* MATLABSystem: '<S26>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_nc =
      Sub_rover_sw_pwrtrain_24b_391.getLatestMessage(&rtb_SourceBlock_o2_p);

    /* Outputs for Enabled SubSystem: '<S26>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_nc,
      &rtb_SourceBlock_o2_p, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_o);

    /* End of Outputs for SubSystem: '<S26>/Enabled Subsystem' */

    /* MATLABSystem: '<S25>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_mk =
      Sub_rover_sw_pwrtrain_24b_390.getLatestMessage(&rtb_SourceBlock_o2_m1);

    /* Outputs for Enabled SubSystem: '<S25>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_mk,
      &rtb_SourceBlock_o2_m1, &rover_sw_pwrtrain_24b_B.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S25>/Enabled Subsystem' */

    /* MATLABSystem: '<S33>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_n =
      Sub_rover_sw_pwrtrain_24b_521.getLatestMessage(&rtb_SourceBlock_o2_ln);

    /* Outputs for Enabled SubSystem: '<S33>/Enabled Subsystem' */
    rover_sw_pwr_EnabledSubsystem_g(rover_sw_pwrtrain_24b_B.SourceBlock_o1_n,
      &rtb_SourceBlock_o2_ln, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_g);

    /* End of Outputs for SubSystem: '<S33>/Enabled Subsystem' */

    /* MATLABSystem: '<S32>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_nq =
      Sub_rover_sw_pwrtrain_24b_986.getLatestMessage(&rtb_SourceBlock_o2_l);

    /* Outputs for Enabled SubSystem: '<S32>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_nq,
      &rtb_SourceBlock_o2_l, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_dt);

    /* End of Outputs for SubSystem: '<S32>/Enabled Subsystem' */

    /* MATLABSystem: '<S34>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_l =
      Sub_rover_sw_pwrtrain_24b_987.getLatestMessage(&rtb_SourceBlock_o2_c);

    /* Outputs for Enabled SubSystem: '<S34>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_l,
      &rtb_SourceBlock_o2_c, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_ov);

    /* End of Outputs for SubSystem: '<S34>/Enabled Subsystem' */

    /* MATLABSystem: '<S35>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_k =
      Sub_rover_sw_pwrtrain_24b_988.getLatestMessage(&rtb_SourceBlock_o2_m);

    /* Outputs for Enabled SubSystem: '<S35>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_k,
      &rtb_SourceBlock_o2_m, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_b);

    /* End of Outputs for SubSystem: '<S35>/Enabled Subsystem' */

    /* MATLABSystem: '<S36>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_f =
      Sub_rover_sw_pwrtrain_24b_989.getLatestMessage(&rtb_SourceBlock_o2_b);

    /* Outputs for Enabled SubSystem: '<S36>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_f,
      &rtb_SourceBlock_o2_b, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_lp);

    /* End of Outputs for SubSystem: '<S36>/Enabled Subsystem' */

    /* Product: '<S2>/Divide' incorporates:
     *  Constant: '<S2>/Constant1'
     *  Sum: '<S2>/Add'
     */
    rover_sw_pwrtrain_24b_B.dist =
      (((rover_sw_pwrtrain_24b_B.EnabledSubsystem_dt.In1.data +
         rover_sw_pwrtrain_24b_B.EnabledSubsystem_ov.In1.data) +
        rover_sw_pwrtrain_24b_B.EnabledSubsystem_b.In1.data) +
       rover_sw_pwrtrain_24b_B.EnabledSubsystem_lp.In1.data) / 4.0;

    /* Chart: '<S53>/Chart' incorporates:
     *  Constant: '<S60>/Lower Limit'
     *  Constant: '<S60>/Upper Limit'
     *  Logic: '<S60>/AND'
     *  RelationalOperator: '<S60>/Lower Test'
     *  RelationalOperator: '<S60>/Upper Test'
     */
    if (rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c < 511) {
      rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c = static_cast<uint16_T>
        (rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c + 1);
    }

    if (rover_sw_pwrtrain_24b_DW.is_active_c6_rover_sw_pwrtrain_ == 0) {
      rover_sw_pwrtrain_24b_DW.is_active_c6_rover_sw_pwrtrain_ = 1U;
      rover_sw_pwrtrain_24b_DW.is_c6_rover_sw_pwrtrain_24b =
        rover_sw_p_IN_TRAILHEAD_UNARMED;
      rover_sw_pwrtrain_24b_B.throttle_cmd = 0.0F;
      rover_sw_pwrtrain_24b_B.steering_cmd = 0.0F;
      rover_sw_pwrtrain_24b_B.mil_lamp_pwm = 1200;
    } else {
      switch (rover_sw_pwrtrain_24b_DW.is_c6_rover_sw_pwrtrain_24b) {
       case rover_sw_pwr_IN_TRAILHEAD_ARMED:
        if ((rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c >= 500) &&
            (rover_sw_pwrtrain_24b_B.dist < 20.0)) {
          rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ARMED = 0;
          rover_sw_pwrtrain_24b_DW.is_c6_rover_sw_pwrtrain_24b =
            rover__IN_TRAILHEAD_ERROR_STATE;
          rover_sw_pwrtrain_24b_B.throttle_cmd = 0.0F;
          rover_sw_pwrtrain_24b_B.steering_cmd = 0.0F;
          rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c = 0U;
          rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ERROR_STATE =
            rover_sw_pwrtrain_24b_IN_mil_on;
        } else if (!rover_sw_pwrtrain_24b_B.EnabledSubsystem_g.In1.data) {
          rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ARMED = 0;
          rover_sw_pwrtrain_24b_DW.is_c6_rover_sw_pwrtrain_24b =
            rover_sw_p_IN_TRAILHEAD_UNARMED;
          rover_sw_pwrtrain_24b_B.throttle_cmd = 0.0F;
          rover_sw_pwrtrain_24b_B.steering_cmd = 0.0F;
          rover_sw_pwrtrain_24b_B.mil_lamp_pwm = 1200;
        } else {
          rover_sw_pwrtrain_24b_B.steering_cmd =
            rover_sw_pwrtrain_24b_B.EnabledSubsystem_o.In1.data;
          if (rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ARMED ==
              rover_sw_pwrtrain_IN_FAST_DECEL) {
            if (rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data >
                rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_d) {
              rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ARMED =
                rover_sw_pwrtrain_IN_SLOW_ACCEL;
              rover_sw_pwrtrain_24b_DW.max_increase_g = 0.005;
            } else {
              rover_sw_pwrtrain_24b_B.throttle_cmd =
                rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_d -
                static_cast<real32_T>(rover_sw_pwrtrain_24b_DW.max_decrease_l);
              if ((rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data >=
                   rover_sw_pwrtrain_24b_B.throttle_cmd) || rtIsNaNF
                  (rover_sw_pwrtrain_24b_B.throttle_cmd)) {
                rover_sw_pwrtrain_24b_B.throttle_cmd =
                  rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data;
              }
            }

            /* case IN_SLOW_ACCEL: */
          } else if (rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data <=
                     rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_d) {
            rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ARMED =
              rover_sw_pwrtrain_IN_FAST_DECEL;
            rover_sw_pwrtrain_24b_DW.max_decrease_l = 0.05;
          } else {
            rover_sw_pwrtrain_24b_B.throttle_cmd =
              rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_d + static_cast<real32_T>
              (rover_sw_pwrtrain_24b_DW.max_increase_g);
            if ((rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data <=
                 rover_sw_pwrtrain_24b_B.throttle_cmd) || rtIsNaNF
                (rover_sw_pwrtrain_24b_B.throttle_cmd)) {
              rover_sw_pwrtrain_24b_B.throttle_cmd =
                rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data;
            }
          }
        }
        break;

       case rover__IN_TRAILHEAD_ERROR_STATE:
        if (!rover_sw_pwrtrain_24b_B.EnabledSubsystem_g.In1.data) {
          rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ERROR_STATE = 0;
          rover_sw_pwrtrain_24b_DW.is_c6_rover_sw_pwrtrain_24b =
            rover_sw_p_IN_TRAILHEAD_UNARMED;
          rover_sw_pwrtrain_24b_B.throttle_cmd = 0.0F;
          rover_sw_pwrtrain_24b_B.steering_cmd = 0.0F;
          rover_sw_pwrtrain_24b_B.mil_lamp_pwm = 1200;
        } else if (rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ERROR_STATE ==
                   rover_sw_pwrtrain_24_IN_mil_off) {
          if (rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c >= 100) {
            rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c = 0U;
            rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ERROR_STATE =
              rover_sw_pwrtrain_24b_IN_mil_on;
          } else {
            rover_sw_pwrtrain_24b_B.mil_lamp_pwm = 500;
          }

          /* case IN_mil_on: */
        } else if (rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c >= 300) {
          rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c = 0U;
          rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ERROR_STATE =
            rover_sw_pwrtrain_24_IN_mil_off;
        } else {
          rover_sw_pwrtrain_24b_B.mil_lamp_pwm = 1150;
        }
        break;

       case rover_IN_TRAILHEAD_READY_TO_ARM:
        if (rover_sw_pwrtrain_24b_B.EnabledSubsystem_g.In1.data &&
            (rover_sw_pwrtrain_24b_B.dist >= 20.0) && (static_cast<real32_T>
             (fabs(static_cast<real_T>
                   (rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data))) >= 0.1))
        {
          rover_sw_pwrtrain_24b_DW.temporalCounter_i1_c = 0U;
          rover_sw_pwrtrain_24b_DW.is_c6_rover_sw_pwrtrain_24b =
            rover_sw_pwr_IN_TRAILHEAD_ARMED;
          rover_sw_pwrtrain_24b_B.mil_lamp_pwm = 1500;
          rover_sw_pwrtrain_24b_DW.is_TRAILHEAD_ARMED =
            rover_sw_pwrtrain_IN_SLOW_ACCEL;
          rover_sw_pwrtrain_24b_DW.max_increase_g = 0.005;
        }
        break;

       default:
        /* case IN_TRAILHEAD_UNARMED: */
        if ((!rover_sw_pwrtrain_24b_B.EnabledSubsystem_g.In1.data) &&
            (rover_sw_pwrtrain_24b_B.dist >= 20.0) &&
            ((rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data >= -0.1F) &&
             (rover_sw_pwrtrain_24b_B.EnabledSubsystem.In1.data <= 0.1F))) {
          rover_sw_pwrtrain_24b_DW.is_c6_rover_sw_pwrtrain_24b =
            rover_IN_TRAILHEAD_READY_TO_ARM;
          rover_sw_pwrtrain_24b_B.throttle_cmd = 0.0F;
          rover_sw_pwrtrain_24b_B.steering_cmd = 0.0F;
          rover_sw_pwrtrain_24b_B.mil_lamp_pwm = 1300;
        }
        break;
      }
    }

    /* End of Chart: '<S53>/Chart' */

    /* BusAssignment: '<S1>/Bus Assignment4' */
    rover_sw_pwrtrain_24b_B.BusAssignment4.data =
      rover_sw_pwrtrain_24b_B.mil_lamp_pwm;

    /* MATLABSystem: '<S18>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1006.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment4);

    /* BusAssignment: '<S68>/Bus Assignment1' incorporates:
     *  Product: '<S76>/Product'
     *  Sum: '<S76>/Plus'
     */
    rover_sw_pwrtrain_24b_B.BusAssignment1_h.data =
      rover_sw_pwrtrain_24b_B.throttle_cmd *
      rover_sw_pwrtrain_24b_B.steering_cmd +
      rover_sw_pwrtrain_24b_B.throttle_cmd;

    /* MATLABSystem: '<S72>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1057.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment1_h);

    /* MATLABSystem: '<S74>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1059.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment1_h);

    /* BusAssignment: '<S68>/Bus Assignment2' incorporates:
     *  Product: '<S76>/Product1'
     *  Sum: '<S76>/Plus2'
     */
    rover_sw_pwrtrain_24b_B.BusAssignment2.data =
      rover_sw_pwrtrain_24b_B.throttle_cmd -
      rover_sw_pwrtrain_24b_B.throttle_cmd *
      rover_sw_pwrtrain_24b_B.steering_cmd;

    /* MATLABSystem: '<S73>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1058.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment2);

    /* MATLABSystem: '<S75>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1060.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment2);

    /* MATLABSystem: '<S31>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_m =
      Sub_rover_sw_pwrtrain_24b_392.getLatestMessage(&rtb_SourceBlock_o2_lk);

    /* Outputs for Enabled SubSystem: '<S31>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_m,
      &rtb_SourceBlock_o2_lk, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_p);

    /* End of Outputs for SubSystem: '<S31>/Enabled Subsystem' */

    /* Switch: '<S2>/Switch' */
    if (rover_sw_pwrtrain_24b_B.SourceBlock_o1_m) {
      /* Switch: '<S2>/Switch' incorporates:
       *  Abs: '<S2>/Abs'
       */
      rover_sw_pwrtrain_24b_B.Switch = static_cast<real32_T>(fabs
        (static_cast<real_T>(rover_sw_pwrtrain_24b_B.EnabledSubsystem_p.In1.data)));
    } else {
      /* Switch: '<S2>/Switch' incorporates:
       *  Constant: '<S2>/Constant'
       */
      rover_sw_pwrtrain_24b_B.Switch = 20000.0;
    }

    /* End of Switch: '<S2>/Switch' */

    /* Sum: '<S65>/Add' incorporates:
     *  Gain: '<S65>/1-alpha'
     *  Gain: '<S65>/alpha'
     *  UnitDelay: '<S65>/Unit Delay'
     */
    rover_sw_pwrtrain_24b_B.minDistance = 0.4 * rover_sw_pwrtrain_24b_B.Switch +
      0.6 * rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE;

    /* Switch: '<S65>/Switch' incorporates:
     *  Constant: '<S65>/Constant'
     *  Constant: '<S65>/Constant2'
     *  Constant: '<S66>/Constant'
     *  Constant: '<S67>/Constant'
     *  RelationalOperator: '<S66>/Compare'
     *  RelationalOperator: '<S67>/Compare'
     *  Switch: '<S65>/Switch1'
     */
    if (rover_sw_pwrtrain_24b_B.minDistance <= 50.0) {
      rover_sw_pwrtrain_24b_B.Switch_b = MILD;
    } else if (rover_sw_pwrtrain_24b_B.minDistance <= 80.0) {
      /* Switch: '<S65>/Switch1' incorporates:
       *  Constant: '<S65>/Constant1'
       */
      rover_sw_pwrtrain_24b_B.Switch_b = SEVERE;
    } else {
      rover_sw_pwrtrain_24b_B.Switch_b = NONE;
    }

    /* End of Switch: '<S65>/Switch' */

    /* Logic: '<S2>/OR' */
    rtb_OR = (rover_sw_pwrtrain_24b_B.SourceBlock_o1_mk ||
              rover_sw_pwrtrain_24b_B.SourceBlock_o1_nc ||
              rover_sw_pwrtrain_24b_B.SourceBlock_o1_n);

    /* Chart: '<S69>/adaptive_throttle_contller' incorporates:
     *  Constant: '<S69>/Constant'
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
            (rover_sw_pwrtrain_24b_B.throttle_cmd <= -0.1)) {
          rover_sw_pwrtrain_24b_DW.is_COLLISION_FLT_FAST_STOP = 0;
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
          rover_sw_pwrtrain_24b_DW.is_LOST_COMM_OPERATION = 0;
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
          rover_sw_pwrtrain_24b_DW.is_LOST_COMM_OPERATION = 0;
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
          rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION = 0;
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
          rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION = 0;
          rover_sw_pwrtrain_24b_DW.is_c3_rover_sw_pwrtrain_24b =
            rover_sw_p_IN_LOSS_COMM_MONITOR;
          rover_sw_pwrtrain_24b_DW.loss_tmr = 0.0;
          rover_sw_pwrtrain_24b_DW.comm_ref_throttle =
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i;
        } else if (rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION ==
                   rover_sw_pwrtrain_IN_FAST_DECEL) {
          /*  20% per cycle */
          rover_sw_pwrtrain_24b_B.throttle_out =
            rover_sw_pwrtrain_24b_B.throttle_cmd >=
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i -
            rover_sw_pwrtrain_24b_DW.max_decrease ? static_cast<real_T>
            (rover_sw_pwrtrain_24b_B.throttle_cmd) :
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i -
              rover_sw_pwrtrain_24b_DW.max_decrease;

          /* case IN_SLOW_ACCEL: */
        } else if (rover_sw_pwrtrain_24b_B.throttle_cmd <=
                   rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i) {
          rover_sw_pwrtrain_24b_DW.is_NORMAL_OPERATION =
            rover_sw_pwrtrain_IN_FAST_DECEL;

          /*  Faster deceleration allowed */
          rover_sw_pwrtrain_24b_DW.max_decrease = 0.05;
        } else {
          /*  1% per cycle */
          rover_sw_pwrtrain_24b_B.throttle_out =
            rover_sw_pwrtrain_24b_B.throttle_cmd <=
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i +
            rover_sw_pwrtrain_24b_DW.max_increase ? static_cast<real_T>
            (rover_sw_pwrtrain_24b_B.throttle_cmd) :
            rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_i +
              rover_sw_pwrtrain_24b_DW.max_increase;
        }
        break;
      }
    }

    /* End of Chart: '<S69>/adaptive_throttle_contller' */

    /* DataTypeConversion: '<S1>/Data Type Conversion1' */
    rover_sw_pwrtrain_24b_B.minDistance = floor
      (rover_sw_pwrtrain_24b_B.decay_mode);
    if (rtIsNaN(rover_sw_pwrtrain_24b_B.minDistance) || rtIsInf
        (rover_sw_pwrtrain_24b_B.minDistance)) {
      rover_sw_pwrtrain_24b_B.minDistance = 0.0;
    } else {
      rover_sw_pwrtrain_24b_B.minDistance = fmod
        (rover_sw_pwrtrain_24b_B.minDistance, 4.294967296E+9);
    }

    /* BusAssignment: '<S1>/Bus Assignment1' incorporates:
     *  DataTypeConversion: '<S1>/Data Type Conversion1'
     */
    rover_sw_pwrtrain_24b_B.BusAssignment1.data =
      rover_sw_pwrtrain_24b_B.minDistance < 0.0 ? -static_cast<int32_T>(
      static_cast<uint32_T>(-rover_sw_pwrtrain_24b_B.minDistance)) :
      static_cast<int32_T>(static_cast<uint32_T>
      (rover_sw_pwrtrain_24b_B.minDistance));

    /* MATLABSystem: '<S14>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_907.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment1);

    /* DataTypeConversion: '<S1>/Data Type Conversion5' */
    rover_sw_pwrtrain_24b_B.minDistance = floor
      (rover_sw_pwrtrain_24b_B.throttle_out);
    if (rtIsNaN(rover_sw_pwrtrain_24b_B.minDistance) || rtIsInf
        (rover_sw_pwrtrain_24b_B.minDistance)) {
      rover_sw_pwrtrain_24b_B.minDistance = 0.0;
    } else {
      rover_sw_pwrtrain_24b_B.minDistance = fmod
        (rover_sw_pwrtrain_24b_B.minDistance, 4.294967296E+9);
    }

    /* BusAssignment: '<S1>/Bus Assignment5' incorporates:
     *  DataTypeConversion: '<S1>/Data Type Conversion5'
     */
    rover_sw_pwrtrain_24b_B.BusAssignment5.data =
      rover_sw_pwrtrain_24b_B.minDistance < 0.0 ? -static_cast<int32_T>(
      static_cast<uint32_T>(-rover_sw_pwrtrain_24b_B.minDistance)) :
      static_cast<int32_T>(static_cast<uint32_T>
      (rover_sw_pwrtrain_24b_B.minDistance));

    /* MATLABSystem: '<S19>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_903.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment5);

    /* DataTypeConversion: '<S1>/Data Type Conversion' */
    rover_sw_pwrtrain_24b_B.minDistance = floor(rover_sw_pwrtrain_24b_B.dist);
    if (rtIsNaN(rover_sw_pwrtrain_24b_B.minDistance) || rtIsInf
        (rover_sw_pwrtrain_24b_B.minDistance)) {
      rover_sw_pwrtrain_24b_B.minDistance = 0.0;
    } else {
      rover_sw_pwrtrain_24b_B.minDistance = fmod
        (rover_sw_pwrtrain_24b_B.minDistance, 4.294967296E+9);
    }

    /* BusAssignment: '<S1>/Bus Assignment7' incorporates:
     *  DataTypeConversion: '<S1>/Data Type Conversion'
     */
    rover_sw_pwrtrain_24b_B.BusAssignment7.data =
      rover_sw_pwrtrain_24b_B.minDistance < 0.0 ? -static_cast<int32_T>(
      static_cast<uint32_T>(-rover_sw_pwrtrain_24b_B.minDistance)) :
      static_cast<int32_T>(static_cast<uint32_T>
      (rover_sw_pwrtrain_24b_B.minDistance));

    /* MATLABSystem: '<S21>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1039.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment7);

    /* MATLABSystem: '<S37>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_b =
      Sub_rover_sw_pwrtrain_24b_1018.getLatestMessage(&rtb_SourceBlock_o2);

    /* Outputs for Enabled SubSystem: '<S37>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_b,
      &rtb_SourceBlock_o2, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_pq);

    /* End of Outputs for SubSystem: '<S37>/Enabled Subsystem' */

    /* Switch: '<S61>/Switch' incorporates:
     *  Constant: '<S63>/Constant'
     *  Constant: '<S64>/Constant'
     *  RelationalOperator: '<S63>/Compare'
     *  RelationalOperator: '<S64>/Compare'
     *  Switch: '<S61>/Switch1'
     */
    if (rover_sw_pwrtrain_24b_B.EnabledSubsystem_pq.In1.data == 0.0F) {
      /* BusAssignment: '<S1>/Bus Assignment6' incorporates:
       *  Constant: '<S61>/Constant'
       */
      rover_sw_pwrtrain_24b_B.BusAssignment6.data = 1150;
    } else if (rover_sw_pwrtrain_24b_B.EnabledSubsystem_pq.In1.data == -1.0F) {
      /* Switch: '<S61>/Switch1' incorporates:
       *  BusAssignment: '<S1>/Bus Assignment6'
       *  Constant: '<S61>/Constant1'
       */
      rover_sw_pwrtrain_24b_B.BusAssignment6.data = 500;
    } else {
      /* BusAssignment: '<S1>/Bus Assignment6' incorporates:
       *  Constant: '<S61>/Constant2'
       *  Switch: '<S61>/Switch1'
       */
      rover_sw_pwrtrain_24b_B.BusAssignment6.data = 2500;
    }

    /* End of Switch: '<S61>/Switch' */

    /* MATLABSystem: '<S20>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1009.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment6);

    /* MATLABSystem: '<S27>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_g =
      Sub_rover_sw_pwrtrain_24b_1130.getLatestMessage(&rtb_SourceBlock_o2_j);

    /* Outputs for Enabled SubSystem: '<S27>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_g,
      &rtb_SourceBlock_o2_j, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_ot);

    /* End of Outputs for SubSystem: '<S27>/Enabled Subsystem' */

    /* MATLABSystem: '<S28>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_a =
      Sub_rover_sw_pwrtrain_24b_1131.getLatestMessage(&rtb_SourceBlock_o2_lm);

    /* Outputs for Enabled SubSystem: '<S28>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_a,
      &rtb_SourceBlock_o2_lm, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_d);

    /* End of Outputs for SubSystem: '<S28>/Enabled Subsystem' */

    /* MATLABSystem: '<S29>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_o =
      Sub_rover_sw_pwrtrain_24b_1132.getLatestMessage(&rtb_SourceBlock_o2_e);

    /* Outputs for Enabled SubSystem: '<S29>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_o,
      &rtb_SourceBlock_o2_e, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_l);

    /* End of Outputs for SubSystem: '<S29>/Enabled Subsystem' */

    /* MATLABSystem: '<S30>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1_fq =
      Sub_rover_sw_pwrtrain_24b_1133.getLatestMessage(&rtb_SourceBlock_o2_cs);

    /* Outputs for Enabled SubSystem: '<S30>/Enabled Subsystem' */
    rover_sw_pwrtr_EnabledSubsystem(rover_sw_pwrtrain_24b_B.SourceBlock_o1_fq,
      &rtb_SourceBlock_o2_cs, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_o4);

    /* End of Outputs for SubSystem: '<S30>/Enabled Subsystem' */

    /* Product: '<S62>/Product' incorporates:
     *  Constant: '<S2>/Constant2'
     *  Constant: '<S62>/const'
     *  Constant: '<S62>/const1'
     *  Constant: '<S62>/gear_ration'
     *  Constant: '<S62>/motor_pole_pair'
     *  Constant: '<S62>/pi'
     *  Constant: '<S62>/wheel_dia'
     *  Product: '<S2>/Divide1'
     *  Product: '<S62>/Divide'
     *  Product: '<S62>/Divide1'
     *  Product: '<S62>/Divide2'
     *  Sum: '<S2>/Add1'
     */
    rover_sw_pwrtrain_24b_B.wheel_spd_kph = static_cast<real32_T>
      (static_cast<real32_T>(static_cast<real32_T>(static_cast<real32_T>
         ((((rover_sw_pwrtrain_24b_B.EnabledSubsystem_ot.In1.data +
             rover_sw_pwrtrain_24b_B.EnabledSubsystem_d.In1.data) +
            rover_sw_pwrtrain_24b_B.EnabledSubsystem_l.In1.data) +
           rover_sw_pwrtrain_24b_B.EnabledSubsystem_o4.In1.data) / 4.0 / 2.0 /
          10.0 * 3.14) * 0.192) / 60.0) * 3.6);

    /* Chart: '<S1>/op_data' */
    if (rover_sw_pwrtrain_24b_DW.is_active_c2_rover_sw_pwrtrain_ == 0) {
      rover_sw_pwrtrain_24b_DW.is_active_c2_rover_sw_pwrtrain_ = 1U;
      rover_sw_pwrtrain_24b_DW.tmr_sec = 0.0F;

      /* BusAssignment: '<S1>/Bus Assignment10' */
      rover_sw_pwrtrain_24b_B.BusAssignment10.data = 0.0F;
    } else {
      rover_sw_pwrtrain_24b_DW.tmr_sec += 0.01F;
      rover_sw_pwrtrain_24b_B.tmr_hr = rover_sw_pwrtrain_24b_DW.tmr_sec /
        3600.0F;

      /* BusAssignment: '<S1>/Bus Assignment10' */
      rover_sw_pwrtrain_24b_B.BusAssignment10.data =
        rover_sw_pwrtrain_24b_B.wheel_spd_kph * rover_sw_pwrtrain_24b_B.tmr_hr;
    }

    /* End of Chart: '<S1>/op_data' */

    /* MATLABSystem: '<S15>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1122.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment10);

    /* BusAssignment: '<S1>/Bus Assignment8' */
    rover_sw_pwrtrain_24b_B.BusAssignment8.data = rover_sw_pwrtrain_24b_B.tmr_hr;

    /* MATLABSystem: '<S22>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1087.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment8);

    /* BusAssignment: '<S1>/Bus Assignment9' */
    rover_sw_pwrtrain_24b_B.BusAssignment9.data =
      rover_sw_pwrtrain_24b_B.wheel_spd_kph;

    /* MATLABSystem: '<S23>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_1115.publish
      (&rover_sw_pwrtrain_24b_B.BusAssignment9);

    /* MATLABSystem: '<S38>/SourceBlock' */
    rover_sw_pwrtrain_24b_B.SourceBlock_o1 =
      Sub_rover_sw_pwrtrain_24b_1042.getLatestMessage(&rtb_SourceBlock_o2_bb);

    /* Outputs for Enabled SubSystem: '<S38>/Enabled Subsystem' */
    rover_sw_pwr_EnabledSubsystem_g(rover_sw_pwrtrain_24b_B.SourceBlock_o1,
      &rtb_SourceBlock_o2_bb, &rover_sw_pwrtrain_24b_B.EnabledSubsystem_g1);

    /* End of Outputs for SubSystem: '<S38>/Enabled Subsystem' */
  }

  if (rtmIsMajorTimeStep((&rover_sw_pwrtrain_24b_M))) {
    if (rtmIsMajorTimeStep((&rover_sw_pwrtrain_24b_M))) {
      /* Update for UnitDelay: '<S53>/Unit Delay' */
      rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE_d =
        rover_sw_pwrtrain_24b_B.throttle_cmd;

      /* Update for UnitDelay: '<S65>/Unit Delay' */
      rover_sw_pwrtrain_24b_DW.UnitDelay_DSTATE = rover_sw_pwrtrain_24b_B.Switch;

      /* Update for UnitDelay: '<S69>/Unit Delay' */
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

  /* Derivatives for Integrator: '<S78>/Integrator' */
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

    /* Start for MATLABSystem: '<S17>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_g.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_g.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_kn = true;
    rover_sw_pwrtrain_24b_DW.obj_g.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_g.isInitialized = 1;
    rover_s_Publisher_setupImpl_myg(&rover_sw_pwrtrain_24b_DW.obj_g);
    rover_sw_pwrtrain_24b_DW.obj_g.isSetupComplete = true;

    /* Start for MATLABSystem: '<S16>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_le.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_le.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_fp = true;
    rover_sw_pwrtrain_24b_DW.obj_le.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_le.isInitialized = 1;
    rover_sw_Publisher_setupImpl_my(&rover_sw_pwrtrain_24b_DW.obj_le);
    rover_sw_pwrtrain_24b_DW.obj_le.isSetupComplete = true;

    /* Start for MATLABSystem: '<S57>/Pure Pursuit' */
    rover_sw_pwrtrain_24b_DW.objisempty = true;
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

    /* End of Start for MATLABSystem: '<S57>/Pure Pursuit' */

    /* Start for MATLABSystem: '<S78>/MATLAB System' */
    rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[0] = false;
    rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[1] = false;
    rover_sw_pwrtrain_24b_DW.obj_j.tunablePropertyChanged[2] = false;
    rover_sw_pwrtrain_24b_DW.objisempty_o = true;
    rover_sw_pwrtrain_24b_DW.obj_j.TrackWidth = 0.5;
    rover_sw_pwrtrain_24b_DW.obj_j.WheelRadius = 0.095;
    rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[0] = (rtMinusInf);
    rover_sw_pwrtrain_24b_DW.obj_j.WheelSpeedRange[1] = (rtInf);
    rover_sw_pwrtrain_24b_DW.obj_j.isInitialized = 1;
    DifferentialDriveKinematics_set(&rover_sw_pwrtrain_24b_DW.obj_j);
    rover_sw_pwrtrain_24b_DW.obj_j.TunablePropsChanged = false;

    /* Start for MATLABSystem: '<S26>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_n.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_n.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_ea = true;
    rover_sw_pwrtrain_24b_DW.obj_n.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_n.isInitialized = 1;
    rover_sw_Subscriber_setupImpl_m(&rover_sw_pwrtrain_24b_DW.obj_n);
    rover_sw_pwrtrain_24b_DW.obj_n.isSetupComplete = true;

    /* Start for MATLABSystem: '<S25>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_ivp.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_ivp.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_c = true;
    rover_sw_pwrtrain_24b_DW.obj_ivp.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_ivp.isInitialized = 1;
    rover_sw_p_Subscriber_setupImpl(&rover_sw_pwrtrain_24b_DW.obj_ivp);
    rover_sw_pwrtrain_24b_DW.obj_ivp.isSetupComplete = true;

    /* Start for MATLABSystem: '<S33>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_mg.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_mg.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_l = true;
    rover_sw_pwrtrain_24b_DW.obj_mg.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_mg.isInitialized = 1;
    r_Subscriber_setupImpl_mygou5bo(&rover_sw_pwrtrain_24b_DW.obj_mg);
    rover_sw_pwrtrain_24b_DW.obj_mg.isSetupComplete = true;

    /* Start for MATLABSystem: '<S32>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_ky.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_ky.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_g0 = true;
    rover_sw_pwrtrain_24b_DW.obj_ky.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_ky.isInitialized = 1;
    ro_Subscriber_setupImpl_mygou5b(&rover_sw_pwrtrain_24b_DW.obj_ky);
    rover_sw_pwrtrain_24b_DW.obj_ky.isSetupComplete = true;

    /* Start for MATLABSystem: '<S34>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_dw.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_dw.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_g = true;
    rover_sw_pwrtrain_24b_DW.obj_dw.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_dw.isInitialized = 1;
    Subscriber_setupImpl_mygou5bow(&rover_sw_pwrtrain_24b_DW.obj_dw);
    rover_sw_pwrtrain_24b_DW.obj_dw.isSetupComplete = true;

    /* Start for MATLABSystem: '<S35>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_in.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_in.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_ob = true;
    rover_sw_pwrtrain_24b_DW.obj_in.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_in.isInitialized = 1;
    Subscriber_setupImpl_mygou5bowb(&rover_sw_pwrtrain_24b_DW.obj_in);
    rover_sw_pwrtrain_24b_DW.obj_in.isSetupComplete = true;

    /* Start for MATLABSystem: '<S36>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_hm.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_hm.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_d = true;
    rover_sw_pwrtrain_24b_DW.obj_hm.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_hm.isInitialized = 1;
    Subscriber_setupImp_mygou5bowbh(&rover_sw_pwrtrain_24b_DW.obj_hm);
    rover_sw_pwrtrain_24b_DW.obj_hm.isSetupComplete = true;

    /* Start for MATLABSystem: '<S18>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_i.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_i.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_eg = true;
    rover_sw_pwrtrain_24b_DW.obj_i.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_i.isInitialized = 1;
    rover__Publisher_setupImpl_mygo(&rover_sw_pwrtrain_24b_DW.obj_i);
    rover_sw_pwrtrain_24b_DW.obj_i.isSetupComplete = true;

    /* Start for MATLABSystem: '<S72>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_d.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_d.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_i = true;
    rover_sw_pwrtrain_24b_DW.obj_d.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_d.isInitialized = 1;
    Publisher_setupImpl_mygou5bowb(&rover_sw_pwrtrain_24b_DW.obj_d);
    rover_sw_pwrtrain_24b_DW.obj_d.isSetupComplete = true;

    /* Start for MATLABSystem: '<S74>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_a.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_a.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_ko = true;
    rover_sw_pwrtrain_24b_DW.obj_a.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_a.isInitialized = 1;
    Publisher_setupImp_mygou5bowbhv(&rover_sw_pwrtrain_24b_DW.obj_a);
    rover_sw_pwrtrain_24b_DW.obj_a.isSetupComplete = true;

    /* Start for MATLABSystem: '<S73>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_m.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_m.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_e = true;
    rover_sw_pwrtrain_24b_DW.obj_m.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_m.isInitialized = 1;
    Publisher_setupImpl_mygou5bowbh(&rover_sw_pwrtrain_24b_DW.obj_m);
    rover_sw_pwrtrain_24b_DW.obj_m.isSetupComplete = true;

    /* Start for MATLABSystem: '<S75>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_k.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_k.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_k = true;
    rover_sw_pwrtrain_24b_DW.obj_k.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_k.isInitialized = 1;
    Publisher_setupIm_mygou5bowbhvx(&rover_sw_pwrtrain_24b_DW.obj_k);
    rover_sw_pwrtrain_24b_DW.obj_k.isSetupComplete = true;

    /* Start for MATLABSystem: '<S31>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_hmj.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_hmj.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_er = true;
    rover_sw_pwrtrain_24b_DW.obj_hmj.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_hmj.isInitialized = 1;
    rov_Subscriber_setupImpl_mygou5(&rover_sw_pwrtrain_24b_DW.obj_hmj);
    rover_sw_pwrtrain_24b_DW.obj_hmj.isSetupComplete = true;

    /* Start for MATLABSystem: '<S14>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_o.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_o.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_ba = true;
    rover_sw_pwrtrain_24b_DW.obj_o.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_o.isInitialized = 1;
    rover_sw_pw_Publisher_setupImpl(&rover_sw_pwrtrain_24b_DW.obj_o);
    rover_sw_pwrtrain_24b_DW.obj_o.isSetupComplete = true;

    /* Start for MATLABSystem: '<S19>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_h.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_h.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_eh = true;
    rover_sw_pwrtrain_24b_DW.obj_h.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_h.isInitialized = 1;
    rover_Publisher_setupImpl_mygou(&rover_sw_pwrtrain_24b_DW.obj_h);
    rover_sw_pwrtrain_24b_DW.obj_h.isSetupComplete = true;

    /* Start for MATLABSystem: '<S21>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_c.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_c.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_jq = true;
    rover_sw_pwrtrain_24b_DW.obj_c.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_c.isInitialized = 1;
    rov_Publisher_setupImpl_mygou5b(&rover_sw_pwrtrain_24b_DW.obj_c);
    rover_sw_pwrtrain_24b_DW.obj_c.isSetupComplete = true;

    /* Start for MATLABSystem: '<S37>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_iv.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_iv.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_b = true;
    rover_sw_pwrtrain_24b_DW.obj_iv.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_iv.isInitialized = 1;
    Subscriber_setupIm_mygou5bowbhv(&rover_sw_pwrtrain_24b_DW.obj_iv);
    rover_sw_pwrtrain_24b_DW.obj_iv.isSetupComplete = true;

    /* Start for MATLABSystem: '<S20>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_f5.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_f5.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_ji = true;
    rover_sw_pwrtrain_24b_DW.obj_f5.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_f5.isInitialized = 1;
    rove_Publisher_setupImpl_mygou5(&rover_sw_pwrtrain_24b_DW.obj_f5);
    rover_sw_pwrtrain_24b_DW.obj_f5.isSetupComplete = true;

    /* Start for MATLABSystem: '<S27>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_b.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_b.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_l0 = true;
    rover_sw_pwrtrain_24b_DW.obj_b.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_b.isInitialized = 1;
    rover_s_Subscriber_setupImpl_my(&rover_sw_pwrtrain_24b_DW.obj_b);
    rover_sw_pwrtrain_24b_DW.obj_b.isSetupComplete = true;

    /* Start for MATLABSystem: '<S28>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_k2.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_k2.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_j = true;
    rover_sw_pwrtrain_24b_DW.obj_k2.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_k2.isInitialized = 1;
    rover__Subscriber_setupImpl_myg(&rover_sw_pwrtrain_24b_DW.obj_k2);
    rover_sw_pwrtrain_24b_DW.obj_k2.isSetupComplete = true;

    /* Start for MATLABSystem: '<S29>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_kyn.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_kyn.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_a5 = true;
    rover_sw_pwrtrain_24b_DW.obj_kyn.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_kyn.isInitialized = 1;
    rover_Subscriber_setupImpl_mygo(&rover_sw_pwrtrain_24b_DW.obj_kyn);
    rover_sw_pwrtrain_24b_DW.obj_kyn.isSetupComplete = true;

    /* Start for MATLABSystem: '<S30>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_cn.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_cn.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_a = true;
    rover_sw_pwrtrain_24b_DW.obj_cn.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_cn.isInitialized = 1;
    rove_Subscriber_setupImpl_mygou(&rover_sw_pwrtrain_24b_DW.obj_cn);
    rover_sw_pwrtrain_24b_DW.obj_cn.isSetupComplete = true;

    /* Start for MATLABSystem: '<S15>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_cl.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_cl.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_bmt = true;
    rover_sw_pwrtrain_24b_DW.obj_cl.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_cl.isInitialized = 1;
    rover_sw__Publisher_setupImpl_m(&rover_sw_pwrtrain_24b_DW.obj_cl);
    rover_sw_pwrtrain_24b_DW.obj_cl.isSetupComplete = true;

    /* Start for MATLABSystem: '<S22>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_l.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_l.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_le = true;
    rover_sw_pwrtrain_24b_DW.obj_l.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_l.isInitialized = 1;
    ro_Publisher_setupImpl_mygou5bo(&rover_sw_pwrtrain_24b_DW.obj_l);
    rover_sw_pwrtrain_24b_DW.obj_l.isSetupComplete = true;

    /* Start for MATLABSystem: '<S23>/SinkBlock' */
    rover_sw_pwrtrain_24b_DW.obj_f.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_f.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_bm = true;
    rover_sw_pwrtrain_24b_DW.obj_f.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_f.isInitialized = 1;
    r_Publisher_setupImpl_mygou5bow(&rover_sw_pwrtrain_24b_DW.obj_f);
    rover_sw_pwrtrain_24b_DW.obj_f.isSetupComplete = true;

    /* Start for MATLABSystem: '<S38>/SourceBlock' */
    rover_sw_pwrtrain_24b_DW.obj_dv.QOSAvoidROSNamespaceConventions = false;
    rover_sw_pwrtrain_24b_DW.obj_dv.matlabCodegenIsDeleted = false;
    rover_sw_pwrtrain_24b_DW.objisempty_f = true;
    rover_sw_pwrtrain_24b_DW.obj_dv.isSetupComplete = false;
    rover_sw_pwrtrain_24b_DW.obj_dv.isInitialized = 1;
    Subscriber_setupI_mygou5bowbhvx(&rover_sw_pwrtrain_24b_DW.obj_dv);
    rover_sw_pwrtrain_24b_DW.obj_dv.isSetupComplete = true;

    /* ConstCode for MATLABSystem: '<S17>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_246.publish
      (&rover_sw_pwrtrain_24b_ConstB.BusAssignment3);

    /* ConstCode for MATLABSystem: '<S16>/SinkBlock' */
    Pub_rover_sw_pwrtrain_24b_826.publish
      (&rover_sw_pwrtrain_24b_ConstB.BusAssignment2);
  }

  {
    int32_T i;
    static const char_T tmp[23] = { 'V', 'e', 'h', 'i', 'c', 'l', 'e', 'S', 'p',
      'e', 'e', 'd', 'H', 'e', 'a', 'd', 'i', 'n', 'g', 'R', 'a', 't', 'e' };

    /* InitializeConditions for Integrator: '<S78>/Integrator' */
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[0] = 0.0;
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[1] = 0.0;
    rover_sw_pwrtrain_24b_X.Integrator_CSTATE[2] = 0.0;

    /* SystemInitialize for Enabled SubSystem: '<S26>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_o);

    /* End of SystemInitialize for SubSystem: '<S26>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S25>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem);

    /* End of SystemInitialize for SubSystem: '<S25>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S33>/Enabled Subsystem' */
    rover_s_EnabledSubsystem_p_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_g);

    /* End of SystemInitialize for SubSystem: '<S33>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S32>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_dt);

    /* End of SystemInitialize for SubSystem: '<S32>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S34>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_ov);

    /* End of SystemInitialize for SubSystem: '<S34>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S35>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_b);

    /* End of SystemInitialize for SubSystem: '<S35>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S36>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_lp);

    /* End of SystemInitialize for SubSystem: '<S36>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S31>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_p);

    /* End of SystemInitialize for SubSystem: '<S31>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S37>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_pq);

    /* End of SystemInitialize for SubSystem: '<S37>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S27>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_ot);

    /* End of SystemInitialize for SubSystem: '<S27>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S28>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_d);

    /* End of SystemInitialize for SubSystem: '<S28>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S29>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_l);

    /* End of SystemInitialize for SubSystem: '<S29>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S30>/Enabled Subsystem' */
    rover_sw__EnabledSubsystem_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_o4);

    /* End of SystemInitialize for SubSystem: '<S30>/Enabled Subsystem' */

    /* SystemInitialize for Enabled SubSystem: '<S38>/Enabled Subsystem' */
    rover_s_EnabledSubsystem_p_Init(&rover_sw_pwrtrain_24b_B.EnabledSubsystem_g1);

    /* End of SystemInitialize for SubSystem: '<S38>/Enabled Subsystem' */

    /* InitializeConditions for MATLABSystem: '<S57>/Pure Pursuit' */
    rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[0] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LookaheadPoint[1] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LastPose[0] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LastPose[1] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.LastPose[2] *= 0.0;
    rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[0] = (rtNaN);
    rover_sw_pwrtrain_24b_DW.obj.ProjectionPoint[1] = (rtNaN);
    rover_sw_pwrtrain_24b_DW.obj.ProjectionLineIndex *= 0.0;

    /* InitializeConditions for MATLABSystem: '<S78>/MATLAB System' */
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

    /* End of InitializeConditions for MATLABSystem: '<S78>/MATLAB System' */
  }
}

/* Model terminate function */
void rover_sw_pwrtrain_24b::terminate()
{
  /* Terminate for MATLABSystem: '<S17>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_g.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_g.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_g.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_g.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_246.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S17>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S16>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_le.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_le.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_le.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_le.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_826.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S16>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S26>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_n.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_n.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_n.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_n.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_391.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S26>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S25>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_ivp.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_ivp.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_ivp.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_ivp.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_390.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S25>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S33>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_mg.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_mg.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_mg.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_mg.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_521.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S33>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S32>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_ky.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_ky.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_ky.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_ky.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_986.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S32>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S34>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_dw.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_dw.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_dw.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_dw.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_987.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S34>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S35>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_in.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_in.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_in.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_in.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_988.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S35>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S36>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_hm.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_hm.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_hm.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_hm.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_989.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S36>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S18>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_i.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_i.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_i.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_i.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1006.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S18>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S72>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_d.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_d.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_d.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_d.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1057.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S72>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S74>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_a.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_a.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_a.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_a.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1059.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S74>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S73>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_m.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_m.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_m.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_m.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1058.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S73>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S75>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_k.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_k.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_k.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_k.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1060.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S75>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S31>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_hmj.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_hmj.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_hmj.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_hmj.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_392.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S31>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S14>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_o.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_o.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_o.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_o.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_907.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S14>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S19>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_h.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_h.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_h.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_903.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S19>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S21>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_c.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_c.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_c.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_c.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1039.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S21>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S37>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_iv.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_iv.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_iv.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_iv.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_1018.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S37>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S20>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_f5.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_f5.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_f5.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_f5.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1009.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S20>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S27>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_b.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_b.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_b.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_b.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_1130.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S27>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S28>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_k2.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_k2.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_k2.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_k2.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_1131.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S28>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S29>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_kyn.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_kyn.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_kyn.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_kyn.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_1132.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S29>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S30>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_cn.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_cn.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_cn.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_cn.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_1133.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S30>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S15>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_cl.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_cl.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_cl.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_cl.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1122.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S15>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S22>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_l.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_l.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_l.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_l.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1087.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S22>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S23>/SinkBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_f.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_f.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_f.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_f.isSetupComplete) {
      Pub_rover_sw_pwrtrain_24b_1115.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S23>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S38>/SourceBlock' */
  if (!rover_sw_pwrtrain_24b_DW.obj_dv.matlabCodegenIsDeleted) {
    rover_sw_pwrtrain_24b_DW.obj_dv.matlabCodegenIsDeleted = true;
    if ((rover_sw_pwrtrain_24b_DW.obj_dv.isInitialized == 1) &&
        rover_sw_pwrtrain_24b_DW.obj_dv.isSetupComplete) {
      Sub_rover_sw_pwrtrain_24b_1042.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S38>/SourceBlock' */
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
