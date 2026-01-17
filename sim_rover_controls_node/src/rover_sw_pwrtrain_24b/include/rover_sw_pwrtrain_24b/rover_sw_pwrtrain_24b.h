/*
 * rover_sw_pwrtrain_24b.h
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

#ifndef rover_sw_pwrtrain_24b_h_
#define rover_sw_pwrtrain_24b_h_
#include <cstring>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros2_initialize.h"
#include "rover_sw_pwrtrain_24b_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <string.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* user code (top of header file) */
#include "vesc_uart.h"
#include "get_telemetry_wrapper.h"

/* Block signals for system '<S14>/Enabled Subsystem' */
struct B_EnabledSubsystem_rover_sw_p_T {
  SL_Bus_std_msgs_Float32 In1;         /* '<S18>/In1' */
};

/* Block signals (default storage) */
struct B_rover_sw_pwrtrain_24b_T {
  SL_Bus_std_msgs_Float32MultiArray BusAssignment;/* '<S27>/Bus Assignment' */
  boolean_T b[1756];
  int16_T tmp_data[878];
  boolean_T bv[878];
  real32_T data[128];                  /* '<S27>/MATLAB Function2' */
  char_T b_zeroDelimTopic[31];
  char_T b_zeroDelimTopic_m[28];
  char_T b_zeroDelimTopic_c[28];
  char_T b_zeroDelimTopic_k[27];
  char_T b_zeroDelimTopic_cx[27];
  char_T b_zeroDelimTopic_b[27];
  char_T b_zeroDelimTopic_p[24];
  real_T Integrator[3];                /* '<S47>/Integrator' */
  char_T b_zeroDelimTopic_cv[22];
  char_T b_zeroDelimTopic_f[20];
  char_T b_zeroDelimTopic_g[19];
  char_T b_zeroDelimTopic_g1[18];
  char_T b_zeroDelimTopic_me[18];
  char_T b_zeroDelimTopic_n[16];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_p;
  real32_T voltages[4];                /* '<S27>/MATLAB Function' */
  real32_T current_motor[4];           /* '<S27>/MATLAB Function' */
  real32_T current_battery[4];         /* '<S27>/MATLAB Function' */
  real32_T rpm[4];                     /* '<S27>/MATLAB Function' */
  real_T lookaheadStartPt[2];
  real_T lookaheadStartPt_l[2];
  real_T dv[2];
  real_T refPt[2];
  real_T Product;                      /* '<S48>/Product' */
  real_T Product1;                     /* '<S48>/Product1' */
  real_T MATLABSystem[3];              /* '<S47>/MATLAB System' */
  real_T throttle_out;                 /* '<S35>/adaptive_throttle_contller' */
  real_T decay_mode;                   /* '<S35>/adaptive_throttle_contller' */
  real_T last_good_time;               /* '<S35>/adaptive_throttle_contller' */
  real_T minDistance;
  real_T i;
  real_T lookaheadIdx;
  real_T Add;                          /* '<S30>/Add' */
  real_T lookaheadStartPt_tmp;
  real_T lookaheadStartPt_tmp_j;
  real_T Switch;                       /* '<S4>/Switch' */
  real_T alpha;
  real_T v12;
  real_T v12_d;
  real_T scale;
  real_T absxk;
  real_T t;
  SL_Bus_std_msgs_Bool In1;            /* '<S21>/In1' */
  int32_T b_i;
  int32_T ret;
  collision_flt Switch_b;              /* '<S30>/Switch' */
  SL_Bus_std_msgs_Int32 BusAssignment3;/* '<S3>/Bus Assignment3' */
  SL_Bus_std_msgs_Int32 BusAssignment2;/* '<S3>/Bus Assignment2' */
  uint8_T fault_codes[4];              /* '<S27>/MATLAB Function' */
  boolean_T SourceBlock_o1_m;          /* '<S16>/SourceBlock' */
  boolean_T SourceBlock_o1_n;          /* '<S15>/SourceBlock' */
  boolean_T SourceBlock_o1_mk;         /* '<S14>/SourceBlock' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_p;/* '<S16>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_o;/* '<S15>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem;/* '<S14>/Enabled Subsystem' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_rover_sw_pwrtrain_24b_T {
  nav_slalgs_internal_PurePursu_T obj; /* '<S25>/Pure Pursuit' */
  robotics_slmobile_internal_bl_T obj_j;/* '<S47>/MATLAB System' */
  ros_slros2_internal_block_Pub_T obj_d;/* '<S57>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_k;/* '<S56>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_ju;/* '<S55>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_n;/* '<S40>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_c;/* '<S39>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_p;/* '<S44>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_a;/* '<S43>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_h;/* '<S13>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_g;/* '<S12>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_l;/* '<S11>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_o;/* '<S10>/SinkBlock' */
  ros_slros2_internal_block_Sub_T obj_m;/* '<S17>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_hm;/* '<S16>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_ng;/* '<S15>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_i;/* '<S14>/SourceBlock' */
  real_T UnitDelay_DSTATE;             /* '<S30>/Unit Delay' */
  real_T UnitDelay_DSTATE_i;           /* '<S35>/Unit Delay' */
  real_T max_decrease;                 /* '<S35>/adaptive_throttle_contller' */
  real_T max_increase;                 /* '<S35>/adaptive_throttle_contller' */
  real_T collision_time;               /* '<S35>/adaptive_throttle_contller' */
  real_T comm_loss_time;               /* '<S35>/adaptive_throttle_contller' */
  real_T comm_ref_throttle;            /* '<S35>/adaptive_throttle_contller' */
  real_T loss_tmr;                     /* '<S35>/adaptive_throttle_contller' */
  real_T phase2_time;                  /* '<S35>/adaptive_throttle_contller' */
  real_T severe_time;                  /* '<S35>/adaptive_throttle_contller' */
  uint16_T temporalCounter_i1;         /* '<S35>/adaptive_throttle_contller' */
  int8_T If_ActiveSubsystem;           /* '<S24>/If' */
  uint8_T is_active_c3_rover_sw_pwrtrain_;/* '<S35>/adaptive_throttle_contller' */
  uint8_T is_c3_rover_sw_pwrtrain_24b; /* '<S35>/adaptive_throttle_contller' */
  uint8_T is_COLLISION_FLT_FAST_STOP;  /* '<S35>/adaptive_throttle_contller' */
  uint8_T is_LOST_COMM_OPERATION;      /* '<S35>/adaptive_throttle_contller' */
  uint8_T is_NORMAL_OPERATION;         /* '<S35>/adaptive_throttle_contller' */
  boolean_T objisempty;                /* '<S57>/SinkBlock' */
  boolean_T objisempty_p;              /* '<S56>/SinkBlock' */
  boolean_T objisempty_o;              /* '<S55>/SinkBlock' */
  boolean_T objisempty_e;              /* '<S25>/Pure Pursuit' */
  boolean_T objisempty_ok;             /* '<S47>/MATLAB System' */
  boolean_T objisempty_j;              /* '<S40>/SinkBlock' */
  boolean_T objisempty_jb;             /* '<S39>/SinkBlock' */
  boolean_T objisempty_d;              /* '<S44>/SinkBlock' */
  boolean_T objisempty_n;              /* '<S43>/SinkBlock' */
  boolean_T objisempty_l;              /* '<S17>/SourceBlock' */
  boolean_T objisempty_er;             /* '<S16>/SourceBlock' */
  boolean_T objisempty_ea;             /* '<S15>/SourceBlock' */
  boolean_T objisempty_c;              /* '<S14>/SourceBlock' */
  boolean_T objisempty_eh;             /* '<S13>/SinkBlock' */
  boolean_T objisempty_k;              /* '<S12>/SinkBlock' */
  boolean_T objisempty_f;              /* '<S11>/SinkBlock' */
  boolean_T objisempty_b;              /* '<S10>/SinkBlock' */
};

/* Continuous states (default storage) */
struct X_rover_sw_pwrtrain_24b_T {
  real_T Integrator_CSTATE[3];         /* '<S47>/Integrator' */
};

/* State derivatives (default storage) */
struct XDot_rover_sw_pwrtrain_24b_T {
  real_T Integrator_CSTATE[3];         /* '<S47>/Integrator' */
};

/* State disabled  */
struct XDis_rover_sw_pwrtrain_24b_T {
  boolean_T Integrator_CSTATE[3];      /* '<S47>/Integrator' */
};

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
struct ODE3_IntgData {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
};

#endif

/* Constant parameters (default storage) */
struct ConstP_rover_sw_pwrtrain_24b_T {
  /* Expression: waypoints
   * Referenced by: '<S26>/Constant4'
   */
  real_T Constant4_Value[1756];
};

/* Constant parameters with dynamic initialization (default storage) */
struct ConstInitP_rover_sw_pwrtrain__T {
  /* Expression: WheelSpeedRange
   * Referenced by: '<S47>/MATLAB System'
   */
  real_T MATLABSystem_WheelSpeedRange[2];
};

/* Real-time Model Data Structure */
struct tag_RTM_rover_sw_pwrtrain_24b_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_rover_sw_pwrtrain_24b_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_rover_sw_pwrtrain_24b_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[3];
  real_T odeF[3][3];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[4];
  } Timing;
};

/* Constant parameters (default storage) */
extern const ConstP_rover_sw_pwrtrain_24b_T rover_sw_pwrtrain_24b_ConstP;

/* Constant parameters with dynamic initialization (default storage) */
extern ConstInitP_rover_sw_pwrtrain__T rover_sw_pwrtrain_24_ConstInitP;/* constant parameters */

/* Class declaration for model rover_sw_pwrtrain_24b */
class rover_sw_pwrtrain_24b
{
  /* public data and function members */
 public:
  /* Real-Time Model get method */
  RT_MODEL_rover_sw_pwrtrain_24_T * getRTM();

  /* model start function */
  void start();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  rover_sw_pwrtrain_24b();

  /* Destructor */
  ~rover_sw_pwrtrain_24b();

  /* private data and function members */
 private:
  /* Block signals */
  B_rover_sw_pwrtrain_24b_T rover_sw_pwrtrain_24b_B;

  /* Block states */
  DW_rover_sw_pwrtrain_24b_T rover_sw_pwrtrain_24b_DW;

  /* Block continuous states */
  X_rover_sw_pwrtrain_24b_T rover_sw_pwrtrain_24b_X;

  /* Block Continuous state disabled vector */
  XDis_rover_sw_pwrtrain_24b_T rover_sw_pwrtrain_24b_XDis;

  /* private member function(s) for subsystem '<S14>/Enabled Subsystem'*/
  static void rover_sw__EnabledSubsystem_Init(B_EnabledSubsystem_rover_sw_p_T
    *localB);
  static void rover_sw_pwrtr_EnabledSubsystem(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Float32 *rtu_In1, B_EnabledSubsystem_rover_sw_p_T *localB);

  /* private member function(s) for subsystem '<Root>'*/
  void rover_sw_p_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover_sw_Subscriber_setupImpl_m(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover_sw_Publisher_setupImpl_my(const ros_slros2_internal_block_Pub_T
    *obj);
  void rover_sw__Publisher_setupImpl_m(const ros_slros2_internal_block_Pub_T
    *obj);
  void rover_s_Subscriber_setupImpl_my(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover__Subscriber_setupImpl_myg(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover_sw_pw_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T
    *obj);
  void rover_s_Publisher_setupImpl_myg(const ros_slros2_internal_block_Pub_T
    *obj);
  void rover__Publisher_setupImpl_mygo(const ros_slros2_internal_block_Pub_T
    *obj);
  void rover_Publisher_setupImpl_mygou(const ros_slros2_internal_block_Pub_T
    *obj);
  void rove_Publisher_setupImpl_mygou5(const ros_slros2_internal_block_Pub_T
    *obj);
  void rov_Publisher_setupImpl_mygou5b(const ros_slros2_internal_block_Pub_T
    *obj);
  void DifferentialDriveKinematics_set(robotics_slmobile_internal_bl_T *obj);
  void ro_Publisher_setupImpl_mygou5bo(const ros_slros2_internal_block_Pub_T
    *obj);
  void r_Publisher_setupImpl_mygou5bow(const ros_slros2_internal_block_Pub_T
    *obj);
  void Publisher_setupImpl_mygou5bowb(const ros_slros2_internal_block_Pub_T *obj);
  real_T rover_sw_pwrtrain_24b_norm(const real_T x[2]);
  real_T rover_sw_pwr_closestPointOnLine(const real_T pt1[2], real_T pt2[2],
    const real_T refPt[2]);

  /* Global mass matrix */

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void rover_sw_pwrtrain_24b_derivatives();

  /* Real-Time Model */
  RT_MODEL_rover_sw_pwrtrain_24_T rover_sw_pwrtrain_24b_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S25>/Display' : Unused code path elimination
 * Block '<S25>/Display1' : Unused code path elimination
 * Block '<S25>/Display2' : Unused code path elimination
 * Block '<S48>/Display' : Unused code path elimination
 * Block '<S3>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S33>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S33>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S33>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S33>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S34>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S34>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S45>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S45>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S47>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S47>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S47>/Reshape' : Reshape block reduction
 * Block '<S25>/Zero-Order Hold' : Eliminated since input and output rates are identical
 * Block '<S25>/Zero-Order Hold1' : Eliminated since input and output rates are identical
 * Block '<S27>/Data Type Conversion1' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'rover_sw_pwrtrain_24b'
 * '<S1>'   : 'rover_sw_pwrtrain_24b/Initialize Function'
 * '<S2>'   : 'rover_sw_pwrtrain_24b/Terminate Function'
 * '<S3>'   : 'rover_sw_pwrtrain_24b/ros_publish'
 * '<S4>'   : 'rover_sw_pwrtrain_24b/ros_subscribe'
 * '<S5>'   : 'rover_sw_pwrtrain_24b/trailhead_main'
 * '<S6>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message1'
 * '<S7>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message2'
 * '<S8>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message3'
 * '<S9>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message5'
 * '<S10>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish1'
 * '<S11>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish2'
 * '<S12>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish3'
 * '<S13>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish5'
 * '<S14>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe'
 * '<S15>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe1'
 * '<S16>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe2'
 * '<S17>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe4'
 * '<S18>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe/Enabled Subsystem'
 * '<S19>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe1/Enabled Subsystem'
 * '<S20>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe2/Enabled Subsystem'
 * '<S21>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe4/Enabled Subsystem'
 * '<S22>'  : 'rover_sw_pwrtrain_24b/trailhead_main/aux_controls'
 * '<S23>'  : 'rover_sw_pwrtrain_24b/trailhead_main/diagnostics'
 * '<S24>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls'
 * '<S25>'  : 'rover_sw_pwrtrain_24b/trailhead_main/path_following'
 * '<S26>'  : 'rover_sw_pwrtrain_24b/trailhead_main/path_generation'
 * '<S27>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry'
 * '<S28>'  : 'rover_sw_pwrtrain_24b/trailhead_main/aux_controls/headlamps_controls'
 * '<S29>'  : 'rover_sw_pwrtrain_24b/trailhead_main/aux_controls/status_indicator_lamp'
 * '<S30>'  : 'rover_sw_pwrtrain_24b/trailhead_main/diagnostics/Subsystem'
 * '<S31>'  : 'rover_sw_pwrtrain_24b/trailhead_main/diagnostics/Subsystem/Compare To Constant'
 * '<S32>'  : 'rover_sw_pwrtrain_24b/trailhead_main/diagnostics/Subsystem/Compare To Constant1'
 * '<S33>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem'
 * '<S34>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem1'
 * '<S35>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/throttle_regulation'
 * '<S36>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem/Blank Message1'
 * '<S37>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem/Blank Message2'
 * '<S38>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem/Compare To Constant'
 * '<S39>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem/Publish1'
 * '<S40>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem/Publish2'
 * '<S41>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem1/Blank Message1'
 * '<S42>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem1/Blank Message2'
 * '<S43>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem1/Publish1'
 * '<S44>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem1/Publish2'
 * '<S45>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem1/vesc_comm'
 * '<S46>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/throttle_regulation/adaptive_throttle_contller'
 * '<S47>'  : 'rover_sw_pwrtrain_24b/trailhead_main/path_following/Differential Drive Kinematic Model'
 * '<S48>'  : 'rover_sw_pwrtrain_24b/trailhead_main/path_following/Zero-Velocity At Goal'
 * '<S49>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry/Blank Message1'
 * '<S50>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry/Blank Message2'
 * '<S51>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry/Blank Message4'
 * '<S52>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry/MATLAB Function'
 * '<S53>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry/MATLAB Function1'
 * '<S54>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry/MATLAB Function2'
 * '<S55>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry/Publish1'
 * '<S56>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry/Publish2'
 * '<S57>'  : 'rover_sw_pwrtrain_24b/trailhead_main/telemetry/Publish4'
 */
#endif                                 /* rover_sw_pwrtrain_24b_h_ */
