/*
 * rover_sw_pwrtrain_24b.h
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

/* Block signals for system '<S25>/Enabled Subsystem' */
struct B_EnabledSubsystem_rover_sw_p_T {
  SL_Bus_std_msgs_Float32 In1;         /* '<S39>/In1' */
};

/* Block signals for system '<S33>/Enabled Subsystem' */
struct B_EnabledSubsystem_rover_sw_e_T {
  SL_Bus_std_msgs_Bool In1;            /* '<S47>/In1' */
};

/* Block signals (default storage) */
struct B_rover_sw_pwrtrain_24b_T {
  boolean_T b[1756];
  int16_T tmp_data[878];
  boolean_T bv[878];
  real_T Integrator[3];                /* '<S78>/Integrator' */
  char_T b_zeroDelimTopic[22];
  char_T b_zeroDelimTopic_m[20];
  char_T b_zeroDelimTopic_c[17];
  char_T b_zeroDelimTopic_k[17];
  char_T b_zeroDelimTopic_cx[17];
  char_T b_zeroDelimTopic_b[17];
  char_T b_zeroDelimTopic_p[17];
  char_T b_zeroDelimTopic_cv[16];
  char_T b_zeroDelimTopic_f[16];
  char_T b_zeroDelimTopic_g[16];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_g;
  sJ4ih70VmKcvCeguWN0mNVF deadline_m;
  sJ4ih70VmKcvCeguWN0mNVF deadline_n;
  sJ4ih70VmKcvCeguWN0mNVF deadline_p;
  sJ4ih70VmKcvCeguWN0mNVF deadline_l;
  sJ4ih70VmKcvCeguWN0mNVF deadline_j;
  sJ4ih70VmKcvCeguWN0mNVF deadline_d;
  sJ4ih70VmKcvCeguWN0mNVF deadline_gu;
  sJ4ih70VmKcvCeguWN0mNVF deadline_ld;
  sJ4ih70VmKcvCeguWN0mNVF deadline_dh;
  sJ4ih70VmKcvCeguWN0mNVF deadline_dy;
  sJ4ih70VmKcvCeguWN0mNVF deadline_lx;
  sJ4ih70VmKcvCeguWN0mNVF deadline_o;
  sJ4ih70VmKcvCeguWN0mNVF deadline_b;
  sJ4ih70VmKcvCeguWN0mNVF deadline_nu;
  sJ4ih70VmKcvCeguWN0mNVF deadline_bs;
  sJ4ih70VmKcvCeguWN0mNVF deadline_ln;
  real_T lookaheadStartPt[2];
  real_T lookaheadStartPt_h[2];
  real_T dv[2];
  real_T refPt[2];
  real_T Product;                      /* '<S79>/Product' */
  real_T Product1;                     /* '<S79>/Product1' */
  real_T MATLABSystem[3];              /* '<S78>/MATLAB System' */
  real_T throttle_out;                 /* '<S69>/adaptive_throttle_contller' */
  real_T decay_mode;                   /* '<S69>/adaptive_throttle_contller' */
  real_T last_good_time;               /* '<S69>/adaptive_throttle_contller' */
  real_T minDistance;
  real_T dist;
  real_T i;
  real_T lookaheadIdx;
  real_T lookaheadStartPt_tmp;
  real_T lookaheadStartPt_tmp_b;
  real_T Switch;                       /* '<S2>/Switch' */
  real_T alpha;
  real_T v12;
  real_T v12_d;
  real_T scale;
  real_T absxk;
  real_T t;
  real32_T throttle_cmd;               /* '<S53>/Chart' */
  real32_T steering_cmd;               /* '<S53>/Chart' */
  real32_T tmr_hr;                     /* '<S1>/op_data' */
  real32_T wheel_spd_kph;              /* '<S62>/Product' */
  int32_T mil_lamp_pwm;                /* '<S53>/Chart' */
  int32_T b_i;
  int32_T ret;
  int32_T trueCount;
  int32_T i_e;
  int32_T tmp_size_idx_0;
  SL_Bus_std_msgs_Int32 BusAssignment4;/* '<S1>/Bus Assignment4' */
  SL_Bus_std_msgs_Int32 BusAssignment1;/* '<S1>/Bus Assignment1' */
  SL_Bus_std_msgs_Int32 BusAssignment5;/* '<S1>/Bus Assignment5' */
  SL_Bus_std_msgs_Int32 BusAssignment7;/* '<S1>/Bus Assignment7' */
  SL_Bus_std_msgs_Int32 BusAssignment6;/* '<S1>/Bus Assignment6' */
  SL_Bus_std_msgs_Float32 BusAssignment1_h;/* '<S68>/Bus Assignment1' */
  SL_Bus_std_msgs_Float32 BusAssignment2;/* '<S68>/Bus Assignment2' */
  SL_Bus_std_msgs_Float32 BusAssignment10;/* '<S1>/Bus Assignment10' */
  SL_Bus_std_msgs_Float32 BusAssignment8;/* '<S1>/Bus Assignment8' */
  SL_Bus_std_msgs_Float32 BusAssignment9;/* '<S1>/Bus Assignment9' */
  collision_flt Switch_b;              /* '<S65>/Switch' */
  boolean_T SourceBlock_o1;            /* '<S38>/SourceBlock' */
  boolean_T SourceBlock_o1_b;          /* '<S37>/SourceBlock' */
  boolean_T SourceBlock_o1_f;          /* '<S36>/SourceBlock' */
  boolean_T SourceBlock_o1_k;          /* '<S35>/SourceBlock' */
  boolean_T SourceBlock_o1_l;          /* '<S34>/SourceBlock' */
  boolean_T SourceBlock_o1_n;          /* '<S33>/SourceBlock' */
  boolean_T SourceBlock_o1_nq;         /* '<S32>/SourceBlock' */
  boolean_T SourceBlock_o1_m;          /* '<S31>/SourceBlock' */
  boolean_T SourceBlock_o1_fq;         /* '<S30>/SourceBlock' */
  boolean_T SourceBlock_o1_o;          /* '<S29>/SourceBlock' */
  boolean_T SourceBlock_o1_a;          /* '<S28>/SourceBlock' */
  boolean_T SourceBlock_o1_g;          /* '<S27>/SourceBlock' */
  boolean_T SourceBlock_o1_nc;         /* '<S26>/SourceBlock' */
  boolean_T SourceBlock_o1_mk;         /* '<S25>/SourceBlock' */
  B_EnabledSubsystem_rover_sw_e_T EnabledSubsystem_g1;/* '<S38>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_pq;/* '<S37>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_lp;/* '<S36>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_b;/* '<S35>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_ov;/* '<S34>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_e_T EnabledSubsystem_g;/* '<S33>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_dt;/* '<S32>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_p;/* '<S31>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_o4;/* '<S30>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_l;/* '<S29>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_d;/* '<S28>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_ot;/* '<S27>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem_o;/* '<S26>/Enabled Subsystem' */
  B_EnabledSubsystem_rover_sw_p_T EnabledSubsystem;/* '<S25>/Enabled Subsystem' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_rover_sw_pwrtrain_24b_T {
  nav_slalgs_internal_PurePursu_T obj; /* '<S57>/Pure Pursuit' */
  robotics_slmobile_internal_bl_T obj_j;/* '<S78>/MATLAB System' */
  ros_slros2_internal_block_Pub_T obj_k;/* '<S75>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_a;/* '<S74>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_m;/* '<S73>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_d;/* '<S72>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_f;/* '<S23>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_l;/* '<S22>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_c;/* '<S21>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_f5;/* '<S20>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_h;/* '<S19>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_i;/* '<S18>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_g;/* '<S17>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_le;/* '<S16>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_cl;/* '<S15>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_o;/* '<S14>/SinkBlock' */
  ros_slros2_internal_block_Sub_T obj_dv;/* '<S38>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_iv;/* '<S37>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_hm;/* '<S36>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_in;/* '<S35>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_dw;/* '<S34>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_mg;/* '<S33>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_ky;/* '<S32>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_hmj;/* '<S31>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_cn;/* '<S30>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_kyn;/* '<S29>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_k2;/* '<S28>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_b;/* '<S27>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_n;/* '<S26>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_ivp;/* '<S25>/SourceBlock' */
  real_T UnitDelay_DSTATE;             /* '<S65>/Unit Delay' */
  real_T UnitDelay_DSTATE_i;           /* '<S69>/Unit Delay' */
  real_T max_decrease;                 /* '<S69>/adaptive_throttle_contller' */
  real_T max_increase;                 /* '<S69>/adaptive_throttle_contller' */
  real_T collision_time;               /* '<S69>/adaptive_throttle_contller' */
  real_T comm_loss_time;               /* '<S69>/adaptive_throttle_contller' */
  real_T comm_ref_throttle;            /* '<S69>/adaptive_throttle_contller' */
  real_T loss_tmr;                     /* '<S69>/adaptive_throttle_contller' */
  real_T phase2_time;                  /* '<S69>/adaptive_throttle_contller' */
  real_T severe_time;                  /* '<S69>/adaptive_throttle_contller' */
  real_T max_increase_g;               /* '<S53>/Chart' */
  real_T max_decrease_l;               /* '<S53>/Chart' */
  real32_T UnitDelay_DSTATE_d;         /* '<S53>/Unit Delay' */
  real32_T tmr_sec;                    /* '<S1>/op_data' */
  uint16_T temporalCounter_i1;         /* '<S69>/adaptive_throttle_contller' */
  uint16_T temporalCounter_i1_c;       /* '<S53>/Chart' */
  uint8_T is_active_c3_rover_sw_pwrtrain_;/* '<S69>/adaptive_throttle_contller' */
  uint8_T is_c3_rover_sw_pwrtrain_24b; /* '<S69>/adaptive_throttle_contller' */
  uint8_T is_COLLISION_FLT_FAST_STOP;  /* '<S69>/adaptive_throttle_contller' */
  uint8_T is_LOST_COMM_OPERATION;      /* '<S69>/adaptive_throttle_contller' */
  uint8_T is_NORMAL_OPERATION;         /* '<S69>/adaptive_throttle_contller' */
  uint8_T is_active_c6_rover_sw_pwrtrain_;/* '<S53>/Chart' */
  uint8_T is_c6_rover_sw_pwrtrain_24b; /* '<S53>/Chart' */
  uint8_T is_TRAILHEAD_ARMED;          /* '<S53>/Chart' */
  uint8_T is_TRAILHEAD_ERROR_STATE;    /* '<S53>/Chart' */
  uint8_T is_active_c2_rover_sw_pwrtrain_;/* '<S1>/op_data' */
  boolean_T objisempty;                /* '<S57>/Pure Pursuit' */
  boolean_T objisempty_o;              /* '<S78>/MATLAB System' */
  boolean_T objisempty_k;              /* '<S75>/SinkBlock' */
  boolean_T objisempty_ko;             /* '<S74>/SinkBlock' */
  boolean_T objisempty_e;              /* '<S73>/SinkBlock' */
  boolean_T objisempty_i;              /* '<S72>/SinkBlock' */
  boolean_T objisempty_f;              /* '<S38>/SourceBlock' */
  boolean_T objisempty_b;              /* '<S37>/SourceBlock' */
  boolean_T objisempty_d;              /* '<S36>/SourceBlock' */
  boolean_T objisempty_ob;             /* '<S35>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S34>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S33>/SourceBlock' */
  boolean_T objisempty_g0;             /* '<S32>/SourceBlock' */
  boolean_T objisempty_er;             /* '<S31>/SourceBlock' */
  boolean_T objisempty_a;              /* '<S30>/SourceBlock' */
  boolean_T objisempty_a5;             /* '<S29>/SourceBlock' */
  boolean_T objisempty_j;              /* '<S28>/SourceBlock' */
  boolean_T objisempty_l0;             /* '<S27>/SourceBlock' */
  boolean_T objisempty_ea;             /* '<S26>/SourceBlock' */
  boolean_T objisempty_c;              /* '<S25>/SourceBlock' */
  boolean_T objisempty_bm;             /* '<S23>/SinkBlock' */
  boolean_T objisempty_le;             /* '<S22>/SinkBlock' */
  boolean_T objisempty_jq;             /* '<S21>/SinkBlock' */
  boolean_T objisempty_ji;             /* '<S20>/SinkBlock' */
  boolean_T objisempty_eh;             /* '<S19>/SinkBlock' */
  boolean_T objisempty_eg;             /* '<S18>/SinkBlock' */
  boolean_T objisempty_kn;             /* '<S17>/SinkBlock' */
  boolean_T objisempty_fp;             /* '<S16>/SinkBlock' */
  boolean_T objisempty_bmt;            /* '<S15>/SinkBlock' */
  boolean_T objisempty_ba;             /* '<S14>/SinkBlock' */
};

/* Continuous states (default storage) */
struct X_rover_sw_pwrtrain_24b_T {
  real_T Integrator_CSTATE[3];         /* '<S78>/Integrator' */
};

/* State derivatives (default storage) */
struct XDot_rover_sw_pwrtrain_24b_T {
  real_T Integrator_CSTATE[3];         /* '<S78>/Integrator' */
};

/* State disabled  */
struct XDis_rover_sw_pwrtrain_24b_T {
  boolean_T Integrator_CSTATE[3];      /* '<S78>/Integrator' */
};

/* Invariant block signals (default storage) */
struct ConstB_rover_sw_pwrtrain_24b_T {
  SL_Bus_std_msgs_Int32 BusAssignment3;/* '<S1>/Bus Assignment3' */
  SL_Bus_std_msgs_Int32 BusAssignment2;/* '<S1>/Bus Assignment2' */
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
   * Referenced by: '<S58>/Constant4'
   */
  real_T Constant4_Value[1756];
};

/* Constant parameters with dynamic initialization (default storage) */
struct ConstInitP_rover_sw_pwrtrain__T {
  /* Expression: WheelSpeedRange
   * Referenced by: '<S78>/MATLAB System'
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
    time_T tArray[2];
  } Timing;
};

extern const ConstB_rover_sw_pwrtrain_24b_T rover_sw_pwrtrain_24b_ConstB;/* constant block i/o */

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

  /* private member function(s) for subsystem '<S25>/Enabled Subsystem'*/
  static void rover_sw__EnabledSubsystem_Init(B_EnabledSubsystem_rover_sw_p_T
    *localB);
  static void rover_sw_pwrtr_EnabledSubsystem(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Float32 *rtu_In1, B_EnabledSubsystem_rover_sw_p_T *localB);

  /* private member function(s) for subsystem '<S33>/Enabled Subsystem'*/
  static void rover_s_EnabledSubsystem_p_Init(B_EnabledSubsystem_rover_sw_e_T
    *localB);
  static void rover_sw_pwr_EnabledSubsystem_g(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_rover_sw_e_T *localB);

  /* private member function(s) for subsystem '<Root>'*/
  void rover_s_Publisher_setupImpl_myg(const ros_slros2_internal_block_Pub_T
    *obj);
  void rover_sw_Publisher_setupImpl_my(const ros_slros2_internal_block_Pub_T
    *obj);
  void DifferentialDriveKinematics_set(robotics_slmobile_internal_bl_T *obj);
  void rover_sw_Subscriber_setupImpl_m(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover_sw_p_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T
    *obj);
  void r_Subscriber_setupImpl_mygou5bo(const ros_slros2_internal_block_Sub_T
    *obj);
  void ro_Subscriber_setupImpl_mygou5b(const ros_slros2_internal_block_Sub_T
    *obj);
  void Subscriber_setupImpl_mygou5bow(const ros_slros2_internal_block_Sub_T *obj);
  void Subscriber_setupImpl_mygou5bowb(const ros_slros2_internal_block_Sub_T
    *obj);
  void Subscriber_setupImp_mygou5bowbh(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover__Publisher_setupImpl_mygo(const ros_slros2_internal_block_Pub_T
    *obj);
  void Publisher_setupImpl_mygou5bowb(const ros_slros2_internal_block_Pub_T *obj);
  void Publisher_setupImp_mygou5bowbhv(const ros_slros2_internal_block_Pub_T
    *obj);
  void Publisher_setupImpl_mygou5bowbh(const ros_slros2_internal_block_Pub_T
    *obj);
  void Publisher_setupIm_mygou5bowbhvx(const ros_slros2_internal_block_Pub_T
    *obj);
  void rov_Subscriber_setupImpl_mygou5(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover_sw_pw_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T
    *obj);
  void rover_Publisher_setupImpl_mygou(const ros_slros2_internal_block_Pub_T
    *obj);
  void rov_Publisher_setupImpl_mygou5b(const ros_slros2_internal_block_Pub_T
    *obj);
  void Subscriber_setupIm_mygou5bowbhv(const ros_slros2_internal_block_Sub_T
    *obj);
  void rove_Publisher_setupImpl_mygou5(const ros_slros2_internal_block_Pub_T
    *obj);
  void rover_s_Subscriber_setupImpl_my(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover__Subscriber_setupImpl_myg(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover_Subscriber_setupImpl_mygo(const ros_slros2_internal_block_Sub_T
    *obj);
  void rove_Subscriber_setupImpl_mygou(const ros_slros2_internal_block_Sub_T
    *obj);
  void rover_sw__Publisher_setupImpl_m(const ros_slros2_internal_block_Pub_T
    *obj);
  void ro_Publisher_setupImpl_mygou5bo(const ros_slros2_internal_block_Pub_T
    *obj);
  void r_Publisher_setupImpl_mygou5bow(const ros_slros2_internal_block_Pub_T
    *obj);
  void Subscriber_setupI_mygou5bowbhvx(const ros_slros2_internal_block_Sub_T
    *obj);
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
 * Block '<S60>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S78>/Data Type Duplicate' : Unused code path elimination
 * Block '<S57>/Display' : Unused code path elimination
 * Block '<S57>/Display1' : Unused code path elimination
 * Block '<S57>/Display2' : Unused code path elimination
 * Block '<S79>/Display' : Unused code path elimination
 * Block '<S1>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S78>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S78>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S78>/Reshape' : Reshape block reduction
 * Block '<S57>/Zero-Order Hold' : Eliminated since input and output rates are identical
 * Block '<S57>/Zero-Order Hold1' : Eliminated since input and output rates are identical
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
 * '<S1>'   : 'rover_sw_pwrtrain_24b/ros_publish'
 * '<S2>'   : 'rover_sw_pwrtrain_24b/ros_subscribe'
 * '<S3>'   : 'rover_sw_pwrtrain_24b/trailhead_main'
 * '<S4>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message1'
 * '<S5>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message10'
 * '<S6>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message2'
 * '<S7>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message3'
 * '<S8>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message4'
 * '<S9>'   : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message5'
 * '<S10>'  : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message6'
 * '<S11>'  : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message7'
 * '<S12>'  : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message8'
 * '<S13>'  : 'rover_sw_pwrtrain_24b/ros_publish/Blank Message9'
 * '<S14>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish1'
 * '<S15>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish10'
 * '<S16>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish2'
 * '<S17>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish3'
 * '<S18>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish4'
 * '<S19>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish5'
 * '<S20>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish6'
 * '<S21>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish7'
 * '<S22>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish8'
 * '<S23>'  : 'rover_sw_pwrtrain_24b/ros_publish/Publish9'
 * '<S24>'  : 'rover_sw_pwrtrain_24b/ros_publish/op_data'
 * '<S25>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe'
 * '<S26>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe1'
 * '<S27>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe10'
 * '<S28>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe11'
 * '<S29>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe12'
 * '<S30>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe13'
 * '<S31>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe2'
 * '<S32>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe3'
 * '<S33>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe4'
 * '<S34>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe5'
 * '<S35>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe6'
 * '<S36>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe7'
 * '<S37>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe8'
 * '<S38>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe9'
 * '<S39>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe/Enabled Subsystem'
 * '<S40>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe1/Enabled Subsystem'
 * '<S41>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe10/Enabled Subsystem'
 * '<S42>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe11/Enabled Subsystem'
 * '<S43>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe12/Enabled Subsystem'
 * '<S44>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe13/Enabled Subsystem'
 * '<S45>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe2/Enabled Subsystem'
 * '<S46>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe3/Enabled Subsystem'
 * '<S47>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe4/Enabled Subsystem'
 * '<S48>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe5/Enabled Subsystem'
 * '<S49>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe6/Enabled Subsystem'
 * '<S50>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe7/Enabled Subsystem'
 * '<S51>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe8/Enabled Subsystem'
 * '<S52>'  : 'rover_sw_pwrtrain_24b/ros_subscribe/Subscribe9/Enabled Subsystem'
 * '<S53>'  : 'rover_sw_pwrtrain_24b/trailhead_main/arming_sequence'
 * '<S54>'  : 'rover_sw_pwrtrain_24b/trailhead_main/aux_controls'
 * '<S55>'  : 'rover_sw_pwrtrain_24b/trailhead_main/diagnostics'
 * '<S56>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls'
 * '<S57>'  : 'rover_sw_pwrtrain_24b/trailhead_main/path_following'
 * '<S58>'  : 'rover_sw_pwrtrain_24b/trailhead_main/path_generation'
 * '<S59>'  : 'rover_sw_pwrtrain_24b/trailhead_main/arming_sequence/Chart'
 * '<S60>'  : 'rover_sw_pwrtrain_24b/trailhead_main/arming_sequence/Interval Test'
 * '<S61>'  : 'rover_sw_pwrtrain_24b/trailhead_main/aux_controls/Subsystem'
 * '<S62>'  : 'rover_sw_pwrtrain_24b/trailhead_main/aux_controls/Subsystem1'
 * '<S63>'  : 'rover_sw_pwrtrain_24b/trailhead_main/aux_controls/Subsystem/Compare To Constant'
 * '<S64>'  : 'rover_sw_pwrtrain_24b/trailhead_main/aux_controls/Subsystem/Compare To Constant1'
 * '<S65>'  : 'rover_sw_pwrtrain_24b/trailhead_main/diagnostics/Subsystem'
 * '<S66>'  : 'rover_sw_pwrtrain_24b/trailhead_main/diagnostics/Subsystem/Compare To Constant'
 * '<S67>'  : 'rover_sw_pwrtrain_24b/trailhead_main/diagnostics/Subsystem/Compare To Constant1'
 * '<S68>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2'
 * '<S69>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/throttle_regulation'
 * '<S70>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Blank Message1'
 * '<S71>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Blank Message2'
 * '<S72>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Publish1'
 * '<S73>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Publish2'
 * '<S74>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Publish3'
 * '<S75>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/Publish4'
 * '<S76>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/Subsystem2/vesc_comm'
 * '<S77>'  : 'rover_sw_pwrtrain_24b/trailhead_main/motor_controls/throttle_regulation/adaptive_throttle_contller'
 * '<S78>'  : 'rover_sw_pwrtrain_24b/trailhead_main/path_following/Differential Drive Kinematic Model'
 * '<S79>'  : 'rover_sw_pwrtrain_24b/trailhead_main/path_following/Zero-Velocity At Goal'
 */
#endif                                 /* rover_sw_pwrtrain_24b_h_ */
