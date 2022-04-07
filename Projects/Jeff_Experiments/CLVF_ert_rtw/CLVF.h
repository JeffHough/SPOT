//
//  CLVF.h
//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Code generation for model "CLVF".
//
//  Model version              : 1.279
//  Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
//  C++ source code generated on : Wed Apr  6 21:40:15 2022
//
//  Target selection: ert.tlc
//  Embedded hardware selection: ARM Compatible->ARM Cortex
//  Code generation objective: Execution efficiency
//  Validation result: Not run


#ifndef RTW_HEADER_CLVF_h_
#define RTW_HEADER_CLVF_h_
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <float.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "MW_gpio.h"
#include "MW_pigs.h"
#include "DAHostLib_Network.h"
#include "owl.hpp"
#include "phasespace_headers.h"
#include "MW_I2C.h"
#include "CLVF_types.h"

// Shared type includes
#include "multiword_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rt_zcfcn.h"
#include "rt_defines.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmStepTask
# define rtmStepTask(rtm, idx)         ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#ifndef rtmTaskCounter
# define rtmTaskCounter(rtm, idx)      ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

// Block signals for system '<S27>/MATLAB Function2'
typedef struct {
  real_T ThrustPer_Final[8];           // '<S27>/MATLAB Function2'
  real_T ThrustPer_Sat[8];
} B_MATLABFunction2_CLVF_T;

// Block signals for system '<S32>/Create Rotation Matrix'
typedef struct {
  real_T C_Ib[4];                      // '<S32>/Create Rotation Matrix'
} B_CreateRotationMatrix_CLVF_T;

// Block signals for system '<S28>/Create Rotation Matrix'
typedef struct {
  real_T C_bI[4];                      // '<S28>/Create Rotation Matrix'
} B_CreateRotationMatrix_CLVF_i_T;

// Block signals for system '<S36>/MATLAB Function'
typedef struct {
  real_T Mat2[64];
  real_T H[24];                        // '<S36>/MATLAB Function'
  real_T b[24];
  real_T v[8];
} B_MATLABFunction_CLVF_T;

// Block signals for system '<S169>/AHRS2'
typedef struct {
  real_T Qw[144];
  real_T Ppost[144];
  real_T H[72];
  real_T b_X[72];
  real_T H_m[72];
  real_T c_A[36];
  real_T xe_post[12];
  real_T Rprior[9];
  real_T h1[9];
  real_T x[9];
  real_T h[9];
  real_T ze[6];
  real_T b_varargout_1[4];
  real_T psquared[4];
  real_T AHRS2_o2[3];                  // '<S169>/AHRS2'
  real_T b_varargout_2[3];
  real_T gravityAccelGyroDiff[3];
  real_T offDiag[3];
  real_T Reast[3];
  real_T c[3];
  boolean_T b[9];
  real_T accelMeasNoiseVar;
  real_T magMeasNoiseVar;
  real_T smax;
  real_T s;
  real_T temp;
  real_T assign_temp_a;
  real_T assign_temp_b;
  real_T assign_temp_c;
  real_T assign_temp_d;
  real_T assign_temp_d_c;
  real_T gyroOffsetErr_idx_0;
  real_T linAccelErr_idx_0;
  real_T gyroOffsetErr_idx_1;
  real_T linAccelErr_idx_1;
  real_T gyroOffsetErr_idx_2;
  real_T linAccelErr_idx_2;
  real_T assign_temp_a_k;
  real_T assign_temp_b_c;
  real_T assign_temp_c_b;
  real_T assign_temp_d_p;
  real_T theta;
  real_T tr;
  real_T d;
  real_T ac2;
  real_T ad2;
  real_T bc2;
  real_T bd2;
  real_T cd2;
  real_T aasq;
  real_T scale;
  real_T absxk;
  int8_T b_ipiv[6];
  int32_T i;
  int32_T i1;
} B_AHRS2_CLVF_T;

// Block states (default storage) for system '<S169>/AHRS2'
typedef struct {
  fusion_simulink_ahrsfilter_CL_T obj; // '<S169>/AHRS2'
  c_fusion_internal_frames_NED__T gobj_1;// '<S169>/AHRS2'
  c_fusion_internal_frames_NED__T gobj_2;// '<S169>/AHRS2'
  boolean_T objisempty;                // '<S169>/AHRS2'
} DW_AHRS2_CLVF_T;

// Block signals for system '<S169>/ChangeOrientation'
typedef struct {
  real_T y[3];                         // '<S169>/ChangeOrientation'
} B_ChangeOrientation_CLVF_T;

// Block signals for system '<S179>/Calculate Running Mean'
typedef struct {
  real_T Mean;                         // '<S182>/Mean'
} B_CalculateRunningMean_CLVF_T;

// Block states (default storage) for system '<S179>/Calculate Running Mean'
typedef struct {
  real_T Delay_DSTATE;                 // '<S182>/Delay'
  real_T Delay1_DSTATE[2];             // '<S182>/Delay1'
  real_T Delay2_DSTATE[4];             // '<S182>/Delay2'
  real_T Delay3_DSTATE[5];             // '<S182>/Delay3'
  real_T Delay4_DSTATE[3];             // '<S182>/Delay4'
  real_T Mean_AccVal;                  // '<S182>/Mean'
} DW_CalculateRunningMean_CLVF_T;

// Block signals for system '<S194>/Calculate Running Mean'
typedef struct {
  real_T TmpSignalConversionAtMean_b[6];
  real_T Subtract;                     // '<S201>/Subtract'
} B_CalculateRunningMean_CLVF_p_T;

// Block states (default storage) for system '<S194>/Calculate Running Mean'
typedef struct {
  real_T Delay_DSTATE;                 // '<S201>/Delay'
  real_T Delay1_DSTATE[2];             // '<S201>/Delay1'
  real_T Delay2_DSTATE[4];             // '<S201>/Delay2'
  real_T Delay3_DSTATE[5];             // '<S201>/Delay3'
  real_T Delay4_DSTATE[3];             // '<S201>/Delay4'
  real_T Mean_AccVal;                  // '<S201>/Mean'
} DW_CalculateRunningMean_CLV_e_T;

// Block signals for system '<S533>/MATLAB Function'
typedef struct {
  real_T y;                            // '<S533>/MATLAB Function'
} B_MATLABFunction_CLVF_g_T;

// Block states (default storage) for system '<Root>/Phase #0:  Wait for Synchronization' 
typedef struct {
  int8_T ThisIFblockdetermineswhetherorn;
    // '<S8>/This IF block determines whether or not to run the BLACK sim//exp'
  int8_T ThisIFblockdetermineswhethero_m;
     // '<S8>/This IF block determines whether or not to run the BLUE sim//exp'
  int8_T ThisIFblockdetermineswhethero_n;
     // '<S8>/This IF block determines whether or not to run the RED sim//exp '
} DW_Phase0WaitforSynchronizati_T;

// Block signals for system '<S281>/MATLAB Function2'
typedef struct {
  real_T Oy[2];                        // '<S281>/MATLAB Function2'
  real_T Ox[2];                        // '<S281>/MATLAB Function2'
} B_MATLABFunction2_CLVF_n_T;

// Block signals for system '<S281>/MATLAB Function3'
typedef struct {
  real_T Ox[2];                        // '<S281>/MATLAB Function3'
} B_MATLABFunction3_CLVF_T;

// Block signals for system '<S281>/MATLAB Function4'
typedef struct {
  real_T e_out;                        // '<S281>/MATLAB Function4'
} B_MATLABFunction4_CLVF_T;

// Block signals for system '<S340>/MATLAB Function2'
typedef struct {
  real_T Oy[2];                        // '<S340>/MATLAB Function2'
  real_T Ox[2];                        // '<S340>/MATLAB Function2'
} B_MATLABFunction2_CLVF_d_T;

// Block signals for system '<S318>/Sub-Phase #1'
typedef struct {
  real_T Subtract;                     // '<S374>/Subtract'
  real_T Delay1;                       // '<S385>/Delay1'
  real_T Subtract1;                    // '<S374>/Subtract1'
  real_T Delay1_i;                     // '<S387>/Delay1'
  real_T Delay1_o;                     // '<S383>/Delay1'
  real_T In1;                          // '<S388>/In1'
  real_T In1_l;                        // '<S386>/In1'
  real_T In1_g;                        // '<S384>/In1'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4;// '<S382>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3;// '<S382>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2;// '<S382>/MATLAB Function2'
} B_SubPhase1_CLVF_T;

// Block states (default storage) for system '<S318>/Sub-Phase #1'
typedef struct {
  real_T Delay1_DSTATE;                // '<S385>/Delay1'
  real_T Delay1_DSTATE_o;              // '<S387>/Delay1'
  real_T Delay1_DSTATE_h;              // '<S383>/Delay1'
  int8_T ifwewentthroughastep_ActiveSubs;// '<S385>/if we went through a "step"' 
  int8_T ifwewentthroughastep_ActiveSu_h;// '<S387>/if we went through a "step"' 
  int8_T ifwewentthroughastep_ActiveSu_j;// '<S383>/if we went through a "step"' 
  uint8_T icLoad;                      // '<S385>/Delay1'
  uint8_T icLoad_k;                    // '<S387>/Delay1'
  uint8_T icLoad_j;                    // '<S383>/Delay1'
} DW_SubPhase1_CLVF_T;

// Block signals for system '<S12>/Change BLUE Behavior'
typedef struct {
  real_T Subtract;                     // '<S438>/Subtract'
  real_T Delay1;                       // '<S460>/Delay1'
  real_T Subtract1;                    // '<S438>/Subtract1'
  real_T Delay1_m;                     // '<S462>/Delay1'
  real_T Delay1_p;                     // '<S458>/Delay1'
  real_T In1;                          // '<S463>/In1'
  real_T In1_k;                        // '<S461>/In1'
  real_T In1_a;                        // '<S459>/In1'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4;// '<S457>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3;// '<S457>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2;// '<S457>/MATLAB Function2'
} B_ChangeBLUEBehavior_CLVF_k_T;

// Block states (default storage) for system '<S12>/Change BLUE Behavior'
typedef struct {
  real_T Delay1_DSTATE;                // '<S460>/Delay1'
  real_T Delay1_DSTATE_n;              // '<S462>/Delay1'
  real_T Delay1_DSTATE_k;              // '<S458>/Delay1'
  int8_T ifwewentthroughastep_ActiveSubs;// '<S460>/if we went through a "step"' 
  int8_T ifwewentthroughastep_ActiveSu_j;// '<S462>/if we went through a "step"' 
  int8_T ifwewentthroughastep_ActiveSu_a;// '<S458>/if we went through a "step"' 
  uint8_T icLoad;                      // '<S460>/Delay1'
  uint8_T icLoad_h;                    // '<S462>/Delay1'
  uint8_T icLoad_l;                    // '<S458>/Delay1'
} DW_ChangeBLUEBehavior_CLVF_i_T;

// Block signals (default storage)
typedef struct {
  real_T Mat2[64];
  real_T Mat2_m[64];
  real_T TmpSignalConversionAtToWork[64];
  real_T transpose[24];                // '<S51>/transpose'
  real_T transpose_p[24];              // '<S42>/transpose'
  real_T transpose_d[24];              // '<S33>/transpose'
  real_T H_bu[24];                     // '<S27>/MATLAB Function'
  real_T b[24];
  real_T StreamPhaseSpacetoPlatform[13];
  real_T C_IB[9];
  real_T y[9];                         // '<S350>/getRotationMatrix'
  real_T acc_ff_tmp[9];
  real_T vc[9];
  real_T C_IB_c[9];
  real_T b_I[9];
  real_T rtb_TmpSignalConversionAtSFu_kk[9];
  real_T d_I[9];
  real_T x[9];
  real_T x_c[9];
  real_T x_b[9];
  real_T rtb_RemoveNegatives_i_p[8];
  char_T cv[45];
  real_T Merge[3];                     // '<S349>/Merge'
  real_T del[3];
  real_T o_hat[3];
  real_T w_dot_I[3];
  real_T vC_T[3];
  real_T e_hat[3];
  real_T r_hat_dot[3];
  real_T TmpSignalConversionAtSFun_n[3];// '<S352>/PD + FF Controller'
  real_T d_T[3];                       // '<S342>/MATLAB Function'
  real_T x_ddot[3];                    // '<S532>/MATLAB Function'
  real_T x_ddot_g[3];                  // '<S528>/MATLAB Function'
  real_T x_ddot_b[3];                  // '<S527>/MATLAB Function'
  real_T rC_D_tmp[3];
  real_T C_IB_cv[3];
  real_T rtb_TmpSignalConversionAtSFun_f[3];
  real_T c_I[3];
  real_T dv[3];
  real_T dv1[3];
  real_T LSM9DS1IMUSensor_o3[3];       // '<S6>/LSM9DS1 IMU Sensor'
  real_T Gain_p[3];                    // '<S171>/Gain'
  real_T DigitalFilter[3];             // '<S237>/Digital Filter'
  real_T Gain_c[3];                    // '<S170>/Gain'
  real_T DigitalFilter_l[3];           // '<S206>/Digital Filter'
  real_T Gain_n[3];                    // '<S169>/Gain'
  real_T DigitalFilter_p[3];           // '<S175>/Digital Filter'
  real_T rtb_Subtract2_l_g[2];
  boolean_T Compare_e[13];
  int8_T b_I_g[9];
  int8_T c_I_m[9];
  int8_T d_I_n[9];
  real_T SetUniversalTimeIfthisisasimula;
                        // '<S15>/Set Universal Time (If this is a simulation)'
  real_T Subtract;                     // '<S482>/Subtract'
  real_T Subtract1;                    // '<S482>/Subtract1'
  real_T In1;                          // '<S519>/In1'
  real_T In1_j;                        // '<S517>/In1'
  real_T In1_d;                        // '<S515>/In1'
  real_T Subtract_o;                   // '<S480>/Subtract'
  real_T Subtract1_c;                  // '<S480>/Subtract1'
  real_T In1_c;                        // '<S493>/In1'
  real_T In1_i;                        // '<S491>/In1'
  real_T In1_g;                        // '<S489>/In1'
  real_T Subtract_l;                   // '<S439>/Subtract'
  real_T Subtract1_g;                  // '<S439>/Subtract1'
  real_T In1_f;                        // '<S476>/In1'
  real_T In1_b;                        // '<S474>/In1'
  real_T In1_h;                        // '<S472>/In1'
  real_T Subtract_d;                   // '<S437>/Subtract'
  real_T Subtract1_d;                  // '<S437>/Subtract1'
  real_T In1_d0;                       // '<S450>/In1'
  real_T In1_fx;                       // '<S448>/In1'
  real_T In1_fk;                       // '<S446>/In1'
  real_T Subtract_k;                   // '<S408>/Subtract'
  real_T Subtract1_a;                  // '<S408>/Subtract1'
  real_T In1_e;                        // '<S433>/In1'
  real_T In1_cy;                       // '<S431>/In1'
  real_T In1_c5;                       // '<S429>/In1'
  real_T Subtract_p;                   // '<S405>/Subtract'
  real_T Subtract1_k;                  // '<S405>/Subtract1'
  real_T In1_e0;                       // '<S419>/In1'
  real_T In1_n;                        // '<S417>/In1'
  real_T In1_he;                       // '<S415>/In1'
  real_T DataStoreRead;                // '<S342>/Data Store Read'
  real_T DataStoreRead1;               // '<S342>/Data Store Read1'
  real_T DataStoreRead2;               // '<S350>/Data Store Read2'
  real_T uHzLPFilter1;                 // '<S342>/1Hz LP Filter1'
  real_T uHzLPFilter;                  // '<S342>/1Hz LP Filter'
  real_T r;                            // '<S342>/Norm1'
  real_T theta;                        // '<S342>/Norm1'
  real_T r_e;                          // '<S342>/Norm'
  real_T theta_b;                      // '<S342>/Norm'
  real_T In1_a;                        // '<S370>/In1'
  real_T newCnt;                       // '<S354>/inDesiredRadius'
  real_T In;                           // '<S358>/In'
  real_T Derivative[3];                // '<S353>/Derivative'
  real_T Sum1[3];                      // '<S353>/Sum1'
  real_T hC_T[3];                      // '<S353>/Body-Fixed Guidance'
  real_T acc_ff[3];                    // '<S353>/Body-Fixed Guidance'
  real_T Sum[3];                       // '<S351>/Sum'
  real_T h[3];                         // '<S351>/Lyapunov Guidance + FF'
  real_T a_ff[3];                      // '<S351>/Lyapunov Guidance + FF'
  real_T In1_jg;                       // '<S345>/In1'
  real_T Subtract_n;                   // '<S321>/Subtract'
  real_T Subtract1_n;                  // '<S321>/Subtract1'
  real_T In1_ft;                       // '<S335>/In1'
  real_T In1_gy;                       // '<S333>/In1'
  real_T In1_l;                        // '<S331>/In1'
  real_T Subtract_f;                   // '<S276>/Subtract'
  real_T Subtract1_dr;                 // '<S276>/Subtract1'
  real_T In1_if;                       // '<S313>/In1'
  real_T In1_cr;                       // '<S311>/In1'
  real_T In1_k;                        // '<S309>/In1'
  real_T Subtract_j;                   // '<S275>/Subtract'
  real_T Subtract1_o;                  // '<S275>/Subtract1'
  real_T In1_jw;                       // '<S300>/In1'
  real_T In1_lc;                       // '<S298>/In1'
  real_T In1_o;                        // '<S296>/In1'
  real_T Subtract_ou;                  // '<S274>/Subtract'
  real_T Subtract1_m;                  // '<S274>/Subtract1'
  real_T In1_gc;                       // '<S287>/In1'
  real_T In1_ej;                       // '<S285>/In1'
  real_T In1_fa;                       // '<S283>/In1'
  real_T In1_p;                        // '<S574>/In1'
  real_T In1_l4;                       // '<S573>/In1'
  real_T In1_n5;                       // '<S572>/In1'
  real_T In1_lx;                       // '<S571>/In1'
  real_T In1_i5;                       // '<S570>/In1'
  real_T In1_aj;                       // '<S569>/In1'
  real_T In1_g0;                       // '<S568>/In1'
  real_T In1_jz;                       // '<S567>/In1'
  real_T In1_hm;                       // '<S564>/In1'
  real_T In1_e1;                       // '<S563>/In1'
  real_T In1_kx;                       // '<S562>/In1'
  real_T In1_lo;                       // '<S561>/In1'
  real_T In1_al;                       // '<S560>/In1'
  real_T In_l[3];                      // '<S559>/In'
  real_T In_e[3];                      // '<S557>/In'
  real_T In_lb[3];                     // '<S555>/In'
  real_T In1_co;                       // '<S264>/In1'
  real_T In1_e2;                       // '<S261>/In1'
  real_T In1_lp;                       // '<S258>/In1'
  real_T In1_eu;                       // '<S252>/In1'
  real_T In1_ajd;                      // '<S248>/In1'
  real_T In1_hk;                       // '<S245>/In1'
  real_T In1_bl;                       // '<S233>/In1'
  real_T In1_px;                       // '<S230>/In1'
  real_T In1_dh;                       // '<S227>/In1'
  real_T In1_df;                       // '<S221>/In1'
  real_T In1_o0;                       // '<S202>/In1'
  real_T In1_gr;                       // '<S199>/In1'
  real_T In1_lw;                       // '<S196>/In1'
  real_T In1_ln;                       // '<S190>/In1'
  real_T In1_m;                        // '<S186>/In1'
  real_T In1_mx;                       // '<S183>/In1'
  real_T dividebydeltaT_f;             // '<S115>/divide by delta T'
  real_T dividebydeltaT_bz;            // '<S116>/divide by delta T'
  real_T Unwrap1;                      // '<S80>/Unwrap1'
  real_T dividebydeltaT_e;             // '<S117>/divide by delta T'
  real_T dividebydeltaT_n;             // '<S109>/divide by delta T'
  real_T dividebydeltaT_i;             // '<S113>/divide by delta T'
  real_T Unwrap;                       // '<S80>/Unwrap'
  real_T dividebydeltaT_m;             // '<S114>/divide by delta T'
  real_T TmpSignalConversionAtSendBLACKS[13];
  real_T In1_o4;                       // '<S136>/In1'
  real_T In1_of;                       // '<S135>/In1'
  real_T In1_eh;                       // '<S134>/In1'
  real_T In1_gn;                       // '<S133>/In1'
  real_T In1_as;                       // '<S132>/In1'
  real_T In1_mp;                       // '<S131>/In1'
  real_T In1_jj;                       // '<S130>/In1'
  real_T In1_eo;                       // '<S129>/In1'
  real_T In1_ls;                       // '<S128>/In1'
  real_T In1_hp;                       // '<S127>/In1'
  real_T In1_ij;                       // '<S126>/In1'
  real_T In1_g2;                       // '<S125>/In1'
  real_T In1_lxy;                      // '<S108>/In1'
  real_T In1_mk[13];                   // '<S107>/In1'
  real_T In1_fg;                       // '<S122>/In1'
  real_T UDPReceive_o1[13];            // '<S79>/UDP Receive'
  real_T dividebydeltaT_p;             // '<S87>/divide by delta T'
  real_T dividebydeltaT_j;             // '<S88>/divide by delta T'
  real_T Unwrap1_p;                    // '<S79>/Unwrap1'
  real_T dividebydeltaT_k;             // '<S89>/divide by delta T'
  real_T dividebydeltaT_iv;            // '<S81>/divide by delta T'
  real_T dividebydeltaT_g;             // '<S85>/divide by delta T'
  real_T Unwrap_o;                     // '<S79>/Unwrap'
  real_T dividebydeltaT_nw;            // '<S86>/divide by delta T'
  real_T In1_ow;                       // '<S104>/In1'
  real_T In1_cv;                       // '<S103>/In1'
  real_T In1_jl;                       // '<S102>/In1'
  real_T In1_bc;                       // '<S101>/In1'
  real_T In1_mh;                       // '<S100>/In1'
  real_T In1_h1;                       // '<S99>/In1'
  real_T In1_nd;                       // '<S98>/In1'
  real_T In1_ix;                       // '<S97>/In1'
  real_T In1_fi;                       // '<S96>/In1'
  real_T In1_hr;                       // '<S95>/In1'
  real_T In1_kw;                       // '<S94>/In1'
  real_T In1_it;                       // '<S93>/In1'
  real_T dividebydeltaT_d;             // '<S147>/divide by delta T'
  real_T dividebydeltaT_ka;            // '<S148>/divide by delta T'
  real_T Unwrap1_pz;                   // '<S77>/Unwrap1'
  real_T dividebydeltaT_l;             // '<S149>/divide by delta T'
  real_T dividebydeltaT_nn;            // '<S141>/divide by delta T'
  real_T dividebydeltaT_pf;            // '<S145>/divide by delta T'
  real_T Unwrap_d;                     // '<S77>/Unwrap'
  real_T dividebydeltaT_lz;            // '<S146>/divide by delta T'
  real_T In1_bj;                       // '<S168>/In1'
  real_T In1_bw;                       // '<S167>/In1'
  real_T In1_ii;                       // '<S166>/In1'
  real_T In1_hy;                       // '<S165>/In1'
  real_T In1_oq;                       // '<S164>/In1'
  real_T In1_gi;                       // '<S163>/In1'
  real_T In1_cp;                       // '<S162>/In1'
  real_T In1_mk3;                      // '<S161>/In1'
  real_T In1_la;                       // '<S160>/In1'
  real_T In1_k0;                       // '<S159>/In1'
  real_T In1_lu;                       // '<S158>/In1'
  real_T In1_o2;                       // '<S157>/In1'
  real_T In1_oz;                       // '<S140>/In1'
  real_T In1_ak[13];                   // '<S139>/In1'
  real_T In1_g5;                       // '<S154>/In1'
  real_T v_rel;
  real_T theta_dot;
  real_T g;
  real_T vc_p;
  real_T dgdr;
  real_T g_dot;
  real_T vc_dot;
  real_T theta_l;
  real_T y6;
  real_T y7;
  real_T RED_Tz_Wrist;
  real_T RED_Tz_Shoulder;
  real_T RED_Tz_Elbow;
  real_T rtb_TmpSignalConversionAtSFun_j;
  real_T rtb_TmpSignalConversionAtSFun_d;
  real_T rtb_TmpSignalConversionAtSFun_g;
  real_T absx11;
  real_T absx21;
  real_T absx31;
  real_T sampleTime;
  real_T d;
  real_T sr;
  real_T si;
  int16_T b_RegisterValue[3];
  uint8_T output_raw[6];
  int32_T ntIdx0;
  int32_T uElOffset0;
  int32_T ntIdx1;
  int32_T uElOffset1;
  int32_T yElIdx;
  int32_T p1;
  int32_T p2;
  int32_T p3;
  int32_T itmp;
  int32_T u0;
  int32_T u1;
  uint32_T lo;
  uint32_T hi;
  ZCEventType zcEvent;
  int8_T rtAction;
  int8_T rtPrevAction;
  uint8_T SwappedDataBytes;
  uint8_T status;
  boolean_T DataTypeConversion;        // '<S121>/Data Type Conversion'
  boolean_T DataTypeConversion_o;      // '<S153>/Data Type Conversion'
  boolean_T rtb_Compare_nd_l;
  boolean_T rtb_Compare_n_d;
  boolean_T rtb_Compare_dy;
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_df;// '<S513>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_bc;// '<S513>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2_mx;// '<S513>/MATLAB Function2'
  B_ChangeBLUEBehavior_CLVF_k_T ChangeBLUEBehavior_b;// '<S13>/Change BLUE Behavior' 
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_d;// '<S487>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_o;// '<S487>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2_g;// '<S487>/MATLAB Function2'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_n;// '<S470>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_c1;// '<S470>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2_i;// '<S470>/MATLAB Function2'
  B_ChangeBLUEBehavior_CLVF_k_T ChangeBLUEBehavior_gj;// '<S12>/Change BLUE Behavior' 
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_l;// '<S444>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_b;// '<S444>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2_p;// '<S444>/MATLAB Function2'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_e;// '<S426>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_k5;// '<S426>/MATLAB Function3'
  B_MATLABFunction2_CLVF_d_T sf_MATLABFunction2_e;// '<S426>/MATLAB Function2'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_p;// '<S413>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_ca;// '<S413>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2_ma;// '<S413>/MATLAB Function2'
  B_SubPhase1_CLVF_T SubPhase4_o;      // '<S318>/Sub-Phase #4'
  B_SubPhase1_CLVF_T SubPhase1_g;      // '<S318>/Sub-Phase #1'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_i;// '<S340>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_f;// '<S340>/MATLAB Function3'
  B_MATLABFunction2_CLVF_d_T sf_MATLABFunction2_c;// '<S340>/MATLAB Function2'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_f;// '<S329>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_a;// '<S329>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2_h;// '<S329>/MATLAB Function2'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_b;// '<S307>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_k;// '<S307>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2_ol;// '<S307>/MATLAB Function2'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4_c;// '<S294>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3_c;// '<S294>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2_m;// '<S294>/MATLAB Function2'
  B_MATLABFunction4_CLVF_T sf_MATLABFunction4;// '<S281>/MATLAB Function4'
  B_MATLABFunction3_CLVF_T sf_MATLABFunction3;// '<S281>/MATLAB Function3'
  B_MATLABFunction2_CLVF_n_T sf_MATLABFunction2_o;// '<S281>/MATLAB Function2'
  B_MATLABFunction_CLVF_g_T sf_MATLABFunction_f2;// '<S535>/MATLAB Function'
  B_MATLABFunction_CLVF_g_T sf_MATLABFunction_he;// '<S534>/MATLAB Function'
  B_MATLABFunction_CLVF_g_T sf_MATLABFunction_f;// '<S533>/MATLAB Function'
  B_CalculateRunningMean_CLVF_p_T CalculateRunningMean_mv;// '<S256>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_e;// '<S255>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_ck;// '<S254>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_avn;// '<S243>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_bv;// '<S242>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_mr;// '<S241>/Calculate Running Mean' 
  B_CreateRotationMatrix_CLVF_T sf_CreateRotationMatrix_c;// '<S171>/Create Rotation Matrix' 
  B_ChangeOrientation_CLVF_T sf_ChangeOrientation_di;// '<S171>/ChangeOrientation' 
  B_AHRS2_CLVF_T AHRS2_pn;             // '<S169>/AHRS2'
  B_CalculateRunningMean_CLVF_p_T CalculateRunningMean_l;// '<S225>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_pa;// '<S224>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_m;// '<S223>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_av;// '<S212>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_a;// '<S211>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_b2;// '<S210>/Calculate Running Mean' 
  B_CreateRotationMatrix_CLVF_T sf_CreateRotationMatrix_nl;// '<S170>/Create Rotation Matrix' 
  B_ChangeOrientation_CLVF_T sf_ChangeOrientation_d;// '<S170>/ChangeOrientation' 
  B_AHRS2_CLVF_T AHRS2_p;              // '<S169>/AHRS2'
  B_CalculateRunningMean_CLVF_p_T CalculateRunningMean_b;// '<S194>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_k;// '<S193>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_ch;// '<S192>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_c;// '<S181>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean_p;// '<S180>/Calculate Running Mean' 
  B_CalculateRunningMean_CLVF_T CalculateRunningMean;// '<S179>/Calculate Running Mean' 
  B_CreateRotationMatrix_CLVF_T sf_CreateRotationMatrix_hj;// '<S169>/Create Rotation Matrix' 
  B_ChangeOrientation_CLVF_T sf_ChangeOrientation;// '<S169>/ChangeOrientation'
  B_AHRS2_CLVF_T AHRS2;                // '<S169>/AHRS2'
  B_CreateRotationMatrix_CLVF_i_T sf_CreateRotationMatrix_n;// '<S46>/Create Rotation Matrix' 
  B_CreateRotationMatrix_CLVF_T sf_CreateRotationMatrix_l;// '<S50>/Create Rotation Matrix' 
  B_MATLABFunction2_CLVF_T sf_MATLABFunction2_l;// '<S45>/MATLAB Function2'
  B_CreateRotationMatrix_CLVF_i_T sf_CreateRotationMatrix_f;// '<S37>/Create Rotation Matrix' 
  B_CreateRotationMatrix_CLVF_T sf_CreateRotationMatrix_i;// '<S41>/Create Rotation Matrix' 
  B_MATLABFunction2_CLVF_T sf_MATLABFunction2_n;// '<S36>/MATLAB Function2'
  B_MATLABFunction_CLVF_T sf_MATLABFunction1_b;// '<S36>/MATLAB Function1'
  B_MATLABFunction_CLVF_T sf_MATLABFunction_i;// '<S36>/MATLAB Function'
  B_CreateRotationMatrix_CLVF_i_T sf_CreateRotationMatrix_h;// '<S28>/Create Rotation Matrix' 
  B_CreateRotationMatrix_CLVF_T sf_CreateRotationMatrix;// '<S32>/Create Rotation Matrix' 
  B_MATLABFunction2_CLVF_T sf_MATLABFunction2;// '<S27>/MATLAB Function2'
} B_CLVF_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  raspi_internal_lsm9ds1Block_C_T obj; // '<S6>/LSM9DS1 IMU Sensor'
  PhaseSpace_ALL_CLVF_T obj_k;         // '<S80>/Stream PhaseSpace to Platform'
  codertarget_linux_blocks_Di_g_T obj_f;// '<S70>/Digital Read'
  raspi_internal_PWMBlock_CLVF_T obj_d;// '<S68>/PWM1'
  raspi_internal_PWMBlock_CLVF_T obj_m;// '<S26>/RED T1 - BLACK T8'
  raspi_internal_PWMBlock_CLVF_T obj_ma;// '<S26>/RED T2 - BLACK T3'
  raspi_internal_PWMBlock_CLVF_T obj_h;// '<S26>/RED T3'
  raspi_internal_PWMBlock_CLVF_T obj_kw;// '<S26>/RED T4 - BLACK T5'
  raspi_internal_PWMBlock_CLVF_T obj_hw;// '<S26>/RED T5 - BLACK T4'
  raspi_internal_PWMBlock_CLVF_T obj_hf;// '<S26>/RED T6 - BLACK T7'
  raspi_internal_PWMBlock_CLVF_T obj_c;// '<S26>/RED T7 - BLACK T6'
  raspi_internal_PWMBlock_CLVF_T obj_fo;// '<S26>/RED T8 - BLACK T1'
  raspi_internal_PWMBlock_CLVF_T obj_ci;// '<S26>/BLACK T2'
  codertarget_linux_blocks_Digi_T obj_cp;// '<S72>/Digital Write'
  codertarget_linux_blocks_Digi_T obj_b;// '<S71>/Digital Write'
  codertarget_linux_blocks_Digi_T obj_a;// '<S21>/Digital Write'
  codertarget_linux_blocks_Digi_T obj_i;// '<S20>/Digital Write'
  real_T Delay1_DSTATE;                // '<S516>/Delay1'
  real_T Delay1_DSTATE_m;              // '<S518>/Delay1'
  real_T Delay1_DSTATE_n;              // '<S514>/Delay1'
  real_T Delay1_DSTATE_o;              // '<S490>/Delay1'
  real_T Delay1_DSTATE_p;              // '<S492>/Delay1'
  real_T Delay1_DSTATE_b;              // '<S488>/Delay1'
  real_T Delay1_DSTATE_i;              // '<S471>/Delay1'
  real_T Delay1_DSTATE_h;              // '<S473>/Delay1'
  real_T Delay1_DSTATE_l;              // '<S475>/Delay1'
  real_T Delay1_DSTATE_lt;             // '<S447>/Delay1'
  real_T Delay1_DSTATE_im;             // '<S449>/Delay1'
  real_T Delay1_DSTATE_f;              // '<S445>/Delay1'
  real_T Delay1_DSTATE_a;              // '<S428>/Delay1'
  real_T uHzLPFilter1_states[2];       // '<S423>/1Hz LP Filter1'
  real_T Delay1_DSTATE_nv;             // '<S430>/Delay1'
  real_T Delay1_DSTATE_m2;             // '<S432>/Delay1'
  real_T Delay1_DSTATE_le;             // '<S414>/Delay1'
  real_T Delay1_DSTATE_nl;             // '<S416>/Delay1'
  real_T Delay1_DSTATE_h2;             // '<S418>/Delay1'
  real_T Delay_DSTATE;                 // '<S354>/Delay'
  real_T Delay1_DSTATE_e;              // '<S344>/Delay1'
  real_T uHzLPFilter_states[2];        // '<S339>/1Hz LP Filter'
  real_T uHzLPFilter1_states_l[2];     // '<S342>/1Hz LP Filter1'
  real_T uHzLPFilter_states_m[2];      // '<S342>/1Hz LP Filter'
  real_T Delay1_DSTATE_hm;             // '<S363>/Delay1'
  real_T Delay1_DSTATE_g;              // '<S332>/Delay1'
  real_T Delay1_DSTATE_g2;             // '<S334>/Delay1'
  real_T Delay1_DSTATE_d;              // '<S330>/Delay1'
  real_T Delay1_DSTATE_ct;             // '<S308>/Delay1'
  real_T Delay1_DSTATE_bm;             // '<S310>/Delay1'
  real_T Delay1_DSTATE_pr;             // '<S312>/Delay1'
  real_T Delay1_DSTATE_fb;             // '<S297>/Delay1'
  real_T Delay1_DSTATE_n1;             // '<S299>/Delay1'
  real_T Delay1_DSTATE_cq;             // '<S295>/Delay1'
  real_T Delay1_DSTATE_k;              // '<S284>/Delay1'
  real_T Delay1_DSTATE_dv;             // '<S286>/Delay1'
  real_T Delay1_DSTATE_j;              // '<S282>/Delay1'
  real_T AccelerationtoVelocity_DSTATE[3];// '<S527>/Acceleration  to Velocity'
  real_T VelocitytoPosition_DSTATE[3]; // '<S527>/Velocity to Position'
  real_T Delay1_DSTATE_lo;             // '<S545>/Delay1'
  real_T Delay1_DSTATE_k3;             // '<S539>/Delay1'
  real_T Delay1_DSTATE_j3;             // '<S546>/Delay1'
  real_T Delay1_DSTATE_gq;             // '<S540>/Delay1'
  real_T Delay1_DSTATE_ea;             // '<S547>/Delay1'
  real_T AccelerationtoVelocity_DSTATE_d[3];// '<S528>/Acceleration  to Velocity' 
  real_T VelocitytoPosition_DSTATE_i[3];// '<S528>/Velocity to Position'
  real_T Delay1_DSTATE_oh;             // '<S548>/Delay1'
  real_T Delay1_DSTATE_ko;             // '<S541>/Delay1'
  real_T Delay1_DSTATE_il;             // '<S549>/Delay1'
  real_T Delay1_DSTATE_mf;             // '<S542>/Delay1'
  real_T Delay1_DSTATE_e1;             // '<S550>/Delay1'
  real_T DiscreteTimeIntegrator_DSTATE;// '<S16>/Discrete-Time Integrator'
  real_T DiscreteTimeIntegrator1_DSTATE;// '<S16>/Discrete-Time Integrator1'
  real_T DiscreteTimeIntegrator2_DSTATE;// '<S16>/Discrete-Time Integrator2'
  real_T AccelerationtoVelocity_DSTATE_h[3];// '<S532>/Acceleration  to Velocity' 
  real_T VelocitytoPosition_DSTATE_f[3];// '<S532>/Velocity to Position'
  real_T Delay1_DSTATE_i3;             // '<S536>/Delay1'
  real_T Delay1_DSTATE_ji;             // '<S537>/Delay1'
  real_T Delay1_DSTATE_mm;             // '<S543>/Delay1'
  real_T Delay1_DSTATE_d4;             // '<S538>/Delay1'
  real_T Delay1_DSTATE_cc;             // '<S544>/Delay1'
  real_T DigitalFilter_FILT_STATES[6]; // '<S237>/Digital Filter'
  real_T DiscreteTimeIntegrator1_DSTAT_i;// '<S243>/Discrete-Time Integrator1'
  real_T DigitalFilter_FILT_STATES_k[6];// '<S206>/Digital Filter'
  real_T DiscreteTimeIntegrator1_DSTAT_a;// '<S212>/Discrete-Time Integrator1'
  real_T DigitalFilter_FILT_STATES_m[6];// '<S175>/Digital Filter'
  real_T DiscreteTimeIntegrator1_DSTAT_m;// '<S181>/Discrete-Time Integrator1'
  real_T Delay1_DSTATE_mo;             // '<S115>/Delay1'
  real_T Delay1_DSTATE_ig;             // '<S110>/Delay1'
  real_T Delay1_DSTATE_lz;             // '<S116>/Delay1'
  real_T Delay1_DSTATE_ew;             // '<S111>/Delay1'
  real_T Delay1_DSTATE_io;             // '<S117>/Delay1'
  real_T Delay1_DSTATE_bt;             // '<S112>/Delay1'
  real_T Delay1_DSTATE_lg;             // '<S109>/Delay1'
  real_T Delay1_DSTATE_kq;             // '<S118>/Delay1'
  real_T Delay1_DSTATE_ph;             // '<S113>/Delay1'
  real_T Delay1_DSTATE_mc;             // '<S119>/Delay1'
  real_T Delay1_DSTATE_phy;            // '<S114>/Delay1'
  real_T Delay1_DSTATE_ol;             // '<S120>/Delay1'
  real_T Delay1_DSTATE_hp;             // '<S87>/Delay1'
  real_T Delay1_DSTATE_bi;             // '<S82>/Delay1'
  real_T Delay1_DSTATE_fo;             // '<S88>/Delay1'
  real_T Delay1_DSTATE_f4;             // '<S83>/Delay1'
  real_T Delay1_DSTATE_lb;             // '<S89>/Delay1'
  real_T Delay1_DSTATE_i2;             // '<S84>/Delay1'
  real_T Delay1_DSTATE_lm;             // '<S81>/Delay1'
  real_T Delay1_DSTATE_ctf;            // '<S90>/Delay1'
  real_T Delay1_DSTATE_du;             // '<S85>/Delay1'
  real_T Delay1_DSTATE_gu;             // '<S91>/Delay1'
  real_T Delay1_DSTATE_jc;             // '<S86>/Delay1'
  real_T Delay1_DSTATE_c4;             // '<S92>/Delay1'
  real_T Delay1_DSTATE_ds;             // '<S147>/Delay1'
  real_T Delay1_DSTATE_bx;             // '<S142>/Delay1'
  real_T Delay1_DSTATE_bq;             // '<S148>/Delay1'
  real_T Delay1_DSTATE_n2;             // '<S143>/Delay1'
  real_T Delay1_DSTATE_az;             // '<S149>/Delay1'
  real_T Delay1_DSTATE_nu;             // '<S144>/Delay1'
  real_T Delay1_DSTATE_d3;             // '<S141>/Delay1'
  real_T Delay1_DSTATE_hy;             // '<S150>/Delay1'
  real_T Delay1_DSTATE_gb;             // '<S145>/Delay1'
  real_T Delay1_DSTATE_fn;             // '<S151>/Delay1'
  real_T Delay1_DSTATE_ek;             // '<S146>/Delay1'
  real_T Delay1_DSTATE_k1;             // '<S152>/Delay1'
  real_T Delay_DSTATE_k;               // '<S58>/Delay'
  real_T RateTransition_Buffer;        // '<S3>/Rate Transition'
  real_T RateTransition1_Buffer;       // '<S3>/Rate Transition1'
  real_T RateTransition2_Buffer;       // '<S3>/Rate Transition2'
  real_T RateTransition3_Buffer;       // '<S3>/Rate Transition3'
  real_T RateTransition4_Buffer;       // '<S3>/Rate Transition4'
  real_T RateTransition5_Buffer;       // '<S3>/Rate Transition5'
  real_T RateTransition6_Buffer;       // '<S3>/Rate Transition6'
  real_T RateTransition7_Buffer;       // '<S3>/Rate Transition7'
  real_T RateTransition8_Buffer;       // '<S3>/Rate Transition8'
  real_T UDPSend_NetworkLib[137];      // '<S15>/UDP Send'
  real_T BLACK_Fx;                     // '<Root>/BLACK_Fx'
  real_T BLUE_Fx;                      // '<Root>/BLACK_Fx1'
  real_T BLACK_Fx_Sat;                 // '<Root>/BLACK_Fx_Sat'
  real_T ARM_Elbow_Py;                 // '<Root>/BLACK_Fx_Sat1'
  real_T BLUE_Fx_Sat;                  // '<Root>/BLACK_Fx_Sat2'
  real_T BLACK_Fy;                     // '<Root>/BLACK_Fy'
  real_T BLUE_Fy;                      // '<Root>/BLACK_Fy1'
  real_T BLACK_Fy_Sat;                 // '<Root>/BLACK_Fy_Sat'
  real_T ARM_Wrist_Px;                 // '<Root>/BLACK_Fy_Sat1'
  real_T BLUE_Fy_Sat;                  // '<Root>/BLACK_Fy_Sat2'
  real_T BLACK_Px;                     // '<Root>/BLACK_Px'
  real_T BLUE_Px;                      // '<Root>/BLACK_Px1'
  real_T BLACK_Py;                     // '<Root>/BLACK_Py'
  real_T BLUE_Py;                      // '<Root>/BLACK_Py1'
  real_T BLACK_Rz;                     // '<Root>/BLACK_Rz'
  real_T BLUE_Rz;                      // '<Root>/BLACK_Rz1'
  real_T BLACK_Tz;                     // '<Root>/BLACK_Tz'
  real_T BLUE_Tz;                      // '<Root>/BLACK_Tz1'
  real_T BLACK_Tz_Sat;                 // '<Root>/BLACK_Tz_Sat'
  real_T ARM_Wrist_Py;                 // '<Root>/BLACK_Tz_Sat1'
  real_T BLUE_Tz_Sat;                  // '<Root>/BLACK_Tz_Sat2'
  real_T SPEED_DESIRED_x;              // '<Root>/Data Store Memory'
  real_T SPEED_DESIRED_y;              // '<Root>/Data Store Memory1'
  real_T RED_Fx;                       // '<Root>/RED_Fx'
  real_T RED_Fx_Sat;                   // '<Root>/RED_Fx_Sat'
  real_T RED_Fy;                       // '<Root>/RED_Fy'
  real_T RED_Fy_Sat;                   // '<Root>/RED_Fy_Sat'
  real_T RED_Px;                       // '<Root>/RED_Px'
  real_T RED_AHRS_Q;                   // '<Root>/RED_Px1'
  real_T BLACK_IMU_Ax_b;               // '<Root>/RED_Px10'
  real_T BLACK_IMU_Ay_b;               // '<Root>/RED_Px11'
  real_T BLACK_IMU_Az_b;               // '<Root>/RED_Px12'
  real_T RED_IMU_Q;                    // '<Root>/RED_Px19'
  real_T RED_AHRS_P;                   // '<Root>/RED_Px2'
  real_T RED_IMU_P;                    // '<Root>/RED_Px20'
  real_T RED_IMU_R;                    // '<Root>/RED_Px21'
  real_T BLACK_IMU_Q;                  // '<Root>/RED_Px22'
  real_T BLACK_IMU_P;                  // '<Root>/RED_Px23'
  real_T BLACK_IMU_R;                  // '<Root>/RED_Px24'
  real_T RED_IMU_Psi;                  // '<Root>/RED_Px25'
  real_T BLACK_IMU_Psi;                // '<Root>/RED_Px26'
  real_T RED_AHRS_R;                   // '<Root>/RED_Px3'
  real_T RED_IMU_Ax_I;                 // '<Root>/RED_Px31'
  real_T RED_IMU_Ay_I;                 // '<Root>/RED_Px32'
  real_T BLACK_IMU_Ax_I;               // '<Root>/RED_Px37'
  real_T BLACK_IMU_Ay_I;               // '<Root>/RED_Px38'
  real_T BLACK_AHRS_Q;                 // '<Root>/RED_Px4'
  real_T BLACK_AHRS_P;                 // '<Root>/RED_Px5'
  real_T BLACK_AHRS_R;                 // '<Root>/RED_Px6'
  real_T RED_IMU_Ax_b;                 // '<Root>/RED_Px7'
  real_T RED_IMU_Ay_b;                 // '<Root>/RED_Px8'
  real_T RED_IMU_Az_b;                 // '<Root>/RED_Px9'
  real_T RED_Py;                       // '<Root>/RED_Py'
  real_T RED_Rz;                       // '<Root>/RED_Rz'
  real_T RED_Tz;                       // '<Root>/RED_Tz'
  real_T Float_State;                  // '<Root>/RED_Tz7'
  real_T Magnet_State;                 // '<Root>/RED_Tz8'
  real_T RED_Tz_RW;                    // '<Root>/RED_Tz_RW'
  real_T RED_Tz_RW_Sat;                // '<Root>/RED_Tz_RW Sat'
  real_T RED_Tz_Sat;                   // '<Root>/RED_Tz_Sat'
  real_T ARM_Elbow_Px;                 // '<Root>/RED_Tz_Sat1'
  real_T RED_dRz_RW_Sat;               // '<Root>/RED_dRz_RW Sat'
  real_T Univ_Time;                    // '<Root>/Universal_Time'
  real_T RED_Vx;                       // '<Root>/Universal_Time1'
  real_T RED_Ax;                       // '<Root>/Universal_Time10'
  real_T RED_Ay;                       // '<Root>/Universal_Time11'
  real_T BLACK_Ax;                     // '<Root>/Universal_Time12'
  real_T BLACK_Ay;                     // '<Root>/Universal_Time13'
  real_T BLACK_RzDD;                   // '<Root>/Universal_Time14'
  real_T RED_RzDD;                     // '<Root>/Universal_Time15'
  real_T RED_Vy;                       // '<Root>/Universal_Time2'
  real_T RED_RzD;                      // '<Root>/Universal_Time3'
  real_T BLACK_Vx;                     // '<Root>/Universal_Time4'
  real_T BLACK_Vy;                     // '<Root>/Universal_Time5'
  real_T BLACK_RzD;                    // '<Root>/Universal_Time6'
  real_T BLUE_Vx;                      // '<Root>/Universal_Time7'
  real_T BLUE_Vy;                      // '<Root>/Universal_Time8'
  real_T BLUE_RzD;                     // '<Root>/Universal_Time9'
  real_T uHzLPFilter1_tmp;             // '<S423>/1Hz LP Filter1'
  real_T uHzLPFilter_tmp;              // '<S339>/1Hz LP Filter'
  real_T uHzLPFilter1_tmp_i;           // '<S342>/1Hz LP Filter1'
  real_T uHzLPFilter_tmp_g;            // '<S342>/1Hz LP Filter'
  real_T TimeStampA;                   // '<S353>/Derivative'
  real_T LastUAtTimeA[3];              // '<S353>/Derivative'
  real_T TimeStampB;                   // '<S353>/Derivative'
  real_T LastUAtTimeB[3];              // '<S353>/Derivative'
  real_T TimeStampA_i;                 // '<S351>/Derivative'
  real_T LastUAtTimeA_i[3];            // '<S351>/Derivative'
  real_T TimeStampB_p;                 // '<S351>/Derivative'
  real_T LastUAtTimeB_k[3];            // '<S351>/Derivative'
  real_T NextOutput;                   // '<S527>/Random Number'
  real_T NextOutput_k;                 // '<S16>/Random Number1'
  real_T NextOutput_n;                 // '<S16>/Random Number'
  real_T NextOutput_m;                 // '<S16>/Random Number2'
  real_T NextOutput_h;                 // '<S528>/Random Number'
  real_T NextOutput_eo;                // '<S16>/Random Number5'
  real_T NextOutput_i;                 // '<S532>/Random Number'
  real_T NextOutput_mh;                // '<S16>/Random Number7'
  real_T NextOutput_kj;                // '<S16>/Random Number6'
  real_T NextOutput_m2;                // '<S16>/Random Number8'
  real_T Unwrap1_Prev;                 // '<S80>/Unwrap1'
  real_T Unwrap1_Cumsum;               // '<S80>/Unwrap1'
  real_T Unwrap_Prev;                  // '<S80>/Unwrap'
  real_T Unwrap_Cumsum;                // '<S80>/Unwrap'
  real_T SendBLACKStatestoBLACKPlatform_[137];
                                // '<S80>/Send BLACK States to  BLACK Platform'
  real_T UDPReceive_NetworkLib[137];   // '<S79>/UDP Receive'
  real_T Unwrap1_Prev_p;               // '<S79>/Unwrap1'
  real_T Unwrap1_Cumsum_e;             // '<S79>/Unwrap1'
  real_T Unwrap_Prev_c;                // '<S79>/Unwrap'
  real_T Unwrap_Cumsum_m;              // '<S79>/Unwrap'
  real_T Unwrap1_Prev_pv;              // '<S77>/Unwrap1'
  real_T Unwrap1_Cumsum_a;             // '<S77>/Unwrap1'
  real_T Unwrap_Prev_o;                // '<S77>/Unwrap'
  real_T Unwrap_Cumsum_o;              // '<S77>/Unwrap'
  real_T Product1_DWORK4[9];           // '<S51>/Product1'
  real_T Product1_DWORK4_g[9];         // '<S42>/Product1'
  real_T Product1_DWORK4_gx[9];        // '<S33>/Product1'
  struct {
    void *LoggedData;
  } ToWorkspace_PWORK;                 // '<S1>/To Workspace'

  struct {
    void *LoggedData;
  } ToWorkspace_PWORK_h;               // '<S349>/To Workspace'

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK;                // '<S349>/To Workspace2'

  struct {
    void *LoggedData;
  } ToWorkspace_PWORK_k;               // '<S342>/To Workspace'

  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                // '<S342>/To Workspace1'

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK_i;              // '<S342>/To Workspace2'

  struct {
    void *LoggedData;
  } ToWorkspace3_PWORK;                // '<S342>/To Workspace3'

  struct {
    void *LoggedData;
  } ToWorkspace_PWORK_d;               // '<S353>/To Workspace'

  uint32_T RandSeed;                   // '<S527>/Random Number'
  uint32_T RandSeed_d;                 // '<S16>/Random Number1'
  uint32_T RandSeed_b;                 // '<S16>/Random Number'
  uint32_T RandSeed_a;                 // '<S16>/Random Number2'
  uint32_T RandSeed_h;                 // '<S528>/Random Number'
  uint32_T RandSeed_i;                 // '<S16>/Random Number4'
  uint32_T RandSeed_c;                 // '<S16>/Random Number3'
  uint32_T RandSeed_n;                 // '<S16>/Random Number5'
  uint32_T RandSeed_ct;                // '<S532>/Random Number'
  uint32_T RandSeed_dd;                // '<S16>/Random Number7'
  uint32_T RandSeed_ik;                // '<S16>/Random Number6'
  uint32_T RandSeed_bm;                // '<S16>/Random Number8'
  boolean_T Delay_DSTATE_k2;           // '<S121>/Delay'
  boolean_T Delay_DSTATE_m;            // '<S153>/Delay'
  boolean_T Delay1_DSTATE_kt;          // '<S63>/Delay1'
  boolean_T Delay2_DSTATE[2];          // '<S63>/Delay2'
  boolean_T Delay5_DSTATE[3];          // '<S63>/Delay5'
  boolean_T Delay3_DSTATE[4];          // '<S63>/Delay3'
  boolean_T Delay4_DSTATE[5];          // '<S63>/Delay4'
  int8_T SeparatePhases_ActiveSubsystem;// '<Root>/Separate Phases'
  int8_T Ifperforminganexperimentgrabthe;
  // '<S5>/If performing an experiment, grab the PhaseSpace data. Otherwise, use a clock to set time in SIM.' 
  int8_T ThisIFblockdetermineswhethero_b;
  // '<S13>/This IF block determines whether or not to run the BLACK sim//exp'
  int8_T ThisIFblockdetermineswhethero_n;
    // '<S13>/This IF block determines whether or not to run the BLUE sim//exp'
  int8_T ThisIFblockdetermineswhethero_j;
    // '<S13>/This IF block determines whether or not to run the RED sim//exp '
  int8_T ThisIFblockdetermineswhethero_d;
  // '<S12>/This IF block determines whether or not to run the BLACK sim//exp'
  int8_T ThisIFblockdetermineswhether_jc;
    // '<S12>/This IF block determines whether or not to run the BLUE sim//exp'
  int8_T ThisIFblockdetermineswhether_a4;
    // '<S12>/This IF block determines whether or not to run the RED sim//exp '
  int8_T ThisIFblockdetermineswhethero_i;
  // '<S11>/This IF block determines whether or not to run the BLACK sim//exp'
  int8_T ThisIFblockdetermineswhether_aa;
    // '<S11>/This IF block determines whether or not to run the BLUE sim//exp'
  int8_T ThisIFblockdetermineswhethero_f;
    // '<S11>/This IF block determines whether or not to run the RED sim//exp '
  int8_T ExperimentSubPhases_ActiveSubsy;// '<S319>/Experiment Sub-Phases'
  int8_T ExperimentSubPhases_ActiveSub_h;// '<S318>/Experiment Sub-Phases'
  int8_T ExperimentSubPhases_ActiveSub_p;// '<S317>/Experiment Sub-Phases'
  int8_T If_ActiveSubsystem;           // '<S349>/If'
  int8_T ThisIFblockdetermineswhether_dy;
  // '<S10>/This IF block determines whether or not to run the BLACK sim//exp'
  int8_T ThisIFblockdetermineswhether_jm;
    // '<S10>/This IF block determines whether or not to run the BLUE sim//exp'
  int8_T ThisIFblockdetermineswhether_aj;
    // '<S10>/This IF block determines whether or not to run the RED sim//exp '
  int8_T DiscreteTimeIntegrator_PrevRese;// '<S16>/Discrete-Time Integrator'
  int8_T DiscreteTimeIntegrator1_PrevRes;// '<S16>/Discrete-Time Integrator1'
  int8_T DiscreteTimeIntegrator2_PrevRes;// '<S16>/Discrete-Time Integrator2'
  int8_T DiscreteTimeIntegrator1_PrevR_m;// '<S243>/Discrete-Time Integrator1'
  int8_T DiscreteTimeIntegrator1_PrevR_h;// '<S212>/Discrete-Time Integrator1'
  int8_T DiscreteTimeIntegrator1_PrevR_d;// '<S181>/Discrete-Time Integrator1'
  int8_T Checkwhetherbothplatformsarebei;
  // '<S75>/Check whether both platforms are being used, and if so then use RED to send data to BLACK ' 
  int8_T ThisIFblockdetermineswhethero_c;
  // '<S76>/This IF block determines whether or not to run the BLACK sim//exp'
  int8_T ThisIFblockdetermineswhether_cx;
    // '<S76>/This IF block determines whether or not to run the RED sim//exp '
  uint8_T icLoad;                      // '<S516>/Delay1'
  uint8_T icLoad_j;                    // '<S518>/Delay1'
  uint8_T icLoad_a;                    // '<S514>/Delay1'
  uint8_T icLoad_e;                    // '<S490>/Delay1'
  uint8_T icLoad_m;                    // '<S492>/Delay1'
  uint8_T icLoad_d;                    // '<S488>/Delay1'
  uint8_T icLoad_dh;                   // '<S471>/Delay1'
  uint8_T icLoad_a2;                   // '<S473>/Delay1'
  uint8_T icLoad_f;                    // '<S475>/Delay1'
  uint8_T icLoad_dq;                   // '<S447>/Delay1'
  uint8_T icLoad_h;                    // '<S449>/Delay1'
  uint8_T icLoad_o;                    // '<S445>/Delay1'
  uint8_T icLoad_jk;                   // '<S428>/Delay1'
  uint8_T icLoad_fc;                   // '<S430>/Delay1'
  uint8_T icLoad_n;                    // '<S432>/Delay1'
  uint8_T icLoad_c;                    // '<S414>/Delay1'
  uint8_T icLoad_l;                    // '<S416>/Delay1'
  uint8_T icLoad_jz;                   // '<S418>/Delay1'
  uint8_T icLoad_en;                   // '<S344>/Delay1'
  uint8_T icLoad_jp;                   // '<S363>/Delay1'
  uint8_T icLoad_of;                   // '<S364>/Delay1'
  uint8_T icLoad_i;                    // '<S365>/Delay1'
  uint8_T icLoad_ay;                   // '<S366>/Delay1'
  uint8_T icLoad_iz;                   // '<S332>/Delay1'
  uint8_T icLoad_p;                    // '<S334>/Delay1'
  uint8_T icLoad_dm;                   // '<S330>/Delay1'
  uint8_T icLoad_ji;                   // '<S308>/Delay1'
  uint8_T icLoad_a1;                   // '<S310>/Delay1'
  uint8_T icLoad_o0;                   // '<S312>/Delay1'
  uint8_T icLoad_dy;                   // '<S297>/Delay1'
  uint8_T icLoad_hf;                   // '<S299>/Delay1'
  uint8_T icLoad_k;                    // '<S295>/Delay1'
  uint8_T icLoad_ns;                   // '<S284>/Delay1'
  uint8_T icLoad_k1;                   // '<S286>/Delay1'
  uint8_T icLoad_ep;                   // '<S282>/Delay1'
  uint8_T icLoad_jt;                   // '<S545>/Delay1'
  uint8_T icLoad_fs;                   // '<S539>/Delay1'
  uint8_T icLoad_b;                    // '<S546>/Delay1'
  uint8_T icLoad_if;                   // '<S540>/Delay1'
  uint8_T icLoad_eg;                   // '<S547>/Delay1'
  uint8_T icLoad_hr;                   // '<S548>/Delay1'
  uint8_T icLoad_kp;                   // '<S541>/Delay1'
  uint8_T icLoad_g;                    // '<S549>/Delay1'
  uint8_T icLoad_ge;                   // '<S542>/Delay1'
  uint8_T icLoad_ho;                   // '<S550>/Delay1'
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S16>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;// '<S16>/Discrete-Time Integrator1'
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;// '<S16>/Discrete-Time Integrator2'
  uint8_T icLoad_im;                   // '<S536>/Delay1'
  uint8_T icLoad_jx;                   // '<S537>/Delay1'
  uint8_T icLoad_mj;                   // '<S543>/Delay1'
  uint8_T icLoad_nv;                   // '<S538>/Delay1'
  uint8_T icLoad_nx;                   // '<S544>/Delay1'
  uint8_T DiscreteTimeIntegrator1_IC_LO_b;// '<S243>/Discrete-Time Integrator1'
  uint8_T DiscreteTimeIntegrator1_IC_LO_l;// '<S212>/Discrete-Time Integrator1'
  uint8_T DiscreteTimeIntegrator1_IC_LO_f;// '<S181>/Discrete-Time Integrator1'
  uint8_T icLoad_bo;                   // '<S115>/Delay1'
  uint8_T icLoad_lm;                   // '<S110>/Delay1'
  uint8_T icLoad_h2;                   // '<S116>/Delay1'
  uint8_T icLoad_fc0;                  // '<S111>/Delay1'
  uint8_T icLoad_jq;                   // '<S117>/Delay1'
  uint8_T icLoad_ab;                   // '<S112>/Delay1'
  uint8_T icLoad_ez;                   // '<S109>/Delay1'
  uint8_T icLoad_n4;                   // '<S118>/Delay1'
  uint8_T icLoad_ln;                   // '<S113>/Delay1'
  uint8_T icLoad_dk;                   // '<S119>/Delay1'
  uint8_T icLoad_kr;                   // '<S114>/Delay1'
  uint8_T icLoad_gt;                   // '<S120>/Delay1'
  uint8_T icLoad_ha;                   // '<S87>/Delay1'
  uint8_T icLoad_oh;                   // '<S82>/Delay1'
  uint8_T icLoad_ij;                   // '<S88>/Delay1'
  uint8_T icLoad_dr;                   // '<S83>/Delay1'
  uint8_T icLoad_ip;                   // '<S89>/Delay1'
  uint8_T icLoad_lp;                   // '<S84>/Delay1'
  uint8_T icLoad_nj;                   // '<S81>/Delay1'
  uint8_T icLoad_av;                   // '<S90>/Delay1'
  uint8_T icLoad_ib;                   // '<S85>/Delay1'
  uint8_T icLoad_hm;                   // '<S91>/Delay1'
  uint8_T icLoad_jh;                   // '<S86>/Delay1'
  uint8_T icLoad_bb;                   // '<S92>/Delay1'
  uint8_T icLoad_po;                   // '<S147>/Delay1'
  uint8_T icLoad_nx2;                  // '<S142>/Delay1'
  uint8_T icLoad_on;                   // '<S148>/Delay1'
  uint8_T icLoad_lw;                   // '<S143>/Delay1'
  uint8_T icLoad_dx;                   // '<S149>/Delay1'
  uint8_T icLoad_pz;                   // '<S144>/Delay1'
  uint8_T icLoad_jj;                   // '<S141>/Delay1'
  uint8_T icLoad_fv;                   // '<S150>/Delay1'
  uint8_T icLoad_jqu;                  // '<S145>/Delay1'
  uint8_T icLoad_b3;                   // '<S151>/Delay1'
  uint8_T icLoad_cw;                   // '<S146>/Delay1'
  uint8_T icLoad_hrx;                  // '<S152>/Delay1'
  boolean_T Unwrap1_FirstStep;         // '<S80>/Unwrap1'
  boolean_T Unwrap_FirstStep;          // '<S80>/Unwrap'
  boolean_T Unwrap1_FirstStep_j;       // '<S79>/Unwrap1'
  boolean_T Unwrap_FirstStep_e;        // '<S79>/Unwrap'
  boolean_T Unwrap1_FirstStep_jh;      // '<S77>/Unwrap1'
  boolean_T Unwrap_FirstStep_b;        // '<S77>/Unwrap'
  boolean_T EnabledSubsystem1_MODE;    // '<S80>/Enabled Subsystem1'
  boolean_T EnabledSubsystem_MODE;     // '<S80>/Enabled Subsystem'
  boolean_T EnabledSubsystem_MODE_p;   // '<S106>/Enabled Subsystem'
  boolean_T EnabledSubsystem1_MODE_d;  // '<S77>/Enabled Subsystem1'
  boolean_T EnabledSubsystem_MODE_l;   // '<S138>/Enabled Subsystem'
  DW_ChangeBLUEBehavior_CLVF_i_T ChangeBLUEBehavior_b;// '<S13>/Change BLUE Behavior' 
  DW_ChangeBLUEBehavior_CLVF_i_T ChangeBLUEBehavior_gj;// '<S12>/Change BLUE Behavior' 
  DW_SubPhase1_CLVF_T SubPhase4_o;     // '<S318>/Sub-Phase #4'
  DW_SubPhase1_CLVF_T SubPhase1_g;     // '<S318>/Sub-Phase #1'
  DW_Phase0WaitforSynchronizati_T Phase1StartFloating;// '<Root>/Phase #1:  Start Floating ' 
  DW_Phase0WaitforSynchronizati_T Phase0WaitforSynchronization;
                                // '<Root>/Phase #0:  Wait for Synchronization'
  DW_CalculateRunningMean_CLV_e_T CalculateRunningMean_mv;// '<S256>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_e;// '<S255>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_ck;// '<S254>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_avn;// '<S243>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_bv;// '<S242>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_mr;// '<S241>/Calculate Running Mean' 
  DW_AHRS2_CLVF_T AHRS2_pn;            // '<S169>/AHRS2'
  DW_CalculateRunningMean_CLV_e_T CalculateRunningMean_l;// '<S225>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_pa;// '<S224>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_m;// '<S223>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_av;// '<S212>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_a;// '<S211>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_b2;// '<S210>/Calculate Running Mean' 
  DW_AHRS2_CLVF_T AHRS2_p;             // '<S169>/AHRS2'
  DW_CalculateRunningMean_CLV_e_T CalculateRunningMean_b;// '<S194>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_k;// '<S193>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_ch;// '<S192>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_c;// '<S181>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean_p;// '<S180>/Calculate Running Mean' 
  DW_CalculateRunningMean_CLVF_T CalculateRunningMean;// '<S179>/Calculate Running Mean' 
  DW_AHRS2_CLVF_T AHRS2;               // '<S169>/AHRS2'
} DW_CLVF_T;

// Zero-crossing (trigger) state
typedef struct {
  ZCSigState SampleandHold_Trig_ZCE;   // '<S354>/Sample and Hold'
  ZCSigState SampleandHold_Trig_ZCE_l; // '<S535>/Sample and Hold'
  ZCSigState SampleandHold1_Trig_ZCE;  // '<S534>/Sample and Hold1'
  ZCSigState SampleandHold1_Trig_ZCE_g;// '<S533>/Sample and Hold1'
} PrevZCX_CLVF_T;

// Parameters for system: '<S169>/AHRS2'
struct P_AHRS2_CLVF_T_ {
  real_T AHRS2_AccelerometerNoise;     // Expression: 0.0001924722
                                          //  Referenced by: '<S169>/AHRS2'

  real_T AHRS2_GyroscopeNoise;         // Expression: 9.1385e-5
                                          //  Referenced by: '<S169>/AHRS2'

  real_T AHRS2_MagnetometerNoise;      // Expression: 0.1
                                          //  Referenced by: '<S169>/AHRS2'

  real_T AHRS2_GyroscopeDriftNoise;    // Expression: 3.0462e-13
                                          //  Referenced by: '<S169>/AHRS2'

  real_T AHRS2_LinearAccelerationNoise;// Expression: 0.0096236100000000012
                                          //  Referenced by: '<S169>/AHRS2'

  real_T AHRS2_MagneticDisturbanceNoise;// Expression: 0.5
                                           //  Referenced by: '<S169>/AHRS2'

  real_T AHRS2_LinearAccelerationDecayFa;// Expression: 0.5
                                            //  Referenced by: '<S169>/AHRS2'

  real_T AHRS2_MagneticDisturbanceDecayF;// Expression: 0.5
                                            //  Referenced by: '<S169>/AHRS2'

  real_T AHRS2_ExpectedMagneticFieldStre;// Expression: 33
                                            //  Referenced by: '<S169>/AHRS2'

};

// Parameters for system: '<S179>/Calculate Running Mean'
struct P_CalculateRunningMean_CLVF_T_ {
  real_T Out1_Y0;                      // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S182>/Out1'

  real_T Delay_InitialCondition;       // Expression: 0.0
                                          //  Referenced by: '<S182>/Delay'

  real_T Delay1_InitialCondition;      // Expression: 0.0
                                          //  Referenced by: '<S182>/Delay1'

  real_T Delay2_InitialCondition;      // Expression: 0.0
                                          //  Referenced by: '<S182>/Delay2'

  real_T Delay3_InitialCondition;      // Expression: 0.0
                                          //  Referenced by: '<S182>/Delay3'

  real_T Delay4_InitialCondition;      // Expression: 0.0
                                          //  Referenced by: '<S182>/Delay4'

};

// Parameters for system: '<S194>/Calculate Running Mean'
struct P_CalculateRunningMean_CLVF_c_T_ {
  real_T Out1_Y0;                      // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S201>/Out1'

  real_T Constant_Value;               // Expression: 9.81
                                          //  Referenced by: '<S201>/Constant'

  real_T Delay_InitialCondition;       // Expression: 0.0
                                          //  Referenced by: '<S201>/Delay'

  real_T Delay1_InitialCondition;      // Expression: 0.0
                                          //  Referenced by: '<S201>/Delay1'

  real_T Delay2_InitialCondition;      // Expression: 0.0
                                          //  Referenced by: '<S201>/Delay2'

  real_T Delay3_InitialCondition;      // Expression: 0.0
                                          //  Referenced by: '<S201>/Delay3'

  real_T Delay4_InitialCondition;      // Expression: 0.0
                                          //  Referenced by: '<S201>/Delay4'

};

// Parameters for system: '<S8>/Change BLUE Behavior'
struct P_ChangeBLUEBehavior_CLVF_T_ {
  real_T Constant3_Value;              // Expression: 0
                                          //  Referenced by: '<S267>/Constant3'

  real_T Constant4_Value;              // Expression: 0
                                          //  Referenced by: '<S267>/Constant4'

  real_T Constant5_Value;              // Expression: 0
                                          //  Referenced by: '<S267>/Constant5'

  real_T PuckState_Value;              // Expression: 0
                                          //  Referenced by: '<S267>/Puck State'

};

// Parameters for system: '<Root>/Phase #0:  Wait for Synchronization'
struct P_Phase0WaitforSynchronizatio_T_ {
  real_T Constant3_Value;              // Expression: 0
                                          //  Referenced by: '<S266>/Constant3'

  real_T Constant4_Value;              // Expression: 0
                                          //  Referenced by: '<S266>/Constant4'

  real_T Constant5_Value;              // Expression: 0
                                          //  Referenced by: '<S266>/Constant5'

  real_T PuckState_Value;              // Expression: 0
                                          //  Referenced by: '<S266>/Puck State'

  real_T Constant_Value;               // Expression: 0
                                          //  Referenced by: '<S268>/Constant'

  real_T Constant1_Value;              // Expression: 0
                                          //  Referenced by: '<S268>/Constant1'

  real_T Constant2_Value;              // Expression: 0
                                          //  Referenced by: '<S268>/Constant2'

  real_T Constant3_Value_h;            // Expression: 0
                                          //  Referenced by: '<S268>/Constant3'

  real_T Constant4_Value_g;            // Expression: 0
                                          //  Referenced by: '<S268>/Constant4'

  real_T Constant5_Value_e;            // Expression: 0
                                          //  Referenced by: '<S268>/Constant5'

  real_T Constant6_Value;              // Expression: 0
                                          //  Referenced by: '<S268>/Constant6'

  real_T PuckState_Value_b;            // Expression: 0
                                          //  Referenced by: '<S268>/Puck State'

  P_ChangeBLUEBehavior_CLVF_T ChangeBLUEBehavior;// '<S8>/Change BLUE Behavior'
};

// Parameters for system: '<S317>/Sub-Phase #2 '
struct P_SubPhase2_CLVF_T_ {
  real_T Constant_Value;               // Expression: 0
                                          //  Referenced by: '<S322>/Constant'

  real_T PuckState_Value;              // Expression: 0
                                          //  Referenced by: '<S322>/Puck State'

};

// Parameters for system: '<S318>/Sub-Phase #1'
struct P_SubPhase1_CLVF_T_ {
  real_T Out1_Y0;                      // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S384>/Out1'

  real_T Out1_Y0_l;                    // Computed Parameter: Out1_Y0_l
                                          //  Referenced by: '<S386>/Out1'

  real_T Out1_Y0_e;                    // Computed Parameter: Out1_Y0_e
                                          //  Referenced by: '<S388>/Out1'

  real_T PuckState_Value;              // Expression: 1
                                          //  Referenced by: '<S374>/Puck State'

};

// Parameters for system: '<S318>/Sub-Phase #2 '
struct P_SubPhase2_CLVF_h_T_ {
  real_T Constant_Value;               // Expression: 0
                                          //  Referenced by: '<S375>/Constant'

  real_T PuckState_Value;              // Expression: 0
                                          //  Referenced by: '<S375>/Puck State'

};

// Parameters for system: '<S319>/Sub-Phase #2 '
struct P_SubPhase2_CLVF_e_T_ {
  real_T Constant_Value;               // Expression: 0
                                          //  Referenced by: '<S406>/Constant'

  real_T Constant1_Value;              // Expression: 0
                                          //  Referenced by: '<S406>/Constant1'

  real_T Constant4_Value;              // Expression: 0
                                          //  Referenced by: '<S406>/Constant4'

  real_T Constant5_Value;              // Expression: 0
                                          //  Referenced by: '<S406>/Constant5'

  real_T Constant6_Value;              // Expression: 0
                                          //  Referenced by: '<S406>/Constant6'

  real_T PuckState_Value;              // Expression: 0
                                          //  Referenced by: '<S406>/Puck State'

};

// Parameters for system: '<S12>/Change BLUE Behavior'
struct P_ChangeBLUEBehavior_CLVF_c_T_ {
  real_T Out1_Y0;                      // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S459>/Out1'

  real_T Out1_Y0_k;                    // Computed Parameter: Out1_Y0_k
                                          //  Referenced by: '<S461>/Out1'

  real_T Out1_Y0_n;                    // Computed Parameter: Out1_Y0_n
                                          //  Referenced by: '<S463>/Out1'

  real_T PuckState_Value;              // Expression: 1
                                          //  Referenced by: '<S438>/Puck State'

};

// Parameters (default storage)
struct P_CLVF_T_ {
  real_T BLACKMass;                    // Variable: BLACKMass
                                          //  Referenced by: '<S352>/Constant'

  real_T F_thrusters_BLACK[8];         // Variable: F_thrusters_BLACK
                                          //  Referenced by:
                                          //    '<S27>/MATLAB Function'
                                          //    '<S27>/MATLAB Function1'

  real_T F_thrusters_BLUE[8];          // Variable: F_thrusters_BLUE
                                          //  Referenced by:
                                          //    '<S36>/MATLAB Function'
                                          //    '<S36>/MATLAB Function1'

  real_T F_thrusters_RED[8];           // Variable: F_thrusters_RED
                                          //  Referenced by:
                                          //    '<S45>/MATLAB Function'
                                          //    '<S45>/MATLAB Function1'

  real_T Kd_tb;                        // Variable: Kd_tb
                                          //  Referenced by:
                                          //    '<S278>/kd_tb'
                                          //    '<S441>/kd_tb'
                                          //    '<S484>/kd_tb'
                                          //    '<S326>/kd_tb'
                                          //    '<S339>/kd_tb'

  real_T Kd_tblue;                     // Variable: Kd_tblue
                                          //  Referenced by:
                                          //    '<S291>/kd_tb'
                                          //    '<S454>/kd_tb'
                                          //    '<S497>/kd_tb'
                                          //    '<S379>/kd_tb'
                                          //    '<S392>/kd_tb'

  real_T Kd_tr;                        // Variable: Kd_tr
                                          //  Referenced by:
                                          //    '<S304>/kd_tr'
                                          //    '<S467>/kd_tr'
                                          //    '<S510>/kd_tr'
                                          //    '<S410>/kd_tr'
                                          //    '<S423>/kd_tr'

  real_T Kd_xb;                        // Variable: Kd_xb
                                          //  Referenced by:
                                          //    '<S279>/kd_xb'
                                          //    '<S442>/kd_xb'
                                          //    '<S485>/kd_xb'
                                          //    '<S327>/kd_xb'

  real_T Kd_xblue;                     // Variable: Kd_xblue
                                          //  Referenced by:
                                          //    '<S292>/kd_xb'
                                          //    '<S455>/kd_xb'
                                          //    '<S498>/kd_xb'
                                          //    '<S380>/kd_xb'
                                          //    '<S393>/kd_xb'

  real_T Kd_xr;                        // Variable: Kd_xr
                                          //  Referenced by:
                                          //    '<S305>/kd_xr'
                                          //    '<S468>/kd_xr'
                                          //    '<S511>/kd_xr'
                                          //    '<S411>/kd_xr'
                                          //    '<S424>/kd_xr'

  real_T Kd_yb;                        // Variable: Kd_yb
                                          //  Referenced by:
                                          //    '<S280>/kd_yb'
                                          //    '<S443>/kd_yb'
                                          //    '<S486>/kd_yb'
                                          //    '<S328>/kd_yb'

  real_T Kd_yblue;                     // Variable: Kd_yblue
                                          //  Referenced by:
                                          //    '<S293>/kd_yb'
                                          //    '<S456>/kd_yb'
                                          //    '<S499>/kd_yb'
                                          //    '<S381>/kd_yb'
                                          //    '<S394>/kd_yb'

  real_T Kd_yr;                        // Variable: Kd_yr
                                          //  Referenced by:
                                          //    '<S306>/kd_yr'
                                          //    '<S469>/kd_yr'
                                          //    '<S512>/kd_yr'
                                          //    '<S412>/kd_yr'
                                          //    '<S425>/kd_yr'

  real_T Kp_tb;                        // Variable: Kp_tb
                                          //  Referenced by:
                                          //    '<S278>/kp_tb'
                                          //    '<S441>/kp_tb'
                                          //    '<S484>/kp_tb'
                                          //    '<S326>/kp_tb'
                                          //    '<S339>/kp_tb'

  real_T Kp_tblue;                     // Variable: Kp_tblue
                                          //  Referenced by:
                                          //    '<S291>/kp_tb'
                                          //    '<S454>/kp_tb'
                                          //    '<S497>/kp_tb'
                                          //    '<S379>/kp_tb'
                                          //    '<S392>/kp_tb'

  real_T Kp_tr;                        // Variable: Kp_tr
                                          //  Referenced by:
                                          //    '<S304>/kp_tr'
                                          //    '<S467>/kp_tr'
                                          //    '<S510>/kp_tr'
                                          //    '<S410>/kp_tr'
                                          //    '<S423>/kp_tr'

  real_T Kp_xb;                        // Variable: Kp_xb
                                          //  Referenced by:
                                          //    '<S279>/kp_xb'
                                          //    '<S442>/kp_xb'
                                          //    '<S485>/kp_xb'
                                          //    '<S327>/kp_xb'

  real_T Kp_xblue;                     // Variable: Kp_xblue
                                          //  Referenced by:
                                          //    '<S292>/kp_xb'
                                          //    '<S455>/kp_xb'
                                          //    '<S498>/kp_xb'
                                          //    '<S380>/kp_xb'
                                          //    '<S393>/kp_xb'

  real_T Kp_xr;                        // Variable: Kp_xr
                                          //  Referenced by:
                                          //    '<S305>/kp_xr'
                                          //    '<S468>/kp_xr'
                                          //    '<S511>/kp_xr'
                                          //    '<S411>/kp_xr'
                                          //    '<S424>/kp_xr'

  real_T Kp_yb;                        // Variable: Kp_yb
                                          //  Referenced by:
                                          //    '<S280>/kp_yb'
                                          //    '<S443>/kp_yb'
                                          //    '<S486>/kp_yb'
                                          //    '<S328>/kp_yb'

  real_T Kp_yblue;                     // Variable: Kp_yblue
                                          //  Referenced by:
                                          //    '<S293>/kp_yb'
                                          //    '<S456>/kp_yb'
                                          //    '<S499>/kp_yb'
                                          //    '<S381>/kp_yb'
                                          //    '<S394>/kp_yb'

  real_T Kp_yr;                        // Variable: Kp_yr
                                          //  Referenced by:
                                          //    '<S306>/kp_yr'
                                          //    '<S469>/kp_yr'
                                          //    '<S512>/kp_yr'
                                          //    '<S412>/kp_yr'
                                          //    '<S425>/kp_yr'

  real_T Phase0_End;                   // Variable: Phase0_End
                                          //  Referenced by: '<Root>/Constant4'

  real_T Phase1_End;                   // Variable: Phase1_End
                                          //  Referenced by: '<Root>/Constant'

  real_T Phase2_End;                   // Variable: Phase2_End
                                          //  Referenced by: '<Root>/Constant1'

  real_T Phase3_End;                   // Variable: Phase3_End
                                          //  Referenced by: '<Root>/Constant2'

  real_T Phase3_SubPhase1_End;         // Variable: Phase3_SubPhase1_End
                                          //  Referenced by:
                                          //    '<S317>/Constant4'
                                          //    '<S318>/Constant4'
                                          //    '<S319>/Constant4'

  real_T Phase3_SubPhase2_End;         // Variable: Phase3_SubPhase2_End
                                          //  Referenced by:
                                          //    '<S317>/Constant1'
                                          //    '<S318>/Constant1'
                                          //    '<S319>/Constant1'

  real_T Phase3_SubPhase3_End_BLACK;   // Variable: Phase3_SubPhase3_End_BLACK
                                          //  Referenced by:
                                          //    '<S317>/Constant2'
                                          //    '<S318>/Constant2'

  real_T Phase3_SubPhase3_End_RED;     // Variable: Phase3_SubPhase3_End_RED
                                          //  Referenced by:
                                          //    '<S319>/Constant2'
                                          //    '<S427>/Constant1'

  real_T Phase3_SubPhase4_End;         // Variable: Phase3_SubPhase4_End
                                          //  Referenced by:
                                          //    '<S317>/Constant3'
                                          //    '<S318>/Constant3'
                                          //    '<S319>/Constant3'

  real_T Phase4_End;                   // Variable: Phase4_End
                                          //  Referenced by: '<Root>/Constant3'

  real_T Phase5_End;                   // Variable: Phase5_End
                                          //  Referenced by: '<Root>/Constant6'

  real_T WhoAmI;                       // Variable: WhoAmI
                                          //  Referenced by:
                                          //    '<S3>/Constant'
                                          //    '<S4>/Constant'
                                          //    '<S6>/Constant1'
                                          //    '<S8>/Constant'
                                          //    '<S9>/Constant'
                                          //    '<S10>/Constant'
                                          //    '<S11>/Constant'
                                          //    '<S12>/Constant'
                                          //    '<S13>/Constant'
                                          //    '<S14>/Constant'
                                          //    '<S76>/Constant'

  real_T a;                            // Variable: a
                                          //  Referenced by: '<S351>/Constant1'

  real_T aTimesOVec[3];                // Variable: aTimesOVec
                                          //  Referenced by: '<S354>/Constant'

  real_T a_prime;                      // Variable: a_prime
                                          //  Referenced by: '<S353>/Constant4'

  real_T acceptableRadius;             // Variable: acceptableRadius
                                          //  Referenced by: '<S354>/Constant1'

  real_T b;                            // Variable: b
                                          //  Referenced by: '<S351>/Constant2'

  real_T cntThreshold;                 // Variable: cntThreshold
                                          //  Referenced by: '<S354>/Constant2'

  real_T d[3];                         // Variable: d
                                          //  Referenced by:
                                          //    '<S342>/MATLAB Function'
                                          //    '<S353>/Constant1'

  real_T den_d[3];                     // Variable: den_d
                                          //  Referenced by:
                                          //    '<S339>/1Hz LP Filter'
                                          //    '<S342>/1Hz LP Filter'
                                          //    '<S342>/1Hz LP Filter1'
                                          //    '<S423>/1Hz LP Filter1'

  real_T drop_states_BLACK[3];         // Variable: drop_states_BLACK
                                          //  Referenced by: '<S527>/Velocity to Position'

  real_T drop_states_BLUE[3];          // Variable: drop_states_BLUE
                                          //  Referenced by: '<S528>/Velocity to Position'

  real_T drop_states_RED[3];           // Variable: drop_states_RED
                                          //  Referenced by: '<S532>/Velocity to Position'

  real_T finalAngle;                   // Variable: finalAngle
                                          //  Referenced by: '<S353>/Constant5'

  real_T home_states_BLACK[3];         // Variable: home_states_BLACK
                                          //  Referenced by:
                                          //    '<S437>/Desired Attitude (BLACK)'
                                          //    '<S437>/Desired Px (BLACK)'
                                          //    '<S437>/Desired Py (BLACK)'
                                          //    '<S480>/Constant'
                                          //    '<S480>/Constant2'
                                          //    '<S480>/Constant3'

  real_T home_states_BLUE[3];          // Variable: home_states_BLUE
                                          //  Referenced by:
                                          //    '<S438>/Desired Attitude (BLUE)'
                                          //    '<S438>/Desired Px (BLUE)'
                                          //    '<S438>/Desired Py (BLUE)'
                                          //    '<S481>/Desired Attitude (BLUE)'
                                          //    '<S481>/Desired Px (BLUE)'
                                          //    '<S481>/Desired Py (BLUE)'

  real_T home_states_RED[3];           // Variable: home_states_RED
                                          //  Referenced by:
                                          //    '<S439>/Constant'
                                          //    '<S439>/Constant1'
                                          //    '<S439>/Constant3'
                                          //    '<S482>/Constant'
                                          //    '<S482>/Constant2'
                                          //    '<S482>/Constant3'

  real_T init_states_BLACK[3];         // Variable: init_states_BLACK
                                          //  Referenced by:
                                          //    '<S274>/Desired Attitude (BLACK)'
                                          //    '<S274>/Desired Px (BLACK)'
                                          //    '<S274>/Desired Py (BLACK)'
                                          //    '<S321>/Desired Attitude (BLACK)'
                                          //    '<S321>/Desired X-Position (BLACK)'
                                          //    '<S321>/Desired Y-Position (BLACK)'

  real_T init_states_BLUE[3];          // Variable: init_states_BLUE
                                          //  Referenced by:
                                          //    '<S275>/Desired Attitude (BLUE)'
                                          //    '<S275>/Desired Px (BLUE)'
                                          //    '<S275>/Desired Py (BLUE)'
                                          //    '<S374>/Desired Attitude (BLUE)'
                                          //    '<S374>/Desired Px (BLUE)'
                                          //    '<S374>/Desired Py (BLUE)'
                                          //    '<S377>/Desired Attitude (BLUE)'
                                          //    '<S377>/Desired Px (BLUE)'
                                          //    '<S377>/Desired Py (BLUE)'

  real_T init_states_RED[3];           // Variable: init_states_RED
                                          //  Referenced by:
                                          //    '<S276>/Constant'
                                          //    '<S276>/Constant1'
                                          //    '<S276>/Constant3'
                                          //    '<S405>/Desired Attitude (RED)'
                                          //    '<S405>/Desired X-Position (RED)'
                                          //    '<S405>/Desired Y-Position (RED)'
                                          //    '<S408>/Desired X-Position (RED)'
                                          //    '<S408>/Desired Y-Position (RED)'

  real_T ka;                           // Variable: ka
                                          //  Referenced by: '<S351>/Constant3'

  real_T kc;                           // Variable: kc
                                          //  Referenced by: '<S351>/Constant4'

  real_T kd;                           // Variable: kd
                                          //  Referenced by: '<S352>/Constant1'

  real_T model_param[6];               // Variable: model_param
                                          //  Referenced by:
                                          //    '<S527>/MATLAB Function'
                                          //    '<S528>/MATLAB Function'
                                          //    '<S532>/MATLAB Function'

  real_T noise_variance_BLACK;         // Variable: noise_variance_BLACK
                                          //  Referenced by: '<S527>/Random Number'

  real_T noise_variance_BLUE;          // Variable: noise_variance_BLUE
                                          //  Referenced by: '<S528>/Random Number'

  real_T noise_variance_RED;           // Variable: noise_variance_RED
                                          //  Referenced by: '<S532>/Random Number'

  real_T num_d[3];                     // Variable: num_d
                                          //  Referenced by:
                                          //    '<S339>/1Hz LP Filter'
                                          //    '<S342>/1Hz LP Filter'
                                          //    '<S342>/1Hz LP Filter1'
                                          //    '<S423>/1Hz LP Filter1'

  real_T o_hat_B[3];                   // Variable: o_hat_B
                                          //  Referenced by:
                                          //    '<S342>/Norm'
                                          //    '<S351>/Constant'

  real_T o_hat_prime[3];               // Variable: o_hat_prime
                                          //  Referenced by:
                                          //    '<S342>/Norm1'
                                          //    '<S353>/Constant2'

  real_T platformSelection;            // Variable: platformSelection
                                          //  Referenced by:
                                          //    '<S75>/Which PLATFORM is being used?'
                                          //    '<S80>/Stream PhaseSpace to Platform'

  real_T rT_I0[3];                     // Variable: rT_I0
                                          //  Referenced by: '<S408>/Constant'

  real_T serverRate;                   // Variable: serverRate
                                          //  Referenced by:
                                          //    '<S533>/Constant1'
                                          //    '<S534>/Constant1'
                                          //    '<S535>/Constant'
                                          //    '<S536>/divide by delta T'
                                          //    '<S537>/divide by delta T'
                                          //    '<S538>/divide by delta T'
                                          //    '<S539>/divide by delta T'
                                          //    '<S540>/divide by delta T'
                                          //    '<S541>/divide by delta T'
                                          //    '<S542>/divide by delta T'
                                          //    '<S543>/divide by delta T'
                                          //    '<S544>/divide by delta T'
                                          //    '<S545>/divide by delta T'
                                          //    '<S546>/divide by delta T'
                                          //    '<S547>/divide by delta T'
                                          //    '<S548>/divide by delta T'
                                          //    '<S549>/divide by delta T'
                                          //    '<S550>/divide by delta T'
                                          //    '<S80>/Stream PhaseSpace to Platform'
                                          //    '<S141>/divide by delta T'
                                          //    '<S142>/divide by delta T'
                                          //    '<S143>/divide by delta T'
                                          //    '<S144>/divide by delta T'
                                          //    '<S145>/divide by delta T'
                                          //    '<S146>/divide by delta T'
                                          //    '<S147>/divide by delta T'
                                          //    '<S148>/divide by delta T'
                                          //    '<S149>/divide by delta T'
                                          //    '<S150>/divide by delta T'
                                          //    '<S151>/divide by delta T'
                                          //    '<S152>/divide by delta T'
                                          //    '<S282>/divide by delta T'
                                          //    '<S284>/divide by delta T'
                                          //    '<S286>/divide by delta T'
                                          //    '<S295>/divide by delta T'
                                          //    '<S297>/divide by delta T'
                                          //    '<S299>/divide by delta T'
                                          //    '<S308>/divide by delta T'
                                          //    '<S310>/divide by delta T'
                                          //    '<S312>/divide by delta T'
                                          //    '<S445>/divide by delta T'
                                          //    '<S447>/divide by delta T'
                                          //    '<S449>/divide by delta T'
                                          //    '<S458>/divide by delta T'
                                          //    '<S460>/divide by delta T'
                                          //    '<S462>/divide by delta T'
                                          //    '<S471>/divide by delta T'
                                          //    '<S473>/divide by delta T'
                                          //    '<S475>/divide by delta T'
                                          //    '<S488>/divide by delta T'
                                          //    '<S490>/divide by delta T'
                                          //    '<S492>/divide by delta T'
                                          //    '<S501>/divide by delta T'
                                          //    '<S503>/divide by delta T'
                                          //    '<S505>/divide by delta T'
                                          //    '<S514>/divide by delta T'
                                          //    '<S516>/divide by delta T'
                                          //    '<S518>/divide by delta T'
                                          //    '<S81>/divide by delta T'
                                          //    '<S82>/divide by delta T'
                                          //    '<S83>/divide by delta T'
                                          //    '<S84>/divide by delta T'
                                          //    '<S85>/divide by delta T'
                                          //    '<S86>/divide by delta T'
                                          //    '<S87>/divide by delta T'
                                          //    '<S88>/divide by delta T'
                                          //    '<S89>/divide by delta T'
                                          //    '<S90>/divide by delta T'
                                          //    '<S91>/divide by delta T'
                                          //    '<S92>/divide by delta T'
                                          //    '<S109>/divide by delta T'
                                          //    '<S110>/divide by delta T'
                                          //    '<S111>/divide by delta T'
                                          //    '<S112>/divide by delta T'
                                          //    '<S113>/divide by delta T'
                                          //    '<S114>/divide by delta T'
                                          //    '<S115>/divide by delta T'
                                          //    '<S116>/divide by delta T'
                                          //    '<S117>/divide by delta T'
                                          //    '<S118>/divide by delta T'
                                          //    '<S119>/divide by delta T'
                                          //    '<S120>/divide by delta T'
                                          //    '<S330>/divide by delta T'
                                          //    '<S332>/divide by delta T'
                                          //    '<S334>/divide by delta T'
                                          //    '<S344>/divide by delta T'
                                          //    '<S383>/divide by delta T'
                                          //    '<S385>/divide by delta T'
                                          //    '<S387>/divide by delta T'
                                          //    '<S396>/divide by delta T'
                                          //    '<S398>/divide by delta T'
                                          //    '<S400>/divide by delta T'
                                          //    '<S414>/divide by delta T'
                                          //    '<S416>/divide by delta T'
                                          //    '<S418>/divide by delta T'
                                          //    '<S428>/divide by delta T'
                                          //    '<S430>/divide by delta T'
                                          //    '<S432>/divide by delta T'
                                          //    '<S363>/divide by delta T'

  real_T simMode;                      // Variable: simMode
                                          //  Referenced by:
                                          //    '<S7>/Constant'
                                          //    '<S19>/Constant'
                                          //    '<S25>/Constant'
                                          //    '<S57>/Constant'
                                          //    '<S74>/Constant'
                                          //    '<S172>/Constant'
                                          //    '<S269>/Constant'
                                          //    '<S273>/Constant'
                                          //    '<S277>/Constant'
                                          //    '<S320>/Constant'
                                          //    '<S440>/Constant'
                                          //    '<S483>/Constant'
                                          //    '<S526>/Constant'
                                          //    '<S78>/Constant'

  real_T theta_d;                      // Variable: theta_d
                                          //  Referenced by: '<S353>/Constant3'

  real_T thruster_dist2CG_BLACK[8];    // Variable: thruster_dist2CG_BLACK
                                          //  Referenced by:
                                          //    '<S27>/MATLAB Function'
                                          //    '<S27>/MATLAB Function1'

  real_T thruster_dist2CG_BLUE[8];     // Variable: thruster_dist2CG_BLUE
                                          //  Referenced by:
                                          //    '<S36>/MATLAB Function'
                                          //    '<S36>/MATLAB Function1'

  real_T thruster_dist2CG_RED[8];      // Variable: thruster_dist2CG_RED
                                          //  Referenced by:
                                          //    '<S45>/MATLAB Function'
                                          //    '<S45>/MATLAB Function1'

  real_T v_max;                        // Variable: v_max
                                          //  Referenced by: '<S353>/Constant'

  real_T w_body;                       // Variable: w_body
                                          //  Referenced by: '<S408>/Gain'

  real_T BLACKPWM_Y0;                  // Computed Parameter: BLACKPWM_Y0
                                          //  Referenced by: '<S22>/BLACK PWM'

  real_T RemoveNegatives_UpperSat;     // Expression: 1000
                                          //  Referenced by: '<S27>/Remove Negatives'

  real_T RemoveNegatives_LowerSat;     // Expression: 0
                                          //  Referenced by: '<S27>/Remove Negatives'

  real_T BLUEPWM_Y0;                   // Computed Parameter: BLUEPWM_Y0
                                          //  Referenced by: '<S23>/BLUE PWM'

  real_T RemoveNegatives_UpperSat_p;   // Expression: 1000
                                          //  Referenced by: '<S36>/Remove Negatives'

  real_T RemoveNegatives_LowerSat_c;   // Expression: 0
                                          //  Referenced by: '<S36>/Remove Negatives'

  real_T REDPWM_Y0;                    // Computed Parameter: REDPWM_Y0
                                          //  Referenced by: '<S24>/RED PWM'

  real_T RemoveNegatives_UpperSat_d;   // Expression: 1000
                                          //  Referenced by: '<S45>/Remove Negatives'

  real_T RemoveNegatives_LowerSat_a;   // Expression: 0
                                          //  Referenced by: '<S45>/Remove Negatives'

  real_T DigitalRead_SampleTime;       // Expression: sampleTime
                                          //  Referenced by: '<S70>/Digital Read'

  real_T SaturateTorque_UpperSat;      // Expression: 0.1
                                          //  Referenced by: '<S58>/Saturate Torque'

  real_T SaturateTorque_LowerSat;      // Expression: -0.1
                                          //  Referenced by: '<S58>/Saturate Torque'

  real_T Delay_InitialCondition;       // Expression: 0.0
                                          //  Referenced by: '<S58>/Delay'

  real_T GearboxRatio_Value;           // Expression: 3375/64
                                          //  Referenced by: '<S62>/Gearbox  Ratio'

  real_T SaturateMotor_UpperSat;       // Expression: 7000
                                          //  Referenced by: '<S59>/Saturate Motor'

  real_T SaturateMotor_LowerSat;       // Expression: -7000
                                          //  Referenced by: '<S59>/Saturate Motor'

  real_T SaturateRPM_UpperSat;         // Expression: (7000*64/3375)
                                          //  Referenced by: '<S58>/Saturate RPM'

  real_T SaturateRPM_LowerSat;         // Expression: -(7000*64/3375)
                                          //  Referenced by: '<S58>/Saturate RPM'

  real_T Out1_Y0;                      // Computed Parameter: Out1_Y0
                                          //  Referenced by: '<S154>/Out1'

  real_T Out1_Y0_b;                    // Computed Parameter: Out1_Y0_b
                                          //  Referenced by: '<S139>/Out1'

  real_T ActualTime_Y0;                // Computed Parameter: ActualTime_Y0
                                          //  Referenced by: '<S140>/Actual Time'

  real_T Out1_Y0_o;                    // Computed Parameter: Out1_Y0_o
                                          //  Referenced by: '<S157>/Out1'

  real_T Out1_Y0_k;                    // Computed Parameter: Out1_Y0_k
                                          //  Referenced by: '<S158>/Out1'

  real_T Out1_Y0_m;                    // Computed Parameter: Out1_Y0_m
                                          //  Referenced by: '<S159>/Out1'

  real_T Out1_Y0_j;                    // Computed Parameter: Out1_Y0_j
                                          //  Referenced by: '<S160>/Out1'

  real_T Out1_Y0_f;                    // Computed Parameter: Out1_Y0_f
                                          //  Referenced by: '<S161>/Out1'

  real_T Out1_Y0_p;                    // Computed Parameter: Out1_Y0_p
                                          //  Referenced by: '<S162>/Out1'

  real_T Out1_Y0_i;                    // Computed Parameter: Out1_Y0_i
                                          //  Referenced by: '<S163>/Out1'

  real_T Out1_Y0_l;                    // Computed Parameter: Out1_Y0_l
                                          //  Referenced by: '<S164>/Out1'

  real_T Out1_Y0_mc;                   // Computed Parameter: Out1_Y0_mc
                                          //  Referenced by: '<S165>/Out1'

  real_T Out1_Y0_ie;                   // Computed Parameter: Out1_Y0_ie
                                          //  Referenced by: '<S166>/Out1'

  real_T Out1_Y0_a;                    // Computed Parameter: Out1_Y0_a
                                          //  Referenced by: '<S167>/Out1'

  real_T Out1_Y0_ief;                  // Computed Parameter: Out1_Y0_ief
                                          //  Referenced by: '<S168>/Out1'

  real_T Constant_Value;               // Expression: 0
                                          //  Referenced by: '<S137>/Constant'

  real_T Out1_Y0_k4;                   // Computed Parameter: Out1_Y0_k4
                                          //  Referenced by: '<S93>/Out1'

  real_T Out1_Y0_c;                    // Computed Parameter: Out1_Y0_c
                                          //  Referenced by: '<S94>/Out1'

  real_T Out1_Y0_l0;                   // Computed Parameter: Out1_Y0_l0
                                          //  Referenced by: '<S95>/Out1'

  real_T Out1_Y0_e;                    // Computed Parameter: Out1_Y0_e
                                          //  Referenced by: '<S96>/Out1'

  real_T Out1_Y0_d;                    // Computed Parameter: Out1_Y0_d
                                          //  Referenced by: '<S97>/Out1'

  real_T Out1_Y0_g;                    // Computed Parameter: Out1_Y0_g
                                          //  Referenced by: '<S98>/Out1'

  real_T Out1_Y0_ft;                   // Computed Parameter: Out1_Y0_ft
                                          //  Referenced by: '<S99>/Out1'

  real_T Out1_Y0_j1;                   // Computed Parameter: Out1_Y0_j1
                                          //  Referenced by: '<S100>/Out1'

  real_T Out1_Y0_ii;                   // Computed Parameter: Out1_Y0_ii
                                          //  Referenced by: '<S101>/Out1'

  real_T Out1_Y0_i1;                   // Computed Parameter: Out1_Y0_i1
                                          //  Referenced by: '<S102>/Out1'

  real_T Out1_Y0_pa;                   // Computed Parameter: Out1_Y0_pa
                                          //  Referenced by: '<S103>/Out1'

  real_T Out1_Y0_h;                    // Computed Parameter: Out1_Y0_h
                                          //  Referenced by: '<S104>/Out1'

  real_T Out1_Y0_ee;                   // Computed Parameter: Out1_Y0_ee
                                          //  Referenced by: '<S122>/Out1'

  real_T Out1_Y0_ob;                   // Computed Parameter: Out1_Y0_ob
                                          //  Referenced by: '<S107>/Out1'

  real_T ActualTime_Y0_f;              // Computed Parameter: ActualTime_Y0_f
                                          //  Referenced by: '<S108>/Actual Time'

  real_T Out1_Y0_oy;                   // Computed Parameter: Out1_Y0_oy
                                          //  Referenced by: '<S125>/Out1'

  real_T Out1_Y0_l2;                   // Computed Parameter: Out1_Y0_l2
                                          //  Referenced by: '<S126>/Out1'

  real_T Out1_Y0_ow;                   // Computed Parameter: Out1_Y0_ow
                                          //  Referenced by: '<S127>/Out1'

  real_T Out1_Y0_mx;                   // Computed Parameter: Out1_Y0_mx
                                          //  Referenced by: '<S128>/Out1'

  real_T Out1_Y0_gm;                   // Computed Parameter: Out1_Y0_gm
                                          //  Referenced by: '<S129>/Out1'

  real_T Out1_Y0_h4;                   // Computed Parameter: Out1_Y0_h4
                                          //  Referenced by: '<S130>/Out1'

  real_T Out1_Y0_kh;                   // Computed Parameter: Out1_Y0_kh
                                          //  Referenced by: '<S131>/Out1'

  real_T Out1_Y0_fm;                   // Computed Parameter: Out1_Y0_fm
                                          //  Referenced by: '<S132>/Out1'

  real_T Out1_Y0_er;                   // Computed Parameter: Out1_Y0_er
                                          //  Referenced by: '<S133>/Out1'

  real_T Out1_Y0_il;                   // Computed Parameter: Out1_Y0_il
                                          //  Referenced by: '<S134>/Out1'

  real_T Out1_Y0_mr;                   // Computed Parameter: Out1_Y0_mr
                                          //  Referenced by: '<S135>/Out1'

  real_T Out1_Y0_jf;                   // Computed Parameter: Out1_Y0_jf
                                          //  Referenced by: '<S136>/Out1'

  real_T Constant_Value_f;             // Expression: 0
                                          //  Referenced by: '<S105>/Constant'

  real_T Out1_Y0_n;                    // Computed Parameter: Out1_Y0_n
                                          //  Referenced by: '<S183>/Out1'

  real_T Out1_Y0_kc;                   // Computed Parameter: Out1_Y0_kc
                                          //  Referenced by: '<S186>/Out1'

  real_T Out1_Y0_n3;                   // Computed Parameter: Out1_Y0_n3
                                          //  Referenced by: '<S190>/Out1'

  real_T Out1_Y0_hs;                   // Computed Parameter: Out1_Y0_hs
                                          //  Referenced by: '<S196>/Out1'

  real_T Out1_Y0_gz;                   // Computed Parameter: Out1_Y0_gz
                                          //  Referenced by: '<S199>/Out1'

  real_T Out1_Y0_cg;                   // Computed Parameter: Out1_Y0_cg
                                          //  Referenced by: '<S202>/Out1'

  real_T Constant_Value_i;             // Expression: 0
                                          //  Referenced by: '<S189>/Constant'

  real_T Gain_Gain;                    // Expression: pi/180
                                          //  Referenced by: '<S169>/Gain'

  real_T Gain1_Gain;                   // Expression: 9.81
                                          //  Referenced by: '<S169>/Gain1'

  real_T DiscreteTimeIntegrator1_gainval;
                          // Computed Parameter: DiscreteTimeIntegrator1_gainval
                             //  Referenced by: '<S181>/Discrete-Time Integrator1'

  real_T Out1_Y0_o0;                   // Computed Parameter: Out1_Y0_o0
                                          //  Referenced by: '<S214>/Out1'

  real_T Out1_Y0_jb;                   // Computed Parameter: Out1_Y0_jb
                                          //  Referenced by: '<S217>/Out1'

  real_T Out1_Y0_dr;                   // Computed Parameter: Out1_Y0_dr
                                          //  Referenced by: '<S221>/Out1'

  real_T Out1_Y0_kcv;                  // Computed Parameter: Out1_Y0_kcv
                                          //  Referenced by: '<S227>/Out1'

  real_T Out1_Y0_ie0;                  // Computed Parameter: Out1_Y0_ie0
                                          //  Referenced by: '<S230>/Out1'

  real_T Out1_Y0_gi;                   // Computed Parameter: Out1_Y0_gi
                                          //  Referenced by: '<S233>/Out1'

  real_T Constant_Value_m;             // Expression: 0
                                          //  Referenced by: '<S220>/Constant'

  real_T Gain_Gain_k;                  // Expression: pi/180
                                          //  Referenced by: '<S170>/Gain'

  real_T Gain1_Gain_c;                 // Expression: 9.81
                                          //  Referenced by: '<S170>/Gain1'

  real_T DiscreteTimeIntegrator1_gainv_p;
                          // Computed Parameter: DiscreteTimeIntegrator1_gainv_p
                             //  Referenced by: '<S212>/Discrete-Time Integrator1'

  real_T Out1_Y0_az;                   // Computed Parameter: Out1_Y0_az
                                          //  Referenced by: '<S245>/Out1'

  real_T Out1_Y0_cc;                   // Computed Parameter: Out1_Y0_cc
                                          //  Referenced by: '<S248>/Out1'

  real_T Out1_Y0_m0;                   // Computed Parameter: Out1_Y0_m0
                                          //  Referenced by: '<S252>/Out1'

  real_T Out1_Y0_cz;                   // Computed Parameter: Out1_Y0_cz
                                          //  Referenced by: '<S258>/Out1'

  real_T Out1_Y0_ai;                   // Computed Parameter: Out1_Y0_ai
                                          //  Referenced by: '<S261>/Out1'

  real_T Out1_Y0_ef;                   // Computed Parameter: Out1_Y0_ef
                                          //  Referenced by: '<S264>/Out1'

  real_T Constant_Value_fd;            // Expression: 0
                                          //  Referenced by: '<S251>/Constant'

  real_T Gain_Gain_f;                  // Expression: pi/180
                                          //  Referenced by: '<S171>/Gain'

  real_T Gain1_Gain_b;                 // Expression: 9.81
                                          //  Referenced by: '<S171>/Gain1'

  real_T DiscreteTimeIntegrator1_gainv_g;
                          // Computed Parameter: DiscreteTimeIntegrator1_gainv_g
                             //  Referenced by: '<S243>/Discrete-Time Integrator1'

  real_T _Y0[3];                       // Expression: initCond
                                          //  Referenced by: '<S555>/ '

  real_T _Y0_m[3];                     // Expression: initCond
                                          //  Referenced by: '<S557>/ '

  real_T _Y0_n[3];                     // Expression: initCond
                                          //  Referenced by: '<S559>/ '

  real_T Out1_Y0_eq;                   // Computed Parameter: Out1_Y0_eq
                                          //  Referenced by: '<S560>/Out1'

  real_T Out1_Y0_ir;                   // Computed Parameter: Out1_Y0_ir
                                          //  Referenced by: '<S561>/Out1'

  real_T Out1_Y0_ny;                   // Computed Parameter: Out1_Y0_ny
                                          //  Referenced by: '<S562>/Out1'

  real_T Out1_Y0_ko;                   // Computed Parameter: Out1_Y0_ko
                                          //  Referenced by: '<S563>/Out1'

  real_T Out1_Y0_ar;                   // Computed Parameter: Out1_Y0_ar
                                          //  Referenced by: '<S564>/Out1'

  real_T Out1_Y0_lp;                   // Computed Parameter: Out1_Y0_lp
                                          //  Referenced by: '<S565>/Out1'

  real_T Out1_Y0_l2n;                  // Computed Parameter: Out1_Y0_l2n
                                          //  Referenced by: '<S566>/Out1'

  real_T Out1_Y0_ep;                   // Computed Parameter: Out1_Y0_ep
                                          //  Referenced by: '<S567>/Out1'

  real_T Out1_Y0_ln;                   // Computed Parameter: Out1_Y0_ln
                                          //  Referenced by: '<S568>/Out1'

  real_T Out1_Y0_lj;                   // Computed Parameter: Out1_Y0_lj
                                          //  Referenced by: '<S569>/Out1'

  real_T Out1_Y0_h2;                   // Computed Parameter: Out1_Y0_h2
                                          //  Referenced by: '<S570>/Out1'

  real_T Out1_Y0_i3;                   // Computed Parameter: Out1_Y0_i3
                                          //  Referenced by: '<S571>/Out1'

  real_T Out1_Y0_mh;                   // Computed Parameter: Out1_Y0_mh
                                          //  Referenced by: '<S572>/Out1'

  real_T Out1_Y0_cb;                   // Computed Parameter: Out1_Y0_cb
                                          //  Referenced by: '<S573>/Out1'

  real_T Out1_Y0_d1;                   // Computed Parameter: Out1_Y0_d1
                                          //  Referenced by: '<S574>/Out1'

  real_T Constant_Value_h;             // Expression: 0
                                          //  Referenced by: '<S529>/Constant'

  real_T Constant_Value_j;             // Expression: 0
                                          //  Referenced by: '<S530>/Constant'

  real_T Constant_Value_c;             // Expression: 0
                                          //  Referenced by: '<S531>/Constant'

  real_T AccelerationtoVelocity_gainval;
                           // Computed Parameter: AccelerationtoVelocity_gainval
                              //  Referenced by: '<S527>/Acceleration  to Velocity'

  real_T AccelerationtoVelocity_IC;    // Expression: 0
                                          //  Referenced by: '<S527>/Acceleration  to Velocity'

  real_T VelocitytoPosition_gainval;
                               // Computed Parameter: VelocitytoPosition_gainval
                                  //  Referenced by: '<S527>/Velocity to Position'

  real_T RandomNumber_Mean;            // Expression: 0
                                          //  Referenced by: '<S527>/Random Number'

  real_T RandomNumber_Seed;            // Expression: 0
                                          //  Referenced by: '<S527>/Random Number'

  real_T RandomNumber1_Mean;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number1'

  real_T RandomNumber1_StdDev;       // Computed Parameter: RandomNumber1_StdDev
                                        //  Referenced by: '<S16>/Random Number1'

  real_T RandomNumber1_Seed;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number1'

  real_T RandomNumber_Mean_o;          // Expression: 0
                                          //  Referenced by: '<S16>/Random Number'

  real_T RandomNumber_StdDev;         // Computed Parameter: RandomNumber_StdDev
                                         //  Referenced by: '<S16>/Random Number'

  real_T RandomNumber_Seed_f;          // Expression: 0
                                          //  Referenced by: '<S16>/Random Number'

  real_T RandomNumber2_Mean;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number2'

  real_T RandomNumber2_StdDev;       // Computed Parameter: RandomNumber2_StdDev
                                        //  Referenced by: '<S16>/Random Number2'

  real_T RandomNumber2_Seed;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number2'

  real_T AccelerationtoVelocity_gainva_a;
                          // Computed Parameter: AccelerationtoVelocity_gainva_a
                             //  Referenced by: '<S528>/Acceleration  to Velocity'

  real_T AccelerationtoVelocity_IC_n;  // Expression: 0
                                          //  Referenced by: '<S528>/Acceleration  to Velocity'

  real_T VelocitytoPosition_gainval_a;
                             // Computed Parameter: VelocitytoPosition_gainval_a
                                //  Referenced by: '<S528>/Velocity to Position'

  real_T RandomNumber_Mean_n;          // Expression: 0
                                          //  Referenced by: '<S528>/Random Number'

  real_T RandomNumber_Seed_g;          // Expression: 0
                                          //  Referenced by: '<S528>/Random Number'

  real_T RandomNumber4_Mean;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number4'

  real_T RandomNumber4_StdDev;       // Computed Parameter: RandomNumber4_StdDev
                                        //  Referenced by: '<S16>/Random Number4'

  real_T RandomNumber4_Seed;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number4'

  real_T RandomNumber3_Mean;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number3'

  real_T RandomNumber3_StdDev;       // Computed Parameter: RandomNumber3_StdDev
                                        //  Referenced by: '<S16>/Random Number3'

  real_T RandomNumber3_Seed;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number3'

  real_T RandomNumber5_Mean;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number5'

  real_T RandomNumber5_StdDev;       // Computed Parameter: RandomNumber5_StdDev
                                        //  Referenced by: '<S16>/Random Number5'

  real_T RandomNumber5_Seed;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number5'

  real_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<S16>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator1_gainv_f;
                          // Computed Parameter: DiscreteTimeIntegrator1_gainv_f
                             //  Referenced by: '<S16>/Discrete-Time Integrator1'

  real_T DiscreteTimeIntegrator2_gainval;
                          // Computed Parameter: DiscreteTimeIntegrator2_gainval
                             //  Referenced by: '<S16>/Discrete-Time Integrator2'

  real_T AccelerationtoVelocity_gainva_i;
                          // Computed Parameter: AccelerationtoVelocity_gainva_i
                             //  Referenced by: '<S532>/Acceleration  to Velocity'

  real_T AccelerationtoVelocity_IC_a;  // Expression: 0
                                          //  Referenced by: '<S532>/Acceleration  to Velocity'

  real_T VelocitytoPosition_gainval_ad;
                            // Computed Parameter: VelocitytoPosition_gainval_ad
                               //  Referenced by: '<S532>/Velocity to Position'

  real_T RandomNumber_Mean_c;          // Expression: 0
                                          //  Referenced by: '<S532>/Random Number'

  real_T RandomNumber_Seed_p;          // Expression: 0
                                          //  Referenced by: '<S532>/Random Number'

  real_T RandomNumber7_Mean;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number7'

  real_T RandomNumber7_StdDev;       // Computed Parameter: RandomNumber7_StdDev
                                        //  Referenced by: '<S16>/Random Number7'

  real_T RandomNumber7_Seed;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number7'

  real_T RandomNumber6_Mean;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number6'

  real_T RandomNumber6_StdDev;       // Computed Parameter: RandomNumber6_StdDev
                                        //  Referenced by: '<S16>/Random Number6'

  real_T RandomNumber6_Seed;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number6'

  real_T RandomNumber8_Mean;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number8'

  real_T RandomNumber8_StdDev;       // Computed Parameter: RandomNumber8_StdDev
                                        //  Referenced by: '<S16>/Random Number8'

  real_T RandomNumber8_Seed;           // Expression: 0
                                          //  Referenced by: '<S16>/Random Number8'

  real_T Out1_Y0_mp;                   // Computed Parameter: Out1_Y0_mp
                                          //  Referenced by: '<S283>/Out1'

  real_T Out1_Y0_av;                   // Computed Parameter: Out1_Y0_av
                                          //  Referenced by: '<S285>/Out1'

  real_T Out1_Y0_g1;                   // Computed Parameter: Out1_Y0_g1
                                          //  Referenced by: '<S287>/Out1'

  real_T PuckState_Value;              // Expression: 1
                                          //  Referenced by: '<S274>/Puck State'

  real_T Out1_Y0_ov;                   // Computed Parameter: Out1_Y0_ov
                                          //  Referenced by: '<S296>/Out1'

  real_T Out1_Y0_a4;                   // Computed Parameter: Out1_Y0_a4
                                          //  Referenced by: '<S298>/Out1'

  real_T Out1_Y0_nv;                   // Computed Parameter: Out1_Y0_nv
                                          //  Referenced by: '<S300>/Out1'

  real_T PuckState_Value_o;            // Expression: 1
                                          //  Referenced by: '<S275>/Puck State'

  real_T Out1_Y0_fr;                   // Computed Parameter: Out1_Y0_fr
                                          //  Referenced by: '<S309>/Out1'

  real_T Out1_Y0_bk;                   // Computed Parameter: Out1_Y0_bk
                                          //  Referenced by: '<S311>/Out1'

  real_T Out1_Y0_j2;                   // Computed Parameter: Out1_Y0_j2
                                          //  Referenced by: '<S313>/Out1'

  real_T Constant2_Value;              // Expression: 0
                                          //  Referenced by: '<S276>/Constant2'

  real_T Constant4_Value;              // Expression: 0
                                          //  Referenced by: '<S276>/Constant4'

  real_T Constant5_Value;              // Expression: 0
                                          //  Referenced by: '<S276>/Constant5'

  real_T Constant6_Value;              // Expression: 0
                                          //  Referenced by: '<S276>/Constant6'

  real_T PuckState_Value_l;            // Expression: 1
                                          //  Referenced by: '<S276>/Puck State'

  real_T Out1_Y0_gc;                   // Computed Parameter: Out1_Y0_gc
                                          //  Referenced by: '<S331>/Out1'

  real_T Out1_Y0_hu;                   // Computed Parameter: Out1_Y0_hu
                                          //  Referenced by: '<S333>/Out1'

  real_T Out1_Y0_efk;                  // Computed Parameter: Out1_Y0_efk
                                          //  Referenced by: '<S335>/Out1'

  real_T PuckState_Value_g;            // Expression: 1
                                          //  Referenced by: '<S321>/Puck State'

  real_T Out1_Y0_bx;                   // Computed Parameter: Out1_Y0_bx
                                          //  Referenced by: '<S345>/Out1'

  real_T _Y0_e;                        // Expression: initCond
                                          //  Referenced by: '<S358>/ '

  real_T Out1_Y0_fl;                   // Computed Parameter: Out1_Y0_fl
                                          //  Referenced by: '<S367>/Out1'

  real_T Out1_Y0_j3;                   // Computed Parameter: Out1_Y0_j3
                                          //  Referenced by: '<S368>/Out1'

  real_T Out1_Y0_px;                   // Computed Parameter: Out1_Y0_px
                                          //  Referenced by: '<S369>/Out1'

  real_T Out1_Y0_ni;                   // Computed Parameter: Out1_Y0_ni
                                          //  Referenced by: '<S370>/Out1'

  real_T Constant2_Value_j;            // Expression: 0
                                          //  Referenced by: '<S342>/Constant2'

  real_T Constant3_Value;              // Expression: 0
                                          //  Referenced by: '<S342>/Constant3'

  real_T Delay_InitialCondition_h;     // Expression: 0
                                          //  Referenced by: '<S354>/Delay'

  real_T uHzLPFilter_InitialStates;    // Expression: 0
                                          //  Referenced by: '<S339>/1Hz LP Filter'

  real_T Constant1_Value;              // Expression: 0
                                          //  Referenced by: '<S342>/Constant1'

  real_T uHzLPFilter1_InitialStates;   // Expression: 0
                                          //  Referenced by: '<S342>/1Hz LP Filter1'

  real_T uHzLPFilter_InitialStates_k;  // Expression: 0
                                          //  Referenced by: '<S342>/1Hz LP Filter'

  real_T Constant_Value_e[3];          // Expression: [0;0;0]
                                          //  Referenced by: '<S349>/Constant'

  real_T Constant1_Value_e;            // Expression: 0
                                          //  Referenced by: '<S350>/Constant1'

  real_T Constant_Value_d;             // Expression: 0
                                          //  Referenced by: '<S350>/Constant'

  real_T Constant_Value_hr[3];         // Expression: [0;0;0]
                                          //  Referenced by: '<S341>/Constant'

  real_T PuckState_Value_gw;           // Expression: 1
                                          //  Referenced by: '<S324>/Puck State'

  real_T Out1_Y0_na;                   // Computed Parameter: Out1_Y0_na
                                          //  Referenced by: '<S415>/Out1'

  real_T Out1_Y0_mn;                   // Computed Parameter: Out1_Y0_mn
                                          //  Referenced by: '<S417>/Out1'

  real_T Out1_Y0_lx;                   // Computed Parameter: Out1_Y0_lx
                                          //  Referenced by: '<S419>/Out1'

  real_T Constant1_Value_a;            // Expression: 0
                                          //  Referenced by: '<S405>/Constant1'

  real_T Constant4_Value_b;            // Expression: 0
                                          //  Referenced by: '<S405>/Constant4'

  real_T Constant5_Value_c;            // Expression: 0
                                          //  Referenced by: '<S405>/Constant5'

  real_T Constant6_Value_b;            // Expression: 0
                                          //  Referenced by: '<S405>/Constant6'

  real_T PuckState_Value_lb;           // Expression: 1
                                          //  Referenced by: '<S405>/Puck State'

  real_T Out1_Y0_bj;                   // Computed Parameter: Out1_Y0_bj
                                          //  Referenced by: '<S429>/Out1'

  real_T Out1_Y0_ki;                   // Computed Parameter: Out1_Y0_ki
                                          //  Referenced by: '<S431>/Out1'

  real_T Out1_Y0_ky;                   // Computed Parameter: Out1_Y0_ky
                                          //  Referenced by: '<S433>/Out1'

  real_T uHzLPFilter1_InitialStates_o; // Expression: 0
                                          //  Referenced by: '<S423>/1Hz LP Filter1'

  real_T Constant1_Value_p;            // Expression: 0
                                          //  Referenced by: '<S408>/Constant1'

  real_T Constant4_Value_j;            // Expression: 0
                                          //  Referenced by: '<S408>/Constant4'

  real_T Constant5_Value_i;            // Expression: 0
                                          //  Referenced by: '<S408>/Constant5'

  real_T Constant6_Value_p;            // Expression: 0
                                          //  Referenced by: '<S408>/Constant6'

  real_T PuckState_Value_h;            // Expression: 1
                                          //  Referenced by: '<S408>/Puck State'

  real_T Out1_Y0_jr;                   // Computed Parameter: Out1_Y0_jr
                                          //  Referenced by: '<S446>/Out1'

  real_T Out1_Y0_mrw;                  // Computed Parameter: Out1_Y0_mrw
                                          //  Referenced by: '<S448>/Out1'

  real_T Out1_Y0_i0;                   // Computed Parameter: Out1_Y0_i0
                                          //  Referenced by: '<S450>/Out1'

  real_T PuckState_Value_f;            // Expression: 1
                                          //  Referenced by: '<S437>/Puck State'

  real_T Out1_Y0_nf;                   // Computed Parameter: Out1_Y0_nf
                                          //  Referenced by: '<S472>/Out1'

  real_T Out1_Y0_gb;                   // Computed Parameter: Out1_Y0_gb
                                          //  Referenced by: '<S474>/Out1'

  real_T Out1_Y0_da;                   // Computed Parameter: Out1_Y0_da
                                          //  Referenced by: '<S476>/Out1'

  real_T Constant2_Value_g;            // Expression: 0
                                          //  Referenced by: '<S439>/Constant2'

  real_T Constant4_Value_h;            // Expression: 0
                                          //  Referenced by: '<S439>/Constant4'

  real_T Constant5_Value_h;            // Expression: 0
                                          //  Referenced by: '<S439>/Constant5'

  real_T Constant6_Value_pg;           // Expression: 0
                                          //  Referenced by: '<S439>/Constant6'

  real_T PuckState_Value_od;           // Expression: 1
                                          //  Referenced by: '<S439>/Puck State'

  real_T Out1_Y0_gx;                   // Computed Parameter: Out1_Y0_gx
                                          //  Referenced by: '<S489>/Out1'

  real_T Out1_Y0_fj;                   // Computed Parameter: Out1_Y0_fj
                                          //  Referenced by: '<S491>/Out1'

  real_T Out1_Y0_bm;                   // Computed Parameter: Out1_Y0_bm
                                          //  Referenced by: '<S493>/Out1'

  real_T PuckState_Value_ge;           // Expression: 1
                                          //  Referenced by: '<S480>/Puck State'

  real_T Out1_Y0_i3j;                  // Computed Parameter: Out1_Y0_i3j
                                          //  Referenced by: '<S515>/Out1'

  real_T Out1_Y0_bs;                   // Computed Parameter: Out1_Y0_bs
                                          //  Referenced by: '<S517>/Out1'

  real_T Out1_Y0_c5;                   // Computed Parameter: Out1_Y0_c5
                                          //  Referenced by: '<S519>/Out1'

  real_T Constant1_Value_k;            // Expression: 0
                                          //  Referenced by: '<S482>/Constant1'

  real_T Constant4_Value_p;            // Expression: 0
                                          //  Referenced by: '<S482>/Constant4'

  real_T Constant5_Value_o;            // Expression: 0
                                          //  Referenced by: '<S482>/Constant5'

  real_T Constant6_Value_a;            // Expression: 0
                                          //  Referenced by: '<S482>/Constant6'

  real_T PuckState_Value_e;            // Expression: 1
                                          //  Referenced by: '<S482>/Puck State'

  real_T Constant3_Value_d;            // Expression: 0
                                          //  Referenced by: '<S523>/Constant3'

  real_T Constant4_Value_e;            // Expression: 0
                                          //  Referenced by: '<S523>/Constant4'

  real_T Constant5_Value_n;            // Expression: 0
                                          //  Referenced by: '<S523>/Constant5'

  real_T PuckState_Value_gd;           // Expression: 0
                                          //  Referenced by: '<S523>/Puck State'

  real_T Constant_Value_jq;            // Expression: 0
                                          //  Referenced by: '<S525>/Constant'

  real_T Constant1_Value_l;            // Expression: 0
                                          //  Referenced by: '<S525>/Constant1'

  real_T Constant2_Value_m;            // Expression: 0
                                          //  Referenced by: '<S525>/Constant2'

  real_T Constant4_Value_o;            // Expression: 0
                                          //  Referenced by: '<S525>/Constant4'

  real_T Constant5_Value_f;            // Expression: 0
                                          //  Referenced by: '<S525>/Constant5'

  real_T Constant6_Value_j;            // Expression: 0
                                          //  Referenced by: '<S525>/Constant6'

  real_T PuckState_Value_hv;           // Expression: 0
                                          //  Referenced by: '<S525>/Puck State'

  real_T Gain_Gain_a;                  // Expression: -1
                                          //  Referenced by: '<S525>/Gain'

  real_T Saturation_UpperSat;          // Expression: 0.005
                                          //  Referenced by: '<S525>/Saturation'

  real_T Saturation_LowerSat;          // Expression: -0.005
                                          //  Referenced by: '<S525>/Saturation'

  real_T RED_Px33_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px33'

  real_T RED_Px36_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px36'

  real_T RED_Px39_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px39'

  real_T RED_Tz10_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz10'

  real_T RED_Tz11_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz11'

  real_T RED_Tz4_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz4'

  real_T RED_Tz5_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz5'

  real_T RED_Tz6_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz6'

  real_T RED_Tz9_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz9'

  real_T BLACK_Fx_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fx'

  real_T BLACK_Fx1_InitialValue;       // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fx1'

  real_T BLACK_Fx_Sat_InitialValue;    // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fx_Sat'

  real_T BLACK_Fx_Sat1_InitialValue;   // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fx_Sat1'

  real_T BLACK_Fx_Sat2_InitialValue;   // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fx_Sat2'

  real_T BLACK_Fy_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fy'

  real_T BLACK_Fy1_InitialValue;       // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fy1'

  real_T BLACK_Fy_Sat_InitialValue;    // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fy_Sat'

  real_T BLACK_Fy_Sat1_InitialValue;   // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fy_Sat1'

  real_T BLACK_Fy_Sat2_InitialValue;   // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Fy_Sat2'

  real_T BLACK_Px_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Px'

  real_T BLACK_Px1_InitialValue;       // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Px1'

  real_T BLACK_Py_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Py'

  real_T BLACK_Py1_InitialValue;       // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Py1'

  real_T BLACK_Rz_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Rz'

  real_T BLACK_Rz1_InitialValue;       // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Rz1'

  real_T BLACK_Tz_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Tz'

  real_T BLACK_Tz1_InitialValue;       // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Tz1'

  real_T BLACK_Tz_Sat_InitialValue;    // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Tz_Sat'

  real_T BLACK_Tz_Sat1_InitialValue;   // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Tz_Sat1'

  real_T BLACK_Tz_Sat2_InitialValue;   // Expression: 0
                                          //  Referenced by: '<Root>/BLACK_Tz_Sat2'

  real_T DataStoreMemory_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Data Store Memory'

  real_T DataStoreMemory1_InitialValue;// Expression: 0
                                          //  Referenced by: '<Root>/Data Store Memory1'

  real_T RED_Fx_InitialValue;          // Expression: 0
                                          //  Referenced by: '<Root>/RED_Fx'

  real_T RED_Fx_Sat_InitialValue;      // Expression: 0
                                          //  Referenced by: '<Root>/RED_Fx_Sat'

  real_T RED_Fy_InitialValue;          // Expression: 0
                                          //  Referenced by: '<Root>/RED_Fy'

  real_T RED_Fy_Sat_InitialValue;      // Expression: 0
                                          //  Referenced by: '<Root>/RED_Fy_Sat'

  real_T RED_Px_InitialValue;          // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px'

  real_T RED_Px1_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px1'

  real_T RED_Px10_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px10'

  real_T RED_Px11_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px11'

  real_T RED_Px12_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px12'

  real_T RED_Px13_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px13'

  real_T RED_Px14_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px14'

  real_T RED_Px15_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px15'

  real_T RED_Px16_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px16'

  real_T RED_Px17_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px17'

  real_T RED_Px18_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px18'

  real_T RED_Px19_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px19'

  real_T RED_Px2_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px2'

  real_T RED_Px20_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px20'

  real_T RED_Px21_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px21'

  real_T RED_Px22_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px22'

  real_T RED_Px23_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px23'

  real_T RED_Px24_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px24'

  real_T RED_Px25_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px25'

  real_T RED_Px26_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px26'

  real_T RED_Px27_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px27'

  real_T RED_Px28_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px28'

  real_T RED_Px29_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px29'

  real_T RED_Px3_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px3'

  real_T RED_Px30_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px30'

  real_T RED_Px31_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px31'

  real_T RED_Px32_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px32'

  real_T RED_Px34_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px34'

  real_T RED_Px35_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px35'

  real_T RED_Px37_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px37'

  real_T RED_Px38_InitialValue;        // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px38'

  real_T RED_Px4_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px4'

  real_T RED_Px5_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px5'

  real_T RED_Px6_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px6'

  real_T RED_Px7_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px7'

  real_T RED_Px8_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px8'

  real_T RED_Px9_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Px9'

  real_T RED_Py_InitialValue;          // Expression: 0
                                          //  Referenced by: '<Root>/RED_Py'

  real_T RED_Rz_InitialValue;          // Expression: 0
                                          //  Referenced by: '<Root>/RED_Rz'

  real_T RED_Tz_InitialValue;          // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz'

  real_T RED_Tz1_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz1'

  real_T RED_Tz2_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz2'

  real_T RED_Tz3_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz3'

  real_T RED_Tz7_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz7'

  real_T RED_Tz8_InitialValue;         // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz8'

  real_T RED_Tz_RW_InitialValue;       // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz_RW'

  real_T RED_Tz_RWSat_InitialValue;    // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz_RW Sat'

  real_T RED_Tz_RWSat1_InitialValue;   // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz_RW Sat1'

  real_T RED_Tz_RW1_InitialValue;      // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz_RW1'

  real_T RED_Tz_Sat_InitialValue;      // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz_Sat'

  real_T RED_Tz_Sat1_InitialValue;     // Expression: 0
                                          //  Referenced by: '<Root>/RED_Tz_Sat1'

  real_T RED_dRz_RWSat_InitialValue;   // Expression: 0
                                          //  Referenced by: '<Root>/RED_dRz_RW Sat'

  real_T Universal_Time_InitialValue;  // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time'

  real_T Universal_Time1_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time1'

  real_T Universal_Time10_InitialValue;// Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time10'

  real_T Universal_Time11_InitialValue;// Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time11'

  real_T Universal_Time12_InitialValue;// Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time12'

  real_T Universal_Time13_InitialValue;// Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time13'

  real_T Universal_Time14_InitialValue;// Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time14'

  real_T Universal_Time15_InitialValue;// Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time15'

  real_T Universal_Time2_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time2'

  real_T Universal_Time3_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time3'

  real_T Universal_Time4_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time4'

  real_T Universal_Time5_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time5'

  real_T Universal_Time6_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time6'

  real_T Universal_Time7_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time7'

  real_T Universal_Time8_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time8'

  real_T Universal_Time9_InitialValue; // Expression: 0
                                          //  Referenced by: '<Root>/Universal_Time9'

  int32_T UDPReceive_Port;             // Computed Parameter: UDPReceive_Port
                                          //  Referenced by: '<S79>/UDP Receive'

  int32_T SendBLACKStatestoBLACKPlatform_;
                          // Computed Parameter: SendBLACKStatestoBLACKPlatform_
                             //  Referenced by: '<S80>/Send BLACK States to  BLACK Platform'

  int32_T UDPSend_Port;                // Computed Parameter: UDPSend_Port
                                          //  Referenced by: '<S15>/UDP Send'

  boolean_T Delay1_InitialCondition;
                                  // Computed Parameter: Delay1_InitialCondition
                                     //  Referenced by: '<S63>/Delay1'

  boolean_T Delay2_InitialCondition;
                                  // Computed Parameter: Delay2_InitialCondition
                                     //  Referenced by: '<S63>/Delay2'

  boolean_T Delay5_InitialCondition;
                                  // Computed Parameter: Delay5_InitialCondition
                                     //  Referenced by: '<S63>/Delay5'

  boolean_T Delay3_InitialCondition;
                                  // Computed Parameter: Delay3_InitialCondition
                                     //  Referenced by: '<S63>/Delay3'

  boolean_T Delay4_InitialCondition;
                                  // Computed Parameter: Delay4_InitialCondition
                                     //  Referenced by: '<S63>/Delay4'

  boolean_T Delay_InitialCondition_m;
                                 // Computed Parameter: Delay_InitialCondition_m
                                    //  Referenced by: '<S153>/Delay'

  boolean_T Delay_InitialCondition_g;
                                 // Computed Parameter: Delay_InitialCondition_g
                                    //  Referenced by: '<S121>/Delay'

  P_ChangeBLUEBehavior_CLVF_T ChangeBLUEBehavior_k;// '<S14>/Change BLUE Behavior' 
  P_ChangeBLUEBehavior_CLVF_c_T ChangeBLUEBehavior_b;// '<S13>/Change BLUE Behavior' 
  P_ChangeBLUEBehavior_CLVF_c_T ChangeBLUEBehavior_gj;// '<S12>/Change BLUE Behavior' 
  P_SubPhase2_CLVF_e_T SubPhase3_jo;   // '<S319>/Sub-Phase #3 '
  P_SubPhase2_CLVF_e_T SubPhase2_d;    // '<S319>/Sub-Phase #2 '
  P_SubPhase1_CLVF_T SubPhase4_o;      // '<S318>/Sub-Phase #4'
  P_SubPhase2_CLVF_h_T SubPhase3_j;    // '<S318>/Sub-Phase #3 '
  P_SubPhase2_CLVF_h_T SubPhase2_l;    // '<S318>/Sub-Phase #2 '
  P_SubPhase1_CLVF_T SubPhase1_g;      // '<S318>/Sub-Phase #1'
  P_SubPhase2_CLVF_T SubPhase3;        // '<S317>/Sub-Phase #3 '
  P_SubPhase2_CLVF_T SubPhase2;        // '<S317>/Sub-Phase #2 '
  P_Phase0WaitforSynchronizatio_T Phase1StartFloating;// '<Root>/Phase #1:  Start Floating ' 
  P_Phase0WaitforSynchronizatio_T Phase0WaitforSynchronization;
                                // '<Root>/Phase #0:  Wait for Synchronization'
  P_CalculateRunningMean_CLVF_c_T CalculateRunningMean_mv;// '<S256>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_e;// '<S255>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_ck;// '<S254>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_avn;// '<S243>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_bv;// '<S242>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_mr;// '<S241>/Calculate Running Mean' 
  P_AHRS2_CLVF_T AHRS2_pn;             // '<S169>/AHRS2'
  P_CalculateRunningMean_CLVF_c_T CalculateRunningMean_l;// '<S225>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_pa;// '<S224>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_m;// '<S223>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_av;// '<S212>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_a;// '<S211>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_b2;// '<S210>/Calculate Running Mean' 
  P_AHRS2_CLVF_T AHRS2_p;              // '<S169>/AHRS2'
  P_CalculateRunningMean_CLVF_c_T CalculateRunningMean_b;// '<S194>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_k;// '<S193>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_ch;// '<S192>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_c;// '<S181>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean_p;// '<S180>/Calculate Running Mean' 
  P_CalculateRunningMean_CLVF_T CalculateRunningMean;// '<S179>/Calculate Running Mean' 
  P_AHRS2_CLVF_T AHRS2;                // '<S169>/AHRS2'
};

// Real-time Model Data Structure
struct tag_RTM_CLVF_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    struct {
      boolean_T TID1_2;
    } RateInteraction;

    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_CLVF_T CLVF_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern B_CLVF_T CLVF_B;

#ifdef __cplusplus

}
#endif

// Block states (default storage)
extern DW_CLVF_T CLVF_DW;

// Zero-crossing (trigger) state
extern PrevZCX_CLVF_T CLVF_PrevZCX;

// External function called from main
#ifdef __cplusplus

extern "C" {

#endif

  extern void CLVF_SetEventsForThisBaseStep(boolean_T *eventFlags);

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void CLVF_SetEventsForThisBaseStep(boolean_T *eventFlags);
  extern void CLVF_initialize(void);
  extern void CLVF_step(int_T tid);
  extern void CLVF_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_CLVF_T *const CLVF_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S123>/Check Signal Attributes' : Unused code path elimination
//  Block '<S124>/Check Signal Attributes' : Unused code path elimination
//  Block '<S155>/Check Signal Attributes' : Unused code path elimination
//  Block '<S156>/Check Signal Attributes' : Unused code path elimination
//  Block '<S178>/Check Signal Attributes' : Unused code path elimination
//  Block '<S209>/Check Signal Attributes' : Unused code path elimination
//  Block '<S240>/Check Signal Attributes' : Unused code path elimination
//  Block '<S350>/1Hz LP Filter1' : Unused code path elimination
//  Block '<S364>/divide by delta T' : Unused code path elimination
//  Block '<S365>/divide by delta T' : Unused code path elimination
//  Block '<S366>/divide by delta T' : Unused code path elimination
//  Block '<S342>/Constant4' : Unused code path elimination
//  Block '<S79>/Reshape' : Reshape block reduction
//  Block '<S169>/Reshape' : Reshape block reduction
//  Block '<S169>/Reshape1' : Reshape block reduction
//  Block '<S169>/Reshape2' : Reshape block reduction
//  Block '<S169>/Reshape3' : Reshape block reduction
//  Block '<S170>/Reshape' : Reshape block reduction
//  Block '<S170>/Reshape1' : Reshape block reduction
//  Block '<S170>/Reshape2' : Reshape block reduction
//  Block '<S170>/Reshape3' : Reshape block reduction
//  Block '<S171>/Reshape' : Reshape block reduction
//  Block '<S171>/Reshape1' : Reshape block reduction
//  Block '<S171>/Reshape2' : Reshape block reduction
//  Block '<S171>/Reshape3' : Reshape block reduction


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CLVF'
//  '<S1>'   : 'CLVF/Data Logger Subsystem'
//  '<S2>'   : 'CLVF/Float & Magnet Controls'
//  '<S3>'   : 'CLVF/From Force//Torque to PWM Signal'
//  '<S4>'   : 'CLVF/From Torque, Command RW'
//  '<S5>'   : 'CLVF/Grab PhaseSpace Data via UDP'
//  '<S6>'   : 'CLVF/Gyroscope & Acceleration Algorithms'
//  '<S7>'   : 'CLVF/Is this a  simulation?'
//  '<S8>'   : 'CLVF/Phase #0:  Wait for Synchronization'
//  '<S9>'   : 'CLVF/Phase #1:  Start Floating '
//  '<S10>'  : 'CLVF/Phase #2:  Move to  Initial Position'
//  '<S11>'  : 'CLVF/Phase #3: Experiment'
//  '<S12>'  : 'CLVF/Phase #4:  Return Home'
//  '<S13>'  : 'CLVF/Phase #5:  Hold Home'
//  '<S14>'  : 'CLVF/Phase #6:  Stop Floating and Spin Down RW'
//  '<S15>'  : 'CLVF/Real-Time Visualization'
//  '<S16>'  : 'CLVF/Simulate Plant Dynamics'
//  '<S17>'  : 'CLVF/Subsystem'
//  '<S18>'  : 'CLVF/Float & Magnet Controls/Change Behavior'
//  '<S19>'  : 'CLVF/Float & Magnet Controls/Is this a  simulation?'
//  '<S20>'  : 'CLVF/Float & Magnet Controls/Change Behavior/GPIO for Magnet'
//  '<S21>'  : 'CLVF/Float & Magnet Controls/Change Behavior/GPIO for Pucks'
//  '<S22>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior'
//  '<S23>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior'
//  '<S24>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior'
//  '<S25>'  : 'CLVF/From Force//Torque to PWM Signal/Is this a  simulation?'
//  '<S26>'  : 'CLVF/From Force//Torque to PWM Signal/Send Commands to PWM Blocks'
//  '<S27>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF'
//  '<S28>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior/Rotate Forces to Body'
//  '<S29>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/MATLAB Function'
//  '<S30>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/MATLAB Function1'
//  '<S31>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/MATLAB Function2'
//  '<S32>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/Rotate Forces to Inertial'
//  '<S33>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/pseudo-inverse'
//  '<S34>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/Rotate Forces to Inertial/Create Rotation Matrix'
//  '<S35>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLACK Behavior/Rotate Forces to Body/Create Rotation Matrix'
//  '<S36>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior/Calculate Thruster  ON//OFF'
//  '<S37>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior/Rotate Forces to Body'
//  '<S38>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior/Calculate Thruster  ON//OFF/MATLAB Function'
//  '<S39>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior/Calculate Thruster  ON//OFF/MATLAB Function1'
//  '<S40>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior/Calculate Thruster  ON//OFF/MATLAB Function2'
//  '<S41>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior/Calculate Thruster  ON//OFF/Rotate Forces to Inertial'
//  '<S42>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior/Calculate Thruster  ON//OFF/pseudo-inverse'
//  '<S43>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior/Calculate Thruster  ON//OFF/Rotate Forces to Inertial/Create Rotation Matrix'
//  '<S44>'  : 'CLVF/From Force//Torque to PWM Signal/Change BLUE Behavior/Rotate Forces to Body/Create Rotation Matrix'
//  '<S45>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior/Calculate Thruster  ON//OFF'
//  '<S46>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior/Rotate Forces to Body'
//  '<S47>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior/Calculate Thruster  ON//OFF/MATLAB Function'
//  '<S48>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior/Calculate Thruster  ON//OFF/MATLAB Function1'
//  '<S49>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior/Calculate Thruster  ON//OFF/MATLAB Function2'
//  '<S50>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior/Calculate Thruster  ON//OFF/Rotate Forces to Inertial'
//  '<S51>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior/Calculate Thruster  ON//OFF/pseudo-inverse'
//  '<S52>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior/Calculate Thruster  ON//OFF/Rotate Forces to Inertial/Create Rotation Matrix'
//  '<S53>'  : 'CLVF/From Force//Torque to PWM Signal/Change RED Behavior/Rotate Forces to Body/Create Rotation Matrix'
//  '<S54>'  : 'CLVF/From Torque, Command RW/Change BLACK Behavior'
//  '<S55>'  : 'CLVF/From Torque, Command RW/Change BLUE Behavior'
//  '<S56>'  : 'CLVF/From Torque, Command RW/Change RED Behavior'
//  '<S57>'  : 'CLVF/From Torque, Command RW/Is this a  simulation?'
//  '<S58>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Calculate Saturated RPM Signal'
//  '<S59>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM'
//  '<S60>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Calculate Saturated RPM Signal/Calculate RW RPM Increment'
//  '<S61>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Check if RW is Ready'
//  '<S62>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Convert Wheel Rate to  Motor Rate'
//  '<S63>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Delay Bank'
//  '<S64>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Get RW Direction'
//  '<S65>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Obtain RW Status'
//  '<S66>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/RPM to PWM Value'
//  '<S67>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Send Direction to Motor Controller'
//  '<S68>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Send PWM to Motor Controller'
//  '<S69>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Turn on Motor'
//  '<S70>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Obtain RW Status/GPIO Read1'
//  '<S71>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Send Direction to Motor Controller/GPIO Write'
//  '<S72>'  : 'CLVF/From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Turn on Motor/GPIO Write1'
//  '<S73>'  : 'CLVF/Grab PhaseSpace Data via UDP/Initialize Universal Time (Simulation)'
//  '<S74>'  : 'CLVF/Grab PhaseSpace Data via UDP/Is this a  simulation?1'
//  '<S75>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States'
//  '<S76>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM'
//  '<S77>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM'
//  '<S78>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Is this a  simulation?'
//  '<S79>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States'
//  '<S80>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States'
//  '<S81>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x1'
//  '<S82>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x10'
//  '<S83>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x11'
//  '<S84>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x12'
//  '<S85>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x2'
//  '<S86>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x3'
//  '<S87>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x4'
//  '<S88>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x5'
//  '<S89>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x6'
//  '<S90>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x7'
//  '<S91>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x8'
//  '<S92>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x9'
//  '<S93>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x1/Hold this value'
//  '<S94>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x10/Hold this value'
//  '<S95>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x11/Hold this value'
//  '<S96>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x12/Hold this value'
//  '<S97>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x2/Hold this value'
//  '<S98>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x3/Hold this value'
//  '<S99>'  : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x4/Hold this value'
//  '<S100>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x5/Hold this value'
//  '<S101>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x6/Hold this value'
//  '<S102>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x7/Hold this value'
//  '<S103>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x8/Hold this value'
//  '<S104>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain BLACK States/x_dot -> x9/Hold this value'
//  '<S105>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/Compare To Zero'
//  '<S106>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/Correct Time'
//  '<S107>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/Enabled Subsystem'
//  '<S108>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/Enabled Subsystem1'
//  '<S109>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x1'
//  '<S110>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x10'
//  '<S111>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x11'
//  '<S112>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x12'
//  '<S113>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x2'
//  '<S114>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x3'
//  '<S115>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x4'
//  '<S116>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x5'
//  '<S117>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x6'
//  '<S118>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x7'
//  '<S119>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x8'
//  '<S120>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x9'
//  '<S121>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/Correct Time/Edge Detector'
//  '<S122>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/Correct Time/Enabled Subsystem'
//  '<S123>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/Correct Time/Edge Detector/Check Signal Attributes'
//  '<S124>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/Correct Time/Edge Detector/Check Signal Attributes1'
//  '<S125>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x1/Hold this value'
//  '<S126>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x10/Hold this value'
//  '<S127>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x11/Hold this value'
//  '<S128>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x12/Hold this value'
//  '<S129>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x2/Hold this value'
//  '<S130>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x3/Hold this value'
//  '<S131>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x4/Hold this value'
//  '<S132>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x5/Hold this value'
//  '<S133>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x6/Hold this value'
//  '<S134>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x7/Hold this value'
//  '<S135>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x8/Hold this value'
//  '<S136>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED+BLACK, or RED+BLACK+ARM/Obtain RED States/x_dot -> x9/Hold this value'
//  '<S137>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/Compare To Zero'
//  '<S138>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/Correct Time'
//  '<S139>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/Enabled Subsystem'
//  '<S140>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/Enabled Subsystem1'
//  '<S141>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x1'
//  '<S142>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x10'
//  '<S143>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x11'
//  '<S144>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x12'
//  '<S145>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x2'
//  '<S146>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x3'
//  '<S147>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x4'
//  '<S148>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x5'
//  '<S149>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x6'
//  '<S150>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x7'
//  '<S151>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x8'
//  '<S152>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x9'
//  '<S153>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/Correct Time/Edge Detector'
//  '<S154>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/Correct Time/Enabled Subsystem'
//  '<S155>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/Correct Time/Edge Detector/Check Signal Attributes'
//  '<S156>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/Correct Time/Edge Detector/Check Signal Attributes1'
//  '<S157>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x1/Hold this value'
//  '<S158>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x10/Hold this value'
//  '<S159>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x11/Hold this value'
//  '<S160>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x12/Hold this value'
//  '<S161>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x2/Hold this value'
//  '<S162>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x3/Hold this value'
//  '<S163>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x4/Hold this value'
//  '<S164>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x5/Hold this value'
//  '<S165>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x6/Hold this value'
//  '<S166>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x7/Hold this value'
//  '<S167>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x8/Hold this value'
//  '<S168>' : 'CLVF/Grab PhaseSpace Data via UDP/Use Hardware to Obtain States/Using RED, BLACK, BLUE, or RED + ARM/x_dot -> x9/Hold this value'
//  '<S169>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior'
//  '<S170>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior'
//  '<S171>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior'
//  '<S172>' : 'CLVF/Gyroscope & Acceleration Algorithms/Is this a  simulation?'
//  '<S173>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/ChangeOrientation'
//  '<S174>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Create Rotation Matrix'
//  '<S175>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Digital Filter Design'
//  '<S176>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias'
//  '<S177>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)'
//  '<S178>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Digital Filter Design/Check Signal Attributes'
//  '<S179>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroX'
//  '<S180>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroY'
//  '<S181>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroZ'
//  '<S182>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroX/Calculate Running Mean'
//  '<S183>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroX/Pass Current Gyro'
//  '<S184>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroX/Subsystem'
//  '<S185>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroY/Calculate Running Mean'
//  '<S186>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroY/Pass Current Gyro'
//  '<S187>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroY/Subsystem'
//  '<S188>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroZ/Calculate Running Mean'
//  '<S189>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroZ/Compare To Zero1'
//  '<S190>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroZ/Pass Current Gyro'
//  '<S191>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias/Correct GyroZ/Subsystem'
//  '<S192>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelX'
//  '<S193>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelY'
//  '<S194>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelZ'
//  '<S195>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelX/Calculate Running Mean'
//  '<S196>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelX/Pass Current Accel'
//  '<S197>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelX/Subsystem'
//  '<S198>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelY/Calculate Running Mean'
//  '<S199>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelY/Pass Current Accel'
//  '<S200>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelY/Subsystem'
//  '<S201>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelZ/Calculate Running Mean'
//  '<S202>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelZ/Pass Current Accel'
//  '<S203>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLACK Behavior/Fix Initial Bias (Accel)/Correct AccelZ/Subsystem'
//  '<S204>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/ChangeOrientation'
//  '<S205>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Create Rotation Matrix'
//  '<S206>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Digital Filter Design'
//  '<S207>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias'
//  '<S208>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)'
//  '<S209>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Digital Filter Design/Check Signal Attributes'
//  '<S210>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroX'
//  '<S211>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroY'
//  '<S212>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroZ'
//  '<S213>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroX/Calculate Running Mean'
//  '<S214>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroX/Pass Current Gyro'
//  '<S215>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroX/Subsystem'
//  '<S216>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroY/Calculate Running Mean'
//  '<S217>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroY/Pass Current Gyro'
//  '<S218>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroY/Subsystem'
//  '<S219>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroZ/Calculate Running Mean'
//  '<S220>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroZ/Compare To Zero1'
//  '<S221>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroZ/Pass Current Gyro'
//  '<S222>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias/Correct GyroZ/Subsystem'
//  '<S223>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelX'
//  '<S224>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelY'
//  '<S225>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelZ'
//  '<S226>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelX/Calculate Running Mean'
//  '<S227>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelX/Pass Current Accel'
//  '<S228>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelX/Subsystem'
//  '<S229>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelY/Calculate Running Mean'
//  '<S230>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelY/Pass Current Accel'
//  '<S231>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelY/Subsystem'
//  '<S232>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelZ/Calculate Running Mean'
//  '<S233>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelZ/Pass Current Accel'
//  '<S234>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change BLUE Behavior/Fix Initial Bias (Accel)/Correct AccelZ/Subsystem'
//  '<S235>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/ChangeOrientation'
//  '<S236>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Create Rotation Matrix'
//  '<S237>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Digital Filter Design1'
//  '<S238>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias'
//  '<S239>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)'
//  '<S240>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Digital Filter Design1/Check Signal Attributes'
//  '<S241>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroX'
//  '<S242>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroY'
//  '<S243>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroZ'
//  '<S244>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroX/Calculate Running Mean'
//  '<S245>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroX/Pass Current Gyro'
//  '<S246>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroX/Subsystem'
//  '<S247>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroY/Calculate Running Mean'
//  '<S248>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroY/Pass Current Gyro'
//  '<S249>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroY/Subsystem'
//  '<S250>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroZ/Calculate Running Mean'
//  '<S251>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroZ/Compare To Zero1'
//  '<S252>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroZ/Pass Current Gyro'
//  '<S253>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias/Correct GyroZ/Subsystem'
//  '<S254>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelX'
//  '<S255>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelY'
//  '<S256>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelZ'
//  '<S257>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelX/Calculate Running Mean'
//  '<S258>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelX/Pass Current Accel'
//  '<S259>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelX/Subsystem'
//  '<S260>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelY/Calculate Running Mean'
//  '<S261>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelY/Pass Current Accel'
//  '<S262>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelY/Subsystem'
//  '<S263>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelZ/Calculate Running Mean'
//  '<S264>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelZ/Pass Current Accel'
//  '<S265>' : 'CLVF/Gyroscope & Acceleration Algorithms/Change RED Behavior/Fix Initial Bias (Accel)/Correct AccelZ/Subsystem'
//  '<S266>' : 'CLVF/Phase #0:  Wait for Synchronization/Change BLACK Behavior'
//  '<S267>' : 'CLVF/Phase #0:  Wait for Synchronization/Change BLUE Behavior'
//  '<S268>' : 'CLVF/Phase #0:  Wait for Synchronization/Change RED Behavior'
//  '<S269>' : 'CLVF/Phase #0:  Wait for Synchronization/Is this a  simulation?'
//  '<S270>' : 'CLVF/Phase #1:  Start Floating /Change BLACK Behavior'
//  '<S271>' : 'CLVF/Phase #1:  Start Floating /Change BLUE Behavior'
//  '<S272>' : 'CLVF/Phase #1:  Start Floating /Change RED Behavior'
//  '<S273>' : 'CLVF/Phase #1:  Start Floating /Is this a  simulation?'
//  '<S274>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior'
//  '<S275>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior'
//  '<S276>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior'
//  '<S277>' : 'CLVF/Phase #2:  Move to  Initial Position/Is this a  simulation?'
//  '<S278>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Custom Discrete PD (Attitude)'
//  '<S279>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Custom Discrete PD (X-Position)'
//  '<S280>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Custom Discrete PD (Y-Position)'
//  '<S281>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Hough Control'
//  '<S282>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S283>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S284>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S285>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S286>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S287>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S288>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Hough Control/MATLAB Function2'
//  '<S289>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Hough Control/MATLAB Function3'
//  '<S290>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLACK Behavior/Hough Control/MATLAB Function4'
//  '<S291>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Custom Discrete PD (Attitude)'
//  '<S292>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Custom Discrete PD (X-Position)'
//  '<S293>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Custom Discrete PD (Y-Position)'
//  '<S294>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Hough Control'
//  '<S295>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S296>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S297>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S298>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S299>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S300>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S301>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Hough Control/MATLAB Function2'
//  '<S302>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Hough Control/MATLAB Function3'
//  '<S303>' : 'CLVF/Phase #2:  Move to  Initial Position/Change BLUE Behavior/Hough Control/MATLAB Function4'
//  '<S304>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Custom Discrete PD (Attitude)'
//  '<S305>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Custom Discrete PD (X-Position)'
//  '<S306>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Custom Discrete PD (Y-Position)'
//  '<S307>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Hough Control'
//  '<S308>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S309>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S310>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S311>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S312>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S313>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S314>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Hough Control/MATLAB Function2'
//  '<S315>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Hough Control/MATLAB Function3'
//  '<S316>' : 'CLVF/Phase #2:  Move to  Initial Position/Change RED Behavior/Hough Control/MATLAB Function4'
//  '<S317>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior'
//  '<S318>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior'
//  '<S319>' : 'CLVF/Phase #3: Experiment/Change RED Behavior'
//  '<S320>' : 'CLVF/Phase #3: Experiment/Is this a  simulation?'
//  '<S321>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1'
//  '<S322>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #2 '
//  '<S323>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #3 '
//  '<S324>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4'
//  '<S325>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Subsystem'
//  '<S326>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Custom Discrete PD (Attitude)'
//  '<S327>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Custom Discrete PD (X-Position)'
//  '<S328>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Custom Discrete PD (Y-Position)'
//  '<S329>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Hough Control'
//  '<S330>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S331>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S332>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S333>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S334>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S335>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S336>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Hough Control/MATLAB Function2'
//  '<S337>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Hough Control/MATLAB Function3'
//  '<S338>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #1/Hough Control/MATLAB Function4'
//  '<S339>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Custom Discrete PD (Attitude)'
//  '<S340>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Hough Control'
//  '<S341>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC'
//  '<S342>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/States in Target-Fixed Frame'
//  '<S343>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/getDesiredRotation'
//  '<S344>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S345>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S346>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Hough Control/MATLAB Function2'
//  '<S347>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Hough Control/MATLAB Function3'
//  '<S348>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Hough Control/MATLAB Function4'
//  '<S349>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System'
//  '<S350>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation'
//  '<S351>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/CLVF'
//  '<S352>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/Control'
//  '<S353>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/Lyapunov'
//  '<S354>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/SWITCH'
//  '<S355>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/CLVF/Lyapunov Guidance + FF'
//  '<S356>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/Control/PD + FF Controller'
//  '<S357>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/Lyapunov/Body-Fixed Guidance'
//  '<S358>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/SWITCH/Sample and Hold'
//  '<S359>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/SWITCH/inDesiredRadius'
//  '<S360>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/SWITCH/sufficientTime'
//  '<S361>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/Vec Derivative'
//  '<S362>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/getRotationMatrix'
//  '<S363>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/x_dot -> x3'
//  '<S364>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/Vec Derivative/x_dot -> x1'
//  '<S365>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/Vec Derivative/x_dot -> x2'
//  '<S366>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/Vec Derivative/x_dot -> x3'
//  '<S367>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/Vec Derivative/x_dot -> x1/Hold this value'
//  '<S368>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/Vec Derivative/x_dot -> x2/Hold this value'
//  '<S369>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/Vec Derivative/x_dot -> x3/Hold this value'
//  '<S370>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/x_dot -> x3/Hold this value'
//  '<S371>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/States in Target-Fixed Frame/MATLAB Function'
//  '<S372>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/States in Target-Fixed Frame/Norm'
//  '<S373>' : 'CLVF/Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/States in Target-Fixed Frame/Norm1'
//  '<S374>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1'
//  '<S375>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #2 '
//  '<S376>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #3 '
//  '<S377>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4'
//  '<S378>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Subsystem'
//  '<S379>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Custom Discrete PD (Attitude)'
//  '<S380>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Custom Discrete PD (X-Position)'
//  '<S381>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Custom Discrete PD (Y-Position)'
//  '<S382>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Hough Control'
//  '<S383>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S384>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S385>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S386>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S387>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S388>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S389>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Hough Control/MATLAB Function2'
//  '<S390>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Hough Control/MATLAB Function3'
//  '<S391>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #1/Hough Control/MATLAB Function4'
//  '<S392>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Custom Discrete PD (Attitude)'
//  '<S393>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Custom Discrete PD (X-Position)'
//  '<S394>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Custom Discrete PD (Y-Position)'
//  '<S395>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Hough Control'
//  '<S396>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S397>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S398>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S399>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S400>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S401>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S402>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Hough Control/MATLAB Function2'
//  '<S403>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Hough Control/MATLAB Function3'
//  '<S404>' : 'CLVF/Phase #3: Experiment/Change BLUE Behavior/Sub-Phase #4/Hough Control/MATLAB Function4'
//  '<S405>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1'
//  '<S406>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #2 '
//  '<S407>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #3 '
//  '<S408>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4'
//  '<S409>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Subsystem'
//  '<S410>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Custom Discrete PD (Attitude)'
//  '<S411>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Custom Discrete PD (X-Position)'
//  '<S412>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Custom Discrete PD (Y-Position)'
//  '<S413>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Hough Control'
//  '<S414>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S415>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S416>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S417>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S418>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S419>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S420>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Hough Control/MATLAB Function2'
//  '<S421>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Hough Control/MATLAB Function3'
//  '<S422>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #1/Hough Control/MATLAB Function4'
//  '<S423>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Custom Discrete PD (Attitude)'
//  '<S424>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Custom Discrete PD (X-Position)'
//  '<S425>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Custom Discrete PD (Y-Position)'
//  '<S426>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Hough Control'
//  '<S427>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Subsystem'
//  '<S428>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S429>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S430>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S431>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S432>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S433>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S434>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Hough Control/MATLAB Function2'
//  '<S435>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Hough Control/MATLAB Function3'
//  '<S436>' : 'CLVF/Phase #3: Experiment/Change RED Behavior/Sub-Phase #4/Hough Control/MATLAB Function4'
//  '<S437>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior'
//  '<S438>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior'
//  '<S439>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior'
//  '<S440>' : 'CLVF/Phase #4:  Return Home/Is this a  simulation?'
//  '<S441>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Custom Discrete PD (Attitude)'
//  '<S442>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Custom Discrete PD (X-Position)'
//  '<S443>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Custom Discrete PD (Y-Position)'
//  '<S444>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Hough Control'
//  '<S445>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S446>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S447>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S448>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S449>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S450>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S451>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Hough Control/MATLAB Function2'
//  '<S452>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Hough Control/MATLAB Function3'
//  '<S453>' : 'CLVF/Phase #4:  Return Home/Change BLACK Behavior/Hough Control/MATLAB Function4'
//  '<S454>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Custom Discrete PD (Attitude)'
//  '<S455>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Custom Discrete PD (X-Position)'
//  '<S456>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Custom Discrete PD (Y-Position)'
//  '<S457>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Hough Control'
//  '<S458>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S459>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S460>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S461>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S462>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S463>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S464>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Hough Control/MATLAB Function2'
//  '<S465>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Hough Control/MATLAB Function3'
//  '<S466>' : 'CLVF/Phase #4:  Return Home/Change BLUE Behavior/Hough Control/MATLAB Function4'
//  '<S467>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Custom Discrete PD (Attitude)'
//  '<S468>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Custom Discrete PD (X-Position)'
//  '<S469>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Custom Discrete PD (Y-Position)'
//  '<S470>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Hough Control'
//  '<S471>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S472>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S473>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S474>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S475>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S476>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S477>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Hough Control/MATLAB Function2'
//  '<S478>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Hough Control/MATLAB Function3'
//  '<S479>' : 'CLVF/Phase #4:  Return Home/Change RED Behavior/Hough Control/MATLAB Function4'
//  '<S480>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior'
//  '<S481>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior'
//  '<S482>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior'
//  '<S483>' : 'CLVF/Phase #5:  Hold Home/Is this a  simulation?'
//  '<S484>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Custom Discrete PD (Attitude)'
//  '<S485>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Custom Discrete PD (X-Position)'
//  '<S486>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Custom Discrete PD (Y-Position)'
//  '<S487>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Hough Control'
//  '<S488>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S489>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S490>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S491>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S492>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S493>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S494>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Hough Control/MATLAB Function2'
//  '<S495>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Hough Control/MATLAB Function3'
//  '<S496>' : 'CLVF/Phase #5:  Hold Home/Change BLACK Behavior/Hough Control/MATLAB Function4'
//  '<S497>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Custom Discrete PD (Attitude)'
//  '<S498>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Custom Discrete PD (X-Position)'
//  '<S499>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Custom Discrete PD (Y-Position)'
//  '<S500>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Hough Control'
//  '<S501>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S502>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S503>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S504>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S505>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S506>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S507>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Hough Control/MATLAB Function2'
//  '<S508>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Hough Control/MATLAB Function3'
//  '<S509>' : 'CLVF/Phase #5:  Hold Home/Change BLUE Behavior/Hough Control/MATLAB Function4'
//  '<S510>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Custom Discrete PD (Attitude)'
//  '<S511>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Custom Discrete PD (X-Position)'
//  '<S512>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Custom Discrete PD (Y-Position)'
//  '<S513>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Hough Control'
//  '<S514>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Custom Discrete PD (Attitude)/x_dot -> x1'
//  '<S515>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Custom Discrete PD (Attitude)/x_dot -> x1/Hold this value'
//  '<S516>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Custom Discrete PD (X-Position)/x_dot -> x1'
//  '<S517>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Custom Discrete PD (X-Position)/x_dot -> x1/Hold this value'
//  '<S518>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1'
//  '<S519>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Custom Discrete PD (Y-Position)/x_dot -> x1/Hold this value'
//  '<S520>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Hough Control/MATLAB Function2'
//  '<S521>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Hough Control/MATLAB Function3'
//  '<S522>' : 'CLVF/Phase #5:  Hold Home/Change RED Behavior/Hough Control/MATLAB Function4'
//  '<S523>' : 'CLVF/Phase #6:  Stop Floating and Spin Down RW/Change BLACK Behavior'
//  '<S524>' : 'CLVF/Phase #6:  Stop Floating and Spin Down RW/Change BLUE Behavior'
//  '<S525>' : 'CLVF/Phase #6:  Stop Floating and Spin Down RW/Change RED Behavior'
//  '<S526>' : 'CLVF/Phase #6:  Stop Floating and Spin Down RW/Is this a  simulation?'
//  '<S527>' : 'CLVF/Simulate Plant Dynamics/BLACK Dynamics Model'
//  '<S528>' : 'CLVF/Simulate Plant Dynamics/BLUE  Dynamics Model'
//  '<S529>' : 'CLVF/Simulate Plant Dynamics/Compare To Zero'
//  '<S530>' : 'CLVF/Simulate Plant Dynamics/Compare To Zero1'
//  '<S531>' : 'CLVF/Simulate Plant Dynamics/Compare To Zero2'
//  '<S532>' : 'CLVF/Simulate Plant Dynamics/RED Dynamics Model'
//  '<S533>' : 'CLVF/Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (BLACK)'
//  '<S534>' : 'CLVF/Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (BLUE)'
//  '<S535>' : 'CLVF/Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (RED)'
//  '<S536>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x1'
//  '<S537>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x10'
//  '<S538>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x11'
//  '<S539>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x13'
//  '<S540>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x14'
//  '<S541>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x16'
//  '<S542>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x17'
//  '<S543>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x2'
//  '<S544>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x3'
//  '<S545>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x4'
//  '<S546>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x5'
//  '<S547>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x6'
//  '<S548>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x7'
//  '<S549>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x8'
//  '<S550>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x9'
//  '<S551>' : 'CLVF/Simulate Plant Dynamics/BLACK Dynamics Model/MATLAB Function'
//  '<S552>' : 'CLVF/Simulate Plant Dynamics/BLUE  Dynamics Model/MATLAB Function'
//  '<S553>' : 'CLVF/Simulate Plant Dynamics/RED Dynamics Model/MATLAB Function'
//  '<S554>' : 'CLVF/Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (BLACK)/MATLAB Function'
//  '<S555>' : 'CLVF/Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (BLACK)/Sample and Hold1'
//  '<S556>' : 'CLVF/Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (BLUE)/MATLAB Function'
//  '<S557>' : 'CLVF/Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (BLUE)/Sample and Hold1'
//  '<S558>' : 'CLVF/Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (RED)/MATLAB Function'
//  '<S559>' : 'CLVF/Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (RED)/Sample and Hold'
//  '<S560>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x1/Hold this value'
//  '<S561>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x10/Hold this value'
//  '<S562>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x11/Hold this value'
//  '<S563>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x13/Hold this value'
//  '<S564>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x14/Hold this value'
//  '<S565>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x16/Hold this value'
//  '<S566>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x17/Hold this value'
//  '<S567>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x2/Hold this value'
//  '<S568>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x3/Hold this value'
//  '<S569>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x4/Hold this value'
//  '<S570>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x5/Hold this value'
//  '<S571>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x6/Hold this value'
//  '<S572>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x7/Hold this value'
//  '<S573>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x8/Hold this value'
//  '<S574>' : 'CLVF/Simulate Plant Dynamics/x_dot -> x9/Hold this value'

#endif                                 // RTW_HEADER_CLVF_h_
