//
//  CLVF_private.h
//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Code generation for model "CLVF".
//
//  Model version              : 1.279
//  Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
//  C++ source code generated on : Wed Apr  6 21:29:06 2022
//
//  Target selection: ert.tlc
//  Embedded hardware selection: ARM Compatible->ARM Cortex
//  Code generation objective: Execution efficiency
//  Validation result: Not run


#ifndef RTW_HEADER_CLVF_private_h_
#define RTW_HEADER_CLVF_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "zero_crossing_types.h"
#include "CLVF.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTFinal
# define rtmSetTFinal(rtm, val)        ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern real_T rt_roundd_snf(real_T u);
extern void CLVF_MATLABFunction2(const real_T rtu_ThrustPer[8],
  B_MATLABFunction2_CLVF_T *localB);
extern void CLVF_CreateRotationMatrix(real_T rtu_Rz,
  B_CreateRotationMatrix_CLVF_T *localB);
extern void CLVF_CreateRotationMatrix_h(real_T rtu_Rz,
  B_CreateRotationMatrix_CLVF_i_T *localB);
extern void CLVF_MATLABFunction(B_MATLABFunction_CLVF_T *localB);
extern void CLVF_AHRS2_Init(const real_T rtu_0[3], DW_AHRS2_CLVF_T *localDW,
  P_AHRS2_CLVF_T *localP);
extern void CLVF_AHRS2(const real_T rtu_0[3], const real_T rtu_1[3], const
  real_T rtu_2[3], B_AHRS2_CLVF_T *localB, DW_AHRS2_CLVF_T *localDW,
  P_AHRS2_CLVF_T *localP);
extern void CLVF_ChangeOrientation(real_T rtu_u, real_T rtu_u_m, real_T rtu_u_e,
  B_ChangeOrientation_CLVF_T *localB);
extern void C_CalculateRunningMean_Init(B_CalculateRunningMean_CLVF_T *localB,
  DW_CalculateRunningMean_CLVF_T *localDW, P_CalculateRunningMean_CLVF_T *localP);
extern void CLVF_CalculateRunningMean(real_T rtu_In1,
  B_CalculateRunningMean_CLVF_T *localB, DW_CalculateRunningMean_CLVF_T *localDW);
extern void CalculateRunningMean_b_Init(B_CalculateRunningMean_CLVF_p_T *localB,
  DW_CalculateRunningMean_CLV_e_T *localDW, P_CalculateRunningMean_CLVF_c_T
  *localP);
extern void CLVF_CalculateRunningMean_b(real_T rtu_In1,
  B_CalculateRunningMean_CLVF_p_T *localB, DW_CalculateRunningMean_CLV_e_T
  *localDW, P_CalculateRunningMean_CLVF_c_T *localP);
extern void CLVF_MATLABFunction_f(real_T rtu_TIME, real_T rtu_SERVER,
  B_MATLABFunction_CLVF_g_T *localB);
extern void CLVF_ChangeBLUEBehavior(P_ChangeBLUEBehavior_CLVF_T *localP, real_T *
  rtd_BLUE_Fx, real_T *rtd_BLUE_Fy, real_T *rtd_BLUE_Tz, real_T *rtd_Float_State);
extern void Phase0WaitforSynchroni_Init(DW_Phase0WaitforSynchronizati_T *localDW);
extern void Phase0WaitforSynchronizatio(DW_Phase0WaitforSynchronizati_T *localDW,
  P_Phase0WaitforSynchronizatio_T *localP, real_T *rtd_BLACK_Fx, real_T
  *rtd_BLACK_Fy, real_T *rtd_BLACK_Tz, real_T *rtd_BLUE_Fx, real_T *rtd_BLUE_Fy,
  real_T *rtd_BLUE_Tz, real_T *rtd_Float_State, real_T *rtd_RED_Fx, real_T
  *rtd_RED_Fy, real_T *rtd_RED_Tz, real_T *rtd_RED_Tz_Elbow, real_T
  *rtd_RED_Tz_RW, real_T *rtd_RED_Tz_Shoulder, real_T *rtd_RED_Tz_Wrist);
extern void CLVF_MATLABFunction2_o(real_T rtu_z, B_MATLABFunction2_CLVF_n_T
  *localB);
extern void CLVF_MATLABFunction3(real_T rtu_z, B_MATLABFunction3_CLVF_T *localB);
extern void CLVF_MATLABFunction4(const real_T rtu_Oy[2], const real_T rtu_e_in[2],
  B_MATLABFunction4_CLVF_T *localB);
extern void CLVF_SubPhase2(P_SubPhase2_CLVF_T *localP, real_T *rtd_BLACK_Fx,
  real_T *rtd_BLACK_Fy, real_T *rtd_BLACK_Tz, real_T *rtd_Float_State);
extern void CLVF_MATLABFunction2_c(real_T rtu_z, B_MATLABFunction2_CLVF_d_T
  *localB);
extern void CLVF_SubPhase1_Init(B_SubPhase1_CLVF_T *localB, DW_SubPhase1_CLVF_T *
  localDW, P_SubPhase1_CLVF_T *localP);
extern void CLVF_SubPhase1_Update(B_SubPhase1_CLVF_T *localB,
  DW_SubPhase1_CLVF_T *localDW);
extern void CLVF_SubPhase1(B_SubPhase1_CLVF_T *localB, DW_SubPhase1_CLVF_T
  *localDW, P_SubPhase1_CLVF_T *localP, real_T *rtd_BLUE_Fx, real_T *rtd_BLUE_Fy,
  real_T *rtd_BLUE_Px, real_T *rtd_BLUE_Py, real_T *rtd_BLUE_Rz, real_T
  *rtd_BLUE_Tz, real_T *rtd_Float_State);
extern void CLVF_SubPhase2_l(P_SubPhase2_CLVF_h_T *localP, real_T *rtd_BLUE_Fx,
  real_T *rtd_BLUE_Fy, real_T *rtd_BLUE_Tz, real_T *rtd_Float_State);
extern void CLVF_SubPhase2_d(P_SubPhase2_CLVF_e_T *localP, real_T
  *rtd_Float_State, real_T *rtd_RED_Fx, real_T *rtd_RED_Fy, real_T *rtd_RED_Tz,
  real_T *rtd_RED_Tz_Elbow, real_T *rtd_RED_Tz_RW, real_T *rtd_RED_Tz_Shoulder,
  real_T *rtd_RED_Tz_Wrist);
extern void C_ChangeBLUEBehavior_a_Init(B_ChangeBLUEBehavior_CLVF_k_T *localB,
  DW_ChangeBLUEBehavior_CLVF_i_T *localDW, P_ChangeBLUEBehavior_CLVF_c_T *localP);
extern void C_ChangeBLUEBehavior_Update(B_ChangeBLUEBehavior_CLVF_k_T *localB,
  DW_ChangeBLUEBehavior_CLVF_i_T *localDW);
extern void CLVF_ChangeBLUEBehavior_g(B_ChangeBLUEBehavior_CLVF_k_T *localB,
  DW_ChangeBLUEBehavior_CLVF_i_T *localDW, P_ChangeBLUEBehavior_CLVF_c_T *localP,
  real_T *rtd_BLUE_Fx, real_T *rtd_BLUE_Fy, real_T *rtd_BLUE_Px, real_T
  *rtd_BLUE_Py, real_T *rtd_BLUE_Rz, real_T *rtd_BLUE_Tz, real_T
  *rtd_Float_State);
extern void CLVF_step0(void);
extern void CLVF_step2(void);

#endif                                 // RTW_HEADER_CLVF_private_h_
