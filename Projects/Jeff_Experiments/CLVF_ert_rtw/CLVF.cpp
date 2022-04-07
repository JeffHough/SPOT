//
//  CLVF.cpp
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


#include "CLVF.h"
#include "CLVF_private.h"

// Block signals (default storage)
B_CLVF_T CLVF_B;

// Block states (default storage)
DW_CLVF_T CLVF_DW;

// Previous zero-crossings (trigger) states
PrevZCX_CLVF_T CLVF_PrevZCX;

// Real-time model
RT_MODEL_CLVF_T CLVF_M_ = RT_MODEL_CLVF_T();
RT_MODEL_CLVF_T *const CLVF_M = &CLVF_M_;

// Forward declaration for local functions
static void CLVF_AHRSFilterBase_resetImpl(fusion_simulink_ahrsfilter_CL_T *obj);
static void IMUFusionCommon_computeAngularV(const real_T gfast[3], const real_T
  offset[3], real_T av[3]);
static void CLVF_NED_ecompass(const real_T a[3], const real_T m[3], real_T R[9],
  B_AHRS2_CLVF_T *localB);
static void CLVF_quaternioncg_quaternioncg(const real_T varargin_1[9], real_T
  *obj_a, real_T *obj_b, real_T *obj_c, real_T *obj_d, B_AHRS2_CLVF_T *localB);
static void CLV_quaternioncg_quaternioncg_f(const real_T varargin_1[3], real_T
  *obj_a, real_T *obj_b, real_T *obj_c, real_T *obj_d, B_AHRS2_CLVF_T *localB);
static void CLVF_quaternionBase_mtimes(real_T x_a, real_T x_b, real_T x_c,
  real_T x_d, real_T y_a, real_T y_b, real_T y_c, real_T y_d, real_T *o_a,
  real_T *o_b, real_T *o_c, real_T *o_d);
static void IMUFusionCommon_predictOrientat(const
  fusion_simulink_ahrsfilter_CL_T *obj, const real_T gfast[3], const real_T
  offset[3], real_T qorient_a, real_T qorient_b, real_T qorient_c, real_T
  qorient_d, real_T *b_qorient_a, real_T *b_qorient_b, real_T *b_qorient_c,
  real_T *b_qorient_d, B_AHRS2_CLVF_T *localB);
static void CLVF_quaternionBase_rotmat(real_T q_a, real_T q_b, real_T q_c,
  real_T q_d, real_T r[9], B_AHRS2_CLVF_T *localB);
static void CLVF_IMUFusionCommon_buildHPart(const real_T v[3], real_T h[9],
  B_AHRS2_CLVF_T *localB);
static real_T CLVF_norm(const real_T x[3], B_AHRS2_CLVF_T *localB);
static void CLVF_quaternionBase_conj(real_T q_a, real_T q_b, real_T q_c, real_T
  q_d, real_T *b_q_a, real_T *b_q_b, real_T *b_q_c, real_T *b_q_d);
static void CLVF_quaternionBase_uminus(real_T obj_a, real_T obj_b, real_T obj_c,
  real_T obj_d, real_T *b_obj_a, real_T *b_obj_b, real_T *b_obj_c, real_T
  *b_obj_d);
static void CLVF_quaternionBase_normalize(real_T q_a, real_T q_b, real_T q_c,
  real_T q_d, real_T *b_q_a, real_T *b_q_b, real_T *b_q_c, real_T *b_q_d);
static void CLVF_quaternioncg_parenAssign(real_T rhs_a, real_T rhs_b, real_T
  rhs_c, real_T rhs_d, real_T *o_a, real_T *o_b, real_T *o_c, real_T *o_d);
static void CLVF_ahrsfilter_stepImpl(fusion_simulink_ahrsfilter_CL_T *obj, const
  real_T accelIn[3], const real_T gyroIn[3], const real_T magIn[3], real_T
  orientOut[4], real_T av[3], B_AHRS2_CLVF_T *localB);

// Forward declaration for local functions
static real_T CLVF_rt_atan2d_snf(real_T u0, real_T u1);
static real_T CLVF_rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u);
static real_T CLVF_rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u);
static void CLVF_rt_invd3x3_snf(const real_T u[9], real_T y[9]);
static void CLVF_SystemCore_release(const codertarget_linux_blocks_Digi_T *obj);
static void CLVF_SystemCore_delete(const codertarget_linux_blocks_Digi_T *obj);
static void matlabCodegenHandle_matlabCodeg(codertarget_linux_blocks_Digi_T *obj);
static void CLVF_SystemCore_release_g(const codertarget_linux_blocks_Digi_T *obj);
static void CLVF_SystemCore_delete_g(const codertarget_linux_blocks_Digi_T *obj);
static void matlabCodegenHandle_matlabCod_g(codertarget_linux_blocks_Digi_T *obj);
static void SystemCore_rele_gffipocpdxml04g(const PhaseSpace_ALL_CLVF_T *obj);
static void SystemCore_dele_gffipocpdxml04g(const PhaseSpace_ALL_CLVF_T *obj);
static void matlabCodegenHa_gffipocpdxml04g(PhaseSpace_ALL_CLVF_T *obj);
static void C_SystemCore_rele_a(const raspi_internal_lsm9ds1Block_C_T *obj);
static void C_SystemCore_dele_a(const raspi_internal_lsm9ds1Block_C_T *obj);
static void CLVF_matlabCodegenHa_a(raspi_internal_lsm9ds1Block_C_T *obj);
static void C_SystemCore_rele_b(j_codertarget_raspi_internal__T *obj);
static void C_SystemCore_dele_b(j_codertarget_raspi_internal__T *obj);
static void CLVF_matlabCodegenHa_b(j_codertarget_raspi_internal__T *obj);
static void C_SystemCore_rele_f(i_codertarget_raspi_internal__T *obj);
static void C_SystemCore_dele_f(i_codertarget_raspi_internal__T *obj);
static void CLVF_matlabCodegenHa_f(i_codertarget_raspi_internal__T *obj);
static void CLVF_SystemCore_release_gf(const raspi_internal_PWMBlock_CLVF_T *obj);
static void CLVF_SystemCore_delete_gf(const raspi_internal_PWMBlock_CLVF_T *obj);
static void matlabCodegenHandle_matlabCo_gf(raspi_internal_PWMBlock_CLVF_T *obj);
static void SystemCore_release_gffipocpdxm(const codertarget_linux_blocks_Di_g_T
  *obj);
static void C_SystemCore_delete_gffipocpdxm(const
  codertarget_linux_blocks_Di_g_T *obj);
static void matlabCodegenHandle_gffipocpdxm(codertarget_linux_blocks_Di_g_T *obj);
static void SystemCore_release_gffipocpdxml(const
  codertarget_linux_blocks_Digi_T *obj);
static void SystemCore_delete_gffipocpdxml(const codertarget_linux_blocks_Digi_T
  *obj);
static void matlabCodegenHandl_gffipocpdxml(codertarget_linux_blocks_Digi_T *obj);
static void SystemCore_relea_gffipocpdxml04(const
  codertarget_linux_blocks_Digi_T *obj);
static void SystemCore_delet_gffipocpdxml04(const
  codertarget_linux_blocks_Digi_T *obj);
static void matlabCodegenHan_gffipocpdxml04(codertarget_linux_blocks_Digi_T *obj);
static raspi_internal_lsm9ds1Block_C_T *CLVF_lsm9ds1Block_lsm9ds1Block
  (raspi_internal_lsm9ds1Block_C_T *obj);
static void CLVF_SystemCore_setup(raspi_internal_lsm9ds1Block_C_T *obj);
static void rate_monotonic_scheduler(void);

//
// Set which subrates need to run this base step (base rate always runs).
// This function must be called prior to calling the model step function
// in order to "remember" which rates need to run this base step.  The
// buffering of events allows for overlapping preemption.
//
void CLVF_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  // Task runs when its counter is zero, computed via rtmStepTask macro
  eventFlags[2] = ((boolean_T)rtmStepTask(CLVF_M, 2));
}

//
//   This function updates active task flag for each subrate
// and rate transition flags for tasks that exchange data.
// The function assumes rate-monotonic multitasking scheduler.
// The function must be called at model base rate so that
// the generated code self-manages all its subrates and rate
// transition flags.
//
static void rate_monotonic_scheduler(void)
{
  // To ensure a deterministic data transfer between two rates,
  //  data is transferred at the priority of a fast task and the frequency
  //  of the slow task.  The following flags indicate when the data transfer
  //  happens.  That is, a rate interaction flag is set true when both rates
  //  will run, and false otherwise.


  // tid 1 shares data with slower tid rate: 2
  if (CLVF_M->Timing.TaskCounters.TID[1] == 0) {
    CLVF_M->Timing.RateInteraction.TID1_2 = (CLVF_M->Timing.TaskCounters.TID[2] ==
      0);
  }

  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (CLVF_M->Timing.TaskCounters.TID[2])++;
  if ((CLVF_M->Timing.TaskCounters.TID[2]) > 1) {// Sample time: [0.1s, 0.0s]
    CLVF_M->Timing.TaskCounters.TID[2] = 0;
  }
}

//
// Output and update for atomic system:
//    '<S27>/MATLAB Function2'
//    '<S36>/MATLAB Function2'
//    '<S45>/MATLAB Function2'
//
void CLVF_MATLABFunction2(const real_T rtu_ThrustPer[8],
  B_MATLABFunction2_CLVF_T *localB)
{
  real_T ex;
  int32_T idx;
  int32_T k;
  boolean_T tmp;
  boolean_T exitg1;

  // MATLAB Function 'From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/MATLAB Function2': '<S31>:1' 
  // '<S31>:1:3' ValveTime       = 0.007;
  // '<S31>:1:4' TControl        = 0.1;
  // '<S31>:1:5' ThrustPer_Final = zeros(8,1);
  // '<S31>:1:7' if max(ThrustPer) > 1
  tmp = rtIsNaN(rtu_ThrustPer[0]);
  if (!tmp) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 9)) {
      if (!rtIsNaN(rtu_ThrustPer[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = rtu_ThrustPer[0];
  } else {
    ex = rtu_ThrustPer[idx - 1];
    while (idx + 1 < 9) {
      if (ex < rtu_ThrustPer[idx]) {
        ex = rtu_ThrustPer[idx];
      }

      idx++;
    }
  }

  if (ex > 1.0) {
    // '<S31>:1:8' ThrustPer_Sat = ThrustPer/max(ThrustPer);
    if (!tmp) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 9)) {
        if (!rtIsNaN(rtu_ThrustPer[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      ex = rtu_ThrustPer[0];
    } else {
      ex = rtu_ThrustPer[idx - 1];
      while (idx + 1 < 9) {
        if (ex < rtu_ThrustPer[idx]) {
          ex = rtu_ThrustPer[idx];
        }

        idx++;
      }
    }

    for (idx = 0; idx < 8; idx++) {
      localB->ThrustPer_Sat[idx] = rtu_ThrustPer[idx] / ex;
    }
  } else {
    // '<S31>:1:9' else
    // '<S31>:1:10' ThrustPer_Sat = ThrustPer;
    memcpy(&localB->ThrustPer_Sat[0], &rtu_ThrustPer[0], sizeof(real_T) << 3U);
  }

  // '<S31>:1:13' for i = 1:8
  for (idx = 0; idx < 8; idx++) {
    // '<S31>:1:14' if ThrustPer_Sat(i) > ValveTime/TControl
    if (localB->ThrustPer_Sat[idx] > 0.069999999999999993) {
      // '<S31>:1:15' ThrustPer_Final(i) = ThrustPer_Sat(i);
      localB->ThrustPer_Final[idx] = localB->ThrustPer_Sat[idx];
    } else {
      // '<S31>:1:16' else
      // '<S31>:1:17' ThrustPer_Final(i) = 0;
      localB->ThrustPer_Final[idx] = 0.0;
    }
  }
}

//
// Output and update for atomic system:
//    '<S32>/Create Rotation Matrix'
//    '<S41>/Create Rotation Matrix'
//    '<S50>/Create Rotation Matrix'
//    '<S169>/Create Rotation Matrix'
//    '<S170>/Create Rotation Matrix'
//    '<S171>/Create Rotation Matrix'
//
void CLVF_CreateRotationMatrix(real_T rtu_Rz, B_CreateRotationMatrix_CLVF_T
  *localB)
{
  real_T r;
  real_T t;
  real_T C_bI_idx_0;
  real_T C_bI_idx_2;
  real_T C_bI_idx_1;

  // MATLAB Function 'From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/Rotate Forces to Inertial/Create Rotation Matrix': '<S34>:1' 
  // '<S34>:1:3' C_bI = [  cos(Rz) sin(Rz)
  // '<S34>:1:4'          -sin(Rz) cos(Rz) ];
  r = sin(rtu_Rz);
  t = cos(rtu_Rz);
  C_bI_idx_0 = t;
  C_bI_idx_2 = r;
  C_bI_idx_1 = -r;

  // '<S34>:1:6' C_Ib = inv(C_bI);
  if (fabs(-r) > fabs(t)) {
    r = t / -r;
    t = 1.0 / (r * t - C_bI_idx_2);
    localB->C_Ib[0] = C_bI_idx_0 / C_bI_idx_1 * t;
    localB->C_Ib[1] = -t;
    localB->C_Ib[2] = -C_bI_idx_2 / C_bI_idx_1 * t;
    localB->C_Ib[3] = r * t;
  } else {
    r = -r / t;
    t = 1.0 / (t - r * C_bI_idx_2);
    localB->C_Ib[0] = C_bI_idx_0 / C_bI_idx_0 * t;
    localB->C_Ib[1] = -r * t;
    localB->C_Ib[2] = -C_bI_idx_2 / C_bI_idx_0 * t;
    localB->C_Ib[3] = t;
  }
}

//
// Output and update for atomic system:
//    '<S28>/Create Rotation Matrix'
//    '<S37>/Create Rotation Matrix'
//    '<S46>/Create Rotation Matrix'
//
void CLVF_CreateRotationMatrix_h(real_T rtu_Rz, B_CreateRotationMatrix_CLVF_i_T *
  localB)
{
  real_T C_bI_tmp;
  real_T C_bI_tmp_0;

  // MATLAB Function 'From Force//Torque to PWM Signal/Change BLACK Behavior/Rotate Forces to Body/Create Rotation Matrix': '<S35>:1' 
  // '<S35>:1:3' C_bI = [  cos(Rz) sin(Rz)
  // '<S35>:1:4'          -sin(Rz) cos(Rz) ];
  C_bI_tmp = sin(rtu_Rz);
  C_bI_tmp_0 = cos(rtu_Rz);
  localB->C_bI[0] = C_bI_tmp_0;
  localB->C_bI[2] = C_bI_tmp;
  localB->C_bI[1] = -C_bI_tmp;
  localB->C_bI[3] = C_bI_tmp_0;
}

//
// Output and update for atomic system:
//    '<S36>/MATLAB Function'
//    '<S36>/MATLAB Function1'
//
void CLVF_MATLABFunction(B_MATLABFunction_CLVF_T *localB)
{
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  int32_T H_tmp;
  static const int8_T b[8] = { -1, -1, 0, 0, 1, 1, 0, 0 };

  static const int8_T c[8] = { 0, 0, 1, 1, 0, 0, -1, -1 };

  // MATLAB Function 'From Force//Torque to PWM Signal/Change BLUE Behavior/Calculate Thruster  ON//OFF/MATLAB Function': '<S38>:1' 
  // '<S38>:1:3' Vec1 = [ -1
  // '<S38>:1:4'          -1
  // '<S38>:1:5'           0
  // '<S38>:1:6'           0
  // '<S38>:1:7'           1
  // '<S38>:1:8'           1
  // '<S38>:1:9'           0
  // '<S38>:1:10'           0 ];
  // '<S38>:1:12' Vec2 = [  0
  // '<S38>:1:13'           0
  // '<S38>:1:14'           1
  // '<S38>:1:15'           1
  // '<S38>:1:16'           0
  // '<S38>:1:17'           0
  // '<S38>:1:18'          -1
  // '<S38>:1:19'          -1 ];
  // '<S38>:1:21' Vec3 = thruster_dist2CG_BLUE./1000;
  // '<S38>:1:23' Mat1 = [Vec1, Vec2, Vec3]';
  // '<S38>:1:25' Mat2 = diag((F_thrusters_BLUE./2));
  for (i = 0; i < 8; i++) {
    localB->v[i] = CLVF_P.F_thrusters_BLUE[i] / 2.0;
  }

  memset(&localB->Mat2[0], 0, sizeof(real_T) << 6U);

  // '<S38>:1:27' H    = Mat1*Mat2;
  for (i = 0; i < 8; i++) {
    localB->Mat2[i + (i << 3)] = localB->v[i];
    localB->b[3 * i] = b[i];
    localB->b[3 * i + 1] = c[i];
    localB->b[3 * i + 2] = CLVF_P.thruster_dist2CG_BLUE[i] / 1000.0;
  }

  for (i = 0; i < 8; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      H_tmp = i_0 + 3 * i;
      localB->H[H_tmp] = 0.0;
      for (i_1 = 0; i_1 < 8; i_1++) {
        localB->H[H_tmp] += localB->b[3 * i_1 + i_0] * localB->Mat2[(i << 3) +
          i_1];
      }
    }
  }
}

static void CLVF_AHRSFilterBase_resetImpl(fusion_simulink_ahrsfilter_CL_T *obj)
{
  real_T accelMeasNoiseVar;
  real_T magMeasNoiseVar;
  static const real_T tmp[144] = { 6.0923483957341713E-6, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0923483957341713E-6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0923483957341713E-6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.6154354946677142E-5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.6154354946677142E-5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    7.6154354946677142E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6 };

  int32_T i;
  int32_T tmp_0;
  int32_T tmp_1;
  static const int8_T tmp_2[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  obj->pOrientPost.a = 1.0;
  obj->pOrientPost.b = 0.0;
  obj->pOrientPost.c = 0.0;
  obj->pOrientPost.d = 0.0;
  obj->pGyroOffset[0] = 0.0;
  obj->pMagVec[0] = 0.0;
  obj->pGyroOffset[1] = 0.0;
  obj->pMagVec[1] = 0.0;
  obj->pGyroOffset[2] = 0.0;
  obj->pMagVec[2] = 0.0;
  obj->pMagVec[0] = obj->ExpectedMagneticFieldStrength;
  magMeasNoiseVar = obj->pKalmanPeriod * obj->pKalmanPeriod *
    (obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
  accelMeasNoiseVar = magMeasNoiseVar + (obj->AccelerometerNoise +
    obj->LinearAccelerationNoise);
  magMeasNoiseVar += obj->MagnetometerNoise + obj->MagneticDisturbanceNoise;
  memset(&obj->pQv[0], 0, 36U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    tmp_0 = tmp_2[3 * i];
    obj->pQv[6 * i] = accelMeasNoiseVar * static_cast<real_T>(tmp_0);
    tmp_1 = 6 * (i + 3);
    obj->pQv[tmp_1 + 3] = magMeasNoiseVar * static_cast<real_T>(tmp_0);
    tmp_0 = tmp_2[3 * i + 1];
    obj->pQv[6 * i + 1] = accelMeasNoiseVar * static_cast<real_T>(tmp_0);
    obj->pQv[tmp_1 + 4] = magMeasNoiseVar * static_cast<real_T>(tmp_0);
    tmp_0 = tmp_2[3 * i + 2];
    obj->pQv[6 * i + 2] = accelMeasNoiseVar * static_cast<real_T>(tmp_0);
    obj->pQv[tmp_1 + 5] = magMeasNoiseVar * static_cast<real_T>(tmp_0);
  }

  memcpy(&obj->pQw[0], &tmp[0], 144U * sizeof(real_T));
  obj->pLinAccelPost[0] = 0.0;
  obj->pLinAccelPost[1] = 0.0;
  obj->pLinAccelPost[2] = 0.0;
  obj->pFirstTime = true;
}

static void IMUFusionCommon_computeAngularV(const real_T gfast[3], const real_T
  offset[3], real_T av[3])
{
  av[0] = gfast[0] - offset[0];
  av[1] = gfast[1] - offset[1];
  av[2] = gfast[2] - offset[2];
}

static void CLVF_NED_ecompass(const real_T a[3], const real_T m[3], real_T R[9],
  B_AHRS2_CLVF_T *localB)
{
  boolean_T nanPageIdx;
  boolean_T y[3];
  int32_T iy;
  int32_T xpageoffset;
  boolean_T exitg1;
  localB->Reast[0] = a[1] * m[2] - a[2] * m[1];
  localB->Reast[1] = a[2] * m[0] - a[0] * m[2];
  localB->Reast[2] = a[0] * m[1] - a[1] * m[0];
  R[6] = a[0];
  R[3] = localB->Reast[0];
  R[7] = a[1];
  R[4] = localB->Reast[1];
  R[8] = a[2];
  R[5] = localB->Reast[2];
  R[0] = localB->Reast[1] * a[2] - localB->Reast[2] * a[1];
  R[1] = localB->Reast[2] * a[0] - localB->Reast[0] * a[2];
  R[2] = localB->Reast[0] * a[1] - localB->Reast[1] * a[0];
  for (iy = 0; iy < 9; iy++) {
    localB->x[iy] = R[iy] * R[iy];
  }

  for (iy = 0; iy < 3; iy++) {
    xpageoffset = iy * 3;
    localB->Reast[iy] = localB->x[xpageoffset + 2] + (localB->x[xpageoffset + 1]
      + localB->x[xpageoffset]);
  }

  localB->Reast[0] = sqrt(localB->Reast[0]);
  localB->Reast[1] = sqrt(localB->Reast[1]);
  localB->Reast[2] = sqrt(localB->Reast[2]);
  memcpy(&localB->x[0], &R[0], 9U * sizeof(real_T));
  for (iy = 0; iy < 3; iy++) {
    R[3 * iy] = localB->x[3 * iy] / localB->Reast[iy];
    xpageoffset = 3 * iy + 1;
    R[xpageoffset] = localB->x[xpageoffset] / localB->Reast[iy];
    xpageoffset = 3 * iy + 2;
    R[xpageoffset] = localB->x[xpageoffset] / localB->Reast[iy];
  }

  for (iy = 0; iy < 9; iy++) {
    localB->b[iy] = rtIsNaN(R[iy]);
  }

  y[0] = false;
  y[1] = false;
  y[2] = false;
  xpageoffset = 1;
  exitg1 = false;
  while ((!exitg1) && (xpageoffset <= 3)) {
    if (!localB->b[xpageoffset - 1]) {
      xpageoffset++;
    } else {
      y[0] = true;
      exitg1 = true;
    }
  }

  xpageoffset = 4;
  exitg1 = false;
  while ((!exitg1) && (xpageoffset <= 6)) {
    if (!localB->b[xpageoffset - 1]) {
      xpageoffset++;
    } else {
      y[1] = true;
      exitg1 = true;
    }
  }

  xpageoffset = 7;
  exitg1 = false;
  while ((!exitg1) && (xpageoffset <= 9)) {
    if (!localB->b[xpageoffset - 1]) {
      xpageoffset++;
    } else {
      y[2] = true;
      exitg1 = true;
    }
  }

  nanPageIdx = false;
  iy = 0;
  exitg1 = false;
  while ((!exitg1) && (iy < 3)) {
    if (!y[iy]) {
      iy++;
    } else {
      nanPageIdx = true;
      exitg1 = true;
    }
  }

  if (nanPageIdx) {
    memset(&R[0], 0, 9U * sizeof(real_T));
    R[0] = 1.0;
    R[4] = 1.0;
    R[8] = 1.0;
  }
}

static void CLVF_quaternioncg_quaternioncg(const real_T varargin_1[9], real_T
  *obj_a, real_T *obj_b, real_T *obj_c, real_T *obj_d, B_AHRS2_CLVF_T *localB)
{
  int32_T b_k;
  int32_T b_idx;
  int32_T b;
  boolean_T exitg1;
  localB->tr = (varargin_1[0] + varargin_1[4]) + varargin_1[8];
  localB->psquared[0] = (localB->tr * 2.0 + 1.0) - localB->tr;
  localB->psquared[1] = (2.0 * varargin_1[0] + 1.0) - localB->tr;
  localB->psquared[2] = (2.0 * varargin_1[4] + 1.0) - localB->tr;
  localB->psquared[3] = (2.0 * varargin_1[8] + 1.0) - localB->tr;
  if (!rtIsNaN(localB->psquared[0])) {
    b_idx = 1;
  } else {
    b_idx = 0;
    b_k = 2;
    exitg1 = false;
    while ((!exitg1) && (b_k < 5)) {
      if (!rtIsNaN(localB->psquared[b_k - 1])) {
        b_idx = b_k;
        exitg1 = true;
      } else {
        b_k++;
      }
    }
  }

  if (b_idx == 0) {
    localB->tr = localB->psquared[0];
    b_idx = 1;
  } else {
    localB->tr = localB->psquared[b_idx - 1];
    b = b_idx;
    for (b_k = b + 1; b_k < 5; b_k++) {
      localB->d = localB->psquared[b_k - 1];
      if (localB->tr < localB->d) {
        localB->tr = localB->d;
        b_idx = b_k;
      }
    }
  }

  switch (b_idx) {
   case 1:
    localB->tr = sqrt(localB->tr);
    *obj_a = 0.5 * localB->tr;
    localB->tr = 0.5 / localB->tr;
    *obj_b = (varargin_1[7] - varargin_1[5]) * localB->tr;
    *obj_c = (varargin_1[2] - varargin_1[6]) * localB->tr;
    *obj_d = (varargin_1[3] - varargin_1[1]) * localB->tr;
    break;

   case 2:
    localB->tr = sqrt(localB->tr);
    *obj_b = 0.5 * localB->tr;
    localB->tr = 0.5 / localB->tr;
    *obj_a = (varargin_1[7] - varargin_1[5]) * localB->tr;
    *obj_c = (varargin_1[3] + varargin_1[1]) * localB->tr;
    *obj_d = (varargin_1[2] + varargin_1[6]) * localB->tr;
    break;

   case 3:
    localB->tr = sqrt(localB->tr);
    *obj_c = 0.5 * localB->tr;
    localB->tr = 0.5 / localB->tr;
    *obj_a = (varargin_1[2] - varargin_1[6]) * localB->tr;
    *obj_b = (varargin_1[3] + varargin_1[1]) * localB->tr;
    *obj_d = (varargin_1[7] + varargin_1[5]) * localB->tr;
    break;

   default:
    localB->tr = sqrt(localB->tr);
    *obj_d = 0.5 * localB->tr;
    localB->tr = 0.5 / localB->tr;
    *obj_a = (varargin_1[3] - varargin_1[1]) * localB->tr;
    *obj_b = (varargin_1[2] + varargin_1[6]) * localB->tr;
    *obj_c = (varargin_1[7] + varargin_1[5]) * localB->tr;
    break;
  }

  if (*obj_a < 0.0) {
    *obj_a = -*obj_a;
    *obj_b = -*obj_b;
    *obj_c = -*obj_c;
    *obj_d = -*obj_d;
  }
}

static void CLV_quaternioncg_quaternioncg_f(const real_T varargin_1[3], real_T
  *obj_a, real_T *obj_b, real_T *obj_c, real_T *obj_d, B_AHRS2_CLVF_T *localB)
{
  real_T st;
  *obj_a = 1.0;
  *obj_b = 0.0;
  *obj_c = 0.0;
  *obj_d = 0.0;
  localB->theta = sqrt((varargin_1[0] * varargin_1[0] + varargin_1[1] *
                        varargin_1[1]) + varargin_1[2] * varargin_1[2]);
  st = sin(localB->theta / 2.0);
  if (localB->theta != 0.0) {
    *obj_a = cos(localB->theta / 2.0);
    *obj_b = varargin_1[0] / localB->theta * st;
    *obj_c = varargin_1[1] / localB->theta * st;
    *obj_d = varargin_1[2] / localB->theta * st;
  }
}

static void CLVF_quaternionBase_mtimes(real_T x_a, real_T x_b, real_T x_c,
  real_T x_d, real_T y_a, real_T y_b, real_T y_c, real_T y_d, real_T *o_a,
  real_T *o_b, real_T *o_c, real_T *o_d)
{
  *o_a = ((x_a * y_a - x_b * y_b) - x_c * y_c) - x_d * y_d;
  *o_b = ((x_a * y_b + x_b * y_a) + x_c * y_d) - x_d * y_c;
  *o_c = ((x_a * y_c - x_b * y_d) + x_c * y_a) + x_d * y_b;
  *o_d = ((x_a * y_d + x_b * y_c) - x_c * y_b) + x_d * y_a;
}

static void IMUFusionCommon_predictOrientat(const
  fusion_simulink_ahrsfilter_CL_T *obj, const real_T gfast[3], const real_T
  offset[3], real_T qorient_a, real_T qorient_b, real_T qorient_c, real_T
  qorient_d, real_T *b_qorient_a, real_T *b_qorient_b, real_T *b_qorient_c,
  real_T *b_qorient_d, B_AHRS2_CLVF_T *localB)
{
  localB->c[0] = (gfast[0] - offset[0]) * obj->pSensorPeriod;
  localB->c[1] = (gfast[1] - offset[1]) * obj->pSensorPeriod;
  localB->c[2] = (gfast[2] - offset[2]) * obj->pSensorPeriod;
  CLV_quaternioncg_quaternioncg_f(localB->c, &localB->assign_temp_a_k,
    &localB->assign_temp_b_c, &localB->assign_temp_c_b, &localB->assign_temp_d_p,
    localB);
  CLVF_quaternionBase_mtimes(qorient_a, qorient_b, qorient_c, qorient_d,
    localB->assign_temp_a_k, localB->assign_temp_b_c, localB->assign_temp_c_b,
    localB->assign_temp_d_p, b_qorient_a, b_qorient_b, b_qorient_c, b_qorient_d);
  if (*b_qorient_a < 0.0) {
    *b_qorient_a = -*b_qorient_a;
    *b_qorient_b = -*b_qorient_b;
    *b_qorient_c = -*b_qorient_c;
    *b_qorient_d = -*b_qorient_d;
  }
}

static void CLVF_quaternionBase_rotmat(real_T q_a, real_T q_b, real_T q_c,
  real_T q_d, real_T r[9], B_AHRS2_CLVF_T *localB)
{
  real_T n;
  n = sqrt(((q_a * q_a + q_b * q_b) + q_c * q_c) + q_d * q_d);
  q_a /= n;
  q_b /= n;
  q_c /= n;
  q_d /= n;
  n = q_a * q_b * 2.0;
  localB->ac2 = q_a * q_c * 2.0;
  localB->ad2 = q_a * q_d * 2.0;
  localB->bc2 = q_b * q_c * 2.0;
  localB->bd2 = q_b * q_d * 2.0;
  localB->cd2 = q_c * q_d * 2.0;
  localB->aasq = q_a * q_a * 2.0 - 1.0;
  r[0] = q_b * q_b * 2.0 + localB->aasq;
  r[3] = localB->bc2 + localB->ad2;
  r[6] = localB->bd2 - localB->ac2;
  r[1] = localB->bc2 - localB->ad2;
  r[4] = q_c * q_c * 2.0 + localB->aasq;
  r[7] = localB->cd2 + n;
  r[2] = localB->bd2 + localB->ac2;
  r[5] = localB->cd2 - n;
  r[8] = q_d * q_d * 2.0 + localB->aasq;
}

static void CLVF_IMUFusionCommon_buildHPart(const real_T v[3], real_T h[9],
  B_AHRS2_CLVF_T *localB)
{
  int32_T i;
  int32_T h_tmp;
  memset(&h[0], 0, 9U * sizeof(real_T));
  h[3] = v[2];
  h[6] = -v[1];
  h[7] = v[0];
  for (i = 0; i < 3; i++) {
    localB->h[3 * i] = h[3 * i];
    h_tmp = 3 * i + 1;
    localB->h[h_tmp] = h[h_tmp] - h[i + 3];
    h_tmp = 3 * i + 2;
    localB->h[h_tmp] = h[h_tmp] - h[i + 6];
  }

  memcpy(&h[0], &localB->h[0], 9U * sizeof(real_T));
}

static real_T CLVF_norm(const real_T x[3], B_AHRS2_CLVF_T *localB)
{
  real_T y;
  real_T t;
  localB->scale = 3.3121686421112381E-170;
  localB->absxk = fabs(x[0]);
  if (localB->absxk > 3.3121686421112381E-170) {
    y = 1.0;
    localB->scale = localB->absxk;
  } else {
    t = localB->absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  localB->absxk = fabs(x[1]);
  if (localB->absxk > localB->scale) {
    t = localB->scale / localB->absxk;
    y = y * t * t + 1.0;
    localB->scale = localB->absxk;
  } else {
    t = localB->absxk / localB->scale;
    y += t * t;
  }

  localB->absxk = fabs(x[2]);
  if (localB->absxk > localB->scale) {
    t = localB->scale / localB->absxk;
    y = y * t * t + 1.0;
    localB->scale = localB->absxk;
  } else {
    t = localB->absxk / localB->scale;
    y += t * t;
  }

  return localB->scale * sqrt(y);
}

static void CLVF_quaternionBase_conj(real_T q_a, real_T q_b, real_T q_c, real_T
  q_d, real_T *b_q_a, real_T *b_q_b, real_T *b_q_c, real_T *b_q_d)
{
  *b_q_a = q_a;
  *b_q_b = -q_b;
  *b_q_c = -q_c;
  *b_q_d = -q_d;
}

static void CLVF_quaternionBase_uminus(real_T obj_a, real_T obj_b, real_T obj_c,
  real_T obj_d, real_T *b_obj_a, real_T *b_obj_b, real_T *b_obj_c, real_T
  *b_obj_d)
{
  *b_obj_a = -obj_a;
  *b_obj_b = -obj_b;
  *b_obj_c = -obj_c;
  *b_obj_d = -obj_d;
}

static void CLVF_quaternionBase_normalize(real_T q_a, real_T q_b, real_T q_c,
  real_T q_d, real_T *b_q_a, real_T *b_q_b, real_T *b_q_c, real_T *b_q_d)
{
  real_T n;
  n = sqrt(((q_a * q_a + q_b * q_b) + q_c * q_c) + q_d * q_d);
  *b_q_a = q_a / n;
  *b_q_b = q_b / n;
  *b_q_c = q_c / n;
  *b_q_d = q_d / n;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
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

static void CLVF_quaternioncg_parenAssign(real_T rhs_a, real_T rhs_b, real_T
  rhs_c, real_T rhs_d, real_T *o_a, real_T *o_b, real_T *o_c, real_T *o_d)
{
  *o_a = rhs_a;
  *o_b = rhs_b;
  *o_c = rhs_c;
  *o_d = rhs_d;
}

static void CLVF_ahrsfilter_stepImpl(fusion_simulink_ahrsfilter_CL_T *obj, const
  real_T accelIn[3], const real_T gyroIn[3], const real_T magIn[3], real_T
  orientOut[4], real_T av[3], B_AHRS2_CLVF_T *localB)
{
  boolean_T isJamming;
  int32_T jp;
  int32_T jj;
  int32_T jp1j;
  int32_T c;
  int32_T ix;
  int32_T iy;
  int32_T jA;
  int32_T jy;
  int32_T c_0;
  int32_T H_tmp;
  int32_T H_tmp_0;
  static const int8_T tmp[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T tmp_0[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  IMUFusionCommon_computeAngularV(gyroIn, obj->pGyroOffset, av);
  if (obj->pFirstTime) {
    obj->pFirstTime = false;
    CLVF_NED_ecompass(accelIn, magIn, localB->Rprior, localB);
    CLVF_quaternioncg_quaternioncg(localB->Rprior, &obj->pOrientPost.a,
      &obj->pOrientPost.b, &obj->pOrientPost.c, &obj->pOrientPost.d, localB);
  }

  for (c_0 = 0; c_0 < 3; c_0++) {
    localB->gravityAccelGyroDiff[c_0] = obj->pGyroOffset[c_0];
  }

  IMUFusionCommon_predictOrientat(obj, gyroIn, localB->gravityAccelGyroDiff,
    obj->pOrientPost.a, obj->pOrientPost.b, obj->pOrientPost.c,
    obj->pOrientPost.d, &localB->temp, &localB->smax, &localB->s,
    &localB->gyroOffsetErr_idx_0, localB);
  obj->pOrientPrior.d = localB->gyroOffsetErr_idx_0;
  obj->pOrientPrior.c = localB->s;
  obj->pOrientPrior.b = localB->smax;
  obj->pOrientPrior.a = localB->temp;
  CLVF_quaternionBase_rotmat(obj->pOrientPrior.a, obj->pOrientPrior.b,
    obj->pOrientPrior.c, obj->pOrientPrior.d, localB->Rprior, localB);
  obj->pLinAccelPrior[0] = obj->LinearAccelerationDecayFactor *
    obj->pLinAccelPost[0];
  localB->gravityAccelGyroDiff[0] = (accelIn[0] + obj->pLinAccelPrior[0]) -
    localB->Rprior[6];
  obj->pLinAccelPrior[1] = obj->LinearAccelerationDecayFactor *
    obj->pLinAccelPost[1];
  localB->gravityAccelGyroDiff[1] = (accelIn[1] + obj->pLinAccelPrior[1]) -
    localB->Rprior[7];
  obj->pLinAccelPrior[2] = obj->LinearAccelerationDecayFactor *
    obj->pLinAccelPost[2];
  localB->gravityAccelGyroDiff[2] = (accelIn[2] + obj->pLinAccelPrior[2]) -
    localB->Rprior[8];
  CLVF_IMUFusionCommon_buildHPart(&localB->Rprior[6], localB->h1, localB);
  for (c_0 = 0; c_0 < 3; c_0++) {
    localB->offDiag[c_0] = localB->Rprior[c_0 + 6] * obj->pMagVec[2] +
      (localB->Rprior[c_0 + 3] * obj->pMagVec[1] + localB->Rprior[c_0] *
       obj->pMagVec[0]);
  }

  CLVF_IMUFusionCommon_buildHPart(localB->offDiag, localB->Rprior, localB);
  for (c_0 = 0; c_0 < 3; c_0++) {
    localB->temp = localB->Rprior[3 * c_0];
    localB->smax = localB->h1[3 * c_0];
    localB->H[6 * c_0] = localB->smax;
    jp = 6 * (c_0 + 3);
    localB->H[jp] = -localB->smax * obj->pKalmanPeriod;
    H_tmp = 6 * (c_0 + 6);
    localB->H[H_tmp] = tmp[3 * c_0];
    H_tmp_0 = 6 * (c_0 + 9);
    localB->H[H_tmp_0] = 0.0;
    localB->H[6 * c_0 + 3] = localB->temp;
    localB->H[jp + 3] = -localB->temp * obj->pKalmanPeriod;
    localB->H[H_tmp + 3] = 0.0;
    localB->H[H_tmp_0 + 3] = tmp_0[3 * c_0];
    jj = 3 * c_0 + 1;
    localB->H[6 * c_0 + 1] = localB->h1[jj];
    localB->H[jp + 1] = -localB->h1[jj] * obj->pKalmanPeriod;
    localB->H[H_tmp + 1] = tmp[jj];
    localB->H[H_tmp_0 + 1] = 0.0;
    localB->H[6 * c_0 + 4] = localB->Rprior[jj];
    localB->H[jp + 4] = -localB->Rprior[jj] * obj->pKalmanPeriod;
    localB->H[H_tmp + 4] = 0.0;
    localB->H[H_tmp_0 + 4] = tmp_0[jj];
    jj = 3 * c_0 + 2;
    localB->H[6 * c_0 + 2] = localB->h1[jj];
    localB->H[jp + 2] = -localB->h1[jj] * obj->pKalmanPeriod;
    localB->H[H_tmp + 2] = tmp[jj];
    localB->H[H_tmp_0 + 2] = 0.0;
    localB->H[6 * c_0 + 5] = localB->Rprior[jj];
    localB->H[jp + 5] = -localB->Rprior[jj] * obj->pKalmanPeriod;
    localB->H[H_tmp + 5] = 0.0;
    localB->H[H_tmp_0 + 5] = tmp_0[jj];
  }

  memcpy(&localB->Qw[0], &obj->pQw[0], 144U * sizeof(real_T));
  for (c_0 = 0; c_0 < 12; c_0++) {
    for (H_tmp = 0; H_tmp < 6; H_tmp++) {
      jp = H_tmp + 6 * c_0;
      localB->H_m[jp] = 0.0;
      for (jj = 0; jj < 12; jj++) {
        localB->H_m[jp] += localB->H[6 * jj + H_tmp] * obj->pQw[12 * c_0 + jj];
      }
    }
  }

  for (c_0 = 0; c_0 < 6; c_0++) {
    for (H_tmp = 0; H_tmp < 6; H_tmp++) {
      localB->temp = 0.0;
      for (jj = 0; jj < 12; jj++) {
        localB->temp += localB->H_m[6 * jj + H_tmp] * localB->H[6 * jj + c_0];
      }

      localB->c_A[c_0 + 6 * H_tmp] = obj->pQv[6 * c_0 + H_tmp] + localB->temp;
    }

    localB->b_ipiv[c_0] = static_cast<int8_T>(c_0 + 1);
  }

  for (jp = 0; jp < 5; jp++) {
    H_tmp = jp * 7 + 2;
    jj = jp * 7;
    c = 6 - jp;
    iy = 1;
    ix = H_tmp - 2;
    localB->smax = fabs(localB->c_A[jj]);
    for (jA = 2; jA <= c; jA++) {
      ix++;
      localB->s = fabs(localB->c_A[ix]);
      if (localB->s > localB->smax) {
        iy = jA;
        localB->smax = localB->s;
      }
    }

    if (localB->c_A[(H_tmp + iy) - 3] != 0.0) {
      if (iy - 1 != 0) {
        localB->b_ipiv[jp] = static_cast<int8_T>(jp + iy);
        ix = jp;
        iy = (jp + iy) - 1;
        for (jA = 0; jA < 6; jA++) {
          localB->temp = localB->c_A[ix];
          localB->c_A[ix] = localB->c_A[iy];
          localB->c_A[iy] = localB->temp;
          ix += 6;
          iy += 6;
        }
      }

      iy = H_tmp - jp;
      for (ix = H_tmp; ix <= iy + 4; ix++) {
        localB->c_A[ix - 1] /= localB->c_A[jj];
      }
    }

    c = 4 - jp;
    jA = jj;
    jy = jj + 6;
    for (jj = 0; jj <= c; jj++) {
      localB->temp = localB->c_A[jy];
      if (localB->c_A[jy] != 0.0) {
        ix = H_tmp - 1;
        c_0 = jA - jp;
        for (jp1j = jA + 8; jp1j <= c_0 + 12; jp1j++) {
          localB->c_A[jp1j - 1] += localB->c_A[ix] * -localB->temp;
          ix++;
        }
      }

      jy += 6;
      jA += 6;
    }
  }

  for (c_0 = 0; c_0 < 12; c_0++) {
    for (H_tmp = 0; H_tmp < 6; H_tmp++) {
      H_tmp_0 = c_0 + 12 * H_tmp;
      localB->b_X[H_tmp_0] = 0.0;
      for (jj = 0; jj < 12; jj++) {
        localB->b_X[H_tmp_0] += obj->pQw[12 * jj + c_0] * localB->H[6 * jj +
          H_tmp];
      }
    }
  }

  for (jp = 0; jp < 6; jp++) {
    jj = 12 * jp - 1;
    jp1j = 6 * jp - 1;
    iy = jp - 1;
    for (c = 0; c <= iy; c++) {
      jy = 12 * c - 1;
      c_0 = (c + jp1j) + 1;
      if (localB->c_A[c_0] != 0.0) {
        for (H_tmp = 0; H_tmp < 12; H_tmp++) {
          ix = H_tmp + 1;
          H_tmp_0 = ix + jj;
          localB->b_X[H_tmp_0] -= localB->c_A[c_0] * localB->b_X[ix + jy];
        }
      }
    }

    localB->temp = 1.0 / localB->c_A[(jp + jp1j) + 1];
    for (c = 0; c < 12; c++) {
      H_tmp_0 = (c + jj) + 1;
      localB->b_X[H_tmp_0] *= localB->temp;
    }
  }

  for (c = 5; c >= 0; c--) {
    jj = 12 * c - 1;
    jp1j = 6 * c - 1;
    for (jA = c + 2; jA < 7; jA++) {
      jy = (jA - 1) * 12 - 1;
      c_0 = jA + jp1j;
      if (localB->c_A[c_0] != 0.0) {
        for (H_tmp = 0; H_tmp < 12; H_tmp++) {
          ix = H_tmp + 1;
          H_tmp_0 = ix + jj;
          localB->b_X[H_tmp_0] -= localB->c_A[c_0] * localB->b_X[ix + jy];
        }
      }
    }
  }

  for (c = 4; c >= 0; c--) {
    if (c + 1 != localB->b_ipiv[c]) {
      jp = localB->b_ipiv[c] - 1;
      for (H_tmp = 0; H_tmp < 12; H_tmp++) {
        c_0 = 12 * c + H_tmp;
        localB->temp = localB->b_X[c_0];
        H_tmp_0 = 12 * jp + H_tmp;
        localB->b_X[c_0] = localB->b_X[H_tmp_0];
        localB->b_X[H_tmp_0] = localB->temp;
      }
    }
  }

  localB->ze[0] = localB->gravityAccelGyroDiff[0];
  localB->ze[3] = magIn[0] - localB->offDiag[0];
  localB->ze[1] = localB->gravityAccelGyroDiff[1];
  localB->ze[4] = magIn[1] - localB->offDiag[1];
  localB->ze[2] = localB->gravityAccelGyroDiff[2];
  localB->ze[5] = magIn[2] - localB->offDiag[2];
  for (c_0 = 0; c_0 < 3; c_0++) {
    localB->offDiag[c_0] = 0.0;
    for (H_tmp = 0; H_tmp < 6; H_tmp++) {
      localB->offDiag[c_0] += localB->b_X[(12 * H_tmp + c_0) + 9] * localB->
        ze[H_tmp];
    }
  }

  localB->temp = CLVF_norm(localB->offDiag, localB);
  isJamming = (localB->temp * localB->temp > obj->ExpectedMagneticFieldStrength *
               obj->ExpectedMagneticFieldStrength * 4.0);
  if (isJamming) {
    for (c_0 = 0; c_0 < 9; c_0++) {
      localB->h1[c_0] = localB->b_X[c_0 + 24] * localB->gravityAccelGyroDiff[2]
        + (localB->b_X[c_0 + 12] * localB->gravityAccelGyroDiff[1] + localB->
           b_X[c_0] * localB->gravityAccelGyroDiff[0]);
    }

    localB->gravityAccelGyroDiff[0] = localB->h1[0];
    localB->gyroOffsetErr_idx_0 = localB->h1[3];
    localB->linAccelErr_idx_0 = localB->h1[6];
    localB->gravityAccelGyroDiff[1] = localB->h1[1];
    localB->gyroOffsetErr_idx_1 = localB->h1[4];
    localB->linAccelErr_idx_1 = localB->h1[7];
    localB->gravityAccelGyroDiff[2] = localB->h1[2];
    localB->gyroOffsetErr_idx_2 = localB->h1[5];
    localB->linAccelErr_idx_2 = localB->h1[8];
  } else {
    for (c_0 = 0; c_0 < 12; c_0++) {
      localB->xe_post[c_0] = 0.0;
      for (H_tmp = 0; H_tmp < 6; H_tmp++) {
        localB->xe_post[c_0] += localB->b_X[12 * H_tmp + c_0] * localB->ze[H_tmp];
      }
    }

    localB->gravityAccelGyroDiff[0] = localB->xe_post[0];
    localB->gyroOffsetErr_idx_0 = localB->xe_post[3];
    localB->linAccelErr_idx_0 = localB->xe_post[6];
    localB->gravityAccelGyroDiff[1] = localB->xe_post[1];
    localB->gyroOffsetErr_idx_1 = localB->xe_post[4];
    localB->linAccelErr_idx_1 = localB->xe_post[7];
    localB->gravityAccelGyroDiff[2] = localB->xe_post[2];
    localB->gyroOffsetErr_idx_2 = localB->xe_post[5];
    localB->linAccelErr_idx_2 = localB->xe_post[8];
  }

  CLV_quaternioncg_quaternioncg_f(localB->gravityAccelGyroDiff, &localB->temp,
    &localB->smax, &localB->s, &localB->assign_temp_d_c, localB);
  CLVF_quaternionBase_conj(localB->temp, localB->smax, localB->s,
    localB->assign_temp_d_c, &localB->assign_temp_a, &localB->assign_temp_b,
    &localB->assign_temp_c, &localB->assign_temp_d);
  CLVF_quaternionBase_mtimes(obj->pOrientPrior.a, obj->pOrientPrior.b,
    obj->pOrientPrior.c, obj->pOrientPrior.d, localB->assign_temp_a,
    localB->assign_temp_b, localB->assign_temp_c, localB->assign_temp_d,
    &obj->pOrientPost.a, &obj->pOrientPost.b, &obj->pOrientPost.c,
    &obj->pOrientPost.d);
  if (obj->pOrientPost.a < 0.0) {
    CLVF_quaternionBase_uminus(obj->pOrientPost.a, obj->pOrientPost.b,
      obj->pOrientPost.c, obj->pOrientPost.d, &obj->pOrientPost.a,
      &obj->pOrientPost.b, &obj->pOrientPost.c, &obj->pOrientPost.d);
  }

  CLVF_quaternionBase_normalize(obj->pOrientPost.a, obj->pOrientPost.b,
    obj->pOrientPost.c, obj->pOrientPost.d, &obj->pOrientPost.a,
    &obj->pOrientPost.b, &obj->pOrientPost.c, &obj->pOrientPost.d);
  CLVF_quaternionBase_rotmat(obj->pOrientPost.a, obj->pOrientPost.b,
    obj->pOrientPost.c, obj->pOrientPost.d, localB->h1, localB);
  obj->pGyroOffset[0] -= localB->gyroOffsetErr_idx_0;
  obj->pLinAccelPost[0] = obj->pLinAccelPrior[0] - localB->linAccelErr_idx_0;
  obj->pGyroOffset[1] -= localB->gyroOffsetErr_idx_1;
  obj->pLinAccelPost[1] = obj->pLinAccelPrior[1] - localB->linAccelErr_idx_1;
  obj->pGyroOffset[2] -= localB->gyroOffsetErr_idx_2;
  obj->pLinAccelPost[2] = obj->pLinAccelPrior[2] - localB->linAccelErr_idx_2;
  if (!isJamming) {
    for (H_tmp = 0; H_tmp < 3; H_tmp++) {
      jp = H_tmp * 3 - 1;
      localB->gravityAccelGyroDiff[H_tmp] = (localB->h1[jp + 1] *
        localB->offDiag[0] + localB->h1[jp + 2] * localB->offDiag[1]) +
        localB->h1[jp + 3] * localB->offDiag[2];
    }

    localB->gravityAccelGyroDiff[0] = obj->pMagVec[0] -
      localB->gravityAccelGyroDiff[0];
    localB->temp = rt_atan2d_snf(obj->pMagVec[2] - localB->gravityAccelGyroDiff
      [2], localB->gravityAccelGyroDiff[0]);
    if (localB->temp < -1.5707963267948966) {
      localB->temp = -1.5707963267948966;
    }

    if (localB->temp > 1.5707963267948966) {
      localB->temp = 1.5707963267948966;
    }

    obj->pMagVec[0] = 0.0;
    obj->pMagVec[1] = 0.0;
    obj->pMagVec[2] = 0.0;
    obj->pMagVec[0] = cos(localB->temp);
    obj->pMagVec[2] = sin(localB->temp);
    obj->pMagVec[0] *= obj->ExpectedMagneticFieldStrength;
    obj->pMagVec[1] *= obj->ExpectedMagneticFieldStrength;
    obj->pMagVec[2] *= obj->ExpectedMagneticFieldStrength;
  }

  for (c_0 = 0; c_0 < 12; c_0++) {
    for (H_tmp = 0; H_tmp < 6; H_tmp++) {
      jp = H_tmp + 6 * c_0;
      localB->H_m[jp] = 0.0;
      for (jj = 0; jj < 12; jj++) {
        localB->H_m[jp] += localB->H[6 * jj + H_tmp] * localB->Qw[12 * c_0 + jj];
      }
    }
  }

  for (c_0 = 0; c_0 < 12; c_0++) {
    for (H_tmp = 0; H_tmp < 12; H_tmp++) {
      localB->temp = 0.0;
      for (jj = 0; jj < 6; jj++) {
        localB->temp += localB->b_X[12 * jj + c_0] * localB->H_m[6 * H_tmp + jj];
      }

      jp = 12 * H_tmp + c_0;
      localB->Ppost[jp] = localB->Qw[jp] - localB->temp;
    }
  }

  memset(&localB->Qw[0], 0, 144U * sizeof(real_T));
  localB->temp = obj->pKalmanPeriod * obj->pKalmanPeriod;
  localB->smax = obj->GyroscopeDriftNoise + obj->GyroscopeNoise;
  localB->Qw[0] = (localB->Ppost[39] + localB->smax) * localB->temp +
    localB->Ppost[0];
  localB->Qw[39] = localB->Ppost[39] + obj->GyroscopeDriftNoise;
  localB->offDiag[0] = -obj->pKalmanPeriod * localB->Qw[39];
  localB->Qw[13] = (localB->Ppost[52] + localB->smax) * localB->temp +
    localB->Ppost[13];
  localB->Qw[52] = localB->Ppost[52] + obj->GyroscopeDriftNoise;
  localB->offDiag[1] = -obj->pKalmanPeriod * localB->Qw[52];
  localB->Qw[26] = (localB->Ppost[65] + localB->smax) * localB->temp +
    localB->Ppost[26];
  localB->Qw[65] = localB->Ppost[65] + obj->GyroscopeDriftNoise;
  localB->offDiag[2] = -obj->pKalmanPeriod * localB->Qw[65];
  localB->Qw[3] = localB->offDiag[0];
  localB->Qw[16] = localB->offDiag[1];
  localB->Qw[29] = localB->offDiag[2];
  localB->Qw[36] = localB->offDiag[0];
  localB->Qw[49] = localB->offDiag[1];
  localB->Qw[62] = localB->offDiag[2];
  localB->temp = obj->LinearAccelerationDecayFactor *
    obj->LinearAccelerationDecayFactor;
  localB->Qw[78] = localB->temp * localB->Ppost[78] +
    obj->LinearAccelerationNoise;
  localB->Qw[91] = localB->temp * localB->Ppost[91] +
    obj->LinearAccelerationNoise;
  localB->Qw[104] = localB->temp * localB->Ppost[104] +
    obj->LinearAccelerationNoise;
  localB->temp = obj->MagneticDisturbanceDecayFactor *
    obj->MagneticDisturbanceDecayFactor;
  localB->Qw[117] = localB->temp * localB->Ppost[117] +
    obj->MagneticDisturbanceNoise;
  localB->Qw[130] = localB->temp * localB->Ppost[130] +
    obj->MagneticDisturbanceNoise;
  localB->Qw[143] = localB->temp * localB->Ppost[143] +
    obj->MagneticDisturbanceNoise;
  memcpy(&obj->pQw[0], &localB->Qw[0], 144U * sizeof(real_T));
  CLVF_quaternioncg_parenAssign(obj->pOrientPost.a, obj->pOrientPost.b,
    obj->pOrientPost.c, obj->pOrientPost.d, &orientOut[0], &orientOut[1],
    &orientOut[2], &orientOut[3]);
}

//
// System initialize for atomic system:
//    synthesized block
//    synthesized block
//    synthesized block
//
void CLVF_AHRS2_Init(const real_T rtu_0[3], DW_AHRS2_CLVF_T *localDW,
                     P_AHRS2_CLVF_T *localP)
{
  // Start for MATLABSystem: '<S169>/AHRS2'
  localDW->obj.isInitialized = 0;
  localDW->objisempty = true;
  localDW->obj.AccelerometerNoise = localP->AHRS2_AccelerometerNoise;
  localDW->obj.GyroscopeNoise = localP->AHRS2_GyroscopeNoise;
  localDW->obj.MagnetometerNoise = localP->AHRS2_MagnetometerNoise;
  localDW->obj.GyroscopeDriftNoise = localP->AHRS2_GyroscopeDriftNoise;
  localDW->obj.LinearAccelerationNoise = localP->AHRS2_LinearAccelerationNoise;
  localDW->obj.MagneticDisturbanceNoise = localP->AHRS2_MagneticDisturbanceNoise;
  localDW->obj.LinearAccelerationDecayFactor =
    localP->AHRS2_LinearAccelerationDecayFa;
  localDW->obj.MagneticDisturbanceDecayFactor =
    localP->AHRS2_MagneticDisturbanceDecayF;
  localDW->obj.ExpectedMagneticFieldStrength =
    localP->AHRS2_ExpectedMagneticFieldStre;
  localDW->obj.isInitialized = 1;
  localDW->obj.pInputPrototype[0] = rtu_0[0];
  localDW->obj.pInputPrototype[1] = rtu_0[1];
  localDW->obj.pInputPrototype[2] = rtu_0[2];
  localDW->obj.pSensorPeriod = 0.05;
  localDW->obj.pKalmanPeriod = localDW->obj.pSensorPeriod;
  localDW->obj.pRefSys = &localDW->gobj_2;
  localDW->obj.TunablePropsChanged = false;

  // InitializeConditions for MATLABSystem: '<S169>/AHRS2'
  CLVF_AHRSFilterBase_resetImpl(&localDW->obj);
}

//
// Output and update for atomic system:
//    synthesized block
//    synthesized block
//    synthesized block
//
void CLVF_AHRS2(const real_T rtu_0[3], const real_T rtu_1[3], const real_T
                rtu_2[3], B_AHRS2_CLVF_T *localB, DW_AHRS2_CLVF_T *localDW,
                P_AHRS2_CLVF_T *localP)
{
  int32_T tmp;
  static const int8_T tmp_0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  // MATLABSystem: '<S169>/AHRS2'
  if (localDW->obj.AccelerometerNoise != localP->AHRS2_AccelerometerNoise) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.AccelerometerNoise = localP->AHRS2_AccelerometerNoise;
  }

  if (localDW->obj.GyroscopeNoise != localP->AHRS2_GyroscopeNoise) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.GyroscopeNoise = localP->AHRS2_GyroscopeNoise;
  }

  if (localDW->obj.MagnetometerNoise != localP->AHRS2_MagnetometerNoise) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.MagnetometerNoise = localP->AHRS2_MagnetometerNoise;
  }

  if (localDW->obj.GyroscopeDriftNoise != localP->AHRS2_GyroscopeDriftNoise) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.GyroscopeDriftNoise = localP->AHRS2_GyroscopeDriftNoise;
  }

  if (localDW->obj.LinearAccelerationNoise !=
      localP->AHRS2_LinearAccelerationNoise) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.LinearAccelerationNoise = localP->AHRS2_LinearAccelerationNoise;
  }

  if (localDW->obj.MagneticDisturbanceNoise !=
      localP->AHRS2_MagneticDisturbanceNoise) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.MagneticDisturbanceNoise =
      localP->AHRS2_MagneticDisturbanceNoise;
  }

  if (localDW->obj.LinearAccelerationDecayFactor !=
      localP->AHRS2_LinearAccelerationDecayFa) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.LinearAccelerationDecayFactor =
      localP->AHRS2_LinearAccelerationDecayFa;
  }

  if (localDW->obj.MagneticDisturbanceDecayFactor !=
      localP->AHRS2_MagneticDisturbanceDecayF) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.MagneticDisturbanceDecayFactor =
      localP->AHRS2_MagneticDisturbanceDecayF;
  }

  if (localDW->obj.ExpectedMagneticFieldStrength !=
      localP->AHRS2_ExpectedMagneticFieldStre) {
    if (localDW->obj.isInitialized == 1) {
      localDW->obj.TunablePropsChanged = true;
    }

    localDW->obj.ExpectedMagneticFieldStrength =
      localP->AHRS2_ExpectedMagneticFieldStre;
  }

  if (localDW->obj.TunablePropsChanged) {
    localDW->obj.TunablePropsChanged = false;
    localB->magMeasNoiseVar = localDW->obj.pKalmanPeriod *
      localDW->obj.pKalmanPeriod * (localDW->obj.GyroscopeDriftNoise +
      localDW->obj.GyroscopeNoise);
    localB->accelMeasNoiseVar = localB->magMeasNoiseVar +
      (localDW->obj.AccelerometerNoise + localDW->obj.LinearAccelerationNoise);
    localB->magMeasNoiseVar += localDW->obj.MagnetometerNoise +
      localDW->obj.MagneticDisturbanceNoise;
    memset(&localDW->obj.pQv[0], 0, 36U * sizeof(real_T));
    for (localB->i = 0; localB->i < 3; localB->i++) {
      localB->i1 = tmp_0[3 * localB->i];
      localDW->obj.pQv[6 * localB->i] = localB->accelMeasNoiseVar * static_cast<
        real_T>(localB->i1);
      tmp = 6 * (localB->i + 3);
      localDW->obj.pQv[tmp + 3] = localB->magMeasNoiseVar * static_cast<real_T>
        (localB->i1);
      localB->i1 = tmp_0[3 * localB->i + 1];
      localDW->obj.pQv[6 * localB->i + 1] = localB->accelMeasNoiseVar *
        static_cast<real_T>(localB->i1);
      localDW->obj.pQv[tmp + 4] = localB->magMeasNoiseVar * static_cast<real_T>
        (localB->i1);
      localB->i1 = tmp_0[3 * localB->i + 2];
      localDW->obj.pQv[6 * localB->i + 2] = localB->accelMeasNoiseVar *
        static_cast<real_T>(localB->i1);
      localDW->obj.pQv[tmp + 5] = localB->magMeasNoiseVar * static_cast<real_T>
        (localB->i1);
    }
  }

  CLVF_ahrsfilter_stepImpl(&localDW->obj, rtu_0, rtu_1, rtu_2,
    localB->b_varargout_1, localB->b_varargout_2, localB);
  localB->AHRS2_o2[0] = localB->b_varargout_2[0];
  localB->AHRS2_o2[1] = localB->b_varargout_2[1];
  localB->AHRS2_o2[2] = localB->b_varargout_2[2];

  // End of MATLABSystem: '<S169>/AHRS2'
}

//
// Output and update for atomic system:
//    '<S169>/ChangeOrientation'
//    '<S170>/ChangeOrientation'
//    '<S171>/ChangeOrientation'
//
void CLVF_ChangeOrientation(real_T rtu_u, real_T rtu_u_m, real_T rtu_u_e,
  B_ChangeOrientation_CLVF_T *localB)
{
  int32_T i;
  static const int8_T a[9] = { 0, -1, 0, 1, 0, 0, 0, 0, 1 };

  // MATLAB Function 'Gyroscope & Acceleration Algorithms/Change BLACK Behavior/ChangeOrientation': '<S173>:1' 
  // '<S173>:1:3' y = rotz(-90)*u';
  for (i = 0; i < 3; i++) {
    localB->y[i] = 0.0;

    // SignalConversion generated from: '<S173>/ SFunction '
    localB->y[i] += static_cast<real_T>(a[i]) * rtu_u;
    localB->y[i] += static_cast<real_T>(a[i + 3]) * rtu_u_m;
    localB->y[i] += static_cast<real_T>(a[i + 6]) * rtu_u_e;
  }
}

//
// System initialize for action system:
//    '<S179>/Calculate Running Mean'
//    '<S180>/Calculate Running Mean'
//    '<S181>/Calculate Running Mean'
//    '<S192>/Calculate Running Mean'
//    '<S193>/Calculate Running Mean'
//    '<S210>/Calculate Running Mean'
//    '<S211>/Calculate Running Mean'
//    '<S212>/Calculate Running Mean'
//    '<S223>/Calculate Running Mean'
//    '<S224>/Calculate Running Mean'
//    ...
//
void C_CalculateRunningMean_Init(B_CalculateRunningMean_CLVF_T *localB,
  DW_CalculateRunningMean_CLVF_T *localDW, P_CalculateRunningMean_CLVF_T *localP)
{
  int32_T i;

  // InitializeConditions for Delay: '<S182>/Delay'
  localDW->Delay_DSTATE = localP->Delay_InitialCondition;

  // InitializeConditions for Delay: '<S182>/Delay1'
  localDW->Delay1_DSTATE[0] = localP->Delay1_InitialCondition;
  localDW->Delay1_DSTATE[1] = localP->Delay1_InitialCondition;

  // InitializeConditions for Delay: '<S182>/Delay2'
  localDW->Delay2_DSTATE[0] = localP->Delay2_InitialCondition;
  localDW->Delay2_DSTATE[1] = localP->Delay2_InitialCondition;
  localDW->Delay2_DSTATE[2] = localP->Delay2_InitialCondition;
  localDW->Delay2_DSTATE[3] = localP->Delay2_InitialCondition;

  // InitializeConditions for Delay: '<S182>/Delay3'
  for (i = 0; i < 5; i++) {
    localDW->Delay3_DSTATE[i] = localP->Delay3_InitialCondition;
  }

  // End of InitializeConditions for Delay: '<S182>/Delay3'

  // InitializeConditions for Delay: '<S182>/Delay4'
  localDW->Delay4_DSTATE[0] = localP->Delay4_InitialCondition;
  localDW->Delay4_DSTATE[1] = localP->Delay4_InitialCondition;
  localDW->Delay4_DSTATE[2] = localP->Delay4_InitialCondition;

  // SystemInitialize for Outport: '<S182>/Out1'
  localB->Mean = localP->Out1_Y0;
}

//
// Output and update for action system:
//    '<S179>/Calculate Running Mean'
//    '<S180>/Calculate Running Mean'
//    '<S181>/Calculate Running Mean'
//    '<S192>/Calculate Running Mean'
//    '<S193>/Calculate Running Mean'
//    '<S210>/Calculate Running Mean'
//    '<S211>/Calculate Running Mean'
//    '<S212>/Calculate Running Mean'
//    '<S223>/Calculate Running Mean'
//    '<S224>/Calculate Running Mean'
//    ...
//
void CLVF_CalculateRunningMean(real_T rtu_In1, B_CalculateRunningMean_CLVF_T
  *localB, DW_CalculateRunningMean_CLVF_T *localDW)
{
  int32_T n;
  int32_T str;
  int_T idxWidth;
  real_T rtb_TmpSignalConversionAtMeanIn[6];

  // SignalConversion generated from: '<S182>/Mean' incorporates:
  //   Delay: '<S182>/Delay'
  //   Delay: '<S182>/Delay1'
  //   Delay: '<S182>/Delay2'
  //   Delay: '<S182>/Delay3'
  //   Delay: '<S182>/Delay4'

  rtb_TmpSignalConversionAtMeanIn[0] = rtu_In1;
  rtb_TmpSignalConversionAtMeanIn[1] = localDW->Delay_DSTATE;
  rtb_TmpSignalConversionAtMeanIn[2] = localDW->Delay1_DSTATE[0U];
  rtb_TmpSignalConversionAtMeanIn[3] = localDW->Delay4_DSTATE[0U];
  rtb_TmpSignalConversionAtMeanIn[4] = localDW->Delay2_DSTATE[0U];
  rtb_TmpSignalConversionAtMeanIn[5] = localDW->Delay3_DSTATE[0U];

  // S-Function (sdspstatfcns): '<S182>/Mean'
  for (idxWidth = 0; idxWidth < 1; idxWidth++) {
    localDW->Mean_AccVal = rtb_TmpSignalConversionAtMeanIn[idxWidth];
    str = 1;
    for (n = 4; n >= 0; n--) {
      localDW->Mean_AccVal += rtb_TmpSignalConversionAtMeanIn[idxWidth + str];
      str++;
    }

    localB->Mean = localDW->Mean_AccVal / 6.0;
  }

  // End of S-Function (sdspstatfcns): '<S182>/Mean'

  // Update for Delay: '<S182>/Delay'
  localDW->Delay_DSTATE = rtu_In1;

  // Update for Delay: '<S182>/Delay1'
  localDW->Delay1_DSTATE[0] = localDW->Delay1_DSTATE[1];
  localDW->Delay1_DSTATE[1] = rtu_In1;

  // Update for Delay: '<S182>/Delay2'
  localDW->Delay2_DSTATE[0] = localDW->Delay2_DSTATE[1];
  localDW->Delay2_DSTATE[1] = localDW->Delay2_DSTATE[2];
  localDW->Delay2_DSTATE[2] = localDW->Delay2_DSTATE[3];
  localDW->Delay2_DSTATE[3] = rtu_In1;

  // Update for Delay: '<S182>/Delay3'
  localDW->Delay3_DSTATE[0] = localDW->Delay3_DSTATE[1];
  localDW->Delay3_DSTATE[1] = localDW->Delay3_DSTATE[2];
  localDW->Delay3_DSTATE[2] = localDW->Delay3_DSTATE[3];
  localDW->Delay3_DSTATE[3] = localDW->Delay3_DSTATE[4];
  localDW->Delay3_DSTATE[4] = rtu_In1;

  // Update for Delay: '<S182>/Delay4'
  localDW->Delay4_DSTATE[0] = localDW->Delay4_DSTATE[1];
  localDW->Delay4_DSTATE[1] = localDW->Delay4_DSTATE[2];
  localDW->Delay4_DSTATE[2] = rtu_In1;
}

//
// System initialize for action system:
//    '<S194>/Calculate Running Mean'
//    '<S225>/Calculate Running Mean'
//    '<S256>/Calculate Running Mean'
//
void CalculateRunningMean_b_Init(B_CalculateRunningMean_CLVF_p_T *localB,
  DW_CalculateRunningMean_CLV_e_T *localDW, P_CalculateRunningMean_CLVF_c_T
  *localP)
{
  int32_T i;

  // InitializeConditions for Delay: '<S201>/Delay'
  localDW->Delay_DSTATE = localP->Delay_InitialCondition;

  // InitializeConditions for Delay: '<S201>/Delay1'
  localDW->Delay1_DSTATE[0] = localP->Delay1_InitialCondition;
  localDW->Delay1_DSTATE[1] = localP->Delay1_InitialCondition;

  // InitializeConditions for Delay: '<S201>/Delay2'
  localDW->Delay2_DSTATE[0] = localP->Delay2_InitialCondition;
  localDW->Delay2_DSTATE[1] = localP->Delay2_InitialCondition;
  localDW->Delay2_DSTATE[2] = localP->Delay2_InitialCondition;
  localDW->Delay2_DSTATE[3] = localP->Delay2_InitialCondition;

  // InitializeConditions for Delay: '<S201>/Delay3'
  for (i = 0; i < 5; i++) {
    localDW->Delay3_DSTATE[i] = localP->Delay3_InitialCondition;
  }

  // End of InitializeConditions for Delay: '<S201>/Delay3'

  // InitializeConditions for Delay: '<S201>/Delay4'
  localDW->Delay4_DSTATE[0] = localP->Delay4_InitialCondition;
  localDW->Delay4_DSTATE[1] = localP->Delay4_InitialCondition;
  localDW->Delay4_DSTATE[2] = localP->Delay4_InitialCondition;

  // SystemInitialize for Outport: '<S201>/Out1'
  localB->Subtract = localP->Out1_Y0;
}

//
// Output and update for action system:
//    '<S194>/Calculate Running Mean'
//    '<S225>/Calculate Running Mean'
//    '<S256>/Calculate Running Mean'
//
void CLVF_CalculateRunningMean_b(real_T rtu_In1, B_CalculateRunningMean_CLVF_p_T
  *localB, DW_CalculateRunningMean_CLV_e_T *localDW,
  P_CalculateRunningMean_CLVF_c_T *localP)
{
  int32_T n;
  int32_T str;
  int_T idxWidth;
  real_T rtb_Mean;

  // Delay: '<S201>/Delay4'
  rtb_Mean = localDW->Delay4_DSTATE[0];

  // SignalConversion generated from: '<S201>/Mean' incorporates:
  //   Delay: '<S201>/Delay'
  //   Delay: '<S201>/Delay1'
  //   Delay: '<S201>/Delay2'
  //   Delay: '<S201>/Delay3'
  //   Delay: '<S201>/Delay4'

  localB->TmpSignalConversionAtMean_b[0] = rtu_In1;
  localB->TmpSignalConversionAtMean_b[1] = localDW->Delay_DSTATE;
  localB->TmpSignalConversionAtMean_b[2] = localDW->Delay1_DSTATE[0U];
  localB->TmpSignalConversionAtMean_b[3] = localDW->Delay4_DSTATE[0U];
  localB->TmpSignalConversionAtMean_b[4] = localDW->Delay2_DSTATE[0U];
  localB->TmpSignalConversionAtMean_b[5] = localDW->Delay3_DSTATE[0U];

  // S-Function (sdspstatfcns): '<S201>/Mean'
  for (idxWidth = 0; idxWidth < 1; idxWidth++) {
    localDW->Mean_AccVal = localB->TmpSignalConversionAtMean_b[idxWidth];
    str = 1;
    for (n = 4; n >= 0; n--) {
      localDW->Mean_AccVal += localB->TmpSignalConversionAtMean_b[idxWidth + str];
      str++;
    }

    rtb_Mean = localDW->Mean_AccVal / 6.0;
  }

  // End of S-Function (sdspstatfcns): '<S201>/Mean'

  // Sum: '<S201>/Subtract' incorporates:
  //   Constant: '<S201>/Constant'

  localB->Subtract = rtb_Mean - localP->Constant_Value;

  // Update for Delay: '<S201>/Delay'
  localDW->Delay_DSTATE = rtu_In1;

  // Update for Delay: '<S201>/Delay1'
  localDW->Delay1_DSTATE[0] = localDW->Delay1_DSTATE[1];
  localDW->Delay1_DSTATE[1] = rtu_In1;

  // Update for Delay: '<S201>/Delay2'
  localDW->Delay2_DSTATE[0] = localDW->Delay2_DSTATE[1];
  localDW->Delay2_DSTATE[1] = localDW->Delay2_DSTATE[2];
  localDW->Delay2_DSTATE[2] = localDW->Delay2_DSTATE[3];
  localDW->Delay2_DSTATE[3] = rtu_In1;

  // Update for Delay: '<S201>/Delay3'
  localDW->Delay3_DSTATE[0] = localDW->Delay3_DSTATE[1];
  localDW->Delay3_DSTATE[1] = localDW->Delay3_DSTATE[2];
  localDW->Delay3_DSTATE[2] = localDW->Delay3_DSTATE[3];
  localDW->Delay3_DSTATE[3] = localDW->Delay3_DSTATE[4];
  localDW->Delay3_DSTATE[4] = rtu_In1;

  // Update for Delay: '<S201>/Delay4'
  localDW->Delay4_DSTATE[0] = localDW->Delay4_DSTATE[1];
  localDW->Delay4_DSTATE[1] = localDW->Delay4_DSTATE[2];
  localDW->Delay4_DSTATE[2] = rtu_In1;
}

//
// Output and update for atomic system:
//    '<S533>/MATLAB Function'
//    '<S534>/MATLAB Function'
//    '<S535>/MATLAB Function'
//
void CLVF_MATLABFunction_f(real_T rtu_TIME, real_T rtu_SERVER,
  B_MATLABFunction_CLVF_g_T *localB)
{
  real_T r;
  boolean_T rEQ0;
  real_T q;

  // MATLAB Function 'Simulate Plant Dynamics/Sample and Hold  Data to Match  PhaseSpace (BLACK)/MATLAB Function': '<S554>:1' 
  // '<S554>:1:3' if mod(TIME,SERVER) == 0
  r = rtu_TIME;
  if (rtu_SERVER == 0.0) {
    if (rtu_TIME == 0.0) {
      r = rtu_SERVER;
    }
  } else if (rtIsNaN(rtu_TIME)) {
    r = (rtNaN);
  } else if (rtIsNaN(rtu_SERVER)) {
    r = (rtNaN);
  } else if (rtIsInf(rtu_TIME)) {
    r = (rtNaN);
  } else if (rtu_TIME == 0.0) {
    r = 0.0 / rtu_SERVER;
  } else if (rtIsInf(rtu_SERVER)) {
    if ((rtu_SERVER < 0.0) != (rtu_TIME < 0.0)) {
      r = rtu_SERVER;
    }
  } else {
    r = fmod(rtu_TIME, rtu_SERVER);
    rEQ0 = (r == 0.0);
    if ((!rEQ0) && (rtu_SERVER > floor(rtu_SERVER))) {
      q = fabs(rtu_TIME / rtu_SERVER);
      rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }

    if (rEQ0) {
      r = rtu_SERVER * 0.0;
    } else {
      if ((rtu_TIME < 0.0) != (rtu_SERVER < 0.0)) {
        r += rtu_SERVER;
      }
    }
  }

  if (r == 0.0) {
    // '<S554>:1:4' y = 1;
    localB->y = 1.0;
  } else {
    // '<S554>:1:5' else
    // '<S554>:1:6' y = 0;
    localB->y = 0.0;
  }
}

//
// Output and update for action system:
//    '<S8>/Change BLUE Behavior'
//    '<S9>/Change BLUE Behavior'
//    '<S14>/Change BLUE Behavior'
//
void CLVF_ChangeBLUEBehavior(P_ChangeBLUEBehavior_CLVF_T *localP, real_T
  *rtd_BLUE_Fx, real_T *rtd_BLUE_Fy, real_T *rtd_BLUE_Tz, real_T
  *rtd_Float_State)
{
  // DataStoreWrite: '<S267>/BLUE_Fx' incorporates:
  //   Constant: '<S267>/Constant3'

  *rtd_BLUE_Fx = localP->Constant3_Value;

  // DataStoreWrite: '<S267>/BLUE_Fy' incorporates:
  //   Constant: '<S267>/Constant4'

  *rtd_BLUE_Fy = localP->Constant4_Value;

  // DataStoreWrite: '<S267>/BLUE_Tz' incorporates:
  //   Constant: '<S267>/Constant5'

  *rtd_BLUE_Tz = localP->Constant5_Value;

  // DataStoreWrite: '<S267>/Data Store Write4' incorporates:
  //   Constant: '<S267>/Puck State'

  *rtd_Float_State = localP->PuckState_Value;
}

//
// System initialize for action system:
//    '<Root>/Phase #0:  Wait for Synchronization'
//    '<Root>/Phase #1:  Start Floating '
//
void Phase0WaitforSynchroni_Init(DW_Phase0WaitforSynchronizati_T *localDW)
{
  // Start for If: '<S8>/This IF block determines whether or not to run the BLACK sim//exp' 
  localDW->ThisIFblockdetermineswhetherorn = -1;

  // Start for If: '<S8>/This IF block determines whether or not to run the BLUE sim//exp' 
  localDW->ThisIFblockdetermineswhethero_m = -1;

  // Start for If: '<S8>/This IF block determines whether or not to run the RED sim//exp ' 
  localDW->ThisIFblockdetermineswhethero_n = -1;
}

//
// Output and update for action system:
//    '<Root>/Phase #0:  Wait for Synchronization'
//    '<Root>/Phase #1:  Start Floating '
//
void Phase0WaitforSynchronizatio(DW_Phase0WaitforSynchronizati_T *localDW,
  P_Phase0WaitforSynchronizatio_T *localP, real_T *rtd_BLACK_Fx, real_T
  *rtd_BLACK_Fy, real_T *rtd_BLACK_Tz, real_T *rtd_BLUE_Fx, real_T *rtd_BLUE_Fy,
  real_T *rtd_BLUE_Tz, real_T *rtd_Float_State, real_T *rtd_RED_Fx, real_T
  *rtd_RED_Fy, real_T *rtd_RED_Tz, real_T *rtd_RED_Tz_Elbow, real_T
  *rtd_RED_Tz_RW, real_T *rtd_RED_Tz_Shoulder, real_T *rtd_RED_Tz_Wrist)
{
  // If: '<S8>/This IF block determines whether or not to run the BLACK sim//exp' incorporates:
  //   Constant: '<S269>/Constant'
  //   Constant: '<S8>/Constant'

  localDW->ThisIFblockdetermineswhetherorn = -1;
  if ((CLVF_P.WhoAmI == 2.0) || (CLVF_P.simMode == 1.0)) {
    localDW->ThisIFblockdetermineswhetherorn = 0;

    // Outputs for IfAction SubSystem: '<S8>/Change BLACK Behavior' incorporates:
    //   ActionPort: '<S266>/Action Port'

    // DataStoreWrite: '<S266>/BLACK_Fx' incorporates:
    //   Constant: '<S266>/Constant3'

    *rtd_BLACK_Fx = localP->Constant3_Value;

    // DataStoreWrite: '<S266>/BLACK_Fy' incorporates:
    //   Constant: '<S266>/Constant4'

    *rtd_BLACK_Fy = localP->Constant4_Value;

    // DataStoreWrite: '<S266>/BLACK_Tz' incorporates:
    //   Constant: '<S266>/Constant5'

    *rtd_BLACK_Tz = localP->Constant5_Value;

    // DataStoreWrite: '<S266>/Data Store Write4' incorporates:
    //   Constant: '<S266>/Puck State'

    *rtd_Float_State = localP->PuckState_Value;

    // End of Outputs for SubSystem: '<S8>/Change BLACK Behavior'
  }

  // End of If: '<S8>/This IF block determines whether or not to run the BLACK sim//exp' 

  // If: '<S8>/This IF block determines whether or not to run the BLUE sim//exp' incorporates:
  //   Constant: '<S269>/Constant'
  //   Constant: '<S8>/Constant'

  localDW->ThisIFblockdetermineswhethero_m = -1;
  if ((CLVF_P.WhoAmI == 3.0) || (CLVF_P.simMode == 1.0)) {
    localDW->ThisIFblockdetermineswhethero_m = 0;

    // Outputs for IfAction SubSystem: '<S8>/Change BLUE Behavior' incorporates:
    //   ActionPort: '<S267>/Action Port'

    CLVF_ChangeBLUEBehavior(&localP->ChangeBLUEBehavior, rtd_BLUE_Fx,
      rtd_BLUE_Fy, rtd_BLUE_Tz, rtd_Float_State);

    // End of Outputs for SubSystem: '<S8>/Change BLUE Behavior'
  }

  // End of If: '<S8>/This IF block determines whether or not to run the BLUE sim//exp' 

  // If: '<S8>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
  //   Constant: '<S269>/Constant'
  //   Constant: '<S8>/Constant'

  localDW->ThisIFblockdetermineswhethero_n = -1;
  if ((CLVF_P.WhoAmI == 1.0) || (CLVF_P.simMode == 1.0)) {
    localDW->ThisIFblockdetermineswhethero_n = 0;

    // Outputs for IfAction SubSystem: '<S8>/Change RED Behavior' incorporates:
    //   ActionPort: '<S268>/Action Port'

    // DataStoreWrite: '<S268>/RED_Fx' incorporates:
    //   Constant: '<S268>/Constant'

    *rtd_RED_Fx = localP->Constant_Value;

    // DataStoreWrite: '<S268>/RED_Fy' incorporates:
    //   Constant: '<S268>/Constant1'

    *rtd_RED_Fy = localP->Constant1_Value;

    // DataStoreWrite: '<S268>/RED_Tz' incorporates:
    //   Constant: '<S268>/Constant2'

    *rtd_RED_Tz = localP->Constant2_Value;

    // DataStoreWrite: '<S268>/RED_Tz_RW' incorporates:
    //   Constant: '<S268>/Constant3'

    *rtd_RED_Tz_RW = localP->Constant3_Value_h;

    // DataStoreWrite: '<S268>/Data Store Write' incorporates:
    //   Constant: '<S268>/Constant4'

    *rtd_RED_Tz_Shoulder = localP->Constant4_Value_g;

    // DataStoreWrite: '<S268>/Data Store Write1' incorporates:
    //   Constant: '<S268>/Constant5'

    *rtd_RED_Tz_Elbow = localP->Constant5_Value_e;

    // DataStoreWrite: '<S268>/Data Store Write2' incorporates:
    //   Constant: '<S268>/Constant6'

    *rtd_RED_Tz_Wrist = localP->Constant6_Value;

    // DataStoreWrite: '<S268>/Data Store Write4' incorporates:
    //   Constant: '<S268>/Puck State'

    *rtd_Float_State = localP->PuckState_Value_b;

    // End of Outputs for SubSystem: '<S8>/Change RED Behavior'
  }

  // End of If: '<S8>/This IF block determines whether or not to run the RED sim//exp ' 
}

//
// Output and update for atomic system:
//    '<S281>/MATLAB Function2'
//    '<S294>/MATLAB Function2'
//    '<S307>/MATLAB Function2'
//    '<S329>/MATLAB Function2'
//    '<S382>/MATLAB Function2'
//    '<S395>/MATLAB Function2'
//    '<S413>/MATLAB Function2'
//    '<S444>/MATLAB Function2'
//    '<S457>/MATLAB Function2'
//    '<S470>/MATLAB Function2'
//    ...
//
void CLVF_MATLABFunction2_o(real_T rtu_z, B_MATLABFunction2_CLVF_n_T *localB)
{
  real_T Ox_tmp_tmp;
  real_T Ox_tmp_tmp_0;

  // MATLAB Function 'Phase #2:  Move to  Initial Position/Change BLACK Behavior/Hough Control/MATLAB Function2': '<S288>:1' 
  // '<S288>:1:6' Ox = [cos(z) sin(z); -sin(z) cos(z)]*[1;0];
  Ox_tmp_tmp = sin(rtu_z);
  Ox_tmp_tmp_0 = cos(rtu_z);
  localB->Ox[0] = 0.0;
  localB->Ox[0] += Ox_tmp_tmp_0;
  localB->Ox[0] += Ox_tmp_tmp * 0.0;
  localB->Ox[1] = 0.0;
  localB->Ox[1] += -Ox_tmp_tmp;
  localB->Ox[1] += Ox_tmp_tmp_0 * 0.0;

  // '<S288>:1:7' Oy = [cos(z) sin(z); -sin(z) cos(z)]*[0;1];
  localB->Oy[0] = 0.0;
  localB->Oy[0] += cos(rtu_z) * 0.0;
  localB->Oy[0] += Ox_tmp_tmp;
  localB->Oy[1] = 0.0;
  localB->Oy[1] += -sin(rtu_z) * 0.0;
  localB->Oy[1] += Ox_tmp_tmp_0;
}

//
// Output and update for atomic system:
//    '<S281>/MATLAB Function3'
//    '<S294>/MATLAB Function3'
//    '<S307>/MATLAB Function3'
//    '<S329>/MATLAB Function3'
//    '<S340>/MATLAB Function3'
//    '<S382>/MATLAB Function3'
//    '<S395>/MATLAB Function3'
//    '<S413>/MATLAB Function3'
//    '<S426>/MATLAB Function3'
//    '<S444>/MATLAB Function3'
//    ...
//
void CLVF_MATLABFunction3(real_T rtu_z, B_MATLABFunction3_CLVF_T *localB)
{
  real_T Ox_tmp;
  real_T Ox_tmp_0;

  // MATLAB Function 'Phase #2:  Move to  Initial Position/Change BLACK Behavior/Hough Control/MATLAB Function3': '<S289>:1' 
  // '<S289>:1:6' Ox = [cos(z) sin(z); -sin(z) cos(z)]*[1;0];
  Ox_tmp = sin(rtu_z);
  Ox_tmp_0 = cos(rtu_z);
  localB->Ox[0] = 0.0;
  localB->Ox[0] += Ox_tmp_0;
  localB->Ox[0] += Ox_tmp * 0.0;
  localB->Ox[1] = 0.0;
  localB->Ox[1] += -Ox_tmp;
  localB->Ox[1] += Ox_tmp_0 * 0.0;
}

//
// Output and update for atomic system:
//    '<S281>/MATLAB Function4'
//    '<S294>/MATLAB Function4'
//    '<S307>/MATLAB Function4'
//    '<S329>/MATLAB Function4'
//    '<S340>/MATLAB Function4'
//    '<S382>/MATLAB Function4'
//    '<S395>/MATLAB Function4'
//    '<S413>/MATLAB Function4'
//    '<S426>/MATLAB Function4'
//    '<S444>/MATLAB Function4'
//    ...
//
void CLVF_MATLABFunction4(const real_T rtu_Oy[2], const real_T rtu_e_in[2],
  B_MATLABFunction4_CLVF_T *localB)
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;

  // MATLAB Function 'Phase #2:  Move to  Initial Position/Change BLACK Behavior/Hough Control/MATLAB Function4': '<S290>:1' 
  // '<S290>:1:3' a = sign(Oy'*e_in);
  // '<S290>:1:4' e_out = -a*norm(e_in);
  scale = 3.3121686421112381E-170;
  absxk = fabs(rtu_e_in[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(rtu_e_in[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = rtu_Oy[0] * rtu_e_in[0] + rtu_Oy[1] * rtu_e_in[1];
  y = scale * sqrt(y);
  if (absxk < 0.0) {
    absxk = -1.0;
  } else if (absxk > 0.0) {
    absxk = 1.0;
  } else if (absxk == 0.0) {
    absxk = 0.0;
  } else {
    absxk = (rtNaN);
  }

  localB->e_out = -absxk * y;
}

//
// Output and update for action system:
//    '<S317>/Sub-Phase #2 '
//    '<S317>/Sub-Phase #3 '
//
void CLVF_SubPhase2(P_SubPhase2_CLVF_T *localP, real_T *rtd_BLACK_Fx, real_T
                    *rtd_BLACK_Fy, real_T *rtd_BLACK_Tz, real_T *rtd_Float_State)
{
  // DataStoreWrite: '<S322>/BLACK_Fx' incorporates:
  //   Constant: '<S322>/Constant'

  *rtd_BLACK_Fx = localP->Constant_Value;

  // DataStoreWrite: '<S322>/BLACK_Fy' incorporates:
  //   Constant: '<S322>/Constant'

  *rtd_BLACK_Fy = localP->Constant_Value;

  // DataStoreWrite: '<S322>/BLACK_Tz' incorporates:
  //   Constant: '<S322>/Constant'

  *rtd_BLACK_Tz = localP->Constant_Value;

  // DataStoreWrite: '<S322>/Data Store Write3' incorporates:
  //   Constant: '<S322>/Puck State'

  *rtd_Float_State = localP->PuckState_Value;
}

//
// Output and update for atomic system:
//    '<S340>/MATLAB Function2'
//    '<S426>/MATLAB Function2'
//
void CLVF_MATLABFunction2_c(real_T rtu_z, B_MATLABFunction2_CLVF_d_T *localB)
{
  real_T Ox_tmp_tmp;
  real_T Ox_tmp_tmp_0;

  // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Hough Control/MATLAB Function2': '<S346>:1' 
  // '<S346>:1:6' Ox = [cos(z) sin(z); -sin(z) cos(z)]*[1;0];
  Ox_tmp_tmp = sin(rtu_z);
  Ox_tmp_tmp_0 = cos(rtu_z);
  localB->Ox[0] = 0.0;
  localB->Ox[0] += Ox_tmp_tmp_0;
  localB->Ox[0] += Ox_tmp_tmp * 0.0;
  localB->Ox[1] = 0.0;
  localB->Ox[1] += -Ox_tmp_tmp;
  localB->Ox[1] += Ox_tmp_tmp_0 * 0.0;

  // '<S346>:1:7' Oy = [cos(z) sin(z); -sin(z) cos(z)]*[0;1];
  localB->Oy[0] = 0.0;
  localB->Oy[0] += cos(rtu_z) * 0.0;
  localB->Oy[0] += Ox_tmp_tmp;
  localB->Oy[1] = 0.0;
  localB->Oy[1] += -sin(rtu_z) * 0.0;
  localB->Oy[1] += Ox_tmp_tmp_0;
}

//
// System initialize for action system:
//    '<S318>/Sub-Phase #1'
//    '<S318>/Sub-Phase #4'
//
void CLVF_SubPhase1_Init(B_SubPhase1_CLVF_T *localB, DW_SubPhase1_CLVF_T
  *localDW, P_SubPhase1_CLVF_T *localP)
{
  // Start for If: '<S385>/if we went through a "step"'
  localDW->ifwewentthroughastep_ActiveSubs = -1;

  // Start for If: '<S387>/if we went through a "step"'
  localDW->ifwewentthroughastep_ActiveSu_h = -1;

  // Start for If: '<S383>/if we went through a "step"'
  localDW->ifwewentthroughastep_ActiveSu_j = -1;

  // InitializeConditions for Delay: '<S385>/Delay1'
  localDW->icLoad = 1U;

  // InitializeConditions for Delay: '<S387>/Delay1'
  localDW->icLoad_k = 1U;

  // InitializeConditions for Delay: '<S383>/Delay1'
  localDW->icLoad_j = 1U;

  // SystemInitialize for IfAction SubSystem: '<S385>/Hold this value'
  // SystemInitialize for Outport: '<S386>/Out1'
  localB->In1_l = localP->Out1_Y0_l;

  // End of SystemInitialize for SubSystem: '<S385>/Hold this value'

  // SystemInitialize for IfAction SubSystem: '<S387>/Hold this value'
  // SystemInitialize for Outport: '<S388>/Out1'
  localB->In1 = localP->Out1_Y0_e;

  // End of SystemInitialize for SubSystem: '<S387>/Hold this value'

  // SystemInitialize for IfAction SubSystem: '<S383>/Hold this value'
  // SystemInitialize for Outport: '<S384>/Out1'
  localB->In1_g = localP->Out1_Y0;

  // End of SystemInitialize for SubSystem: '<S383>/Hold this value'
}

//
// Outputs for action system:
//    '<S318>/Sub-Phase #1'
//    '<S318>/Sub-Phase #4'
//
void CLVF_SubPhase1(B_SubPhase1_CLVF_T *localB, DW_SubPhase1_CLVF_T *localDW,
                    P_SubPhase1_CLVF_T *localP, real_T *rtd_BLUE_Fx, real_T
                    *rtd_BLUE_Fy, real_T *rtd_BLUE_Px, real_T *rtd_BLUE_Py,
                    real_T *rtd_BLUE_Rz, real_T *rtd_BLUE_Tz, real_T
                    *rtd_Float_State)
{
  real_T rtb_Sum6_os;
  real_T rtb_Subtract2_m[2];

  // Sum: '<S374>/Subtract' incorporates:
  //   Constant: '<S374>/Desired Px (BLUE)'

  localB->Subtract = CLVF_P.init_states_BLUE[0] - *rtd_BLUE_Px;

  // Delay: '<S385>/Delay1'
  if (localDW->icLoad != 0) {
    localDW->Delay1_DSTATE = localB->Subtract;
  }

  localB->Delay1 = localDW->Delay1_DSTATE;

  // End of Delay: '<S385>/Delay1'

  // Sum: '<S385>/Sum6'
  rtb_Sum6_os = localB->Subtract - localB->Delay1;

  // If: '<S385>/if we went through a "step"' incorporates:
  //   Inport: '<S386>/In1'

  localDW->ifwewentthroughastep_ActiveSubs = -1;
  if (rtb_Sum6_os != 0.0) {
    localDW->ifwewentthroughastep_ActiveSubs = 0;

    // Outputs for IfAction SubSystem: '<S385>/Hold this value' incorporates:
    //   ActionPort: '<S386>/Action Port'

    localB->In1_l = rtb_Sum6_os;

    // End of Outputs for SubSystem: '<S385>/Hold this value'
  }

  // End of If: '<S385>/if we went through a "step"'

  // Sum: '<S380>/Sum3' incorporates:
  //   Gain: '<S380>/kd_xb'
  //   Gain: '<S380>/kp_xb'
  //   Gain: '<S385>/divide by delta T'

  *rtd_BLUE_Fx = 1.0 / CLVF_P.serverRate * localB->In1_l * CLVF_P.Kd_xblue +
    CLVF_P.Kp_xblue * localB->Subtract;

  // Sum: '<S374>/Subtract1' incorporates:
  //   Constant: '<S374>/Desired Py (BLUE)'

  localB->Subtract1 = CLVF_P.init_states_BLUE[1] - *rtd_BLUE_Py;

  // Delay: '<S387>/Delay1'
  if (localDW->icLoad_k != 0) {
    localDW->Delay1_DSTATE_o = localB->Subtract1;
  }

  localB->Delay1_i = localDW->Delay1_DSTATE_o;

  // End of Delay: '<S387>/Delay1'

  // Sum: '<S387>/Sum6'
  rtb_Sum6_os = localB->Subtract1 - localB->Delay1_i;

  // If: '<S387>/if we went through a "step"' incorporates:
  //   Inport: '<S388>/In1'

  localDW->ifwewentthroughastep_ActiveSu_h = -1;
  if (rtb_Sum6_os != 0.0) {
    localDW->ifwewentthroughastep_ActiveSu_h = 0;

    // Outputs for IfAction SubSystem: '<S387>/Hold this value' incorporates:
    //   ActionPort: '<S388>/Action Port'

    localB->In1 = rtb_Sum6_os;

    // End of Outputs for SubSystem: '<S387>/Hold this value'
  }

  // End of If: '<S387>/if we went through a "step"'

  // Sum: '<S381>/Sum3' incorporates:
  //   Gain: '<S381>/kd_yb'
  //   Gain: '<S381>/kp_yb'
  //   Gain: '<S387>/divide by delta T'

  *rtd_BLUE_Fy = 1.0 / CLVF_P.serverRate * localB->In1 * CLVF_P.Kd_yblue +
    CLVF_P.Kp_yblue * localB->Subtract1;

  // MATLAB Function: '<S382>/MATLAB Function2' incorporates:
  //   Constant: '<S374>/Desired Attitude (BLUE)'

  CLVF_MATLABFunction2_o(CLVF_P.init_states_BLUE[2], &localB->sf_MATLABFunction2);

  // MATLAB Function: '<S382>/MATLAB Function3'
  CLVF_MATLABFunction3(*rtd_BLUE_Rz, &localB->sf_MATLABFunction3);

  // Sum: '<S382>/Subtract2'
  rtb_Subtract2_m[0] = localB->sf_MATLABFunction2.Ox[0] -
    localB->sf_MATLABFunction3.Ox[0];
  rtb_Subtract2_m[1] = localB->sf_MATLABFunction2.Ox[1] -
    localB->sf_MATLABFunction3.Ox[1];

  // MATLAB Function: '<S382>/MATLAB Function4'
  CLVF_MATLABFunction4(localB->sf_MATLABFunction2.Oy, rtb_Subtract2_m,
                       &localB->sf_MATLABFunction4);

  // Delay: '<S383>/Delay1'
  if (localDW->icLoad_j != 0) {
    localDW->Delay1_DSTATE_h = localB->sf_MATLABFunction4.e_out;
  }

  localB->Delay1_o = localDW->Delay1_DSTATE_h;

  // End of Delay: '<S383>/Delay1'

  // Sum: '<S383>/Sum6'
  rtb_Sum6_os = localB->sf_MATLABFunction4.e_out - localB->Delay1_o;

  // If: '<S383>/if we went through a "step"' incorporates:
  //   Inport: '<S384>/In1'

  localDW->ifwewentthroughastep_ActiveSu_j = -1;
  if (rtb_Sum6_os != 0.0) {
    localDW->ifwewentthroughastep_ActiveSu_j = 0;

    // Outputs for IfAction SubSystem: '<S383>/Hold this value' incorporates:
    //   ActionPort: '<S384>/Action Port'

    localB->In1_g = rtb_Sum6_os;

    // End of Outputs for SubSystem: '<S383>/Hold this value'
  }

  // End of If: '<S383>/if we went through a "step"'

  // Sum: '<S379>/Sum3' incorporates:
  //   Gain: '<S379>/kd_tb'
  //   Gain: '<S379>/kp_tb'
  //   Gain: '<S383>/divide by delta T'

  *rtd_BLUE_Tz = 1.0 / CLVF_P.serverRate * localB->In1_g * CLVF_P.Kd_tblue +
    CLVF_P.Kp_tblue * localB->sf_MATLABFunction4.e_out;

  // DataStoreWrite: '<S374>/Data Store Write1' incorporates:
  //   Constant: '<S374>/Puck State'

  *rtd_Float_State = localP->PuckState_Value;
}

//
// Update for action system:
//    '<S318>/Sub-Phase #1'
//    '<S318>/Sub-Phase #4'
//
void CLVF_SubPhase1_Update(B_SubPhase1_CLVF_T *localB, DW_SubPhase1_CLVF_T
  *localDW)
{
  // Update for Delay: '<S385>/Delay1'
  localDW->icLoad = 0U;
  localDW->Delay1_DSTATE = localB->Subtract;

  // Update for Delay: '<S387>/Delay1'
  localDW->icLoad_k = 0U;
  localDW->Delay1_DSTATE_o = localB->Subtract1;

  // Update for Delay: '<S383>/Delay1'
  localDW->icLoad_j = 0U;
  localDW->Delay1_DSTATE_h = localB->sf_MATLABFunction4.e_out;
}

//
// Output and update for action system:
//    '<S318>/Sub-Phase #2 '
//    '<S318>/Sub-Phase #3 '
//
void CLVF_SubPhase2_l(P_SubPhase2_CLVF_h_T *localP, real_T *rtd_BLUE_Fx, real_T *
                      rtd_BLUE_Fy, real_T *rtd_BLUE_Tz, real_T *rtd_Float_State)
{
  // DataStoreWrite: '<S375>/BLUE_Fx' incorporates:
  //   Constant: '<S375>/Constant'

  *rtd_BLUE_Fx = localP->Constant_Value;

  // DataStoreWrite: '<S375>/BLUE_Fy' incorporates:
  //   Constant: '<S375>/Constant'

  *rtd_BLUE_Fy = localP->Constant_Value;

  // DataStoreWrite: '<S375>/BLUE_Tz' incorporates:
  //   Constant: '<S375>/Constant'

  *rtd_BLUE_Tz = localP->Constant_Value;

  // DataStoreWrite: '<S375>/Data Store Write1' incorporates:
  //   Constant: '<S375>/Puck State'

  *rtd_Float_State = localP->PuckState_Value;
}

//
// Output and update for action system:
//    '<S319>/Sub-Phase #2 '
//    '<S319>/Sub-Phase #3 '
//
void CLVF_SubPhase2_d(P_SubPhase2_CLVF_e_T *localP, real_T *rtd_Float_State,
                      real_T *rtd_RED_Fx, real_T *rtd_RED_Fy, real_T *rtd_RED_Tz,
                      real_T *rtd_RED_Tz_Elbow, real_T *rtd_RED_Tz_RW, real_T
                      *rtd_RED_Tz_Shoulder, real_T *rtd_RED_Tz_Wrist)
{
  // DataStoreWrite: '<S406>/RED_Fx' incorporates:
  //   Constant: '<S406>/Constant'

  *rtd_RED_Fx = localP->Constant_Value;

  // DataStoreWrite: '<S406>/RED_Fy' incorporates:
  //   Constant: '<S406>/Constant'

  *rtd_RED_Fy = localP->Constant_Value;

  // DataStoreWrite: '<S406>/RED_Tz' incorporates:
  //   Constant: '<S406>/Constant'

  *rtd_RED_Tz = localP->Constant_Value;

  // DataStoreWrite: '<S406>/RED_Tz_RW' incorporates:
  //   Constant: '<S406>/Constant1'

  *rtd_RED_Tz_RW = localP->Constant1_Value;

  // DataStoreWrite: '<S406>/Data Store Write' incorporates:
  //   Constant: '<S406>/Constant4'

  *rtd_RED_Tz_Shoulder = localP->Constant4_Value;

  // DataStoreWrite: '<S406>/Data Store Write1' incorporates:
  //   Constant: '<S406>/Constant5'

  *rtd_RED_Tz_Elbow = localP->Constant5_Value;

  // DataStoreWrite: '<S406>/Data Store Write2' incorporates:
  //   Constant: '<S406>/Constant6'

  *rtd_RED_Tz_Wrist = localP->Constant6_Value;

  // DataStoreWrite: '<S406>/Data Store Write3' incorporates:
  //   Constant: '<S406>/Puck State'

  *rtd_Float_State = localP->PuckState_Value;
}

//
// System initialize for action system:
//    '<S12>/Change BLUE Behavior'
//    '<S13>/Change BLUE Behavior'
//
void C_ChangeBLUEBehavior_a_Init(B_ChangeBLUEBehavior_CLVF_k_T *localB,
  DW_ChangeBLUEBehavior_CLVF_i_T *localDW, P_ChangeBLUEBehavior_CLVF_c_T *localP)
{
  // Start for If: '<S460>/if we went through a "step"'
  localDW->ifwewentthroughastep_ActiveSubs = -1;

  // Start for If: '<S462>/if we went through a "step"'
  localDW->ifwewentthroughastep_ActiveSu_j = -1;

  // Start for If: '<S458>/if we went through a "step"'
  localDW->ifwewentthroughastep_ActiveSu_a = -1;

  // InitializeConditions for Delay: '<S460>/Delay1'
  localDW->icLoad = 1U;

  // InitializeConditions for Delay: '<S462>/Delay1'
  localDW->icLoad_h = 1U;

  // InitializeConditions for Delay: '<S458>/Delay1'
  localDW->icLoad_l = 1U;

  // SystemInitialize for IfAction SubSystem: '<S460>/Hold this value'
  // SystemInitialize for Outport: '<S461>/Out1'
  localB->In1_k = localP->Out1_Y0_k;

  // End of SystemInitialize for SubSystem: '<S460>/Hold this value'

  // SystemInitialize for IfAction SubSystem: '<S462>/Hold this value'
  // SystemInitialize for Outport: '<S463>/Out1'
  localB->In1 = localP->Out1_Y0_n;

  // End of SystemInitialize for SubSystem: '<S462>/Hold this value'

  // SystemInitialize for IfAction SubSystem: '<S458>/Hold this value'
  // SystemInitialize for Outport: '<S459>/Out1'
  localB->In1_a = localP->Out1_Y0;

  // End of SystemInitialize for SubSystem: '<S458>/Hold this value'
}

//
// Outputs for action system:
//    '<S12>/Change BLUE Behavior'
//    '<S13>/Change BLUE Behavior'
//
void CLVF_ChangeBLUEBehavior_g(B_ChangeBLUEBehavior_CLVF_k_T *localB,
  DW_ChangeBLUEBehavior_CLVF_i_T *localDW, P_ChangeBLUEBehavior_CLVF_c_T *localP,
  real_T *rtd_BLUE_Fx, real_T *rtd_BLUE_Fy, real_T *rtd_BLUE_Px, real_T
  *rtd_BLUE_Py, real_T *rtd_BLUE_Rz, real_T *rtd_BLUE_Tz, real_T
  *rtd_Float_State)
{
  real_T rtb_Sum6_cb;
  real_T rtb_Subtract2_d[2];

  // Sum: '<S438>/Subtract' incorporates:
  //   Constant: '<S438>/Desired Px (BLUE)'

  localB->Subtract = CLVF_P.home_states_BLUE[0] - *rtd_BLUE_Px;

  // Delay: '<S460>/Delay1'
  if (localDW->icLoad != 0) {
    localDW->Delay1_DSTATE = localB->Subtract;
  }

  localB->Delay1 = localDW->Delay1_DSTATE;

  // End of Delay: '<S460>/Delay1'

  // Sum: '<S460>/Sum6'
  rtb_Sum6_cb = localB->Subtract - localB->Delay1;

  // If: '<S460>/if we went through a "step"' incorporates:
  //   Inport: '<S461>/In1'

  localDW->ifwewentthroughastep_ActiveSubs = -1;
  if (rtb_Sum6_cb != 0.0) {
    localDW->ifwewentthroughastep_ActiveSubs = 0;

    // Outputs for IfAction SubSystem: '<S460>/Hold this value' incorporates:
    //   ActionPort: '<S461>/Action Port'

    localB->In1_k = rtb_Sum6_cb;

    // End of Outputs for SubSystem: '<S460>/Hold this value'
  }

  // End of If: '<S460>/if we went through a "step"'

  // Sum: '<S455>/Sum3' incorporates:
  //   Gain: '<S455>/kd_xb'
  //   Gain: '<S455>/kp_xb'
  //   Gain: '<S460>/divide by delta T'

  *rtd_BLUE_Fx = 1.0 / CLVF_P.serverRate * localB->In1_k * CLVF_P.Kd_xblue +
    CLVF_P.Kp_xblue * localB->Subtract;

  // Sum: '<S438>/Subtract1' incorporates:
  //   Constant: '<S438>/Desired Py (BLUE)'

  localB->Subtract1 = CLVF_P.home_states_BLUE[1] - *rtd_BLUE_Py;

  // Delay: '<S462>/Delay1'
  if (localDW->icLoad_h != 0) {
    localDW->Delay1_DSTATE_n = localB->Subtract1;
  }

  localB->Delay1_m = localDW->Delay1_DSTATE_n;

  // End of Delay: '<S462>/Delay1'

  // Sum: '<S462>/Sum6'
  rtb_Sum6_cb = localB->Subtract1 - localB->Delay1_m;

  // If: '<S462>/if we went through a "step"' incorporates:
  //   Inport: '<S463>/In1'

  localDW->ifwewentthroughastep_ActiveSu_j = -1;
  if (rtb_Sum6_cb != 0.0) {
    localDW->ifwewentthroughastep_ActiveSu_j = 0;

    // Outputs for IfAction SubSystem: '<S462>/Hold this value' incorporates:
    //   ActionPort: '<S463>/Action Port'

    localB->In1 = rtb_Sum6_cb;

    // End of Outputs for SubSystem: '<S462>/Hold this value'
  }

  // End of If: '<S462>/if we went through a "step"'

  // Sum: '<S456>/Sum3' incorporates:
  //   Gain: '<S456>/kd_yb'
  //   Gain: '<S456>/kp_yb'
  //   Gain: '<S462>/divide by delta T'

  *rtd_BLUE_Fy = 1.0 / CLVF_P.serverRate * localB->In1 * CLVF_P.Kd_yblue +
    CLVF_P.Kp_yblue * localB->Subtract1;

  // MATLAB Function: '<S457>/MATLAB Function2' incorporates:
  //   Constant: '<S438>/Desired Attitude (BLUE)'

  CLVF_MATLABFunction2_o(CLVF_P.home_states_BLUE[2], &localB->sf_MATLABFunction2);

  // MATLAB Function: '<S457>/MATLAB Function3'
  CLVF_MATLABFunction3(*rtd_BLUE_Rz, &localB->sf_MATLABFunction3);

  // Sum: '<S457>/Subtract2'
  rtb_Subtract2_d[0] = localB->sf_MATLABFunction2.Ox[0] -
    localB->sf_MATLABFunction3.Ox[0];
  rtb_Subtract2_d[1] = localB->sf_MATLABFunction2.Ox[1] -
    localB->sf_MATLABFunction3.Ox[1];

  // MATLAB Function: '<S457>/MATLAB Function4'
  CLVF_MATLABFunction4(localB->sf_MATLABFunction2.Oy, rtb_Subtract2_d,
                       &localB->sf_MATLABFunction4);

  // Delay: '<S458>/Delay1'
  if (localDW->icLoad_l != 0) {
    localDW->Delay1_DSTATE_k = localB->sf_MATLABFunction4.e_out;
  }

  localB->Delay1_p = localDW->Delay1_DSTATE_k;

  // End of Delay: '<S458>/Delay1'

  // Sum: '<S458>/Sum6'
  rtb_Sum6_cb = localB->sf_MATLABFunction4.e_out - localB->Delay1_p;

  // If: '<S458>/if we went through a "step"' incorporates:
  //   Inport: '<S459>/In1'

  localDW->ifwewentthroughastep_ActiveSu_a = -1;
  if (rtb_Sum6_cb != 0.0) {
    localDW->ifwewentthroughastep_ActiveSu_a = 0;

    // Outputs for IfAction SubSystem: '<S458>/Hold this value' incorporates:
    //   ActionPort: '<S459>/Action Port'

    localB->In1_a = rtb_Sum6_cb;

    // End of Outputs for SubSystem: '<S458>/Hold this value'
  }

  // End of If: '<S458>/if we went through a "step"'

  // Sum: '<S454>/Sum3' incorporates:
  //   Gain: '<S454>/kd_tb'
  //   Gain: '<S454>/kp_tb'
  //   Gain: '<S458>/divide by delta T'

  *rtd_BLUE_Tz = 1.0 / CLVF_P.serverRate * localB->In1_a * CLVF_P.Kd_tblue +
    CLVF_P.Kp_tblue * localB->sf_MATLABFunction4.e_out;

  // DataStoreWrite: '<S438>/Data Store Write3' incorporates:
  //   Constant: '<S438>/Puck State'

  *rtd_Float_State = localP->PuckState_Value;
}

//
// Update for action system:
//    '<S12>/Change BLUE Behavior'
//    '<S13>/Change BLUE Behavior'
//
void C_ChangeBLUEBehavior_Update(B_ChangeBLUEBehavior_CLVF_k_T *localB,
  DW_ChangeBLUEBehavior_CLVF_i_T *localDW)
{
  // Update for Delay: '<S460>/Delay1'
  localDW->icLoad = 0U;
  localDW->Delay1_DSTATE = localB->Subtract;

  // Update for Delay: '<S462>/Delay1'
  localDW->icLoad_h = 0U;
  localDW->Delay1_DSTATE_n = localB->Subtract1;

  // Update for Delay: '<S458>/Delay1'
  localDW->icLoad_l = 0U;
  localDW->Delay1_DSTATE_k = localB->sf_MATLABFunction4.e_out;
}

static real_T CLVF_rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      CLVF_B.u0 = 1;
    } else {
      CLVF_B.u0 = -1;
    }

    if (u1 > 0.0) {
      CLVF_B.u1 = 1;
    } else {
      CLVF_B.u1 = -1;
    }

    y = atan2(static_cast<real_T>(CLVF_B.u0), static_cast<real_T>(CLVF_B.u1));
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

static real_T CLVF_rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  // Uniform random number generator (random number between 0 and 1)

  // #define IA      16807                      magic multiplier = 7^5
  // #define IM      2147483647                 modulus = 2^31-1
  // #define IQ      127773                     IM div IA
  // #define IR      2836                       IM modulo IA
  // #define S       4.656612875245797e-10      reciprocal of 2^31-1
  // test = IA * (seed % IQ) - IR * (seed/IQ)
  // seed = test < 0 ? (test + IM) : test
  // return (seed*S)

  CLVF_B.lo = *u % 127773U * 16807U;
  CLVF_B.hi = *u / 127773U * 2836U;
  if (CLVF_B.lo < CLVF_B.hi) {
    *u = 2147483647U - (CLVF_B.hi - CLVF_B.lo);
  } else {
    *u = CLVF_B.lo - CLVF_B.hi;
  }

  return static_cast<real_T>(*u) * 4.6566128752457969E-10;
}

static real_T CLVF_rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T y;

  // Normal (Gaussian) random number generator
  do {
    CLVF_B.sr = 2.0 * CLVF_rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    CLVF_B.si = 2.0 * CLVF_rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    CLVF_B.si = CLVF_B.sr * CLVF_B.sr + CLVF_B.si * CLVF_B.si;
  } while (CLVF_B.si > 1.0);

  y = sqrt(-2.0 * log(CLVF_B.si) / CLVF_B.si) * CLVF_B.sr;
  return y;
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

static void CLVF_rt_invd3x3_snf(const real_T u[9], real_T y[9])
{
  memcpy(&CLVF_B.x[0], &u[0], 9U * sizeof(real_T));
  CLVF_B.p1 = 1;
  CLVF_B.p2 = 3;
  CLVF_B.p3 = 6;
  CLVF_B.absx11 = fabs(u[0]);
  CLVF_B.absx21 = fabs(u[1]);
  CLVF_B.absx31 = fabs(u[2]);
  if ((CLVF_B.absx21 > CLVF_B.absx11) && (CLVF_B.absx21 > CLVF_B.absx31)) {
    CLVF_B.p1 = 4;
    CLVF_B.p2 = 0;
    CLVF_B.x[0] = u[1];
    CLVF_B.x[1] = u[0];
    CLVF_B.x[3] = u[4];
    CLVF_B.x[4] = u[3];
    CLVF_B.x[6] = u[7];
    CLVF_B.x[7] = u[6];
  } else {
    if (CLVF_B.absx31 > CLVF_B.absx11) {
      CLVF_B.p1 = 7;
      CLVF_B.p3 = 0;
      CLVF_B.x[2] = CLVF_B.x[0];
      CLVF_B.x[0] = u[2];
      CLVF_B.x[5] = CLVF_B.x[3];
      CLVF_B.x[3] = u[5];
      CLVF_B.x[8] = CLVF_B.x[6];
      CLVF_B.x[6] = u[8];
    }
  }

  CLVF_B.absx31 = CLVF_B.x[1] / CLVF_B.x[0];
  CLVF_B.x[1] = CLVF_B.absx31;
  CLVF_B.absx11 = CLVF_B.x[2] / CLVF_B.x[0];
  CLVF_B.x[2] = CLVF_B.absx11;
  CLVF_B.x[4] -= CLVF_B.absx31 * CLVF_B.x[3];
  CLVF_B.x[5] -= CLVF_B.absx11 * CLVF_B.x[3];
  CLVF_B.x[7] -= CLVF_B.absx31 * CLVF_B.x[6];
  CLVF_B.x[8] -= CLVF_B.absx11 * CLVF_B.x[6];
  if (fabs(CLVF_B.x[5]) > fabs(CLVF_B.x[4])) {
    CLVF_B.itmp = CLVF_B.p2;
    CLVF_B.p2 = CLVF_B.p3;
    CLVF_B.p3 = CLVF_B.itmp;
    CLVF_B.x[1] = CLVF_B.absx11;
    CLVF_B.x[2] = CLVF_B.absx31;
    CLVF_B.absx11 = CLVF_B.x[4];
    CLVF_B.x[4] = CLVF_B.x[5];
    CLVF_B.x[5] = CLVF_B.absx11;
    CLVF_B.absx11 = CLVF_B.x[7];
    CLVF_B.x[7] = CLVF_B.x[8];
    CLVF_B.x[8] = CLVF_B.absx11;
  }

  CLVF_B.absx31 = CLVF_B.x[5] / CLVF_B.x[4];
  CLVF_B.x[8] -= CLVF_B.absx31 * CLVF_B.x[7];
  CLVF_B.absx11 = (CLVF_B.absx31 * CLVF_B.x[1] - CLVF_B.x[2]) / CLVF_B.x[8];
  CLVF_B.absx21 = -(CLVF_B.x[7] * CLVF_B.absx11 + CLVF_B.x[1]) / CLVF_B.x[4];
  y[CLVF_B.p1 - 1] = ((1.0 - CLVF_B.x[3] * CLVF_B.absx21) - CLVF_B.x[6] *
                      CLVF_B.absx11) / CLVF_B.x[0];
  y[CLVF_B.p1] = CLVF_B.absx21;
  y[CLVF_B.p1 + 1] = CLVF_B.absx11;
  CLVF_B.absx11 = -CLVF_B.absx31 / CLVF_B.x[8];
  CLVF_B.absx21 = (1.0 - CLVF_B.x[7] * CLVF_B.absx11) / CLVF_B.x[4];
  y[CLVF_B.p2] = -(CLVF_B.x[3] * CLVF_B.absx21 + CLVF_B.x[6] * CLVF_B.absx11) /
    CLVF_B.x[0];
  y[CLVF_B.p2 + 1] = CLVF_B.absx21;
  y[CLVF_B.p2 + 2] = CLVF_B.absx11;
  CLVF_B.absx11 = 1.0 / CLVF_B.x[8];
  CLVF_B.absx21 = -CLVF_B.x[7] * CLVF_B.absx11 / CLVF_B.x[4];
  y[CLVF_B.p3] = -(CLVF_B.x[3] * CLVF_B.absx21 + CLVF_B.x[6] * CLVF_B.absx11) /
    CLVF_B.x[0];
  y[CLVF_B.p3 + 1] = CLVF_B.absx21;
  y[CLVF_B.p3 + 2] = CLVF_B.absx11;
}

static void CLVF_SystemCore_release(const codertarget_linux_blocks_Digi_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    MW_gpioTerminate(10U);
  }
}

static void CLVF_SystemCore_delete(const codertarget_linux_blocks_Digi_T *obj)
{
  CLVF_SystemCore_release(obj);
}

static void matlabCodegenHandle_matlabCodeg(codertarget_linux_blocks_Digi_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    CLVF_SystemCore_delete(obj);
  }
}

static void CLVF_SystemCore_release_g(const codertarget_linux_blocks_Digi_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    MW_gpioTerminate(26U);
  }
}

static void CLVF_SystemCore_delete_g(const codertarget_linux_blocks_Digi_T *obj)
{
  CLVF_SystemCore_release_g(obj);
}

static void matlabCodegenHandle_matlabCod_g(codertarget_linux_blocks_Digi_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    CLVF_SystemCore_delete_g(obj);
  }
}

static void SystemCore_rele_gffipocpdxml04g(const PhaseSpace_ALL_CLVF_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    terminate_phasespace();
  }
}

static void SystemCore_dele_gffipocpdxml04g(const PhaseSpace_ALL_CLVF_T *obj)
{
  SystemCore_rele_gffipocpdxml04g(obj);
}

static void matlabCodegenHa_gffipocpdxml04g(PhaseSpace_ALL_CLVF_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    SystemCore_dele_gffipocpdxml04g(obj);
  }
}

static void C_SystemCore_rele_a(const raspi_internal_lsm9ds1Block_C_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    MW_I2C_Close(obj->i2cobj_A_G.MW_I2C_HANDLE);
    MW_I2C_Close(obj->i2cobj_MAG.MW_I2C_HANDLE);
  }
}

static void C_SystemCore_dele_a(const raspi_internal_lsm9ds1Block_C_T *obj)
{
  C_SystemCore_rele_a(obj);
}

static void CLVF_matlabCodegenHa_a(raspi_internal_lsm9ds1Block_C_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    C_SystemCore_dele_a(obj);
  }
}

static void C_SystemCore_rele_b(j_codertarget_raspi_internal__T *obj)
{
  if (obj->isInitialized == 1) {
    obj->isInitialized = 2;
  }
}

static void C_SystemCore_dele_b(j_codertarget_raspi_internal__T *obj)
{
  C_SystemCore_rele_b(obj);
}

static void CLVF_matlabCodegenHa_b(j_codertarget_raspi_internal__T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    C_SystemCore_dele_b(obj);
  }
}

static void C_SystemCore_rele_f(i_codertarget_raspi_internal__T *obj)
{
  if (obj->isInitialized == 1) {
    obj->isInitialized = 2;
  }
}

static void C_SystemCore_dele_f(i_codertarget_raspi_internal__T *obj)
{
  C_SystemCore_rele_f(obj);
}

static void CLVF_matlabCodegenHa_f(i_codertarget_raspi_internal__T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    C_SystemCore_dele_f(obj);
  }
}

static void CLVF_SystemCore_release_gf(const raspi_internal_PWMBlock_CLVF_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    EXT_PWMBlock_terminate(obj->PinNumber);
  }
}

static void CLVF_SystemCore_delete_gf(const raspi_internal_PWMBlock_CLVF_T *obj)
{
  CLVF_SystemCore_release_gf(obj);
}

static void matlabCodegenHandle_matlabCo_gf(raspi_internal_PWMBlock_CLVF_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    CLVF_SystemCore_delete_gf(obj);
  }
}

static void SystemCore_release_gffipocpdxm(const codertarget_linux_blocks_Di_g_T
  *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    MW_gpioTerminate(24U);
  }
}

static void C_SystemCore_delete_gffipocpdxm(const
  codertarget_linux_blocks_Di_g_T *obj)
{
  SystemCore_release_gffipocpdxm(obj);
}

static void matlabCodegenHandle_gffipocpdxm(codertarget_linux_blocks_Di_g_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    C_SystemCore_delete_gffipocpdxm(obj);
  }
}

static void SystemCore_release_gffipocpdxml(const
  codertarget_linux_blocks_Digi_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    MW_gpioTerminate(21U);
  }
}

static void SystemCore_delete_gffipocpdxml(const codertarget_linux_blocks_Digi_T
  *obj)
{
  SystemCore_release_gffipocpdxml(obj);
}

static void matlabCodegenHandl_gffipocpdxml(codertarget_linux_blocks_Digi_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    SystemCore_delete_gffipocpdxml(obj);
  }
}

static void SystemCore_relea_gffipocpdxml04(const
  codertarget_linux_blocks_Digi_T *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    MW_gpioTerminate(25U);
  }
}

static void SystemCore_delet_gffipocpdxml04(const
  codertarget_linux_blocks_Digi_T *obj)
{
  SystemCore_relea_gffipocpdxml04(obj);
}

static void matlabCodegenHan_gffipocpdxml04(codertarget_linux_blocks_Digi_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    SystemCore_delet_gffipocpdxml04(obj);
  }
}

static raspi_internal_lsm9ds1Block_C_T *CLVF_lsm9ds1Block_lsm9ds1Block
  (raspi_internal_lsm9ds1Block_C_T *obj)
{
  raspi_internal_lsm9ds1Block_C_T *b_obj;
  int32_T i;
  static const int8_T tmp[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  for (i = 0; i < 9; i++) {
    obj->CalGyroA[i] = tmp[i];
  }

  obj->CalGyroB[0] = 0.0;
  obj->CalGyroB[1] = 0.0;
  obj->CalGyroB[2] = 0.0;
  for (i = 0; i < 9; i++) {
    obj->CalAccelA[i] = tmp[i];
  }

  obj->CalAccelB[0] = 0.0;
  obj->CalAccelB[1] = 0.0;
  obj->CalAccelB[2] = 0.0;
  for (i = 0; i < 9; i++) {
    obj->CalMagA[i] = tmp[i];
  }

  obj->CalMagB[0] = 0.0;
  obj->CalMagB[1] = 0.0;
  obj->CalMagB[2] = 0.0;
  obj->isInitialized = 0;
  b_obj = obj;
  obj->i2cobj_A_G.isInitialized = 0;
  obj->i2cobj_A_G.matlabCodegenIsDeleted = false;
  obj->i2cobj_MAG.isInitialized = 0;
  obj->i2cobj_MAG.matlabCodegenIsDeleted = false;
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static void CLVF_SystemCore_setup(raspi_internal_lsm9ds1Block_C_T *obj)
{
  MW_I2C_Mode_Type ModeType;
  uint32_T i2cname;
  uint8_T b_SwappedDataBytes[2];
  uint8_T CastedData;
  uint8_T SwappedDataBytes[2];
  obj->isSetupComplete = false;
  obj->isInitialized = 1;
  ModeType = MW_I2C_MASTER;
  i2cname = 1;
  obj->i2cobj_A_G.MW_I2C_HANDLE = MW_I2C_Open(i2cname, ModeType);
  obj->i2cobj_A_G.BusSpeed = 100000U;
  MW_I2C_SetBusSpeed(obj->i2cobj_A_G.MW_I2C_HANDLE, obj->i2cobj_A_G.BusSpeed);
  CastedData = 96U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 16U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_A_G.MW_I2C_HANDLE, 106U, SwappedDataBytes, 2U,
                     false, false);
  CastedData = 0U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 17U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_A_G.MW_I2C_HANDLE, 106U, SwappedDataBytes, 2U,
                     false, false);
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 18U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_A_G.MW_I2C_HANDLE, 106U, SwappedDataBytes, 2U,
                     false, false);
  CastedData = 56U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 30U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_A_G.MW_I2C_HANDLE, 106U, SwappedDataBytes, 2U,
                     false, false);
  CastedData = 0U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 19U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_A_G.MW_I2C_HANDLE, 106U, SwappedDataBytes, 2U,
                     false, false);
  CastedData = 56U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 31U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_A_G.MW_I2C_HANDLE, 106U, SwappedDataBytes, 2U,
                     false, false);
  CastedData = 103U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 32U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_A_G.MW_I2C_HANDLE, 106U, SwappedDataBytes, 2U,
                     false, false);
  CastedData = 0U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 33U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_A_G.MW_I2C_HANDLE, 106U, SwappedDataBytes, 2U,
                     false, false);
  CastedData = 4U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 34U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_A_G.MW_I2C_HANDLE, 106U, SwappedDataBytes, 2U,
                     false, false);
  ModeType = MW_I2C_MASTER;
  i2cname = 1;
  obj->i2cobj_MAG.MW_I2C_HANDLE = MW_I2C_Open(i2cname, ModeType);
  obj->i2cobj_MAG.BusSpeed = 100000U;
  MW_I2C_SetBusSpeed(obj->i2cobj_MAG.MW_I2C_HANDLE, obj->i2cobj_MAG.BusSpeed);
  CastedData = 24U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 32U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_MAG.MW_I2C_HANDLE, 28U, SwappedDataBytes, 2U,
                     false, false);
  CastedData = 0U;
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 33U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_MAG.MW_I2C_HANDLE, 28U, SwappedDataBytes, 2U,
                     false, false);
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 34U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_MAG.MW_I2C_HANDLE, 28U, SwappedDataBytes, 2U,
                     false, false);
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 35U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_MAG.MW_I2C_HANDLE, 28U, SwappedDataBytes, 2U,
                     false, false);
  memcpy((void *)&b_SwappedDataBytes[1], (void *)&CastedData, (uint32_T)((size_t)
          1 * sizeof(uint8_T)));
  b_SwappedDataBytes[0] = 36U;
  memcpy((void *)&SwappedDataBytes[0], (void *)&b_SwappedDataBytes[0], (uint32_T)
         ((size_t)2 * sizeof(uint8_T)));
  MW_I2C_MasterWrite(obj->i2cobj_MAG.MW_I2C_HANDLE, 28U, SwappedDataBytes, 2U,
                     false, false);
  obj->isSetupComplete = true;
}

// Model step function for TID0
void CLVF_step0(void)                  // Sample time: [0.0s, 0.0s]
{
  {                                    // Sample time: [0.0s, 0.0s]
    rate_monotonic_scheduler();
  }

  {
    char_T *sErr;
    real_T (*lastU)[3];
    static const int8_T b[3] = { 0, 0, 1 };

    static const int8_T b_0[8] = { -1, -1, 0, 0, 1, 1, 0, 0 };

    static const int8_T c[8] = { 0, 0, 1, 1, 0, 0, -1, -1 };

    static const int8_T b_1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    // If: '<Root>/Separate Phases' incorporates:
    //   Constant: '<Root>/Constant'
    //   Constant: '<Root>/Constant1'
    //   Constant: '<Root>/Constant2'
    //   Constant: '<Root>/Constant3'
    //   Constant: '<Root>/Constant4'
    //   Constant: '<Root>/Constant6'
    //   DataStoreRead: '<S17>/Universal_Time'

    CLVF_B.rtPrevAction = CLVF_DW.SeparatePhases_ActiveSubsystem;
    if (CLVF_DW.Univ_Time < CLVF_P.Phase0_End) {
      CLVF_B.rtAction = 0;
    } else if (CLVF_DW.Univ_Time < CLVF_P.Phase1_End) {
      CLVF_B.rtAction = 1;
    } else if (CLVF_DW.Univ_Time < CLVF_P.Phase2_End) {
      CLVF_B.rtAction = 2;
    } else if (CLVF_DW.Univ_Time < CLVF_P.Phase3_End) {
      CLVF_B.rtAction = 3;
    } else if (CLVF_DW.Univ_Time < CLVF_P.Phase4_End) {
      CLVF_B.rtAction = 4;
    } else if (CLVF_DW.Univ_Time < CLVF_P.Phase5_End) {
      CLVF_B.rtAction = 5;
    } else {
      CLVF_B.rtAction = 6;
    }

    CLVF_DW.SeparatePhases_ActiveSubsystem = CLVF_B.rtAction;
    if ((CLVF_B.rtPrevAction != CLVF_B.rtAction) && (CLVF_B.rtPrevAction == 3))
    {
      // Disable for If: '<S11>/This IF block determines whether or not to run the BLACK sim//exp' 
      if (CLVF_DW.ThisIFblockdetermineswhethero_i == 0) {
        // Disable for If: '<S317>/Experiment Sub-Phases'
        if (CLVF_DW.ExperimentSubPhases_ActiveSub_p == 3) {
          // Disable for If: '<S349>/If'
          CLVF_DW.If_ActiveSubsystem = -1;
        }

        CLVF_DW.ExperimentSubPhases_ActiveSub_p = -1;
      }

      CLVF_DW.ThisIFblockdetermineswhethero_i = -1;

      // End of Disable for If: '<S11>/This IF block determines whether or not to run the BLACK sim//exp' 

      // Disable for If: '<S11>/This IF block determines whether or not to run the BLUE sim//exp' 
      CLVF_DW.ThisIFblockdetermineswhether_aa = -1;

      // Disable for If: '<S11>/This IF block determines whether or not to run the RED sim//exp ' 
      CLVF_DW.ThisIFblockdetermineswhethero_f = -1;
    }

    switch (CLVF_B.rtAction) {
     case 0:
      // Outputs for IfAction SubSystem: '<Root>/Phase #0:  Wait for Synchronization' incorporates:
      //   ActionPort: '<S8>/Action Port'

      Phase0WaitforSynchronizatio(&CLVF_DW.Phase0WaitforSynchronization,
        &CLVF_P.Phase0WaitforSynchronization, &CLVF_DW.BLACK_Fx,
        &CLVF_DW.BLACK_Fy, &CLVF_DW.BLACK_Tz, &CLVF_DW.BLUE_Fx, &CLVF_DW.BLUE_Fy,
        &CLVF_DW.BLUE_Tz, &CLVF_DW.Float_State, &CLVF_DW.RED_Fx, &CLVF_DW.RED_Fy,
        &CLVF_DW.RED_Tz, &CLVF_B.RED_Tz_Elbow, &CLVF_DW.RED_Tz_RW,
        &CLVF_B.RED_Tz_Shoulder, &CLVF_B.RED_Tz_Wrist);

      // End of Outputs for SubSystem: '<Root>/Phase #0:  Wait for Synchronization' 
      break;

     case 1:
      // Outputs for IfAction SubSystem: '<Root>/Phase #1:  Start Floating ' incorporates:
      //   ActionPort: '<S9>/Action Port'

      Phase0WaitforSynchronizatio(&CLVF_DW.Phase1StartFloating,
        &CLVF_P.Phase1StartFloating, &CLVF_DW.BLACK_Fx, &CLVF_DW.BLACK_Fy,
        &CLVF_DW.BLACK_Tz, &CLVF_DW.BLUE_Fx, &CLVF_DW.BLUE_Fy, &CLVF_DW.BLUE_Tz,
        &CLVF_DW.Float_State, &CLVF_DW.RED_Fx, &CLVF_DW.RED_Fy, &CLVF_DW.RED_Tz,
        &CLVF_B.RED_Tz_Elbow, &CLVF_DW.RED_Tz_RW, &CLVF_B.RED_Tz_Shoulder,
        &CLVF_B.RED_Tz_Wrist);

      // End of Outputs for SubSystem: '<Root>/Phase #1:  Start Floating '
      break;

     case 2:
      // Outputs for IfAction SubSystem: '<Root>/Phase #2:  Move to  Initial Position' incorporates:
      //   ActionPort: '<S10>/Action Port'

      // If: '<S10>/This IF block determines whether or not to run the BLACK sim//exp' incorporates:
      //   Constant: '<S10>/Constant'
      //   Constant: '<S277>/Constant'

      CLVF_DW.ThisIFblockdetermineswhether_dy = -1;
      if ((CLVF_P.WhoAmI == 2.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhether_dy = 0;

        // Outputs for IfAction SubSystem: '<S10>/Change BLACK Behavior' incorporates:
        //   ActionPort: '<S274>/Action Port'

        // Sum: '<S274>/Subtract' incorporates:
        //   Constant: '<S274>/Desired Px (BLACK)'
        //   DataStoreRead: '<S274>/BLACK_Px'

        CLVF_B.Subtract_ou = CLVF_P.init_states_BLACK[0] - CLVF_DW.BLACK_Px;

        // Delay: '<S284>/Delay1'
        if (CLVF_DW.icLoad_ns != 0) {
          CLVF_DW.Delay1_DSTATE_k = CLVF_B.Subtract_ou;
        }

        // Sum: '<S284>/Sum6' incorporates:
        //   Delay: '<S284>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract_ou - CLVF_DW.Delay1_DSTATE_k;

        // If: '<S284>/if we went through a "step"' incorporates:
        //   Inport: '<S285>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S284>/Hold this value' incorporates:
          //   ActionPort: '<S285>/Action Port'

          CLVF_B.In1_ej = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S284>/Hold this value'
        }

        // End of If: '<S284>/if we went through a "step"'

        // Sum: '<S279>/Sum3' incorporates:
        //   DataStoreWrite: '<S274>/BLACK_Fx'
        //   Gain: '<S279>/kd_xb'
        //   Gain: '<S279>/kp_xb'
        //   Gain: '<S284>/divide by delta T'

        CLVF_DW.BLACK_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_ej *
          CLVF_P.Kd_xb + CLVF_P.Kp_xb * CLVF_B.Subtract_ou;

        // Sum: '<S274>/Subtract1' incorporates:
        //   Constant: '<S274>/Desired Py (BLACK)'
        //   DataStoreRead: '<S274>/BLACK_Py '

        CLVF_B.Subtract1_m = CLVF_P.init_states_BLACK[1] - CLVF_DW.BLACK_Py;

        // Delay: '<S286>/Delay1'
        if (CLVF_DW.icLoad_k1 != 0) {
          CLVF_DW.Delay1_DSTATE_dv = CLVF_B.Subtract1_m;
        }

        // Sum: '<S286>/Sum6' incorporates:
        //   Delay: '<S286>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract1_m - CLVF_DW.Delay1_DSTATE_dv;

        // If: '<S286>/if we went through a "step"' incorporates:
        //   Inport: '<S287>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S286>/Hold this value' incorporates:
          //   ActionPort: '<S287>/Action Port'

          CLVF_B.In1_gc = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S286>/Hold this value'
        }

        // End of If: '<S286>/if we went through a "step"'

        // Sum: '<S280>/Sum3' incorporates:
        //   DataStoreWrite: '<S274>/BLACK_Fy'
        //   Gain: '<S280>/kd_yb'
        //   Gain: '<S280>/kp_yb'
        //   Gain: '<S286>/divide by delta T'

        CLVF_DW.BLACK_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_gc *
          CLVF_P.Kd_yb + CLVF_P.Kp_yb * CLVF_B.Subtract1_m;

        // MATLAB Function: '<S281>/MATLAB Function2' incorporates:
        //   Constant: '<S274>/Desired Attitude (BLACK)'

        CLVF_MATLABFunction2_o(CLVF_P.init_states_BLACK[2],
          &CLVF_B.sf_MATLABFunction2_o);

        // MATLAB Function: '<S281>/MATLAB Function3' incorporates:
        //   DataStoreRead: '<S274>/BLACK_Rz'

        CLVF_MATLABFunction3(CLVF_DW.BLACK_Rz, &CLVF_B.sf_MATLABFunction3);

        // Sum: '<S281>/Subtract2'
        CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_o.Ox[0] -
          CLVF_B.sf_MATLABFunction3.Ox[0];
        CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_o.Ox[1] -
          CLVF_B.sf_MATLABFunction3.Ox[1];

        // MATLAB Function: '<S281>/MATLAB Function4'
        CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_o.Oy,
                             CLVF_B.rtb_Subtract2_l_g,
                             &CLVF_B.sf_MATLABFunction4);

        // Delay: '<S282>/Delay1'
        if (CLVF_DW.icLoad_ep != 0) {
          CLVF_DW.Delay1_DSTATE_j = CLVF_B.sf_MATLABFunction4.e_out;
        }

        // Sum: '<S282>/Sum6' incorporates:
        //   Delay: '<S282>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.sf_MATLABFunction4.e_out -
          CLVF_DW.Delay1_DSTATE_j;

        // If: '<S282>/if we went through a "step"' incorporates:
        //   Inport: '<S283>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S282>/Hold this value' incorporates:
          //   ActionPort: '<S283>/Action Port'

          CLVF_B.In1_fa = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S282>/Hold this value'
        }

        // End of If: '<S282>/if we went through a "step"'

        // Sum: '<S278>/Sum3' incorporates:
        //   DataStoreWrite: '<S274>/BLACK_Tz'
        //   Gain: '<S278>/kd_tb'
        //   Gain: '<S278>/kp_tb'
        //   Gain: '<S282>/divide by delta T'

        CLVF_DW.BLACK_Tz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_fa *
          CLVF_P.Kd_tb + CLVF_P.Kp_tb * CLVF_B.sf_MATLABFunction4.e_out;

        // DataStoreWrite: '<S274>/Data Store Write3' incorporates:
        //   Constant: '<S274>/Puck State'

        CLVF_DW.Float_State = CLVF_P.PuckState_Value;

        // End of Outputs for SubSystem: '<S10>/Change BLACK Behavior'
      }

      // End of If: '<S10>/This IF block determines whether or not to run the BLACK sim//exp' 

      // If: '<S10>/This IF block determines whether or not to run the BLUE sim//exp' incorporates:
      //   Constant: '<S10>/Constant'
      //   Constant: '<S277>/Constant'

      CLVF_DW.ThisIFblockdetermineswhether_jm = -1;
      if ((CLVF_P.WhoAmI == 3.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhether_jm = 0;

        // Outputs for IfAction SubSystem: '<S10>/Change BLUE Behavior' incorporates:
        //   ActionPort: '<S275>/Action Port'

        // Sum: '<S275>/Subtract' incorporates:
        //   Constant: '<S275>/Desired Px (BLUE)'
        //   DataStoreRead: '<S275>/BLUE_Px'

        CLVF_B.Subtract_j = CLVF_P.init_states_BLUE[0] - CLVF_DW.BLUE_Px;

        // Delay: '<S297>/Delay1'
        if (CLVF_DW.icLoad_dy != 0) {
          CLVF_DW.Delay1_DSTATE_fb = CLVF_B.Subtract_j;
        }

        // Sum: '<S297>/Sum6' incorporates:
        //   Delay: '<S297>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract_j - CLVF_DW.Delay1_DSTATE_fb;

        // If: '<S297>/if we went through a "step"' incorporates:
        //   Inport: '<S298>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S297>/Hold this value' incorporates:
          //   ActionPort: '<S298>/Action Port'

          CLVF_B.In1_lc = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S297>/Hold this value'
        }

        // End of If: '<S297>/if we went through a "step"'

        // Sum: '<S292>/Sum3' incorporates:
        //   DataStoreWrite: '<S275>/BLUE_Fx'
        //   Gain: '<S292>/kd_xb'
        //   Gain: '<S292>/kp_xb'
        //   Gain: '<S297>/divide by delta T'

        CLVF_DW.BLUE_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_lc *
          CLVF_P.Kd_xblue + CLVF_P.Kp_xblue * CLVF_B.Subtract_j;

        // Sum: '<S275>/Subtract1' incorporates:
        //   Constant: '<S275>/Desired Py (BLUE)'
        //   DataStoreRead: '<S275>/BLUE_Py '

        CLVF_B.Subtract1_o = CLVF_P.init_states_BLUE[1] - CLVF_DW.BLUE_Py;

        // Delay: '<S299>/Delay1'
        if (CLVF_DW.icLoad_hf != 0) {
          CLVF_DW.Delay1_DSTATE_n1 = CLVF_B.Subtract1_o;
        }

        // Sum: '<S299>/Sum6' incorporates:
        //   Delay: '<S299>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract1_o - CLVF_DW.Delay1_DSTATE_n1;

        // If: '<S299>/if we went through a "step"' incorporates:
        //   Inport: '<S300>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S299>/Hold this value' incorporates:
          //   ActionPort: '<S300>/Action Port'

          CLVF_B.In1_jw = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S299>/Hold this value'
        }

        // End of If: '<S299>/if we went through a "step"'

        // Sum: '<S293>/Sum3' incorporates:
        //   DataStoreWrite: '<S275>/BLUE_Fy'
        //   Gain: '<S293>/kd_yb'
        //   Gain: '<S293>/kp_yb'
        //   Gain: '<S299>/divide by delta T'

        CLVF_DW.BLUE_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_jw *
          CLVF_P.Kd_yblue + CLVF_P.Kp_yblue * CLVF_B.Subtract1_o;

        // MATLAB Function: '<S294>/MATLAB Function2' incorporates:
        //   Constant: '<S275>/Desired Attitude (BLUE)'

        CLVF_MATLABFunction2_o(CLVF_P.init_states_BLUE[2],
          &CLVF_B.sf_MATLABFunction2_m);

        // MATLAB Function: '<S294>/MATLAB Function3' incorporates:
        //   DataStoreRead: '<S275>/BLUE_Rz'

        CLVF_MATLABFunction3(CLVF_DW.BLUE_Rz, &CLVF_B.sf_MATLABFunction3_c);

        // Sum: '<S294>/Subtract2'
        CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_m.Ox[0] -
          CLVF_B.sf_MATLABFunction3_c.Ox[0];
        CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_m.Ox[1] -
          CLVF_B.sf_MATLABFunction3_c.Ox[1];

        // MATLAB Function: '<S294>/MATLAB Function4'
        CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_m.Oy,
                             CLVF_B.rtb_Subtract2_l_g,
                             &CLVF_B.sf_MATLABFunction4_c);

        // Delay: '<S295>/Delay1'
        if (CLVF_DW.icLoad_k != 0) {
          CLVF_DW.Delay1_DSTATE_cq = CLVF_B.sf_MATLABFunction4_c.e_out;
        }

        // Sum: '<S295>/Sum6' incorporates:
        //   Delay: '<S295>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.sf_MATLABFunction4_c.e_out -
          CLVF_DW.Delay1_DSTATE_cq;

        // If: '<S295>/if we went through a "step"' incorporates:
        //   Inport: '<S296>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S295>/Hold this value' incorporates:
          //   ActionPort: '<S296>/Action Port'

          CLVF_B.In1_o = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S295>/Hold this value'
        }

        // End of If: '<S295>/if we went through a "step"'

        // Sum: '<S291>/Sum3' incorporates:
        //   DataStoreWrite: '<S275>/BLUE_Tz'
        //   Gain: '<S291>/kd_tb'
        //   Gain: '<S291>/kp_tb'
        //   Gain: '<S295>/divide by delta T'

        CLVF_DW.BLUE_Tz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_o *
          CLVF_P.Kd_tblue + CLVF_P.Kp_tblue * CLVF_B.sf_MATLABFunction4_c.e_out;

        // DataStoreWrite: '<S275>/Data Store Write3' incorporates:
        //   Constant: '<S275>/Puck State'

        CLVF_DW.Float_State = CLVF_P.PuckState_Value_o;

        // End of Outputs for SubSystem: '<S10>/Change BLUE Behavior'
      }

      // End of If: '<S10>/This IF block determines whether or not to run the BLUE sim//exp' 

      // If: '<S10>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
      //   Constant: '<S10>/Constant'
      //   Constant: '<S277>/Constant'

      CLVF_DW.ThisIFblockdetermineswhether_aj = -1;
      if ((CLVF_P.WhoAmI == 1.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhether_aj = 0;

        // Outputs for IfAction SubSystem: '<S10>/Change RED Behavior' incorporates:
        //   ActionPort: '<S276>/Action Port'

        // MATLAB Function: '<S307>/MATLAB Function2' incorporates:
        //   Constant: '<S276>/Constant'

        CLVF_MATLABFunction2_o(CLVF_P.init_states_RED[2],
          &CLVF_B.sf_MATLABFunction2_ol);

        // MATLAB Function: '<S307>/MATLAB Function3' incorporates:
        //   DataStoreRead: '<S276>/RED_Rz'

        CLVF_MATLABFunction3(CLVF_DW.RED_Rz, &CLVF_B.sf_MATLABFunction3_k);

        // Sum: '<S307>/Subtract2'
        CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_ol.Ox[0] -
          CLVF_B.sf_MATLABFunction3_k.Ox[0];
        CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_ol.Ox[1] -
          CLVF_B.sf_MATLABFunction3_k.Ox[1];

        // MATLAB Function: '<S307>/MATLAB Function4'
        CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_ol.Oy,
                             CLVF_B.rtb_Subtract2_l_g,
                             &CLVF_B.sf_MATLABFunction4_b);

        // Delay: '<S308>/Delay1'
        if (CLVF_DW.icLoad_ji != 0) {
          CLVF_DW.Delay1_DSTATE_ct = CLVF_B.sf_MATLABFunction4_b.e_out;
        }

        // Sum: '<S308>/Sum6' incorporates:
        //   Delay: '<S308>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.sf_MATLABFunction4_b.e_out -
          CLVF_DW.Delay1_DSTATE_ct;

        // If: '<S308>/if we went through a "step"' incorporates:
        //   Inport: '<S309>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S308>/Hold this value' incorporates:
          //   ActionPort: '<S309>/Action Port'

          CLVF_B.In1_k = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S308>/Hold this value'
        }

        // End of If: '<S308>/if we went through a "step"'

        // Sum: '<S304>/Sum3' incorporates:
        //   DataStoreWrite: '<S276>/RED_Tz'
        //   Gain: '<S304>/kd_tr'
        //   Gain: '<S304>/kp_tr'
        //   Gain: '<S308>/divide by delta T'

        CLVF_DW.RED_Tz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_k * CLVF_P.Kd_tr +
          CLVF_P.Kp_tr * CLVF_B.sf_MATLABFunction4_b.e_out;

        // Sum: '<S276>/Subtract' incorporates:
        //   Constant: '<S276>/Constant1'
        //   DataStoreRead: '<S276>/RED_Px'

        CLVF_B.Subtract_f = CLVF_P.init_states_RED[0] - CLVF_DW.RED_Px;

        // Delay: '<S310>/Delay1'
        if (CLVF_DW.icLoad_a1 != 0) {
          CLVF_DW.Delay1_DSTATE_bm = CLVF_B.Subtract_f;
        }

        // Sum: '<S310>/Sum6' incorporates:
        //   Delay: '<S310>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract_f - CLVF_DW.Delay1_DSTATE_bm;

        // If: '<S310>/if we went through a "step"' incorporates:
        //   Inport: '<S311>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S310>/Hold this value' incorporates:
          //   ActionPort: '<S311>/Action Port'

          CLVF_B.In1_cr = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S310>/Hold this value'
        }

        // End of If: '<S310>/if we went through a "step"'

        // Sum: '<S305>/Sum3' incorporates:
        //   DataStoreWrite: '<S276>/RED_Fx'
        //   Gain: '<S305>/kd_xr'
        //   Gain: '<S305>/kp_xr'
        //   Gain: '<S310>/divide by delta T'

        CLVF_DW.RED_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_cr * CLVF_P.Kd_xr
          + CLVF_P.Kp_xr * CLVF_B.Subtract_f;

        // DataStoreWrite: '<S276>/RED_Tz_RW1' incorporates:
        //   Constant: '<S276>/Constant2'

        CLVF_DW.RED_Tz_RW = CLVF_P.Constant2_Value;

        // Sum: '<S276>/Subtract1' incorporates:
        //   Constant: '<S276>/Constant3'
        //   DataStoreRead: '<S276>/RED_Py '

        CLVF_B.Subtract1_dr = CLVF_P.init_states_RED[1] - CLVF_DW.RED_Py;

        // Delay: '<S312>/Delay1'
        if (CLVF_DW.icLoad_o0 != 0) {
          CLVF_DW.Delay1_DSTATE_pr = CLVF_B.Subtract1_dr;
        }

        // Sum: '<S312>/Sum6' incorporates:
        //   Delay: '<S312>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract1_dr - CLVF_DW.Delay1_DSTATE_pr;

        // If: '<S312>/if we went through a "step"' incorporates:
        //   Inport: '<S313>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S312>/Hold this value' incorporates:
          //   ActionPort: '<S313>/Action Port'

          CLVF_B.In1_if = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S312>/Hold this value'
        }

        // End of If: '<S312>/if we went through a "step"'

        // Sum: '<S306>/Sum3' incorporates:
        //   DataStoreWrite: '<S276>/RED_Fy'
        //   Gain: '<S306>/kd_yr'
        //   Gain: '<S306>/kp_yr'
        //   Gain: '<S312>/divide by delta T'

        CLVF_DW.RED_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_if * CLVF_P.Kd_yr
          + CLVF_P.Kp_yr * CLVF_B.Subtract1_dr;

        // DataStoreWrite: '<S276>/Data Store Write3' incorporates:
        //   Constant: '<S276>/Puck State'

        CLVF_DW.Float_State = CLVF_P.PuckState_Value_l;

        // End of Outputs for SubSystem: '<S10>/Change RED Behavior'
      }

      // End of If: '<S10>/This IF block determines whether or not to run the RED sim//exp ' 
      // End of Outputs for SubSystem: '<Root>/Phase #2:  Move to  Initial Position' 
      break;

     case 3:
      // Outputs for IfAction SubSystem: '<Root>/Phase #3: Experiment' incorporates:
      //   ActionPort: '<S11>/Action Port'

      // If: '<S11>/This IF block determines whether or not to run the BLACK sim//exp' incorporates:
      //   Constant: '<S11>/Constant'
      //   Constant: '<S320>/Constant'

      CLVF_B.rtPrevAction = CLVF_DW.ThisIFblockdetermineswhethero_i;
      CLVF_B.rtAction = -1;
      if ((CLVF_P.WhoAmI == 2.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_B.rtAction = 0;
      }

      CLVF_DW.ThisIFblockdetermineswhethero_i = CLVF_B.rtAction;
      if ((CLVF_B.rtPrevAction != CLVF_B.rtAction) && (CLVF_B.rtPrevAction == 0))
      {
        // Disable for If: '<S317>/Experiment Sub-Phases'
        if (CLVF_DW.ExperimentSubPhases_ActiveSub_p == 3) {
          // Disable for If: '<S349>/If'
          CLVF_DW.If_ActiveSubsystem = -1;
        }

        CLVF_DW.ExperimentSubPhases_ActiveSub_p = -1;
      }

      if (CLVF_B.rtAction == 0) {
        // Outputs for IfAction SubSystem: '<S11>/Change BLACK Behavior' incorporates:
        //   ActionPort: '<S317>/Action Port'

        // If: '<S317>/Experiment Sub-Phases' incorporates:
        //   Constant: '<S317>/Constant1'
        //   Constant: '<S317>/Constant2'
        //   Constant: '<S317>/Constant3'
        //   Constant: '<S317>/Constant4'
        //   DataStoreRead: '<S325>/Universal_Time'
        //   Inport: '<S358>/In'
        //   MATLAB Function: '<S324>/getDesiredRotation'

        CLVF_B.rtPrevAction = CLVF_DW.ExperimentSubPhases_ActiveSub_p;
        CLVF_B.rtAction = -1;
        if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase1_End) {
          CLVF_B.rtAction = 0;
        } else if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase2_End) {
          CLVF_B.rtAction = 1;
        } else if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase3_End_BLACK) {
          CLVF_B.rtAction = 2;
        } else {
          if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase4_End) {
            CLVF_B.rtAction = 3;
          }
        }

        CLVF_DW.ExperimentSubPhases_ActiveSub_p = CLVF_B.rtAction;
        if ((CLVF_B.rtPrevAction != CLVF_B.rtAction) && (CLVF_B.rtPrevAction ==
             3)) {
          // Disable for If: '<S349>/If'
          CLVF_DW.If_ActiveSubsystem = -1;
        }

        switch (CLVF_B.rtAction) {
         case 0:
          // Outputs for IfAction SubSystem: '<S317>/Sub-Phase #1' incorporates:
          //   ActionPort: '<S321>/Action Port'

          // Sum: '<S321>/Subtract' incorporates:
          //   Constant: '<S321>/Desired X-Position (BLACK)'
          //   DataStoreRead: '<S321>/BLACK_Px'

          CLVF_B.Subtract_n = CLVF_P.init_states_BLACK[0] - CLVF_DW.BLACK_Px;

          // Delay: '<S332>/Delay1'
          if (CLVF_DW.icLoad_iz != 0) {
            CLVF_DW.Delay1_DSTATE_g = CLVF_B.Subtract_n;
          }

          // Sum: '<S332>/Sum6' incorporates:
          //   Delay: '<S332>/Delay1'

          CLVF_B.RED_Tz_Shoulder = CLVF_B.Subtract_n - CLVF_DW.Delay1_DSTATE_g;

          // If: '<S332>/if we went through a "step"' incorporates:
          //   Inport: '<S333>/In1'

          if (CLVF_B.RED_Tz_Shoulder != 0.0) {
            // Outputs for IfAction SubSystem: '<S332>/Hold this value' incorporates:
            //   ActionPort: '<S333>/Action Port'

            CLVF_B.In1_gy = CLVF_B.RED_Tz_Shoulder;

            // End of Outputs for SubSystem: '<S332>/Hold this value'
          }

          // End of If: '<S332>/if we went through a "step"'

          // Sum: '<S327>/Sum3' incorporates:
          //   DataStoreWrite: '<S321>/BLACK_Fx'
          //   Gain: '<S327>/kd_xb'
          //   Gain: '<S327>/kp_xb'
          //   Gain: '<S332>/divide by delta T'

          CLVF_DW.BLACK_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_gy *
            CLVF_P.Kd_xb + CLVF_P.Kp_xb * CLVF_B.Subtract_n;

          // Sum: '<S321>/Subtract1' incorporates:
          //   Constant: '<S321>/Desired Y-Position (BLACK)'
          //   DataStoreRead: '<S321>/BLACK_Py '

          CLVF_B.Subtract1_n = CLVF_P.init_states_BLACK[1] - CLVF_DW.BLACK_Py;

          // Delay: '<S334>/Delay1'
          if (CLVF_DW.icLoad_p != 0) {
            CLVF_DW.Delay1_DSTATE_g2 = CLVF_B.Subtract1_n;
          }

          // Sum: '<S334>/Sum6' incorporates:
          //   Delay: '<S334>/Delay1'

          CLVF_B.RED_Tz_Shoulder = CLVF_B.Subtract1_n - CLVF_DW.Delay1_DSTATE_g2;

          // If: '<S334>/if we went through a "step"' incorporates:
          //   Inport: '<S335>/In1'

          if (CLVF_B.RED_Tz_Shoulder != 0.0) {
            // Outputs for IfAction SubSystem: '<S334>/Hold this value' incorporates:
            //   ActionPort: '<S335>/Action Port'

            CLVF_B.In1_ft = CLVF_B.RED_Tz_Shoulder;

            // End of Outputs for SubSystem: '<S334>/Hold this value'
          }

          // End of If: '<S334>/if we went through a "step"'

          // Sum: '<S328>/Sum3' incorporates:
          //   DataStoreWrite: '<S321>/BLACK_Fy'
          //   Gain: '<S328>/kd_yb'
          //   Gain: '<S328>/kp_yb'
          //   Gain: '<S334>/divide by delta T'

          CLVF_DW.BLACK_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_ft *
            CLVF_P.Kd_yb + CLVF_P.Kp_yb * CLVF_B.Subtract1_n;

          // MATLAB Function: '<S329>/MATLAB Function2' incorporates:
          //   Constant: '<S321>/Desired Attitude (BLACK)'

          CLVF_MATLABFunction2_o(CLVF_P.init_states_BLACK[2],
            &CLVF_B.sf_MATLABFunction2_h);

          // MATLAB Function: '<S329>/MATLAB Function3' incorporates:
          //   DataStoreRead: '<S321>/BLACK_Rz'

          CLVF_MATLABFunction3(CLVF_DW.BLACK_Rz, &CLVF_B.sf_MATLABFunction3_a);

          // Sum: '<S329>/Subtract2'
          CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_h.Ox[0] -
            CLVF_B.sf_MATLABFunction3_a.Ox[0];
          CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_h.Ox[1] -
            CLVF_B.sf_MATLABFunction3_a.Ox[1];

          // MATLAB Function: '<S329>/MATLAB Function4'
          CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_h.Oy,
                               CLVF_B.rtb_Subtract2_l_g,
                               &CLVF_B.sf_MATLABFunction4_f);

          // Delay: '<S330>/Delay1'
          if (CLVF_DW.icLoad_dm != 0) {
            CLVF_DW.Delay1_DSTATE_d = CLVF_B.sf_MATLABFunction4_f.e_out;
          }

          // Sum: '<S330>/Sum6' incorporates:
          //   Delay: '<S330>/Delay1'

          CLVF_B.RED_Tz_Shoulder = CLVF_B.sf_MATLABFunction4_f.e_out -
            CLVF_DW.Delay1_DSTATE_d;

          // If: '<S330>/if we went through a "step"' incorporates:
          //   Inport: '<S331>/In1'

          if (CLVF_B.RED_Tz_Shoulder != 0.0) {
            // Outputs for IfAction SubSystem: '<S330>/Hold this value' incorporates:
            //   ActionPort: '<S331>/Action Port'

            CLVF_B.In1_l = CLVF_B.RED_Tz_Shoulder;

            // End of Outputs for SubSystem: '<S330>/Hold this value'
          }

          // End of If: '<S330>/if we went through a "step"'

          // Sum: '<S326>/Sum3' incorporates:
          //   DataStoreWrite: '<S321>/BLACK_Tz'
          //   Gain: '<S326>/kd_tb'
          //   Gain: '<S326>/kp_tb'
          //   Gain: '<S330>/divide by delta T'

          CLVF_DW.BLACK_Tz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_l *
            CLVF_P.Kd_tb + CLVF_P.Kp_tb * CLVF_B.sf_MATLABFunction4_f.e_out;

          // DataStoreWrite: '<S321>/Data Store Write3' incorporates:
          //   Constant: '<S321>/Puck State'

          CLVF_DW.Float_State = CLVF_P.PuckState_Value_g;

          // End of Outputs for SubSystem: '<S317>/Sub-Phase #1'
          break;

         case 1:
          // Outputs for IfAction SubSystem: '<S317>/Sub-Phase #2 ' incorporates:
          //   ActionPort: '<S322>/Action Port'

          CLVF_SubPhase2(&CLVF_P.SubPhase2, &CLVF_DW.BLACK_Fx, &CLVF_DW.BLACK_Fy,
                         &CLVF_DW.BLACK_Tz, &CLVF_DW.Float_State);

          // End of Outputs for SubSystem: '<S317>/Sub-Phase #2 '
          break;

         case 2:
          // Outputs for IfAction SubSystem: '<S317>/Sub-Phase #3 ' incorporates:
          //   ActionPort: '<S323>/Action Port'

          CLVF_SubPhase2(&CLVF_P.SubPhase3, &CLVF_DW.BLACK_Fx, &CLVF_DW.BLACK_Fy,
                         &CLVF_DW.BLACK_Tz, &CLVF_DW.Float_State);

          // End of Outputs for SubSystem: '<S317>/Sub-Phase #3 '
          break;

         case 3:
          // Outputs for IfAction SubSystem: '<S317>/Sub-Phase #4' incorporates:
          //   ActionPort: '<S324>/Action Port'

          // DataStoreRead: '<S342>/Data Store Read'
          CLVF_B.DataStoreRead = CLVF_DW.BLACK_Px;

          // DataStoreRead: '<S342>/Data Store Read1'
          CLVF_B.DataStoreRead1 = CLVF_DW.BLACK_Py;

          // MATLAB Function: '<S342>/MATLAB Function' incorporates:
          //   DataStoreRead: '<S342>/Data Store Read8'
          //   MATLAB Function: '<S342>/Norm1'

          // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/States in Target-Fixed Frame/MATLAB Function': '<S371>:1' 
          // '<S371>:1:3' d_T = C3(theta_T)'*d;
          // 'C3:6' A = [cos(z) sin(z) 0; -sin(z) cos(z) 0 ; 0 0 1];
          CLVF_B.g = sin(CLVF_DW.RED_Rz);
          CLVF_B.RED_Tz_Elbow = cos(CLVF_DW.RED_Rz);
          CLVF_B.vc[0] = CLVF_B.RED_Tz_Elbow;
          CLVF_B.vc[1] = CLVF_B.g;
          CLVF_B.vc[2] = 0.0;
          CLVF_B.vc[3] = -CLVF_B.g;
          CLVF_B.vc[4] = CLVF_B.RED_Tz_Elbow;
          CLVF_B.vc[5] = 0.0;
          for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++)
          {
            CLVF_B.vc[CLVF_B.uElOffset1 + 6] = b[CLVF_B.uElOffset1];
            CLVF_B.d_T[CLVF_B.uElOffset1] = CLVF_B.vc[CLVF_B.uElOffset1 + 6] *
              CLVF_P.d[2] + (CLVF_B.vc[CLVF_B.uElOffset1 + 3] * CLVF_P.d[1] +
                             CLVF_B.vc[CLVF_B.uElOffset1] * CLVF_P.d[0]);
          }

          // End of MATLAB Function: '<S342>/MATLAB Function'

          // Sum: '<S342>/Sum' incorporates:
          //   Constant: '<S342>/Constant2'
          //   Constant: '<S342>/Constant3'
          //   DataStoreRead: '<S342>/Data Store Read2'
          //   DataStoreRead: '<S342>/Data Store Read3'
          //   Sum: '<S342>/Sum2'

          CLVF_B.d_T[0] = CLVF_B.DataStoreRead - (CLVF_DW.RED_Px + CLVF_B.d_T[0]);
          CLVF_B.d_T[1] = CLVF_B.DataStoreRead1 - (CLVF_DW.RED_Py + CLVF_B.d_T[1]);
          CLVF_B.d_T[2] = CLVF_P.Constant2_Value_j - (CLVF_P.Constant3_Value +
            CLVF_B.d_T[2]);

          // MATLAB Function: '<S342>/Norm1'
          // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/States in Target-Fixed Frame/Norm1': '<S373>:1' 
          // '<S373>:1:3' r = sqrt(sum(rC_D.^2));
          CLVF_B.RED_Tz_Shoulder = sqrt((CLVF_B.d_T[0] * CLVF_B.d_T[0] +
            CLVF_B.d_T[1] * CLVF_B.d_T[1]) + CLVF_B.d_T[2] * CLVF_B.d_T[2]);

          // '<S373>:1:4' r_hat = rC_D/r;
          // '<S373>:1:6' C_BI = C3(theta_T);
          // 'C3:6' A = [cos(z) sin(z) 0; -sin(z) cos(z) 0 ; 0 0 1];
          // '<S373>:1:8' o_docking = C_BI'*o_hat_prime;
          // '<S373>:1:11' theta = acos(r_hat'*o_docking);
          CLVF_B.vc[0] = CLVF_B.RED_Tz_Elbow;
          CLVF_B.vc[1] = CLVF_B.g;
          CLVF_B.vc[2] = 0.0;
          CLVF_B.vc[3] = -CLVF_B.g;
          CLVF_B.vc[4] = CLVF_B.RED_Tz_Elbow;
          CLVF_B.vc[5] = 0.0;
          CLVF_B.vc_dot = 0.0;
          for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++)
          {
            CLVF_B.vc[CLVF_B.uElOffset1 + 6] = b[CLVF_B.uElOffset1];
            CLVF_B.vc_dot += CLVF_B.d_T[CLVF_B.uElOffset1] /
              CLVF_B.RED_Tz_Shoulder * (CLVF_B.vc[CLVF_B.uElOffset1 + 6] *
              CLVF_P.o_hat_prime[2] + (CLVF_B.vc[CLVF_B.uElOffset1 + 3] *
              CLVF_P.o_hat_prime[1] + CLVF_B.vc[CLVF_B.uElOffset1] *
              CLVF_P.o_hat_prime[0]));
          }

          CLVF_B.theta = CLVF_B.vc_dot;
          CLVF_B.theta = acos(CLVF_B.theta);
          CLVF_B.r = CLVF_B.RED_Tz_Shoulder;

          // Sum: '<S349>/Sum' incorporates:
          //   DataStoreRead: '<S342>/Data Store Read2'

          CLVF_B.TmpSignalConversionAtSFun_n[0] = CLVF_B.DataStoreRead -
            CLVF_DW.RED_Px;

          // Outputs for IfAction SubSystem: '<S349>/CLVF' incorporates:
          //   ActionPort: '<S351>/Action Port'

          // If: '<S349>/If' incorporates:
          //   DataStoreRead: '<S342>/Data Store Read3'
          //   MATLAB Function: '<S351>/Lyapunov Guidance + FF'
          //   Sum: '<S349>/Sum'

          CLVF_B.RED_Tz_Wrist = CLVF_B.DataStoreRead1 - CLVF_DW.RED_Py;

          // End of Outputs for SubSystem: '<S349>/CLVF'

          // Sum: '<S349>/Sum'
          CLVF_B.TmpSignalConversionAtSFun_n[1] = CLVF_B.RED_Tz_Wrist;

          // Outputs for IfAction SubSystem: '<S349>/CLVF' incorporates:
          //   ActionPort: '<S351>/Action Port'

          // If: '<S349>/If' incorporates:
          //   Constant: '<S342>/Constant2'
          //   Constant: '<S342>/Constant3'
          //   MATLAB Function: '<S351>/Lyapunov Guidance + FF'
          //   Sum: '<S342>/Sum1'
          //   Sum: '<S349>/Sum'

          CLVF_B.RED_Tz_Elbow = CLVF_P.Constant2_Value_j -
            CLVF_P.Constant3_Value;

          // End of Outputs for SubSystem: '<S349>/CLVF'

          // Sum: '<S349>/Sum'
          CLVF_B.TmpSignalConversionAtSFun_n[2] = CLVF_B.RED_Tz_Elbow;

          // DataStoreRead: '<S350>/Data Store Read2'
          CLVF_B.DataStoreRead2 = CLVF_DW.RED_Rz;

          // MATLAB Function: '<S350>/getRotationMatrix'
          // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/Navigation/getRotationMatrix': '<S362>:1' 
          // '<S362>:1:3' y = C3(u);
          // 'C3:6' A = [cos(z) sin(z) 0; -sin(z) cos(z) 0 ; 0 0 1];
          CLVF_B.y[0] = cos(CLVF_B.DataStoreRead2);
          CLVF_B.y[3] = sin(CLVF_B.DataStoreRead2);
          CLVF_B.y[6] = 0.0;
          CLVF_B.y[1] = -sin(CLVF_B.DataStoreRead2);
          CLVF_B.y[4] = cos(CLVF_B.DataStoreRead2);
          CLVF_B.y[7] = 0.0;
          CLVF_B.y[2] = 0.0;
          CLVF_B.y[5] = 0.0;
          CLVF_B.y[8] = 1.0;

          // MATLAB Function: '<S354>/inDesiredRadius' incorporates:
          //   Constant: '<S354>/Constant'
          //   Constant: '<S354>/Constant1'
          //   Delay: '<S354>/Delay'
          //   Sum: '<S349>/Sum'

          // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/SWITCH/inDesiredRadius': '<S359>:1' 
          // '<S359>:1:6' del = C_BI*rC_T - aTimesOVec;
          // '<S359>:1:9' dist = sqrt(sum(del.^2));
          for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
            CLVF_B.RED_Tz_Shoulder = ((CLVF_B.y[CLVF_B.yElIdx + 3] *
              CLVF_B.RED_Tz_Wrist + CLVF_B.y[CLVF_B.yElIdx] *
              CLVF_B.TmpSignalConversionAtSFun_n[0]) + CLVF_B.y[CLVF_B.yElIdx +
              6] * CLVF_B.RED_Tz_Elbow) - CLVF_P.aTimesOVec[CLVF_B.yElIdx];
            CLVF_B.w_dot_I[CLVF_B.yElIdx] = CLVF_B.RED_Tz_Shoulder *
              CLVF_B.RED_Tz_Shoulder;
          }

          // '<S359>:1:16' if dist <= acceptableDistance
          if (sqrt((CLVF_B.w_dot_I[0] + CLVF_B.w_dot_I[1]) + CLVF_B.w_dot_I[2]) <=
              CLVF_P.acceptableRadius) {
            // '<S359>:1:17' newCnt = currentCnt+1;
            CLVF_B.newCnt = CLVF_DW.Delay_DSTATE + 1.0;
          } else {
            // '<S359>:1:18' else
            // '<S359>:1:19' newCnt = 0;
            CLVF_B.newCnt = 0.0;
          }

          // End of MATLAB Function: '<S354>/inDesiredRadius'

          // MATLAB Function: '<S354>/sufficientTime' incorporates:
          //   Constant: '<S354>/Constant2'

          // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/SWITCH/sufficientTime': '<S360>:1' 
          // '<S360>:1:3' if cnt < cntThreshold
          if (CLVF_B.newCnt < CLVF_P.cntThreshold) {
            // '<S360>:1:4' oneOrZero = 0;
            CLVF_B.RED_Tz_Shoulder = 0.0;
          } else {
            // '<S360>:1:5' else
            // '<S360>:1:6' oneOrZero = 1;
            CLVF_B.RED_Tz_Shoulder = 1.0;
          }

          // End of MATLAB Function: '<S354>/sufficientTime'

          // Outputs for Triggered SubSystem: '<S354>/Sample and Hold' incorporates:
          //   TriggerPort: '<S358>/Trigger'

          CLVF_B.zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
            &CLVF_PrevZCX.SampleandHold_Trig_ZCE,
            (CLVF_B.RED_Tz_Shoulder));
          if (CLVF_B.zcEvent != NO_ZCEVENT) {
            CLVF_B.In = CLVF_B.RED_Tz_Shoulder;
          }

          // End of Outputs for SubSystem: '<S354>/Sample and Hold'
          // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/getDesiredRotation': '<S343>:1' 
          // '<S343>:1:3' rD_C = -rC_D;
          CLVF_B.d_T[0] = -CLVF_B.d_T[0];

          // MATLAB Function: '<S324>/getDesiredRotation' incorporates:
          //   DataStoreRead: '<S324>/RED_Rz'
          //   Inport: '<S358>/In'

          // '<S343>:1:5' if whichField == 0
          if (CLVF_B.In == 0.0) {
            // '<S343>:1:6' thetaC = atan2(rD_C(2),rD_C(1)) - pi/2;
            CLVF_B.RED_Tz_Shoulder = CLVF_rt_atan2d_snf(-CLVF_B.d_T[1],
              CLVF_B.d_T[0]) - 1.5707963267948966;
          } else {
            // '<S343>:1:7' else
            // '<S343>:1:9' thetaC = RED_Z - pi;
            CLVF_B.RED_Tz_Shoulder = CLVF_DW.RED_Rz - 3.1415926535897931;
          }

          // MATLAB Function: '<S340>/MATLAB Function2'
          CLVF_MATLABFunction2_c(CLVF_B.RED_Tz_Shoulder,
            &CLVF_B.sf_MATLABFunction2_c);

          // MATLAB Function: '<S340>/MATLAB Function3' incorporates:
          //   DataStoreRead: '<S324>/Data Store Read3'

          CLVF_MATLABFunction3(CLVF_DW.BLACK_Rz, &CLVF_B.sf_MATLABFunction3_f);

          // Sum: '<S340>/Subtract2'
          CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_c.Ox[0] -
            CLVF_B.sf_MATLABFunction3_f.Ox[0];
          CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_c.Ox[1] -
            CLVF_B.sf_MATLABFunction3_f.Ox[1];

          // MATLAB Function: '<S340>/MATLAB Function4'
          CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_c.Oy,
                               CLVF_B.rtb_Subtract2_l_g,
                               &CLVF_B.sf_MATLABFunction4_i);

          // Delay: '<S344>/Delay1'
          if (CLVF_DW.icLoad_en != 0) {
            CLVF_DW.Delay1_DSTATE_e = CLVF_B.sf_MATLABFunction4_i.e_out;
          }

          // Sum: '<S344>/Sum6' incorporates:
          //   Delay: '<S344>/Delay1'

          CLVF_B.RED_Tz_Shoulder = CLVF_B.sf_MATLABFunction4_i.e_out -
            CLVF_DW.Delay1_DSTATE_e;

          // If: '<S344>/if we went through a "step"' incorporates:
          //   Inport: '<S345>/In1'

          if (CLVF_B.RED_Tz_Shoulder != 0.0) {
            // Outputs for IfAction SubSystem: '<S344>/Hold this value' incorporates:
            //   ActionPort: '<S345>/Action Port'

            CLVF_B.In1_jg = CLVF_B.RED_Tz_Shoulder;

            // End of Outputs for SubSystem: '<S344>/Hold this value'
          }

          // End of If: '<S344>/if we went through a "step"'

          // DiscreteTransferFcn: '<S339>/1Hz LP Filter' incorporates:
          //   Gain: '<S344>/divide by delta T'

          CLVF_DW.uHzLPFilter_tmp = ((1.0 / CLVF_P.serverRate * CLVF_B.In1_jg -
            CLVF_P.den_d[1] * CLVF_DW.uHzLPFilter_states[0]) - CLVF_P.den_d[2] *
            CLVF_DW.uHzLPFilter_states[1]) / CLVF_P.den_d[0];

          // Sum: '<S339>/Sum3' incorporates:
          //   DataStoreWrite: '<S324>/BLACK_Tz'
          //   DiscreteTransferFcn: '<S339>/1Hz LP Filter'
          //   Gain: '<S339>/kd_tb'
          //   Gain: '<S339>/kp_tb'

          CLVF_DW.BLACK_Tz = ((CLVF_P.num_d[0] * CLVF_DW.uHzLPFilter_tmp +
                               CLVF_P.num_d[1] * CLVF_DW.uHzLPFilter_states[0])
                              + CLVF_P.num_d[2] * CLVF_DW.uHzLPFilter_states[1])
            * CLVF_P.Kd_tb + CLVF_P.Kp_tb * CLVF_B.sf_MATLABFunction4_i.e_out;

          // DiscreteTransferFcn: '<S342>/1Hz LP Filter1' incorporates:
          //   DataStoreRead: '<S342>/Data Store Read7'

          CLVF_DW.uHzLPFilter1_tmp_i = ((CLVF_DW.BLACK_Vy - CLVF_P.den_d[1] *
            CLVF_DW.uHzLPFilter1_states_l[0]) - CLVF_P.den_d[2] *
            CLVF_DW.uHzLPFilter1_states_l[1]) / CLVF_P.den_d[0];
          CLVF_B.uHzLPFilter1 = (CLVF_P.num_d[0] * CLVF_DW.uHzLPFilter1_tmp_i +
            CLVF_P.num_d[1] * CLVF_DW.uHzLPFilter1_states_l[0]) + CLVF_P.num_d[2]
            * CLVF_DW.uHzLPFilter1_states_l[1];

          // DiscreteTransferFcn: '<S342>/1Hz LP Filter' incorporates:
          //   DataStoreRead: '<S342>/Data Store Read6'

          CLVF_DW.uHzLPFilter_tmp_g = ((CLVF_DW.BLACK_Vx - CLVF_P.den_d[1] *
            CLVF_DW.uHzLPFilter_states_m[0]) - CLVF_P.den_d[2] *
            CLVF_DW.uHzLPFilter_states_m[1]) / CLVF_P.den_d[0];
          CLVF_B.uHzLPFilter = (CLVF_P.num_d[0] * CLVF_DW.uHzLPFilter_tmp_g +
                                CLVF_P.num_d[1] * CLVF_DW.uHzLPFilter_states_m[0])
            + CLVF_P.num_d[2] * CLVF_DW.uHzLPFilter_states_m[1];

          // Delay: '<S363>/Delay1'
          if (CLVF_DW.icLoad_jp != 0) {
            CLVF_DW.Delay1_DSTATE_hm = CLVF_B.DataStoreRead2;
          }

          // Sum: '<S363>/Sum6' incorporates:
          //   Delay: '<S363>/Delay1'

          CLVF_B.RED_Tz_Shoulder = CLVF_B.DataStoreRead2 -
            CLVF_DW.Delay1_DSTATE_hm;

          // If: '<S363>/if we went through a "step"' incorporates:
          //   Inport: '<S370>/In1'

          if (CLVF_B.RED_Tz_Shoulder != 0.0) {
            // Outputs for IfAction SubSystem: '<S363>/Hold this value' incorporates:
            //   ActionPort: '<S370>/Action Port'

            CLVF_B.In1_a = CLVF_B.RED_Tz_Shoulder;

            // End of Outputs for SubSystem: '<S363>/Hold this value'
          }

          // End of If: '<S363>/if we went through a "step"'

          // Gain: '<S363>/divide by delta T'
          CLVF_B.RED_Tz_Shoulder = 1.0 / CLVF_P.serverRate * CLVF_B.In1_a;

          // If: '<S349>/If' incorporates:
          //   MATLAB Function: '<S351>/Lyapunov Guidance + FF'
          //   MATLAB Function: '<S353>/Body-Fixed Guidance'

          CLVF_B.rtAction = static_cast<int8_T>(!(CLVF_B.In == 0.0));
          CLVF_DW.If_ActiveSubsystem = CLVF_B.rtAction;
          if (CLVF_B.rtAction == 0) {
            // Outputs for IfAction SubSystem: '<S349>/CLVF' incorporates:
            //   ActionPort: '<S351>/Action Port'

            // MATLAB Function: '<S351>/Lyapunov Guidance + FF' incorporates:
            //   Constant: '<S341>/Constant'
            //   Constant: '<S342>/Constant1'
            //   Constant: '<S349>/Constant'
            //   Constant: '<S350>/Constant'
            //   Constant: '<S350>/Constant1'
            //   Constant: '<S351>/Constant'
            //   DataStoreRead: '<S342>/Data Store Read2'
            //   SignalConversion generated from: '<S355>/ SFunction '

            // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/CLVF/Lyapunov Guidance + FF': '<S355>:1' 
            // '<S355>:1:4' C_IB = CT_BI';
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.C_IB[3 * CLVF_B.uElOffset1] = CLVF_B.y[CLVF_B.uElOffset1];
              CLVF_B.C_IB[3 * CLVF_B.uElOffset1 + 1] =
                CLVF_B.y[CLVF_B.uElOffset1 + 3];
              CLVF_B.C_IB[3 * CLVF_B.uElOffset1 + 2] =
                CLVF_B.y[CLVF_B.uElOffset1 + 6];
            }

            // '<S355>:1:5' o_hat = C_IB*o_hat_B;
            // '<S355>:1:7' wT_I = C_IB*wT_B;
            // '<S355>:1:8' w_dot_I = C_IB*w_dot_B;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.vc_p = CLVF_B.C_IB[CLVF_B.uElOffset1 + 3];
              CLVF_B.dgdr = CLVF_B.vc_p * CLVF_P.o_hat_B[1] +
                CLVF_B.C_IB[CLVF_B.uElOffset1] * CLVF_P.o_hat_B[0];
              CLVF_B.vc_dot = CLVF_B.vc_p * CLVF_P.Constant1_Value_e +
                CLVF_B.C_IB[CLVF_B.uElOffset1] * CLVF_P.Constant_Value_d;
              CLVF_B.rtb_TmpSignalConversionAtSFun_j = CLVF_B.vc_p *
                CLVF_P.Constant_Value_hr[1] + CLVF_B.C_IB[CLVF_B.uElOffset1] *
                CLVF_P.Constant_Value_hr[0];
              CLVF_B.vc_p = CLVF_B.C_IB[CLVF_B.uElOffset1 + 6];
              CLVF_B.dgdr += CLVF_B.vc_p * CLVF_P.o_hat_B[2];
              CLVF_B.vc_dot += CLVF_B.vc_p * CLVF_B.RED_Tz_Shoulder;
              CLVF_B.rtb_TmpSignalConversionAtSFun_j += CLVF_B.vc_p *
                CLVF_P.Constant_Value_hr[2];
              CLVF_B.o_hat[CLVF_B.uElOffset1] = CLVF_B.dgdr;
              CLVF_B.d_T[CLVF_B.uElOffset1] = CLVF_B.vc_dot;
              CLVF_B.w_dot_I[CLVF_B.uElOffset1] =
                CLVF_B.rtb_TmpSignalConversionAtSFun_j;
            }

            // '<S355>:1:10' rC_T = rC_I - rT_I;
            CLVF_B.TmpSignalConversionAtSFun_n[0] = CLVF_B.DataStoreRead -
              CLVF_DW.RED_Px;

            // '<S355>:1:11' vC_T = vC_I - vT_I;
            CLVF_B.vC_T[0] = CLVF_B.uHzLPFilter - CLVF_P.Constant_Value_e[0];
            CLVF_B.vC_T[1] = CLVF_B.uHzLPFilter1 - CLVF_P.Constant_Value_e[1];
            CLVF_B.vC_T[2] = CLVF_P.Constant1_Value - CLVF_P.Constant_Value_e[2];

            // '<S355>:1:13' q = 0.000001;
            // '<S355>:1:16' [h, a_ff] = CLVF_quad(rC_T, wT_I, w_dot_I, o_hat, vT_I, aT_I, a, b, ka, kc, vC_T, q); 
            // 'CLVF_quad:12' r = sqrt(sum(r_T.^2));
            CLVF_B.RED_Tz_Shoulder = sqrt((CLVF_B.TmpSignalConversionAtSFun_n[0]
              * CLVF_B.TmpSignalConversionAtSFun_n[0] + CLVF_B.RED_Tz_Wrist *
              CLVF_B.RED_Tz_Wrist) + CLVF_B.RED_Tz_Elbow * CLVF_B.RED_Tz_Elbow);

            // 'CLVF_quad:16' o_mag = sqrt(sum(o_hat.^2));
            // 'CLVF_quad:17' theta = acos(r_T'*o_hat/(r*o_mag));
            // 'CLVF_quad:24' r_hat = r_T./r;
            CLVF_B.rtb_TmpSignalConversionAtSFun_g =
              CLVF_B.TmpSignalConversionAtSFun_n[0] * CLVF_B.o_hat[0];
            CLVF_B.TmpSignalConversionAtSFun_n[0] /= CLVF_B.RED_Tz_Shoulder;

            // MATLAB Function: '<S351>/Lyapunov Guidance + FF'
            CLVF_B.rtb_TmpSignalConversionAtSFun_g += CLVF_B.RED_Tz_Wrist *
              CLVF_B.o_hat[1];
            CLVF_B.TmpSignalConversionAtSFun_n[1] = CLVF_B.RED_Tz_Wrist /
              CLVF_B.RED_Tz_Shoulder;

            // MATLAB Function: '<S351>/Lyapunov Guidance + FF'
            CLVF_B.rtb_TmpSignalConversionAtSFun_g += CLVF_B.RED_Tz_Elbow *
              CLVF_B.o_hat[2];
            CLVF_B.rtb_TmpSignalConversionAtSFun_j = CLVF_B.RED_Tz_Elbow /
              CLVF_B.RED_Tz_Shoulder;
            CLVF_B.TmpSignalConversionAtSFun_n[2] =
              CLVF_B.rtb_TmpSignalConversionAtSFun_j;

            // MATLAB Function: '<S351>/Lyapunov Guidance + FF' incorporates:
            //   Constant: '<S349>/Constant'
            //   Constant: '<S351>/Constant1'
            //   Constant: '<S351>/Constant2'
            //   Constant: '<S351>/Constant3'
            //   Constant: '<S351>/Constant4'

            CLVF_B.theta_l = acos(CLVF_B.rtb_TmpSignalConversionAtSFun_g / (sqrt
              ((CLVF_B.o_hat[0] * CLVF_B.o_hat[0] + CLVF_B.o_hat[1] *
                CLVF_B.o_hat[1]) + CLVF_B.o_hat[2] * CLVF_B.o_hat[2]) *
              CLVF_B.RED_Tz_Shoulder));

            // 'CLVF_quad:27' a_hat = (o_hat - r_hat.*cos(theta))./(sin(theta)+q); 
            CLVF_B.y6 = cos(CLVF_B.theta_l);
            CLVF_B.vc_dot = sin(CLVF_B.theta_l);
            CLVF_B.del[0] = (CLVF_B.o_hat[0] -
                             CLVF_B.TmpSignalConversionAtSFun_n[0] * CLVF_B.y6) /
              (CLVF_B.vc_dot + 1.0E-6);
            CLVF_B.del[1] = (CLVF_B.o_hat[1] -
                             CLVF_B.TmpSignalConversionAtSFun_n[1] * CLVF_B.y6) /
              (CLVF_B.vc_dot + 1.0E-6);
            CLVF_B.del[2] = (CLVF_B.o_hat[2] -
                             CLVF_B.rtb_TmpSignalConversionAtSFun_j * CLVF_B.y6)
              / (CLVF_B.vc_dot + 1.0E-6);

            // 'CLVF_quad:34' if r<a
            if (CLVF_B.RED_Tz_Shoulder < CLVF_P.a) {
              // 'CLVF_quad:35' g = r;
              CLVF_B.g = CLVF_B.RED_Tz_Shoulder;

              // 'CLVF_quad:44' sa = ka*(r/a)*sin(theta);
              CLVF_B.RED_Tz_Wrist = CLVF_B.RED_Tz_Shoulder / CLVF_P.a *
                CLVF_P.ka * CLVF_B.vc_dot;
            } else {
              // 'CLVF_quad:36' else
              // 'CLVF_quad:37' g = a^2./r;
              CLVF_B.g = CLVF_P.a * CLVF_P.a / CLVF_B.RED_Tz_Shoulder;

              // 'CLVF_quad:45' else
              // 'CLVF_quad:46' sa = ka*(a/r)*sin(theta);
              CLVF_B.RED_Tz_Wrist = CLVF_P.a / CLVF_B.RED_Tz_Shoulder *
                CLVF_P.ka * CLVF_B.vc_dot;
            }

            // 'CLVF_quad:43' if r<a
            // 'CLVF_quad:57' if abs(r-a) <= b
            CLVF_B.y7 = fabs(CLVF_B.RED_Tz_Shoulder - CLVF_P.a);
            if (CLVF_B.y7 <= CLVF_P.b) {
              // 'CLVF_quad:58' vc = sign(a-r)  *  (-kc/b^2*abs(r-a)^2 + 2*kc/b*abs(r-a)); 
              CLVF_B.vc_p = CLVF_P.a - CLVF_B.RED_Tz_Shoulder;
              if (CLVF_B.vc_p < 0.0) {
                CLVF_B.vc_p = -1.0;
              } else if (CLVF_B.vc_p > 0.0) {
                CLVF_B.vc_p = 1.0;
              } else if (CLVF_B.vc_p == 0.0) {
                CLVF_B.vc_p = 0.0;
              } else {
                CLVF_B.vc_p = (rtNaN);
              }

              CLVF_B.vc_p *= -CLVF_P.kc / (CLVF_P.b * CLVF_P.b) * (CLVF_B.y7 *
                CLVF_B.y7) + 2.0 * CLVF_P.kc / CLVF_P.b * CLVF_B.y7;
            } else {
              // 'CLVF_quad:59' else
              // 'CLVF_quad:60' vc = kc*sign(a-r);
              CLVF_B.vc_p = CLVF_P.a - CLVF_B.RED_Tz_Shoulder;
              if (CLVF_B.vc_p < 0.0) {
                CLVF_B.vc_p = -1.0;
              } else if (CLVF_B.vc_p > 0.0) {
                CLVF_B.vc_p = 1.0;
              } else if (CLVF_B.vc_p == 0.0) {
                CLVF_B.vc_p = 0.0;
              } else {
                CLVF_B.vc_p = (rtNaN);
              }

              CLVF_B.vc_p *= CLVF_P.kc;
            }

            // 'CLVF_quad:67' h = vc*r_hat + sa*a_hat + g*cross(w_OI,r_hat) + vt_I; 
            // 'CLVF_quad:72' e_hat = cross(r_hat,a_hat);
            CLVF_B.e_hat[0] = CLVF_B.TmpSignalConversionAtSFun_n[1] *
              CLVF_B.del[2] - CLVF_B.rtb_TmpSignalConversionAtSFun_j *
              CLVF_B.del[1];
            CLVF_B.e_hat[1] = CLVF_B.rtb_TmpSignalConversionAtSFun_j *
              CLVF_B.del[0] - CLVF_B.TmpSignalConversionAtSFun_n[0] *
              CLVF_B.del[2];
            CLVF_B.e_hat[2] = CLVF_B.TmpSignalConversionAtSFun_n[0] *
              CLVF_B.del[1] - CLVF_B.TmpSignalConversionAtSFun_n[1] *
              CLVF_B.del[0];

            // 'CLVF_quad:80' r_hat_dot = (eye(3,3) - r_hat*r_hat')/r * vC_T;
            memset(&CLVF_B.C_IB[0], 0, 9U * sizeof(real_T));
            CLVF_B.C_IB[0] = 1.0;
            CLVF_B.C_IB[4] = 1.0;
            CLVF_B.C_IB[8] = 1.0;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.C_IB_c[3 * CLVF_B.uElOffset1] = (CLVF_B.C_IB[3 *
                CLVF_B.uElOffset1] - CLVF_B.TmpSignalConversionAtSFun_n[0] *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1]) /
                CLVF_B.RED_Tz_Shoulder;
              CLVF_B.ntIdx1 = 3 * CLVF_B.uElOffset1 + 1;
              CLVF_B.C_IB_c[CLVF_B.ntIdx1] = (CLVF_B.C_IB[CLVF_B.ntIdx1] -
                CLVF_B.TmpSignalConversionAtSFun_n[1] *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1]) /
                CLVF_B.RED_Tz_Shoulder;
              CLVF_B.ntIdx1 = 3 * CLVF_B.uElOffset1 + 2;
              CLVF_B.C_IB_c[CLVF_B.ntIdx1] = (CLVF_B.C_IB[CLVF_B.ntIdx1] -
                CLVF_B.rtb_TmpSignalConversionAtSFun_j *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1]) /
                CLVF_B.RED_Tz_Shoulder;
            }

            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.r_hat_dot[CLVF_B.uElOffset1] =
                CLVF_B.C_IB_c[CLVF_B.uElOffset1 + 6] * CLVF_B.vC_T[2] +
                (CLVF_B.C_IB_c[CLVF_B.uElOffset1 + 3] * CLVF_B.vC_T[1] +
                 CLVF_B.C_IB_c[CLVF_B.uElOffset1] * CLVF_B.vC_T[0]);
            }

            // 'CLVF_quad:90' d_a_hat_d_r_hat = (eye(3,3) - a_hat*a_hat')/(a_hat'*o_hat+q)  *  (-r_hat*o_hat' - (r_hat'*o_hat)*eye(3,3)); 
            memset(&CLVF_B.C_IB[0], 0, 9U * sizeof(real_T));

            // 'CLVF_quad:91' d_a_hat_d_o_hat = e_hat*e_hat'/(a_hat'*o_hat + q); 
            // 'CLVF_quad:93' a_hat_dot = d_a_hat_d_r_hat*r_hat_dot + d_a_hat_d_o_hat*cross(w_OI,o_hat); 
            // 'CLVF_quad:97' r_dot = r_hat'*vC_T;
            CLVF_B.C_IB[0] = 1.0;
            CLVF_B.C_IB[4] = 1.0;
            CLVF_B.C_IB[8] = 1.0;
            CLVF_B.theta_l = (CLVF_B.del[0] * CLVF_B.o_hat[0] + CLVF_B.del[1] *
                              CLVF_B.o_hat[1]) + CLVF_B.del[2] * CLVF_B.o_hat[2];
            CLVF_B.rtb_TmpSignalConversionAtSFun_g =
              (CLVF_B.TmpSignalConversionAtSFun_n[0] * CLVF_B.o_hat[0] +
               CLVF_B.TmpSignalConversionAtSFun_n[1] * CLVF_B.o_hat[1]) +
              CLVF_B.rtb_TmpSignalConversionAtSFun_j * CLVF_B.o_hat[2];
            CLVF_B.rtb_TmpSignalConversionAtSFun_d =
              (CLVF_B.TmpSignalConversionAtSFun_n[0] * CLVF_B.vC_T[0] +
               CLVF_B.TmpSignalConversionAtSFun_n[1] * CLVF_B.vC_T[1]) +
              CLVF_B.rtb_TmpSignalConversionAtSFun_j * CLVF_B.vC_T[2];

            // 'CLVF_quad:99' theta_dot = -o_hat'/(sin(theta)+q) * r_hat_dot + w_OI'*e_hat; 
            // 'CLVF_quad:102' if r<a
            if (CLVF_B.RED_Tz_Shoulder < CLVF_P.a) {
              // 'CLVF_quad:103' dgdr = 1;
              CLVF_B.dgdr = 1.0;
            } else {
              // 'CLVF_quad:104' else
              // 'CLVF_quad:107' dgdr = -a^2./r^2;
              CLVF_B.dgdr = -(CLVF_P.a * CLVF_P.a) / (CLVF_B.RED_Tz_Shoulder *
                CLVF_B.RED_Tz_Shoulder);
            }

            // 'CLVF_quad:110' g_dot = dgdr * r_dot;
            CLVF_B.g_dot = CLVF_B.dgdr * CLVF_B.rtb_TmpSignalConversionAtSFun_d;

            // 'CLVF_quad:113' if r<a
            if (CLVF_B.RED_Tz_Shoulder < CLVF_P.a) {
              // 'CLVF_quad:114' dsadr = (ka/a)*sin(theta);
              CLVF_B.v_rel = CLVF_P.ka / CLVF_P.a * CLVF_B.vc_dot;

              // 'CLVF_quad:115' dsadtheta = (ka*r/a)*cos(theta);
              CLVF_B.dgdr = CLVF_P.ka * CLVF_B.RED_Tz_Shoulder / CLVF_P.a *
                CLVF_B.y6;
            } else {
              // 'CLVF_quad:116' else
              // 'CLVF_quad:117' dsadr = -(ka*a/r^2)*sin(theta);
              CLVF_B.dgdr = CLVF_P.ka * CLVF_P.a;
              CLVF_B.v_rel = -(CLVF_B.dgdr / (CLVF_B.RED_Tz_Shoulder *
                CLVF_B.RED_Tz_Shoulder)) * CLVF_B.vc_dot;

              // 'CLVF_quad:118' dsadtheta = (ka*a/r)*cos(theta);
              CLVF_B.dgdr = CLVF_B.dgdr / CLVF_B.RED_Tz_Shoulder * CLVF_B.y6;
            }

            // 'CLVF_quad:121' sa_dot = dsadr*r_dot + dsadtheta*theta_dot;
            CLVF_B.dgdr = (((-CLVF_B.o_hat[0] / (CLVF_B.vc_dot + 1.0E-6) *
                             CLVF_B.r_hat_dot[0] + -CLVF_B.o_hat[1] /
                             (CLVF_B.vc_dot + 1.0E-6) * CLVF_B.r_hat_dot[1]) +
                            -CLVF_B.o_hat[2] / (CLVF_B.vc_dot + 1.0E-6) *
                            CLVF_B.r_hat_dot[2]) + ((CLVF_B.d_T[0] *
              CLVF_B.e_hat[0] + CLVF_B.d_T[1] * CLVF_B.e_hat[1]) + CLVF_B.d_T[2]
              * CLVF_B.e_hat[2])) * CLVF_B.dgdr + CLVF_B.v_rel *
              CLVF_B.rtb_TmpSignalConversionAtSFun_d;

            // 'CLVF_quad:130' if abs(r-a) <= b
            if (CLVF_B.y7 <= CLVF_P.b) {
              // 'CLVF_quad:131' if r-a >= 0
              if (CLVF_B.RED_Tz_Shoulder - CLVF_P.a >= 0.0) {
                // 'CLVF_quad:132' dvcdr = 2*kc/b^2*(r-a)-2*kc/b;
                CLVF_B.RED_Tz_Shoulder = 2.0 * CLVF_P.kc / (CLVF_P.b * CLVF_P.b)
                  * (CLVF_B.RED_Tz_Shoulder - CLVF_P.a) - 2.0 * CLVF_P.kc /
                  CLVF_P.b;
              } else {
                // 'CLVF_quad:133' else
                // 'CLVF_quad:134' dvcdr = -2*kc/b^2*(r-a)-2*kc/b;
                CLVF_B.RED_Tz_Shoulder = -2.0 * CLVF_P.kc / (CLVF_P.b * CLVF_P.b)
                  * (CLVF_B.RED_Tz_Shoulder - CLVF_P.a) - 2.0 * CLVF_P.kc /
                  CLVF_P.b;
              }
            } else {
              // 'CLVF_quad:136' else
              // 'CLVF_quad:137' dvcdr = 0;
              CLVF_B.RED_Tz_Shoulder = 0.0;
            }

            // 'CLVF_quad:141' vc_dot = dvcdr*r_dot;
            CLVF_B.vc_dot = CLVF_B.RED_Tz_Shoulder *
              CLVF_B.rtb_TmpSignalConversionAtSFun_d;

            // 'CLVF_quad:145' a_ff = vc_dot*r_hat + vc*r_hat_dot + sa_dot*a_hat + sa*a_hat_dot + g_dot*cross(w_OI,r_hat) ... 
            // 'CLVF_quad:146'                         + g*cross(w_dot_OI, r_hat) + g*cross(w_OI,r_hat_dot) + at_I; 
            CLVF_B.RED_Tz_Shoulder = CLVF_B.d_T[1] *
              CLVF_B.rtb_TmpSignalConversionAtSFun_j - CLVF_B.d_T[2] *
              CLVF_B.TmpSignalConversionAtSFun_n[1];
            CLVF_B.h[0] = (CLVF_B.RED_Tz_Shoulder * CLVF_B.g + (CLVF_B.vc_p *
              CLVF_B.TmpSignalConversionAtSFun_n[0] + CLVF_B.RED_Tz_Wrist *
              CLVF_B.del[0])) + CLVF_P.Constant_Value_e[0];
            CLVF_B.y6 = CLVF_B.d_T[2] * CLVF_B.TmpSignalConversionAtSFun_n[0] -
              CLVF_B.d_T[0] * CLVF_B.rtb_TmpSignalConversionAtSFun_j;
            CLVF_B.h[1] = (CLVF_B.y6 * CLVF_B.g + (CLVF_B.vc_p *
              CLVF_B.TmpSignalConversionAtSFun_n[1] + CLVF_B.RED_Tz_Wrist *
              CLVF_B.del[1])) + CLVF_P.Constant_Value_e[1];
            CLVF_B.rtb_TmpSignalConversionAtSFun_d = CLVF_B.d_T[0] *
              CLVF_B.TmpSignalConversionAtSFun_n[1] - CLVF_B.d_T[1] *
              CLVF_B.TmpSignalConversionAtSFun_n[0];
            CLVF_B.h[2] = (CLVF_B.rtb_TmpSignalConversionAtSFun_d * CLVF_B.g +
                           (CLVF_B.vc_p * CLVF_B.rtb_TmpSignalConversionAtSFun_j
                            + CLVF_B.RED_Tz_Wrist * CLVF_B.del[2])) +
              CLVF_P.Constant_Value_e[2];
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.rtb_TmpSignalConversionAtSFun_f[CLVF_B.uElOffset1] =
                -CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1];
              CLVF_B.C_IB_c[3 * CLVF_B.uElOffset1] = (CLVF_B.C_IB[3 *
                CLVF_B.uElOffset1] - CLVF_B.del[0] *
                CLVF_B.del[CLVF_B.uElOffset1]) / (CLVF_B.theta_l + 1.0E-6);
              CLVF_B.ntIdx1 = 3 * CLVF_B.uElOffset1 + 1;
              CLVF_B.C_IB_c[CLVF_B.ntIdx1] = (CLVF_B.C_IB[CLVF_B.ntIdx1] -
                CLVF_B.del[1] * CLVF_B.del[CLVF_B.uElOffset1]) / (CLVF_B.theta_l
                + 1.0E-6);
              CLVF_B.ntIdx1 = 3 * CLVF_B.uElOffset1 + 2;
              CLVF_B.C_IB_c[CLVF_B.ntIdx1] = (CLVF_B.C_IB[CLVF_B.ntIdx1] -
                CLVF_B.del[2] * CLVF_B.del[CLVF_B.uElOffset1]) / (CLVF_B.theta_l
                + 1.0E-6);
            }

            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.rtb_TmpSignalConversionAtSFu_kk[3 * CLVF_B.uElOffset1] =
                CLVF_B.rtb_TmpSignalConversionAtSFun_f[0] *
                CLVF_B.o_hat[CLVF_B.uElOffset1] - static_cast<real_T>(b_1[3 *
                CLVF_B.uElOffset1]) * CLVF_B.rtb_TmpSignalConversionAtSFun_g;
              CLVF_B.yElIdx = 3 * CLVF_B.uElOffset1 + 1;
              CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.yElIdx] =
                CLVF_B.rtb_TmpSignalConversionAtSFun_f[1] *
                CLVF_B.o_hat[CLVF_B.uElOffset1] - static_cast<real_T>
                (b_1[CLVF_B.yElIdx]) * CLVF_B.rtb_TmpSignalConversionAtSFun_g;
              CLVF_B.yElIdx = 3 * CLVF_B.uElOffset1 + 2;
              CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.yElIdx] =
                CLVF_B.rtb_TmpSignalConversionAtSFun_f[2] *
                CLVF_B.o_hat[CLVF_B.uElOffset1] - static_cast<real_T>
                (b_1[CLVF_B.yElIdx]) * CLVF_B.rtb_TmpSignalConversionAtSFun_g;
            }

            CLVF_B.rtb_TmpSignalConversionAtSFun_g = CLVF_B.d_T[1] *
              CLVF_B.o_hat[2] - CLVF_B.d_T[2] * CLVF_B.o_hat[1];
            CLVF_B.y7 = CLVF_B.d_T[2] * CLVF_B.o_hat[0] - CLVF_B.d_T[0] *
              CLVF_B.o_hat[2];
            CLVF_B.v_rel = CLVF_B.d_T[0] * CLVF_B.o_hat[1] - CLVF_B.d_T[1] *
              CLVF_B.o_hat[0];
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.C_IB_cv[CLVF_B.uElOffset1] = 0.0;
              for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
                CLVF_B.ntIdx1 = CLVF_B.uElOffset1 + 3 * CLVF_B.yElIdx;
                CLVF_B.C_IB[CLVF_B.ntIdx1] = 0.0;
                CLVF_B.C_IB[CLVF_B.ntIdx1] +=
                  CLVF_B.rtb_TmpSignalConversionAtSFu_kk[3 * CLVF_B.yElIdx] *
                  CLVF_B.C_IB_c[CLVF_B.uElOffset1];
                CLVF_B.C_IB[CLVF_B.ntIdx1] +=
                  CLVF_B.rtb_TmpSignalConversionAtSFu_kk[3 * CLVF_B.yElIdx + 1] *
                  CLVF_B.C_IB_c[CLVF_B.uElOffset1 + 3];
                CLVF_B.C_IB[CLVF_B.ntIdx1] +=
                  CLVF_B.rtb_TmpSignalConversionAtSFu_kk[3 * CLVF_B.yElIdx + 2] *
                  CLVF_B.C_IB_c[CLVF_B.uElOffset1 + 6];
                CLVF_B.vc[CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1] =
                  CLVF_B.e_hat[CLVF_B.yElIdx] * CLVF_B.e_hat[CLVF_B.uElOffset1] /
                  (CLVF_B.theta_l + 1.0E-6);
                CLVF_B.C_IB_cv[CLVF_B.uElOffset1] += CLVF_B.C_IB[CLVF_B.ntIdx1] *
                  CLVF_B.r_hat_dot[CLVF_B.yElIdx];
              }
            }

            CLVF_B.rtb_TmpSignalConversionAtSFun_f[0] = CLVF_B.RED_Tz_Shoulder *
              CLVF_B.g_dot;
            CLVF_B.rtb_TmpSignalConversionAtSFun_f[1] = CLVF_B.y6 * CLVF_B.g_dot;
            CLVF_B.rtb_TmpSignalConversionAtSFun_f[2] =
              CLVF_B.rtb_TmpSignalConversionAtSFun_d * CLVF_B.g_dot;
            CLVF_B.e_hat[0] = (CLVF_B.w_dot_I[1] *
                               CLVF_B.rtb_TmpSignalConversionAtSFun_j -
                               CLVF_B.w_dot_I[2] *
                               CLVF_B.TmpSignalConversionAtSFun_n[1]) * CLVF_B.g;
            CLVF_B.e_hat[1] = (CLVF_B.w_dot_I[2] *
                               CLVF_B.TmpSignalConversionAtSFun_n[0] -
                               CLVF_B.w_dot_I[0] *
                               CLVF_B.rtb_TmpSignalConversionAtSFun_j) *
              CLVF_B.g;
            CLVF_B.e_hat[2] = (CLVF_B.w_dot_I[0] *
                               CLVF_B.TmpSignalConversionAtSFun_n[1] -
                               CLVF_B.w_dot_I[1] *
                               CLVF_B.TmpSignalConversionAtSFun_n[0]) * CLVF_B.g;
            CLVF_B.w_dot_I[0] = (CLVF_B.d_T[1] * CLVF_B.r_hat_dot[2] -
                                 CLVF_B.d_T[2] * CLVF_B.r_hat_dot[1]) * CLVF_B.g;
            CLVF_B.w_dot_I[1] = (CLVF_B.d_T[2] * CLVF_B.r_hat_dot[0] -
                                 CLVF_B.d_T[0] * CLVF_B.r_hat_dot[2]) * CLVF_B.g;
            CLVF_B.w_dot_I[2] = (CLVF_B.d_T[0] * CLVF_B.r_hat_dot[1] -
                                 CLVF_B.d_T[1] * CLVF_B.r_hat_dot[0]) * CLVF_B.g;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.a_ff[CLVF_B.uElOffset1] = ((((((CLVF_B.vc_dot *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1] +
                CLVF_B.vc_p * CLVF_B.r_hat_dot[CLVF_B.uElOffset1]) + CLVF_B.dgdr
                * CLVF_B.del[CLVF_B.uElOffset1]) +
                (CLVF_B.C_IB_cv[CLVF_B.uElOffset1] +
                 (CLVF_B.vc[CLVF_B.uElOffset1 + 6] * CLVF_B.v_rel +
                  (CLVF_B.vc[CLVF_B.uElOffset1 + 3] * CLVF_B.y7 +
                   CLVF_B.vc[CLVF_B.uElOffset1] *
                   CLVF_B.rtb_TmpSignalConversionAtSFun_g))) *
                CLVF_B.RED_Tz_Wrist) +
                CLVF_B.rtb_TmpSignalConversionAtSFun_f[CLVF_B.uElOffset1]) +
                CLVF_B.e_hat[CLVF_B.uElOffset1]) +
                CLVF_B.w_dot_I[CLVF_B.uElOffset1]) +
                CLVF_P.Constant_Value_e[CLVF_B.uElOffset1];
            }

            // Derivative: '<S351>/Derivative'
            CLVF_B.RED_Tz_Wrist = CLVF_M->Timing.t[0];
            if ((CLVF_DW.TimeStampA_i >= CLVF_B.RED_Tz_Wrist) &&
                (CLVF_DW.TimeStampB_p >= CLVF_B.RED_Tz_Wrist)) {
              CLVF_B.TmpSignalConversionAtSFun_n[0] = 0.0;
              CLVF_B.TmpSignalConversionAtSFun_n[1] = 0.0;
              CLVF_B.TmpSignalConversionAtSFun_n[2] = 0.0;
            } else {
              CLVF_B.RED_Tz_Shoulder = CLVF_DW.TimeStampA_i;
              lastU = &CLVF_DW.LastUAtTimeA_i;
              if (CLVF_DW.TimeStampA_i < CLVF_DW.TimeStampB_p) {
                if (CLVF_DW.TimeStampB_p < CLVF_B.RED_Tz_Wrist) {
                  CLVF_B.RED_Tz_Shoulder = CLVF_DW.TimeStampB_p;
                  lastU = &CLVF_DW.LastUAtTimeB_k;
                }
              } else {
                if (CLVF_DW.TimeStampA_i >= CLVF_B.RED_Tz_Wrist) {
                  CLVF_B.RED_Tz_Shoulder = CLVF_DW.TimeStampB_p;
                  lastU = &CLVF_DW.LastUAtTimeB_k;
                }
              }

              CLVF_B.RED_Tz_Shoulder = CLVF_B.RED_Tz_Wrist -
                CLVF_B.RED_Tz_Shoulder;
              CLVF_B.TmpSignalConversionAtSFun_n[0] = (CLVF_B.h[0] - (*lastU)[0])
                / CLVF_B.RED_Tz_Shoulder;
              CLVF_B.TmpSignalConversionAtSFun_n[1] = (CLVF_B.h[1] - (*lastU)[1])
                / CLVF_B.RED_Tz_Shoulder;
              CLVF_B.TmpSignalConversionAtSFun_n[2] = (CLVF_B.h[2] - (*lastU)[2])
                / CLVF_B.RED_Tz_Shoulder;
            }

            // End of Derivative: '<S351>/Derivative'

            // Sum: '<S351>/Sum'
            CLVF_B.Sum[0] = CLVF_B.TmpSignalConversionAtSFun_n[0] - CLVF_B.a_ff
              [0];
            CLVF_B.Sum[1] = CLVF_B.TmpSignalConversionAtSFun_n[1] - CLVF_B.a_ff
              [1];
            CLVF_B.Sum[2] = CLVF_B.TmpSignalConversionAtSFun_n[2] - CLVF_B.a_ff
              [2];

            // SignalConversion generated from: '<S351>/a_ff'
            CLVF_B.o_hat[0] = CLVF_B.a_ff[0];

            // SignalConversion generated from: '<S351>/h'
            CLVF_B.Merge[0] = CLVF_B.h[0];

            // SignalConversion generated from: '<S351>/a_ff'
            CLVF_B.o_hat[1] = CLVF_B.a_ff[1];

            // SignalConversion generated from: '<S351>/h'
            CLVF_B.Merge[1] = CLVF_B.h[1];
            CLVF_B.Merge[2] = CLVF_B.h[2];

            // End of Outputs for SubSystem: '<S349>/CLVF'
          } else {
            // Outputs for IfAction SubSystem: '<S349>/Lyapunov' incorporates:
            //   ActionPort: '<S353>/Action Port'

            // SignalConversion generated from: '<S357>/ SFunction ' incorporates:
            //   Constant: '<S350>/Constant'
            //   Constant: '<S350>/Constant1'
            //   MATLAB Function: '<S353>/Body-Fixed Guidance'

            CLVF_B.vC_T[0] = CLVF_P.Constant_Value_d;
            CLVF_B.vC_T[1] = CLVF_P.Constant1_Value_e;
            CLVF_B.vC_T[2] = CLVF_B.RED_Tz_Shoulder;

            // MATLAB Function: '<S353>/Body-Fixed Guidance' incorporates:
            //   Constant: '<S350>/Constant'
            //   Constant: '<S350>/Constant1'
            //   SignalConversion generated from: '<S357>/ SFunction '

            // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/Lyapunov/Body-Fixed Guidance': '<S357>:1' 
            // '<S357>:1:3' C_IB = CT_BI';
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.C_IB[3 * CLVF_B.uElOffset1] = CLVF_B.y[CLVF_B.uElOffset1];
              CLVF_B.C_IB[3 * CLVF_B.uElOffset1 + 1] =
                CLVF_B.y[CLVF_B.uElOffset1 + 3];
              CLVF_B.C_IB[3 * CLVF_B.uElOffset1 + 2] =
                CLVF_B.y[CLVF_B.uElOffset1 + 6];
            }

            // '<S357>:1:5' wT_I = C_IB*wT_B;
            // '<S357>:1:6' w_dot_I = C_IB*w_dot_B;
            // '<S357>:1:8' o_hat_B = o_hat_prime;
            // '<S357>:1:10' o_hat = C_IB * o_hat_prime;
            // '<S357>:1:14' vD_I = vT_I + skew(wT_I)*(C_IB * d);
            // 'skew:6' A = [0 -v(3) v(2);v(3) 0 -v(1) ; -v(2) v(1) 0];
            // '<S357>:1:15' vC_D = vC_T - (C_IB* skew(wT_B)* d);
            // 'skew:6' A = [0 -v(3) v(2);v(3) 0 -v(1) ; -v(2) v(1) 0];
            CLVF_B.vc[0] = 0.0;
            CLVF_B.vc[3] = -CLVF_B.RED_Tz_Shoulder;
            CLVF_B.vc[6] = CLVF_P.Constant1_Value_e;
            CLVF_B.vc[1] = CLVF_B.RED_Tz_Shoulder;
            CLVF_B.vc[4] = 0.0;
            CLVF_B.vc[7] = -CLVF_P.Constant_Value_d;
            CLVF_B.vc[2] = -CLVF_P.Constant1_Value_e;
            CLVF_B.vc[5] = CLVF_P.Constant_Value_d;
            CLVF_B.vc[8] = 0.0;

            // Sum: '<S353>/Sum' incorporates:
            //   Constant: '<S342>/Constant1'
            //   Constant: '<S349>/Constant'

            CLVF_B.rtb_TmpSignalConversionAtSFun_f[0] = CLVF_B.uHzLPFilter -
              CLVF_P.Constant_Value_e[0];
            CLVF_B.rtb_TmpSignalConversionAtSFun_f[1] = CLVF_B.uHzLPFilter1 -
              CLVF_P.Constant_Value_e[1];
            CLVF_B.rtb_TmpSignalConversionAtSFun_f[2] = CLVF_P.Constant1_Value -
              CLVF_P.Constant_Value_e[2];

            // MATLAB Function: '<S353>/Body-Fixed Guidance' incorporates:
            //   Constant: '<S341>/Constant'
            //   Constant: '<S353>/Constant1'
            //   Constant: '<S353>/Constant2'

            // '<S357>:1:19' rD_T = C_IB*d;
            // '<S357>:1:20' rC_D = rC_T - rD_T;
            // '<S357>:1:24' aD_I = C_IB*(skew(wT_B)*skew(wT_B)*d + skew(w_dot_B)*d) ; 
            // 'skew:6' A = [0 -v(3) v(2);v(3) 0 -v(1) ; -v(2) v(1) 0];
            // 'skew:6' A = [0 -v(3) v(2);v(3) 0 -v(1) ; -v(2) v(1) 0];
            // 'skew:6' A = [0 -v(3) v(2);v(3) 0 -v(1) ; -v(2) v(1) 0];
            // '<S357>:1:26' q = 0.0001;
            // '<S357>:1:28' [hC_T, acc_ff] = LVF(rC_D, o_hat, q, v_max, theta_d, wT_I, vD_I, a_prime, finalAngle, vC_D, aD_I, w_dot_I, d, o_hat_B, CT_BI); 
            // 'LVF:10' r = sqrt(sum(r_T.^2));
            for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
              CLVF_B.d_T[CLVF_B.yElIdx] = 0.0;
              CLVF_B.w_dot_I[CLVF_B.yElIdx] = 0.0;
              CLVF_B.o_hat[CLVF_B.yElIdx] = 0.0;
              CLVF_B.C_IB_cv[CLVF_B.yElIdx] = 0.0;
              CLVF_B.rC_D_tmp[CLVF_B.yElIdx] = 0.0;
              for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3;
                   CLVF_B.uElOffset1++) {
                CLVF_B.ntIdx1 = CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1;
                CLVF_B.C_IB_c[CLVF_B.ntIdx1] = 0.0;
                CLVF_B.C_IB_c[CLVF_B.ntIdx1] += CLVF_B.vc[3 * CLVF_B.uElOffset1]
                  * CLVF_B.C_IB[CLVF_B.yElIdx];
                CLVF_B.C_IB_c[CLVF_B.ntIdx1] += CLVF_B.vc[3 * CLVF_B.uElOffset1
                  + 1] * CLVF_B.C_IB[CLVF_B.yElIdx + 3];
                CLVF_B.C_IB_c[CLVF_B.ntIdx1] += CLVF_B.vc[3 * CLVF_B.uElOffset1
                  + 2] * CLVF_B.C_IB[CLVF_B.yElIdx + 6];
                CLVF_B.d_T[CLVF_B.yElIdx] += CLVF_B.C_IB[CLVF_B.ntIdx1] *
                  CLVF_B.vC_T[CLVF_B.uElOffset1];
                CLVF_B.w_dot_I[CLVF_B.yElIdx] += CLVF_B.C_IB[CLVF_B.ntIdx1] *
                  CLVF_P.Constant_Value_hr[CLVF_B.uElOffset1];
                CLVF_B.o_hat[CLVF_B.yElIdx] += CLVF_B.C_IB[CLVF_B.ntIdx1] *
                  CLVF_P.o_hat_prime[CLVF_B.uElOffset1];
                CLVF_B.C_IB_cv[CLVF_B.yElIdx] += CLVF_B.C_IB_c[CLVF_B.ntIdx1] *
                  CLVF_P.d[CLVF_B.uElOffset1];
                CLVF_B.rC_D_tmp[CLVF_B.yElIdx] += CLVF_B.C_IB[CLVF_B.ntIdx1] *
                  CLVF_P.d[CLVF_B.uElOffset1];
              }

              CLVF_B.e_hat[CLVF_B.yElIdx] =
                CLVF_B.rtb_TmpSignalConversionAtSFun_f[CLVF_B.yElIdx] -
                CLVF_B.C_IB_cv[CLVF_B.yElIdx];
              CLVF_B.rtb_TmpSignalConversionAtSFun_g =
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.yElIdx] -
                CLVF_B.rC_D_tmp[CLVF_B.yElIdx];
              CLVF_B.del[CLVF_B.yElIdx] = CLVF_B.rtb_TmpSignalConversionAtSFun_g
                * CLVF_B.rtb_TmpSignalConversionAtSFun_g;
              CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.yElIdx] =
                CLVF_B.rtb_TmpSignalConversionAtSFun_g;
            }

            CLVF_B.RED_Tz_Shoulder = sqrt((CLVF_B.del[0] + CLVF_B.del[1]) +
              CLVF_B.del[2]);

            // 'LVF:14' o_mag = sqrt(sum(o_hat.^2));
            CLVF_B.RED_Tz_Wrist = sqrt((CLVF_B.o_hat[0] * CLVF_B.o_hat[0] +
              CLVF_B.o_hat[1] * CLVF_B.o_hat[1]) + CLVF_B.o_hat[2] *
              CLVF_B.o_hat[2]);

            // 'LVF:15' o_hat = o_hat./o_mag;
            // 'LVF:20' theta = acos(((CT_BI*r_T)'*o_hat_B)/(r*o_mag));
            CLVF_B.vc_dot = 0.0;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.vc_dot += (CLVF_B.y[CLVF_B.uElOffset1 + 6] *
                                CLVF_B.TmpSignalConversionAtSFun_n[2] +
                                (CLVF_B.y[CLVF_B.uElOffset1 + 3] *
                                 CLVF_B.TmpSignalConversionAtSFun_n[1] +
                                 CLVF_B.y[CLVF_B.uElOffset1] *
                                 CLVF_B.TmpSignalConversionAtSFun_n[0])) *
                CLVF_P.o_hat_prime[CLVF_B.uElOffset1];
              CLVF_B.o_hat[CLVF_B.uElOffset1] /= CLVF_B.RED_Tz_Wrist;
            }

            CLVF_B.theta_l = acos(CLVF_B.vc_dot / (CLVF_B.RED_Tz_Shoulder *
              CLVF_B.RED_Tz_Wrist));

            // 'LVF:27' r_hat = r_T./r;
            CLVF_B.TmpSignalConversionAtSFun_n[0] /= CLVF_B.RED_Tz_Shoulder;
            CLVF_B.TmpSignalConversionAtSFun_n[1] /= CLVF_B.RED_Tz_Shoulder;

            // MATLAB Function: '<S353>/Body-Fixed Guidance'
            CLVF_B.rtb_TmpSignalConversionAtSFun_g =
              CLVF_B.TmpSignalConversionAtSFun_n[2] / CLVF_B.RED_Tz_Shoulder;
            CLVF_B.TmpSignalConversionAtSFun_n[2] =
              CLVF_B.rtb_TmpSignalConversionAtSFun_g;

            // MATLAB Function: '<S353>/Body-Fixed Guidance' incorporates:
            //   Constant: '<S341>/Constant'
            //   Constant: '<S349>/Constant'
            //   Constant: '<S350>/Constant'
            //   Constant: '<S350>/Constant1'
            //   Constant: '<S353>/Constant'
            //   Constant: '<S353>/Constant1'
            //   Constant: '<S353>/Constant3'
            //   Constant: '<S353>/Constant4'
            //   Constant: '<S353>/Constant5'

            // 'LVF:31' a_hat = skew(r_hat)*skew(o_hat)*r_hat/(sin(theta) + q);
            // 'skew:6' A = [0 -v(3) v(2);v(3) 0 -v(1) ; -v(2) v(1) 0];
            // 'skew:6' A = [0 -v(3) v(2);v(3) 0 -v(1) ; -v(2) v(1) 0];
            CLVF_B.y6 = sin(CLVF_B.theta_l);
            CLVF_B.vc[0] = 0.0;
            CLVF_B.vc[3] = -CLVF_B.rtb_TmpSignalConversionAtSFun_g;
            CLVF_B.vc[6] = CLVF_B.TmpSignalConversionAtSFun_n[1];
            CLVF_B.vc[1] = CLVF_B.rtb_TmpSignalConversionAtSFun_g;
            CLVF_B.vc[4] = 0.0;
            CLVF_B.vc[7] = -CLVF_B.TmpSignalConversionAtSFun_n[0];
            CLVF_B.vc[2] = -CLVF_B.TmpSignalConversionAtSFun_n[1];
            CLVF_B.vc[5] = CLVF_B.TmpSignalConversionAtSFun_n[0];
            CLVF_B.vc[8] = 0.0;
            CLVF_B.C_IB_c[0] = 0.0;
            CLVF_B.C_IB_c[3] = -CLVF_B.o_hat[2];
            CLVF_B.C_IB_c[6] = CLVF_B.o_hat[1];
            CLVF_B.C_IB_c[1] = CLVF_B.o_hat[2];
            CLVF_B.C_IB_c[4] = 0.0;
            CLVF_B.C_IB_c[7] = -CLVF_B.o_hat[0];
            CLVF_B.C_IB_c[2] = -CLVF_B.o_hat[1];
            CLVF_B.C_IB_c[5] = CLVF_B.o_hat[0];
            CLVF_B.C_IB_c[8] = 0.0;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.RED_Tz_Wrist = 0.0;
              for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
                CLVF_B.ntIdx1 = CLVF_B.uElOffset1 + 3 * CLVF_B.yElIdx;
                CLVF_B.y[CLVF_B.ntIdx1] = 0.0;
                CLVF_B.y[CLVF_B.ntIdx1] += CLVF_B.C_IB_c[3 * CLVF_B.yElIdx] *
                  CLVF_B.vc[CLVF_B.uElOffset1];
                CLVF_B.y[CLVF_B.ntIdx1] += CLVF_B.C_IB_c[3 * CLVF_B.yElIdx + 1] *
                  CLVF_B.vc[CLVF_B.uElOffset1 + 3];
                CLVF_B.y[CLVF_B.ntIdx1] += CLVF_B.C_IB_c[3 * CLVF_B.yElIdx + 2] *
                  CLVF_B.vc[CLVF_B.uElOffset1 + 6];
                CLVF_B.RED_Tz_Wrist += CLVF_B.y[CLVF_B.ntIdx1] *
                  CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.yElIdx];
              }

              CLVF_B.del[CLVF_B.uElOffset1] = CLVF_B.RED_Tz_Wrist / (CLVF_B.y6 +
                0.0001);
            }

            // 'LVF:39' if theta <= theta_d
            if (CLVF_B.theta_l <= CLVF_P.theta_d) {
              // 'LVF:40' theta_N = (theta/theta_d)*pi/2;
              CLVF_B.vc_dot = CLVF_B.theta_l / CLVF_P.theta_d *
                3.1415926535897931 / 2.0;
            } else {
              // 'LVF:41' else
              // 'LVF:42' theta_N = pi/2;
              CLVF_B.vc_dot = 1.5707963267948966;
            }

            // 'LVF:50' g = r;
            // 'LVF:66' if r >= a_prime
            if (CLVF_B.RED_Tz_Shoulder >= CLVF_P.a_prime) {
              // 'LVF:67' rN = finalAngle;
              CLVF_B.g_dot = CLVF_P.finalAngle;
            } else {
              // 'LVF:68' else
              // 'LVF:69' rN = r/a_prime*finalAngle;
              CLVF_B.g_dot = CLVF_B.RED_Tz_Shoulder / CLVF_P.a_prime *
                CLVF_P.finalAngle;
            }

            // 'LVF:73' v_rel = v_max*sin(rN);
            CLVF_B.v_rel = CLVF_P.v_max * sin(CLVF_B.g_dot);

            // 'LVF:76' sa = v_rel*sin(theta_N);
            CLVF_B.g = sin(CLVF_B.vc_dot);
            CLVF_B.RED_Tz_Wrist = CLVF_B.v_rel * CLVF_B.g;

            // 'LVF:79' vc = -v_rel*cos(theta_N);
            CLVF_B.dgdr = cos(CLVF_B.vc_dot);
            CLVF_B.vc_p = -CLVF_B.v_rel * CLVF_B.dgdr;

            // 'LVF:85' h = vc*r_hat + sa*a_hat + g*cross(w_OI,r_hat) + vt_I;
            // 'LVF:89' e_hat = cross(r_hat,a_hat);
            // 'LVF:97' r_hat_dot = (eye(3,3) - r_hat*r_hat')/r * vC_T;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 9; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.b_I_g[CLVF_B.uElOffset1] = 0;
            }

            for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
              CLVF_B.b_I_g[CLVF_B.yElIdx + 3 * CLVF_B.yElIdx] = 1;
              CLVF_B.y[3 * CLVF_B.yElIdx] = CLVF_B.TmpSignalConversionAtSFun_n[0]
                * CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.yElIdx];
              CLVF_B.y[3 * CLVF_B.yElIdx + 1] =
                CLVF_B.TmpSignalConversionAtSFun_n[1] *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.yElIdx];
              CLVF_B.y[3 * CLVF_B.yElIdx + 2] =
                CLVF_B.rtb_TmpSignalConversionAtSFun_g *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.yElIdx];
            }

            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 9; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.b_I[CLVF_B.uElOffset1] = (static_cast<real_T>
                (CLVF_B.b_I_g[CLVF_B.uElOffset1]) - CLVF_B.y[CLVF_B.uElOffset1])
                / CLVF_B.RED_Tz_Shoulder;
            }

            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.r_hat_dot[CLVF_B.uElOffset1] = CLVF_B.b_I[CLVF_B.uElOffset1
                + 6] * CLVF_B.e_hat[2] + (CLVF_B.b_I[CLVF_B.uElOffset1 + 3] *
                CLVF_B.e_hat[1] + CLVF_B.b_I[CLVF_B.uElOffset1] * CLVF_B.e_hat[0]);
            }

            // 'LVF:107' d_a_hat_d_r_hat = (eye(3,3) - a_hat*a_hat')/(sin(theta)+q)  *  (2*o_hat*r_hat'-r_hat*o_hat' - (r_hat'*o_hat)*eye(3,3)); 
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 9; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.b_I_g[CLVF_B.uElOffset1] = 0;
            }

            CLVF_B.b_I_g[0] = 1;
            CLVF_B.b_I_g[4] = 1;
            CLVF_B.b_I_g[8] = 1;
            CLVF_B.rtb_TmpSignalConversionAtSFun_j =
              (CLVF_B.TmpSignalConversionAtSFun_n[0] * CLVF_B.o_hat[0] +
               CLVF_B.TmpSignalConversionAtSFun_n[1] * CLVF_B.o_hat[1]) +
              CLVF_B.rtb_TmpSignalConversionAtSFun_g * CLVF_B.o_hat[2];

            // 'LVF:109' d_a_hat_d_o_hat = (eye(3,3) - r_hat*r_hat')*(eye(3,3) - a_hat*a_hat')/(sin(theta) + q); 
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 9; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.c_I_m[CLVF_B.uElOffset1] = 0;
            }

            CLVF_B.c_I_m[0] = 1;
            CLVF_B.c_I_m[4] = 1;
            CLVF_B.c_I_m[8] = 1;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 9; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.d_I_n[CLVF_B.uElOffset1] = 0;
            }

            // 'LVF:111' a_hat_dot = d_a_hat_d_r_hat*r_hat_dot + d_a_hat_d_o_hat*cross(w_OI,o_hat); 
            // 'LVF:115' r_dot = r_hat'*vC_T;
            // 'LVF:118' theta_dot = -o_hat'/(sin(theta)+q) * r_hat_dot + (-r_hat'*skew(w_OI)*o_hat)/sin(theta); 
            // 'skew:6' A = [0 -v(3) v(2);v(3) 0 -v(1) ; -v(2) v(1) 0];
            CLVF_B.d_I_n[0] = 1;
            CLVF_B.d_I_n[4] = 1;
            CLVF_B.d_I_n[8] = 1;
            CLVF_B.rtb_TmpSignalConversionAtSFun_d =
              (CLVF_B.TmpSignalConversionAtSFun_n[0] * CLVF_B.e_hat[0] +
               CLVF_B.TmpSignalConversionAtSFun_n[1] * CLVF_B.e_hat[1]) +
              CLVF_B.rtb_TmpSignalConversionAtSFun_g * CLVF_B.e_hat[2];
            CLVF_B.vc[0] = 0.0;
            CLVF_B.vc[3] = -CLVF_B.d_T[2];
            CLVF_B.vc[6] = CLVF_B.d_T[1];
            CLVF_B.vc[1] = CLVF_B.d_T[2];
            CLVF_B.vc[4] = 0.0;
            CLVF_B.vc[7] = -CLVF_B.d_T[0];
            CLVF_B.vc[2] = -CLVF_B.d_T[1];
            CLVF_B.vc[5] = CLVF_B.d_T[0];
            CLVF_B.vc[8] = 0.0;
            CLVF_B.y7 = 0.0;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.y7 += (CLVF_B.vc[3 * CLVF_B.uElOffset1 + 2] *
                            -CLVF_B.rtb_TmpSignalConversionAtSFun_g +
                            (CLVF_B.vc[3 * CLVF_B.uElOffset1 + 1] *
                             -CLVF_B.TmpSignalConversionAtSFun_n[1] + CLVF_B.vc
                             [3 * CLVF_B.uElOffset1] *
                             -CLVF_B.TmpSignalConversionAtSFun_n[0])) *
                CLVF_B.o_hat[CLVF_B.uElOffset1];
            }

            CLVF_B.theta_dot = ((-CLVF_B.o_hat[0] / (CLVF_B.y6 + 0.0001) *
                                 CLVF_B.r_hat_dot[0] + -CLVF_B.o_hat[1] /
                                 (CLVF_B.y6 + 0.0001) * CLVF_B.r_hat_dot[1]) +
                                -CLVF_B.o_hat[2] / (CLVF_B.y6 + 0.0001) *
                                CLVF_B.r_hat_dot[2]) + CLVF_B.y7 / CLVF_B.y6;

            // 'LVF:121' dgdr = 1;
            // 'LVF:122' g_dot = dgdr * r_dot;
            // 'LVF:126' if r <= a_prime
            if (CLVF_B.RED_Tz_Shoulder <= CLVF_P.a_prime) {
              // 'LVF:127' dvreldr = v_max*cos(rN)*finalAngle/a_prime;
              CLVF_B.g_dot = CLVF_P.v_max * cos(CLVF_B.g_dot) *
                CLVF_P.finalAngle / CLVF_P.a_prime;
            } else {
              // 'LVF:128' else
              // 'LVF:129' dvreldr = 0;
              CLVF_B.g_dot = 0.0;
            }

            // 'LVF:143' dvcdr = -dvreldr*cos(theta_N);
            // 'LVF:145' if theta < theta_d
            if (CLVF_B.theta_l < CLVF_P.theta_d) {
              // 'LVF:146' dvcdtheta = v_rel*sin(theta_N) * pi/(2*theta_d);
              CLVF_B.vc_dot = CLVF_B.v_rel * sin(CLVF_B.vc_dot) *
                3.1415926535897931 / (2.0 * CLVF_P.theta_d);
            } else {
              // 'LVF:147' else
              // 'LVF:148' dvcdtheta = 0;
              CLVF_B.vc_dot = 0.0;
            }

            // 'LVF:151' vc_dot = dvcdr*r_dot + dvcdtheta*theta_dot;
            CLVF_B.vc_dot = -CLVF_B.g_dot * CLVF_B.dgdr *
              CLVF_B.rtb_TmpSignalConversionAtSFun_d + CLVF_B.vc_dot *
              CLVF_B.theta_dot;

            // 'LVF:162' dsadr = dvreldr*sin(theta_N);
            // 'LVF:164' if theta < theta_d
            if (CLVF_B.theta_l < CLVF_P.theta_d) {
              // 'LVF:165' dsadtheta = v_rel*cos(theta_N) * pi/(2*theta_d);
              CLVF_B.dgdr = CLVF_B.v_rel * CLVF_B.dgdr * 3.1415926535897931 /
                (2.0 * CLVF_P.theta_d);
            } else {
              // 'LVF:166' else
              // 'LVF:167' dsadtheta = 0;
              CLVF_B.dgdr = 0.0;
            }

            // 'LVF:170' sa_dot = dsadr*r_dot + dsadtheta*theta_dot;
            CLVF_B.dgdr = CLVF_B.g_dot * CLVF_B.g *
              CLVF_B.rtb_TmpSignalConversionAtSFun_d + CLVF_B.dgdr *
              CLVF_B.theta_dot;

            // 'LVF:174' a_ff = vc_dot*r_hat + vc*r_hat_dot + sa_dot*a_hat + sa*a_hat_dot + g_dot*cross(w_OI,r_hat) ... 
            // 'LVF:175'                         + g*cross(w_dot_OI, r_hat) + g*cross(w_OI,r_hat_dot) + at_I; 
            CLVF_B.g = CLVF_B.d_T[1] * CLVF_B.rtb_TmpSignalConversionAtSFun_g -
              CLVF_B.d_T[2] * CLVF_B.TmpSignalConversionAtSFun_n[1];
            CLVF_B.C_IB_cv[0] = CLVF_B.g * CLVF_B.RED_Tz_Shoulder;
            CLVF_B.g_dot = CLVF_B.d_T[2] * CLVF_B.TmpSignalConversionAtSFun_n[0]
              - CLVF_B.d_T[0] * CLVF_B.rtb_TmpSignalConversionAtSFun_g;
            CLVF_B.C_IB_cv[1] = CLVF_B.g_dot * CLVF_B.RED_Tz_Shoulder;
            CLVF_B.y7 = CLVF_B.d_T[0] * CLVF_B.TmpSignalConversionAtSFun_n[1] -
              CLVF_B.d_T[1] * CLVF_B.TmpSignalConversionAtSFun_n[0];
            CLVF_B.C_IB_cv[2] = CLVF_B.y7 * CLVF_B.RED_Tz_Shoulder;
            CLVF_B.vc[0] = 0.0;
            CLVF_B.vc[3] = -CLVF_B.d_T[2];
            CLVF_B.vc[6] = CLVF_B.d_T[1];
            CLVF_B.vc[1] = CLVF_B.d_T[2];
            CLVF_B.vc[4] = 0.0;
            CLVF_B.vc[7] = -CLVF_B.d_T[0];
            CLVF_B.vc[2] = -CLVF_B.d_T[1];
            CLVF_B.vc[5] = CLVF_B.d_T[0];
            CLVF_B.vc[8] = 0.0;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.hC_T[CLVF_B.uElOffset1] = (((CLVF_B.vc[CLVF_B.uElOffset1 +
                3] * CLVF_B.rC_D_tmp[1] + CLVF_B.vc[CLVF_B.uElOffset1] *
                CLVF_B.rC_D_tmp[0]) + CLVF_B.vc[CLVF_B.uElOffset1 + 6] *
                CLVF_B.rC_D_tmp[2]) + CLVF_P.Constant_Value_e[CLVF_B.uElOffset1])
                + ((CLVF_B.vc_p *
                    CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1] +
                    CLVF_B.RED_Tz_Wrist * CLVF_B.del[CLVF_B.uElOffset1]) +
                   CLVF_B.C_IB_cv[CLVF_B.uElOffset1]);
              CLVF_B.acc_ff_tmp[3 * CLVF_B.uElOffset1] = CLVF_B.del[0] *
                CLVF_B.del[CLVF_B.uElOffset1];
              CLVF_B.C_IB_c[3 * CLVF_B.uElOffset1] = 2.0 * CLVF_B.o_hat[0] *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1];
              CLVF_B.rtb_TmpSignalConversionAtSFu_kk[3 * CLVF_B.uElOffset1] =
                CLVF_B.TmpSignalConversionAtSFun_n[0] *
                CLVF_B.o_hat[CLVF_B.uElOffset1];
              CLVF_B.yElIdx = 3 * CLVF_B.uElOffset1 + 1;
              CLVF_B.acc_ff_tmp[CLVF_B.yElIdx] = CLVF_B.del[1] *
                CLVF_B.del[CLVF_B.uElOffset1];
              CLVF_B.C_IB_c[CLVF_B.yElIdx] = 2.0 * CLVF_B.o_hat[1] *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1];
              CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.yElIdx] =
                CLVF_B.TmpSignalConversionAtSFun_n[1] *
                CLVF_B.o_hat[CLVF_B.uElOffset1];
              CLVF_B.yElIdx = 3 * CLVF_B.uElOffset1 + 2;
              CLVF_B.acc_ff_tmp[CLVF_B.yElIdx] = CLVF_B.del[2] *
                CLVF_B.del[CLVF_B.uElOffset1];
              CLVF_B.C_IB_c[CLVF_B.yElIdx] = 2.0 * CLVF_B.o_hat[2] *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1];
              CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.yElIdx] =
                CLVF_B.rtb_TmpSignalConversionAtSFun_g *
                CLVF_B.o_hat[CLVF_B.uElOffset1];
            }

            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 9; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.b_I[CLVF_B.uElOffset1] = (static_cast<real_T>
                (CLVF_B.b_I_g[CLVF_B.uElOffset1]) -
                CLVF_B.acc_ff_tmp[CLVF_B.uElOffset1]) / (CLVF_B.y6 + 0.0001);
              CLVF_B.vc[CLVF_B.uElOffset1] = (CLVF_B.C_IB_c[CLVF_B.uElOffset1] -
                CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.uElOffset1]) -
                CLVF_B.rtb_TmpSignalConversionAtSFun_j * static_cast<real_T>
                (b_1[CLVF_B.uElOffset1]);
            }

            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
                CLVF_B.ntIdx1 = CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1;
                CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.ntIdx1] = 0.0;
                CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.ntIdx1] +=
                  CLVF_B.vc[3 * CLVF_B.uElOffset1] * CLVF_B.b_I[CLVF_B.yElIdx];
                CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.ntIdx1] +=
                  CLVF_B.vc[3 * CLVF_B.uElOffset1 + 1] *
                  CLVF_B.b_I[CLVF_B.yElIdx + 3];
                CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.ntIdx1] +=
                  CLVF_B.vc[3 * CLVF_B.uElOffset1 + 2] *
                  CLVF_B.b_I[CLVF_B.yElIdx + 6];
              }
            }

            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 9; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.b_I[CLVF_B.uElOffset1] = static_cast<real_T>
                (CLVF_B.c_I_m[CLVF_B.uElOffset1]) - CLVF_B.y[CLVF_B.uElOffset1];
              CLVF_B.d_I[CLVF_B.uElOffset1] = static_cast<real_T>
                (CLVF_B.d_I_n[CLVF_B.uElOffset1]) -
                CLVF_B.acc_ff_tmp[CLVF_B.uElOffset1];
            }

            CLVF_B.e_hat[0] = CLVF_B.d_T[1] * CLVF_B.o_hat[2] - CLVF_B.d_T[2] *
              CLVF_B.o_hat[1];
            CLVF_B.e_hat[1] = CLVF_B.d_T[2] * CLVF_B.o_hat[0] - CLVF_B.d_T[0] *
              CLVF_B.o_hat[2];
            CLVF_B.e_hat[2] = CLVF_B.d_T[0] * CLVF_B.o_hat[1] - CLVF_B.d_T[1] *
              CLVF_B.o_hat[0];
            CLVF_B.rC_D_tmp[0] = CLVF_B.g *
              CLVF_B.rtb_TmpSignalConversionAtSFun_d;
            CLVF_B.rC_D_tmp[1] = CLVF_B.g_dot *
              CLVF_B.rtb_TmpSignalConversionAtSFun_d;
            CLVF_B.rC_D_tmp[2] = CLVF_B.y7 *
              CLVF_B.rtb_TmpSignalConversionAtSFun_d;
            CLVF_B.C_IB_cv[0] = (CLVF_B.w_dot_I[1] *
                                 CLVF_B.rtb_TmpSignalConversionAtSFun_g -
                                 CLVF_B.w_dot_I[2] *
                                 CLVF_B.TmpSignalConversionAtSFun_n[1]) *
              CLVF_B.RED_Tz_Shoulder;
            CLVF_B.C_IB_cv[1] = (CLVF_B.w_dot_I[2] *
                                 CLVF_B.TmpSignalConversionAtSFun_n[0] -
                                 CLVF_B.w_dot_I[0] *
                                 CLVF_B.rtb_TmpSignalConversionAtSFun_g) *
              CLVF_B.RED_Tz_Shoulder;
            CLVF_B.C_IB_cv[2] = (CLVF_B.w_dot_I[0] *
                                 CLVF_B.TmpSignalConversionAtSFun_n[1] -
                                 CLVF_B.w_dot_I[1] *
                                 CLVF_B.TmpSignalConversionAtSFun_n[0]) *
              CLVF_B.RED_Tz_Shoulder;
            CLVF_B.w_dot_I[0] = (CLVF_B.d_T[1] * CLVF_B.r_hat_dot[2] -
                                 CLVF_B.d_T[2] * CLVF_B.r_hat_dot[1]) *
              CLVF_B.RED_Tz_Shoulder;
            CLVF_B.w_dot_I[1] = (CLVF_B.d_T[2] * CLVF_B.r_hat_dot[0] -
                                 CLVF_B.d_T[0] * CLVF_B.r_hat_dot[2]) *
              CLVF_B.RED_Tz_Shoulder;
            CLVF_B.w_dot_I[2] = (CLVF_B.d_T[0] * CLVF_B.r_hat_dot[1] -
                                 CLVF_B.d_T[1] * CLVF_B.r_hat_dot[0]) *
              CLVF_B.RED_Tz_Shoulder;
            CLVF_B.vc[0] = 0.0;
            CLVF_B.vc[3] = -CLVF_B.vC_T[2];
            CLVF_B.vc[6] = CLVF_P.Constant1_Value_e;
            CLVF_B.vc[1] = CLVF_B.vC_T[2];
            CLVF_B.vc[4] = 0.0;
            CLVF_B.vc[7] = -CLVF_P.Constant_Value_d;
            CLVF_B.vc[2] = -CLVF_P.Constant1_Value_e;
            CLVF_B.vc[5] = CLVF_P.Constant_Value_d;
            CLVF_B.vc[8] = 0.0;
            CLVF_B.C_IB_c[0] = 0.0;
            CLVF_B.C_IB_c[3] = -CLVF_B.vC_T[2];
            CLVF_B.C_IB_c[6] = CLVF_P.Constant1_Value_e;
            CLVF_B.C_IB_c[1] = CLVF_B.vC_T[2];
            CLVF_B.C_IB_c[4] = 0.0;
            CLVF_B.C_IB_c[7] = -CLVF_P.Constant_Value_d;
            CLVF_B.C_IB_c[2] = -CLVF_P.Constant1_Value_e;
            CLVF_B.C_IB_c[5] = CLVF_P.Constant_Value_d;
            CLVF_B.C_IB_c[8] = 0.0;
            CLVF_B.acc_ff_tmp[0] = 0.0;
            CLVF_B.acc_ff_tmp[3] = -CLVF_P.Constant_Value_hr[2];
            CLVF_B.acc_ff_tmp[6] = CLVF_P.Constant_Value_hr[1];
            CLVF_B.acc_ff_tmp[1] = CLVF_P.Constant_Value_hr[2];
            CLVF_B.acc_ff_tmp[4] = 0.0;
            CLVF_B.acc_ff_tmp[7] = -CLVF_P.Constant_Value_hr[0];
            CLVF_B.acc_ff_tmp[2] = -CLVF_P.Constant_Value_hr[1];
            CLVF_B.acc_ff_tmp[5] = CLVF_P.Constant_Value_hr[0];
            CLVF_B.acc_ff_tmp[8] = 0.0;
            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.vC_T[CLVF_B.uElOffset1] = 0.0;
              CLVF_B.c_I[CLVF_B.uElOffset1] = 0.0;
              CLVF_B.rtb_TmpSignalConversionAtSFun_f[CLVF_B.uElOffset1] = 0.0;
              CLVF_B.dv[CLVF_B.uElOffset1] = 0.0;
              for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
                CLVF_B.ntIdx1 = 3 * CLVF_B.yElIdx + CLVF_B.uElOffset1;
                CLVF_B.ntIdx0 = 3 * CLVF_B.yElIdx + 1;
                CLVF_B.uElOffset0 = 3 * CLVF_B.yElIdx + 2;
                CLVF_B.y[CLVF_B.ntIdx1] = 0.0;
                CLVF_B.y[CLVF_B.ntIdx1] += CLVF_B.C_IB_c[3 * CLVF_B.yElIdx] *
                  CLVF_B.vc[CLVF_B.uElOffset1];
                CLVF_B.y[CLVF_B.ntIdx1] += CLVF_B.C_IB_c[CLVF_B.ntIdx0] *
                  CLVF_B.vc[CLVF_B.uElOffset1 + 3];
                CLVF_B.y[CLVF_B.ntIdx1] += CLVF_B.C_IB_c[CLVF_B.uElOffset0] *
                  CLVF_B.vc[CLVF_B.uElOffset1 + 6];
                CLVF_B.vC_T[CLVF_B.uElOffset1] +=
                  CLVF_B.rtb_TmpSignalConversionAtSFu_kk[CLVF_B.ntIdx1] *
                  CLVF_B.r_hat_dot[CLVF_B.yElIdx];
                CLVF_B.c_I[CLVF_B.uElOffset1] += ((CLVF_B.d_I[CLVF_B.ntIdx0] *
                  CLVF_B.b_I[CLVF_B.uElOffset1 + 3] + CLVF_B.d_I[3 *
                  CLVF_B.yElIdx] * CLVF_B.b_I[CLVF_B.uElOffset1]) +
                  CLVF_B.d_I[CLVF_B.uElOffset0] * CLVF_B.b_I[CLVF_B.uElOffset1 +
                  6]) / (CLVF_B.y6 + 0.0001) * CLVF_B.e_hat[CLVF_B.yElIdx];
                CLVF_B.rtb_TmpSignalConversionAtSFun_f[CLVF_B.uElOffset1] +=
                  CLVF_B.y[CLVF_B.ntIdx1] * CLVF_P.d[CLVF_B.yElIdx];
                CLVF_B.dv[CLVF_B.uElOffset1] += CLVF_B.acc_ff_tmp[CLVF_B.ntIdx1]
                  * CLVF_P.d[CLVF_B.yElIdx];
              }

              CLVF_B.dv1[CLVF_B.uElOffset1] =
                CLVF_B.rtb_TmpSignalConversionAtSFun_f[CLVF_B.uElOffset1] +
                CLVF_B.dv[CLVF_B.uElOffset1];
            }

            for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1
                 ++) {
              CLVF_B.acc_ff[CLVF_B.uElOffset1] = ((((((CLVF_B.vc_dot *
                CLVF_B.TmpSignalConversionAtSFun_n[CLVF_B.uElOffset1] +
                CLVF_B.vc_p * CLVF_B.r_hat_dot[CLVF_B.uElOffset1]) + CLVF_B.dgdr
                * CLVF_B.del[CLVF_B.uElOffset1]) +
                (CLVF_B.vC_T[CLVF_B.uElOffset1] + CLVF_B.c_I[CLVF_B.uElOffset1])
                * CLVF_B.RED_Tz_Wrist) + CLVF_B.rC_D_tmp[CLVF_B.uElOffset1]) +
                CLVF_B.C_IB_cv[CLVF_B.uElOffset1]) +
                CLVF_B.w_dot_I[CLVF_B.uElOffset1]) +
                ((CLVF_B.C_IB[CLVF_B.uElOffset1 + 3] * CLVF_B.dv1[1] +
                  CLVF_B.C_IB[CLVF_B.uElOffset1] * CLVF_B.dv1[0]) +
                 CLVF_B.C_IB[CLVF_B.uElOffset1 + 6] * CLVF_B.dv1[2]);
            }

            // Derivative: '<S353>/Derivative'
            CLVF_B.RED_Tz_Wrist = CLVF_M->Timing.t[0];
            if ((CLVF_DW.TimeStampA >= CLVF_B.RED_Tz_Wrist) &&
                (CLVF_DW.TimeStampB >= CLVF_B.RED_Tz_Wrist)) {
              CLVF_B.Derivative[0] = 0.0;
              CLVF_B.Derivative[1] = 0.0;
              CLVF_B.Derivative[2] = 0.0;
            } else {
              CLVF_B.RED_Tz_Shoulder = CLVF_DW.TimeStampA;
              lastU = &CLVF_DW.LastUAtTimeA;
              if (CLVF_DW.TimeStampA < CLVF_DW.TimeStampB) {
                if (CLVF_DW.TimeStampB < CLVF_B.RED_Tz_Wrist) {
                  CLVF_B.RED_Tz_Shoulder = CLVF_DW.TimeStampB;
                  lastU = &CLVF_DW.LastUAtTimeB;
                }
              } else {
                if (CLVF_DW.TimeStampA >= CLVF_B.RED_Tz_Wrist) {
                  CLVF_B.RED_Tz_Shoulder = CLVF_DW.TimeStampB;
                  lastU = &CLVF_DW.LastUAtTimeB;
                }
              }

              CLVF_B.RED_Tz_Shoulder = CLVF_B.RED_Tz_Wrist -
                CLVF_B.RED_Tz_Shoulder;
              CLVF_B.Derivative[0] = (CLVF_B.hC_T[0] - (*lastU)[0]) /
                CLVF_B.RED_Tz_Shoulder;
              CLVF_B.Derivative[1] = (CLVF_B.hC_T[1] - (*lastU)[1]) /
                CLVF_B.RED_Tz_Shoulder;
              CLVF_B.Derivative[2] = (CLVF_B.hC_T[2] - (*lastU)[2]) /
                CLVF_B.RED_Tz_Shoulder;
            }

            // End of Derivative: '<S353>/Derivative'
            // Sum: '<S353>/Sum1'
            CLVF_B.Sum1[0] = CLVF_B.Derivative[0] - CLVF_B.acc_ff[0];
            CLVF_B.Sum1[1] = CLVF_B.Derivative[1] - CLVF_B.acc_ff[1];
            CLVF_B.Sum1[2] = CLVF_B.Derivative[2] - CLVF_B.acc_ff[2];

            // ToWorkspace: '<S353>/To Workspace'
            rt_UpdateLogVar((LogVar *)(LogVar*)
                            (CLVF_DW.ToWorkspace_PWORK_d.LoggedData),
                            &CLVF_B.acc_ff[0], 0);

            // SignalConversion generated from: '<S353>/acc_ff'
            CLVF_B.o_hat[0] = CLVF_B.acc_ff[0];

            // SignalConversion generated from: '<S353>/h'
            CLVF_B.Merge[0] = CLVF_B.hC_T[0];

            // SignalConversion generated from: '<S353>/acc_ff'
            CLVF_B.o_hat[1] = CLVF_B.acc_ff[1];

            // SignalConversion generated from: '<S353>/h'
            CLVF_B.Merge[1] = CLVF_B.hC_T[1];
            CLVF_B.Merge[2] = CLVF_B.hC_T[2];

            // End of Outputs for SubSystem: '<S349>/Lyapunov'
          }

          // DataStoreWrite: '<S324>/RED_Fx' incorporates:
          //   Constant: '<S352>/Constant'
          //   Constant: '<S352>/Constant1'
          //   MATLAB Function: '<S352>/PD + FF Controller'
          //   SignalConversion generated from: '<S356>/ SFunction '

          // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/Lyapunov GNC/CLVF//LVF System/Control/PD + FF Controller': '<S356>:1' 
          // '<S356>:1:3' f = kd*(h - vC_I)+ m*a_ff;
          CLVF_DW.BLACK_Fx = (CLVF_B.Merge[0] - CLVF_B.uHzLPFilter) * CLVF_P.kd
            + CLVF_P.BLACKMass * CLVF_B.o_hat[0];

          // DataStoreWrite: '<S324>/RED_Fy' incorporates:
          //   Constant: '<S352>/Constant'
          //   Constant: '<S352>/Constant1'
          //   MATLAB Function: '<S352>/PD + FF Controller'
          //   SignalConversion generated from: '<S356>/ SFunction '

          CLVF_DW.BLACK_Fy = (CLVF_B.Merge[1] - CLVF_B.uHzLPFilter1) * CLVF_P.kd
            + CLVF_P.BLACKMass * CLVF_B.o_hat[1];

          // DataStoreWrite: '<S349>/Data Store Write'
          CLVF_DW.SPEED_DESIRED_x = CLVF_B.Merge[0];

          // DataStoreWrite: '<S349>/Data Store Write1'
          CLVF_DW.SPEED_DESIRED_y = CLVF_B.Merge[1];

          // ToWorkspace: '<S349>/To Workspace'
          rt_UpdateLogVar((LogVar *)(LogVar*)
                          (CLVF_DW.ToWorkspace_PWORK_h.LoggedData), &CLVF_B.In,
                          0);

          // ToWorkspace: '<S349>/To Workspace2'
          rt_UpdateLogVar((LogVar *)(LogVar*)
                          (CLVF_DW.ToWorkspace2_PWORK.LoggedData),
                          &CLVF_B.Merge[0], 0);

          // DataStoreWrite: '<S324>/Data Store Write3' incorporates:
          //   Constant: '<S324>/Puck State'

          CLVF_DW.Float_State = CLVF_P.PuckState_Value_gw;

          // Sum: '<S342>/Sum1' incorporates:
          //   DataStoreRead: '<S342>/Data Store Read2'
          //   DataStoreRead: '<S342>/Data Store Read3'

          CLVF_B.o_hat[0] = CLVF_B.DataStoreRead - CLVF_DW.RED_Px;
          CLVF_B.o_hat[1] = CLVF_B.DataStoreRead1 - CLVF_DW.RED_Py;

          // MATLAB Function: '<S342>/Norm' incorporates:
          //   DataStoreRead: '<S342>/Data Store Read9'

          // MATLAB Function 'Phase #3: Experiment/Change BLACK Behavior/Sub-Phase #4/States in Target-Fixed Frame/Norm': '<S372>:1' 
          // '<S372>:1:3' r = sqrt(sum(rC_T.^2));
          CLVF_B.RED_Tz_Shoulder = sqrt((CLVF_B.o_hat[0] * CLVF_B.o_hat[0] +
            CLVF_B.o_hat[1] * CLVF_B.o_hat[1]) + CLVF_B.RED_Tz_Elbow *
            CLVF_B.RED_Tz_Elbow);

          // '<S372>:1:4' r_hat = rC_T/r;
          CLVF_B.TmpSignalConversionAtSFun_n[0] = CLVF_B.o_hat[0] /
            CLVF_B.RED_Tz_Shoulder;
          CLVF_B.TmpSignalConversionAtSFun_n[1] = CLVF_B.o_hat[1] /
            CLVF_B.RED_Tz_Shoulder;
          CLVF_B.TmpSignalConversionAtSFun_n[2] = CLVF_B.RED_Tz_Elbow /
            CLVF_B.RED_Tz_Shoulder;

          // '<S372>:1:6' C_BI = C3(theta_T);
          // 'C3:6' A = [cos(z) sin(z) 0; -sin(z) cos(z) 0 ; 0 0 1];
          CLVF_B.RED_Tz_Wrist = sin(CLVF_DW.RED_Rz);
          CLVF_B.vc_p = cos(CLVF_DW.RED_Rz);
          CLVF_B.C_IB[0] = CLVF_B.vc_p;
          CLVF_B.C_IB[3] = CLVF_B.RED_Tz_Wrist;
          CLVF_B.C_IB[6] = 0.0;
          CLVF_B.C_IB[1] = -CLVF_B.RED_Tz_Wrist;
          CLVF_B.C_IB[4] = CLVF_B.vc_p;
          CLVF_B.C_IB[7] = 0.0;
          CLVF_B.C_IB[2] = 0.0;
          CLVF_B.C_IB[5] = 0.0;
          CLVF_B.C_IB[8] = 1.0;

          // '<S372>:1:9' theta = acos(r_hat'*C_BI'*o_hat_B);
          for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++)
          {
            CLVF_B.RED_Tz_Wrist = (CLVF_B.C_IB[CLVF_B.uElOffset1 + 3] *
              CLVF_B.TmpSignalConversionAtSFun_n[1] +
              CLVF_B.TmpSignalConversionAtSFun_n[0] *
              CLVF_B.C_IB[CLVF_B.uElOffset1]) + CLVF_B.C_IB[CLVF_B.uElOffset1 +
              6] * CLVF_B.TmpSignalConversionAtSFun_n[2];
            CLVF_B.o_hat[CLVF_B.uElOffset1] = CLVF_B.RED_Tz_Wrist;
          }

          CLVF_B.theta_b = (CLVF_B.o_hat[0] * CLVF_P.o_hat_B[0] + CLVF_B.o_hat[1]
                            * CLVF_P.o_hat_B[1]) + CLVF_B.o_hat[2] *
            CLVF_P.o_hat_B[2];
          CLVF_B.theta_b = acos(CLVF_B.theta_b);
          CLVF_B.r_e = CLVF_B.RED_Tz_Shoulder;

          // End of MATLAB Function: '<S342>/Norm'

          // ToWorkspace: '<S342>/To Workspace'
          rt_UpdateLogVar((LogVar *)(LogVar*)
                          (CLVF_DW.ToWorkspace_PWORK_k.LoggedData), &CLVF_B.r_e,
                          0);

          // ToWorkspace: '<S342>/To Workspace1'
          rt_UpdateLogVar((LogVar *)(LogVar*)
                          (CLVF_DW.ToWorkspace1_PWORK.LoggedData), &CLVF_B.r, 0);

          // ToWorkspace: '<S342>/To Workspace2'
          rt_UpdateLogVar((LogVar *)(LogVar*)
                          (CLVF_DW.ToWorkspace2_PWORK_i.LoggedData),
                          &CLVF_B.theta_b, 0);

          // ToWorkspace: '<S342>/To Workspace3'
          rt_UpdateLogVar((LogVar *)(LogVar*)
                          (CLVF_DW.ToWorkspace3_PWORK.LoggedData), &CLVF_B.theta,
                          0);

          // End of Outputs for SubSystem: '<S317>/Sub-Phase #4'
          break;
        }

        // End of If: '<S317>/Experiment Sub-Phases'
        // End of Outputs for SubSystem: '<S11>/Change BLACK Behavior'
      }

      // End of If: '<S11>/This IF block determines whether or not to run the BLACK sim//exp' 

      // If: '<S11>/This IF block determines whether or not to run the BLUE sim//exp' incorporates:
      //   Constant: '<S11>/Constant'
      //   Constant: '<S320>/Constant'

      CLVF_DW.ThisIFblockdetermineswhether_aa = -1;
      if ((CLVF_P.WhoAmI == 3.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhether_aa = 0;

        // Outputs for IfAction SubSystem: '<S11>/Change BLUE Behavior' incorporates:
        //   ActionPort: '<S318>/Action Port'

        // If: '<S318>/Experiment Sub-Phases' incorporates:
        //   Constant: '<S318>/Constant1'
        //   Constant: '<S318>/Constant2'
        //   Constant: '<S318>/Constant3'
        //   Constant: '<S318>/Constant4'
        //   DataStoreRead: '<S378>/Universal_Time'

        CLVF_DW.ExperimentSubPhases_ActiveSub_h = -1;
        if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase1_End) {
          CLVF_DW.ExperimentSubPhases_ActiveSub_h = 0;

          // Outputs for IfAction SubSystem: '<S318>/Sub-Phase #1' incorporates:
          //   ActionPort: '<S374>/Action Port'

          CLVF_SubPhase1(&CLVF_B.SubPhase1_g, &CLVF_DW.SubPhase1_g,
                         &CLVF_P.SubPhase1_g, &CLVF_DW.BLUE_Fx, &CLVF_DW.BLUE_Fy,
                         &CLVF_DW.BLUE_Px, &CLVF_DW.BLUE_Py, &CLVF_DW.BLUE_Rz,
                         &CLVF_DW.BLUE_Tz, &CLVF_DW.Float_State);

          // End of Outputs for SubSystem: '<S318>/Sub-Phase #1'
        } else if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase2_End) {
          CLVF_DW.ExperimentSubPhases_ActiveSub_h = 1;

          // Outputs for IfAction SubSystem: '<S318>/Sub-Phase #2 ' incorporates:
          //   ActionPort: '<S375>/Action Port'

          CLVF_SubPhase2_l(&CLVF_P.SubPhase2_l, &CLVF_DW.BLUE_Fx,
                           &CLVF_DW.BLUE_Fy, &CLVF_DW.BLUE_Tz,
                           &CLVF_DW.Float_State);

          // End of Outputs for SubSystem: '<S318>/Sub-Phase #2 '
        } else if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase3_End_BLACK) {
          CLVF_DW.ExperimentSubPhases_ActiveSub_h = 2;

          // Outputs for IfAction SubSystem: '<S318>/Sub-Phase #3 ' incorporates:
          //   ActionPort: '<S376>/Action Port'

          CLVF_SubPhase2_l(&CLVF_P.SubPhase3_j, &CLVF_DW.BLUE_Fx,
                           &CLVF_DW.BLUE_Fy, &CLVF_DW.BLUE_Tz,
                           &CLVF_DW.Float_State);

          // End of Outputs for SubSystem: '<S318>/Sub-Phase #3 '
        } else {
          if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase4_End) {
            CLVF_DW.ExperimentSubPhases_ActiveSub_h = 3;

            // Outputs for IfAction SubSystem: '<S318>/Sub-Phase #4' incorporates:
            //   ActionPort: '<S377>/Action Port'

            CLVF_SubPhase1(&CLVF_B.SubPhase4_o, &CLVF_DW.SubPhase4_o,
                           &CLVF_P.SubPhase4_o, &CLVF_DW.BLUE_Fx,
                           &CLVF_DW.BLUE_Fy, &CLVF_DW.BLUE_Px, &CLVF_DW.BLUE_Py,
                           &CLVF_DW.BLUE_Rz, &CLVF_DW.BLUE_Tz,
                           &CLVF_DW.Float_State);

            // End of Outputs for SubSystem: '<S318>/Sub-Phase #4'
          }
        }

        // End of If: '<S318>/Experiment Sub-Phases'
        // End of Outputs for SubSystem: '<S11>/Change BLUE Behavior'
      }

      // End of If: '<S11>/This IF block determines whether or not to run the BLUE sim//exp' 

      // If: '<S11>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
      //   Constant: '<S11>/Constant'
      //   Constant: '<S320>/Constant'

      CLVF_DW.ThisIFblockdetermineswhethero_f = -1;
      if ((CLVF_P.WhoAmI == 1.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhethero_f = 0;

        // Outputs for IfAction SubSystem: '<S11>/Change RED Behavior' incorporates:
        //   ActionPort: '<S319>/Action Port'

        // If: '<S319>/Experiment Sub-Phases' incorporates:
        //   Constant: '<S319>/Constant1'
        //   Constant: '<S319>/Constant2'
        //   Constant: '<S319>/Constant3'
        //   Constant: '<S319>/Constant4'
        //   DataStoreRead: '<S409>/Universal_Time'

        CLVF_DW.ExperimentSubPhases_ActiveSubsy = -1;
        if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase1_End) {
          CLVF_DW.ExperimentSubPhases_ActiveSubsy = 0;

          // Outputs for IfAction SubSystem: '<S319>/Sub-Phase #1' incorporates:
          //   ActionPort: '<S405>/Action Port'

          // DataStoreWrite: '<S405>/RED_Tz_RW' incorporates:
          //   Constant: '<S405>/Constant1'

          CLVF_DW.RED_Tz_RW = CLVF_P.Constant1_Value_a;

          // MATLAB Function: '<S413>/MATLAB Function2' incorporates:
          //   Constant: '<S405>/Desired Attitude (RED)'

          CLVF_MATLABFunction2_o(CLVF_P.init_states_RED[2],
            &CLVF_B.sf_MATLABFunction2_ma);

          // MATLAB Function: '<S413>/MATLAB Function3' incorporates:
          //   DataStoreRead: '<S405>/RED_Rz'

          CLVF_MATLABFunction3(CLVF_DW.RED_Rz, &CLVF_B.sf_MATLABFunction3_ca);

          // Sum: '<S413>/Subtract2'
          CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_ma.Ox[0] -
            CLVF_B.sf_MATLABFunction3_ca.Ox[0];
          CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_ma.Ox[1] -
            CLVF_B.sf_MATLABFunction3_ca.Ox[1];

          // MATLAB Function: '<S413>/MATLAB Function4'
          CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_ma.Oy,
                               CLVF_B.rtb_Subtract2_l_g,
                               &CLVF_B.sf_MATLABFunction4_p);

          // Delay: '<S414>/Delay1'
          if (CLVF_DW.icLoad_c != 0) {
            CLVF_DW.Delay1_DSTATE_le = CLVF_B.sf_MATLABFunction4_p.e_out;
          }

          // Sum: '<S414>/Sum6' incorporates:
          //   Delay: '<S414>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.sf_MATLABFunction4_p.e_out -
            CLVF_DW.Delay1_DSTATE_le;

          // If: '<S414>/if we went through a "step"' incorporates:
          //   Inport: '<S415>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S414>/Hold this value' incorporates:
            //   ActionPort: '<S415>/Action Port'

            CLVF_B.In1_he = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S414>/Hold this value'
          }

          // End of If: '<S414>/if we went through a "step"'

          // Sum: '<S410>/Sum3' incorporates:
          //   DataStoreWrite: '<S405>/RED_Tz'
          //   Gain: '<S410>/kd_tr'
          //   Gain: '<S410>/kp_tr'
          //   Gain: '<S414>/divide by delta T'

          CLVF_DW.RED_Tz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_he *
            CLVF_P.Kd_tr + CLVF_P.Kp_tr * CLVF_B.sf_MATLABFunction4_p.e_out;

          // Sum: '<S405>/Subtract' incorporates:
          //   Constant: '<S405>/Desired X-Position (RED)'
          //   DataStoreRead: '<S405>/RED_Px'

          CLVF_B.Subtract_p = CLVF_P.init_states_RED[0] - CLVF_DW.RED_Px;

          // Delay: '<S416>/Delay1'
          if (CLVF_DW.icLoad_l != 0) {
            CLVF_DW.Delay1_DSTATE_nl = CLVF_B.Subtract_p;
          }

          // Sum: '<S416>/Sum6' incorporates:
          //   Delay: '<S416>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract_p - CLVF_DW.Delay1_DSTATE_nl;

          // If: '<S416>/if we went through a "step"' incorporates:
          //   Inport: '<S417>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S416>/Hold this value' incorporates:
            //   ActionPort: '<S417>/Action Port'

            CLVF_B.In1_n = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S416>/Hold this value'
          }

          // End of If: '<S416>/if we went through a "step"'

          // Sum: '<S411>/Sum3' incorporates:
          //   DataStoreWrite: '<S405>/RED_Fx'
          //   Gain: '<S411>/kd_xr'
          //   Gain: '<S411>/kp_xr'
          //   Gain: '<S416>/divide by delta T'

          CLVF_DW.RED_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_n * CLVF_P.Kd_xr
            + CLVF_P.Kp_xr * CLVF_B.Subtract_p;

          // Sum: '<S405>/Subtract1' incorporates:
          //   Constant: '<S405>/Desired Y-Position (RED)'
          //   DataStoreRead: '<S405>/RED_Py '

          CLVF_B.Subtract1_k = CLVF_P.init_states_RED[1] - CLVF_DW.RED_Py;

          // Delay: '<S418>/Delay1'
          if (CLVF_DW.icLoad_jz != 0) {
            CLVF_DW.Delay1_DSTATE_h2 = CLVF_B.Subtract1_k;
          }

          // Sum: '<S418>/Sum6' incorporates:
          //   Delay: '<S418>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract1_k - CLVF_DW.Delay1_DSTATE_h2;

          // If: '<S418>/if we went through a "step"' incorporates:
          //   Inport: '<S419>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S418>/Hold this value' incorporates:
            //   ActionPort: '<S419>/Action Port'

            CLVF_B.In1_e0 = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S418>/Hold this value'
          }

          // End of If: '<S418>/if we went through a "step"'

          // Sum: '<S412>/Sum3' incorporates:
          //   DataStoreWrite: '<S405>/RED_Fy'
          //   Gain: '<S412>/kd_yr'
          //   Gain: '<S412>/kp_yr'
          //   Gain: '<S418>/divide by delta T'

          CLVF_DW.RED_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_e0 *
            CLVF_P.Kd_yr + CLVF_P.Kp_yr * CLVF_B.Subtract1_k;

          // DataStoreWrite: '<S405>/Data Store Write3' incorporates:
          //   Constant: '<S405>/Puck State'

          CLVF_DW.Float_State = CLVF_P.PuckState_Value_lb;

          // End of Outputs for SubSystem: '<S319>/Sub-Phase #1'
        } else if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase2_End) {
          CLVF_DW.ExperimentSubPhases_ActiveSubsy = 1;

          // Outputs for IfAction SubSystem: '<S319>/Sub-Phase #2 ' incorporates:
          //   ActionPort: '<S406>/Action Port'

          CLVF_SubPhase2_d(&CLVF_P.SubPhase2_d, &CLVF_DW.Float_State,
                           &CLVF_DW.RED_Fx, &CLVF_DW.RED_Fy, &CLVF_DW.RED_Tz,
                           &CLVF_B.RED_Tz_Elbow, &CLVF_DW.RED_Tz_RW,
                           &CLVF_B.RED_Tz_Shoulder, &CLVF_B.RED_Tz_Wrist);

          // End of Outputs for SubSystem: '<S319>/Sub-Phase #2 '
        } else if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase3_End_RED) {
          CLVF_DW.ExperimentSubPhases_ActiveSubsy = 2;

          // Outputs for IfAction SubSystem: '<S319>/Sub-Phase #3 ' incorporates:
          //   ActionPort: '<S407>/Action Port'

          CLVF_SubPhase2_d(&CLVF_P.SubPhase3_jo, &CLVF_DW.Float_State,
                           &CLVF_DW.RED_Fx, &CLVF_DW.RED_Fy, &CLVF_DW.RED_Tz,
                           &CLVF_B.RED_Tz_Elbow, &CLVF_DW.RED_Tz_RW,
                           &CLVF_B.RED_Tz_Shoulder, &CLVF_B.RED_Tz_Wrist);

          // End of Outputs for SubSystem: '<S319>/Sub-Phase #3 '
        } else {
          if (CLVF_DW.Univ_Time < CLVF_P.Phase3_SubPhase4_End) {
            CLVF_DW.ExperimentSubPhases_ActiveSubsy = 3;

            // Outputs for IfAction SubSystem: '<S319>/Sub-Phase #4' incorporates:
            //   ActionPort: '<S408>/Action Port'

            // MATLAB Function: '<S426>/MATLAB Function2' incorporates:
            //   Constant: '<S408>/Constant'
            //   Constant: '<S427>/Constant1'
            //   DataStoreRead: '<S427>/Universal_Time'
            //   Gain: '<S408>/Gain'
            //   Sum: '<S408>/Sum'
            //   Sum: '<S427>/Subtract'

            CLVF_MATLABFunction2_c(CLVF_P.w_body * (CLVF_DW.Univ_Time -
              CLVF_P.Phase3_SubPhase3_End_RED) + CLVF_P.rT_I0[2],
              &CLVF_B.sf_MATLABFunction2_e);

            // MATLAB Function: '<S426>/MATLAB Function3' incorporates:
            //   DataStoreRead: '<S408>/RED_Rz'

            CLVF_MATLABFunction3(CLVF_DW.RED_Rz, &CLVF_B.sf_MATLABFunction3_k5);

            // Sum: '<S426>/Subtract2'
            CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_e.Ox[0] -
              CLVF_B.sf_MATLABFunction3_k5.Ox[0];
            CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_e.Ox[1] -
              CLVF_B.sf_MATLABFunction3_k5.Ox[1];

            // MATLAB Function: '<S426>/MATLAB Function4'
            CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_e.Oy,
                                 CLVF_B.rtb_Subtract2_l_g,
                                 &CLVF_B.sf_MATLABFunction4_e);

            // Delay: '<S428>/Delay1'
            if (CLVF_DW.icLoad_jk != 0) {
              CLVF_DW.Delay1_DSTATE_a = CLVF_B.sf_MATLABFunction4_e.e_out;
            }

            // Sum: '<S428>/Sum6' incorporates:
            //   Delay: '<S428>/Delay1'

            CLVF_B.RED_Tz_Shoulder = CLVF_B.sf_MATLABFunction4_e.e_out -
              CLVF_DW.Delay1_DSTATE_a;

            // If: '<S428>/if we went through a "step"' incorporates:
            //   Inport: '<S429>/In1'

            if (CLVF_B.RED_Tz_Shoulder != 0.0) {
              // Outputs for IfAction SubSystem: '<S428>/Hold this value' incorporates:
              //   ActionPort: '<S429>/Action Port'

              CLVF_B.In1_c5 = CLVF_B.RED_Tz_Shoulder;

              // End of Outputs for SubSystem: '<S428>/Hold this value'
            }

            // End of If: '<S428>/if we went through a "step"'

            // DiscreteTransferFcn: '<S423>/1Hz LP Filter1' incorporates:
            //   Gain: '<S428>/divide by delta T'

            CLVF_DW.uHzLPFilter1_tmp = ((1.0 / CLVF_P.serverRate * CLVF_B.In1_c5
              - CLVF_P.den_d[1] * CLVF_DW.uHzLPFilter1_states[0]) -
              CLVF_P.den_d[2] * CLVF_DW.uHzLPFilter1_states[1]) / CLVF_P.den_d[0];

            // Sum: '<S423>/Sum3' incorporates:
            //   DataStoreWrite: '<S408>/RED_Tz'
            //   DiscreteTransferFcn: '<S423>/1Hz LP Filter1'
            //   Gain: '<S423>/kd_tr'
            //   Gain: '<S423>/kp_tr'

            CLVF_DW.RED_Tz = ((CLVF_P.num_d[0] * CLVF_DW.uHzLPFilter1_tmp +
                               CLVF_P.num_d[1] * CLVF_DW.uHzLPFilter1_states[0])
                              + CLVF_P.num_d[2] * CLVF_DW.uHzLPFilter1_states[1])
              * CLVF_P.Kd_tr + CLVF_P.Kp_tr * CLVF_B.sf_MATLABFunction4_e.e_out;

            // DataStoreWrite: '<S408>/RED_Tz_RW' incorporates:
            //   Constant: '<S408>/Constant1'

            CLVF_DW.RED_Tz_RW = CLVF_P.Constant1_Value_p;

            // Sum: '<S408>/Subtract' incorporates:
            //   Constant: '<S408>/Desired X-Position (RED)'
            //   DataStoreRead: '<S408>/RED_Px'

            CLVF_B.Subtract_k = CLVF_P.init_states_RED[0] - CLVF_DW.RED_Px;

            // Delay: '<S430>/Delay1'
            if (CLVF_DW.icLoad_fc != 0) {
              CLVF_DW.Delay1_DSTATE_nv = CLVF_B.Subtract_k;
            }

            // Sum: '<S430>/Sum6' incorporates:
            //   Delay: '<S430>/Delay1'

            CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract_k - CLVF_DW.Delay1_DSTATE_nv;

            // If: '<S430>/if we went through a "step"' incorporates:
            //   Inport: '<S431>/In1'

            if (CLVF_B.RED_Tz_Elbow != 0.0) {
              // Outputs for IfAction SubSystem: '<S430>/Hold this value' incorporates:
              //   ActionPort: '<S431>/Action Port'

              CLVF_B.In1_cy = CLVF_B.RED_Tz_Elbow;

              // End of Outputs for SubSystem: '<S430>/Hold this value'
            }

            // End of If: '<S430>/if we went through a "step"'

            // Sum: '<S424>/Sum3' incorporates:
            //   DataStoreWrite: '<S408>/RED_Fx'
            //   Gain: '<S424>/kd_xr'
            //   Gain: '<S424>/kp_xr'
            //   Gain: '<S430>/divide by delta T'

            CLVF_DW.RED_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_cy *
              CLVF_P.Kd_xr + CLVF_P.Kp_xr * CLVF_B.Subtract_k;

            // Sum: '<S408>/Subtract1' incorporates:
            //   Constant: '<S408>/Desired Y-Position (RED)'
            //   DataStoreRead: '<S408>/RED_Py '

            CLVF_B.Subtract1_a = CLVF_P.init_states_RED[1] - CLVF_DW.RED_Py;

            // Delay: '<S432>/Delay1'
            if (CLVF_DW.icLoad_n != 0) {
              CLVF_DW.Delay1_DSTATE_m2 = CLVF_B.Subtract1_a;
            }

            // Sum: '<S432>/Sum6' incorporates:
            //   Delay: '<S432>/Delay1'

            CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract1_a - CLVF_DW.Delay1_DSTATE_m2;

            // If: '<S432>/if we went through a "step"' incorporates:
            //   Inport: '<S433>/In1'

            if (CLVF_B.RED_Tz_Elbow != 0.0) {
              // Outputs for IfAction SubSystem: '<S432>/Hold this value' incorporates:
              //   ActionPort: '<S433>/Action Port'

              CLVF_B.In1_e = CLVF_B.RED_Tz_Elbow;

              // End of Outputs for SubSystem: '<S432>/Hold this value'
            }

            // End of If: '<S432>/if we went through a "step"'

            // Sum: '<S425>/Sum3' incorporates:
            //   DataStoreWrite: '<S408>/RED_Fy'
            //   Gain: '<S425>/kd_yr'
            //   Gain: '<S425>/kp_yr'
            //   Gain: '<S432>/divide by delta T'

            CLVF_DW.RED_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_e *
              CLVF_P.Kd_yr + CLVF_P.Kp_yr * CLVF_B.Subtract1_a;

            // DataStoreWrite: '<S408>/Data Store Write3' incorporates:
            //   Constant: '<S408>/Puck State'

            CLVF_DW.Float_State = CLVF_P.PuckState_Value_h;

            // End of Outputs for SubSystem: '<S319>/Sub-Phase #4'
          }
        }

        // End of If: '<S319>/Experiment Sub-Phases'
        // End of Outputs for SubSystem: '<S11>/Change RED Behavior'
      }

      // End of If: '<S11>/This IF block determines whether or not to run the RED sim//exp ' 
      // End of Outputs for SubSystem: '<Root>/Phase #3: Experiment'
      break;

     case 4:
      // Outputs for IfAction SubSystem: '<Root>/Phase #4:  Return Home' incorporates:
      //   ActionPort: '<S12>/Action Port'

      // If: '<S12>/This IF block determines whether or not to run the BLACK sim//exp' incorporates:
      //   Constant: '<S12>/Constant'
      //   Constant: '<S440>/Constant'

      CLVF_DW.ThisIFblockdetermineswhethero_d = -1;
      if ((CLVF_P.WhoAmI == 2.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhethero_d = 0;

        // Outputs for IfAction SubSystem: '<S12>/Change BLACK Behavior' incorporates:
        //   ActionPort: '<S437>/Action Port'

        // Sum: '<S437>/Subtract' incorporates:
        //   Constant: '<S437>/Desired Px (BLACK)'
        //   DataStoreRead: '<S437>/BLACK_Px'

        CLVF_B.Subtract_d = CLVF_P.home_states_BLACK[0] - CLVF_DW.BLACK_Px;

        // Delay: '<S447>/Delay1'
        if (CLVF_DW.icLoad_dq != 0) {
          CLVF_DW.Delay1_DSTATE_lt = CLVF_B.Subtract_d;
        }

        // Sum: '<S447>/Sum6' incorporates:
        //   Delay: '<S447>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract_d - CLVF_DW.Delay1_DSTATE_lt;

        // If: '<S447>/if we went through a "step"' incorporates:
        //   Inport: '<S448>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S447>/Hold this value' incorporates:
          //   ActionPort: '<S448>/Action Port'

          CLVF_B.In1_fx = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S447>/Hold this value'
        }

        // End of If: '<S447>/if we went through a "step"'

        // Sum: '<S442>/Sum3' incorporates:
        //   DataStoreWrite: '<S437>/BLACK_Fx'
        //   Gain: '<S442>/kd_xb'
        //   Gain: '<S442>/kp_xb'
        //   Gain: '<S447>/divide by delta T'

        CLVF_DW.BLACK_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_fx *
          CLVF_P.Kd_xb + CLVF_P.Kp_xb * CLVF_B.Subtract_d;

        // Sum: '<S437>/Subtract1' incorporates:
        //   Constant: '<S437>/Desired Py (BLACK)'
        //   DataStoreRead: '<S437>/BLACK_Py '

        CLVF_B.Subtract1_d = CLVF_P.home_states_BLACK[1] - CLVF_DW.BLACK_Py;

        // Delay: '<S449>/Delay1'
        if (CLVF_DW.icLoad_h != 0) {
          CLVF_DW.Delay1_DSTATE_im = CLVF_B.Subtract1_d;
        }

        // Sum: '<S449>/Sum6' incorporates:
        //   Delay: '<S449>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract1_d - CLVF_DW.Delay1_DSTATE_im;

        // If: '<S449>/if we went through a "step"' incorporates:
        //   Inport: '<S450>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S449>/Hold this value' incorporates:
          //   ActionPort: '<S450>/Action Port'

          CLVF_B.In1_d0 = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S449>/Hold this value'
        }

        // End of If: '<S449>/if we went through a "step"'

        // Sum: '<S443>/Sum3' incorporates:
        //   DataStoreWrite: '<S437>/BLACK_Fy'
        //   Gain: '<S443>/kd_yb'
        //   Gain: '<S443>/kp_yb'
        //   Gain: '<S449>/divide by delta T'

        CLVF_DW.BLACK_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_d0 *
          CLVF_P.Kd_yb + CLVF_P.Kp_yb * CLVF_B.Subtract1_d;

        // MATLAB Function: '<S444>/MATLAB Function2' incorporates:
        //   Constant: '<S437>/Desired Attitude (BLACK)'

        CLVF_MATLABFunction2_o(CLVF_P.home_states_BLACK[2],
          &CLVF_B.sf_MATLABFunction2_p);

        // MATLAB Function: '<S444>/MATLAB Function3' incorporates:
        //   DataStoreRead: '<S437>/BLACK_Rz'

        CLVF_MATLABFunction3(CLVF_DW.BLACK_Rz, &CLVF_B.sf_MATLABFunction3_b);

        // Sum: '<S444>/Subtract2'
        CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_p.Ox[0] -
          CLVF_B.sf_MATLABFunction3_b.Ox[0];
        CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_p.Ox[1] -
          CLVF_B.sf_MATLABFunction3_b.Ox[1];

        // MATLAB Function: '<S444>/MATLAB Function4'
        CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_p.Oy,
                             CLVF_B.rtb_Subtract2_l_g,
                             &CLVF_B.sf_MATLABFunction4_l);

        // Delay: '<S445>/Delay1'
        if (CLVF_DW.icLoad_o != 0) {
          CLVF_DW.Delay1_DSTATE_f = CLVF_B.sf_MATLABFunction4_l.e_out;
        }

        // Sum: '<S445>/Sum6' incorporates:
        //   Delay: '<S445>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.sf_MATLABFunction4_l.e_out -
          CLVF_DW.Delay1_DSTATE_f;

        // If: '<S445>/if we went through a "step"' incorporates:
        //   Inport: '<S446>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S445>/Hold this value' incorporates:
          //   ActionPort: '<S446>/Action Port'

          CLVF_B.In1_fk = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S445>/Hold this value'
        }

        // End of If: '<S445>/if we went through a "step"'

        // Sum: '<S441>/Sum3' incorporates:
        //   DataStoreWrite: '<S437>/BLACK_Tz'
        //   Gain: '<S441>/kd_tb'
        //   Gain: '<S441>/kp_tb'
        //   Gain: '<S445>/divide by delta T'

        CLVF_DW.BLACK_Tz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_fk *
          CLVF_P.Kd_tb + CLVF_P.Kp_tb * CLVF_B.sf_MATLABFunction4_l.e_out;

        // DataStoreWrite: '<S437>/Data Store Write3' incorporates:
        //   Constant: '<S437>/Puck State'

        CLVF_DW.Float_State = CLVF_P.PuckState_Value_f;

        // End of Outputs for SubSystem: '<S12>/Change BLACK Behavior'
      }

      // End of If: '<S12>/This IF block determines whether or not to run the BLACK sim//exp' 

      // If: '<S12>/This IF block determines whether or not to run the BLUE sim//exp' incorporates:
      //   Constant: '<S12>/Constant'
      //   Constant: '<S440>/Constant'

      CLVF_DW.ThisIFblockdetermineswhether_jc = -1;
      if ((CLVF_P.WhoAmI == 3.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhether_jc = 0;

        // Outputs for IfAction SubSystem: '<S12>/Change BLUE Behavior' incorporates:
        //   ActionPort: '<S438>/Action Port'

        CLVF_ChangeBLUEBehavior_g(&CLVF_B.ChangeBLUEBehavior_gj,
          &CLVF_DW.ChangeBLUEBehavior_gj, &CLVF_P.ChangeBLUEBehavior_gj,
          &CLVF_DW.BLUE_Fx, &CLVF_DW.BLUE_Fy, &CLVF_DW.BLUE_Px, &CLVF_DW.BLUE_Py,
          &CLVF_DW.BLUE_Rz, &CLVF_DW.BLUE_Tz, &CLVF_DW.Float_State);

        // End of Outputs for SubSystem: '<S12>/Change BLUE Behavior'
      }

      // End of If: '<S12>/This IF block determines whether or not to run the BLUE sim//exp' 

      // If: '<S12>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
      //   Constant: '<S12>/Constant'
      //   Constant: '<S440>/Constant'

      CLVF_DW.ThisIFblockdetermineswhether_a4 = -1;
      if ((CLVF_P.WhoAmI == 1.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhether_a4 = 0;

        // Outputs for IfAction SubSystem: '<S12>/Change RED Behavior' incorporates:
        //   ActionPort: '<S439>/Action Port'

        // MATLAB Function: '<S470>/MATLAB Function2' incorporates:
        //   Constant: '<S439>/Constant'

        CLVF_MATLABFunction2_o(CLVF_P.home_states_RED[2],
          &CLVF_B.sf_MATLABFunction2_i);

        // MATLAB Function: '<S470>/MATLAB Function3' incorporates:
        //   DataStoreRead: '<S439>/RED_Rz'

        CLVF_MATLABFunction3(CLVF_DW.RED_Rz, &CLVF_B.sf_MATLABFunction3_c1);

        // Sum: '<S470>/Subtract2'
        CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_i.Ox[0] -
          CLVF_B.sf_MATLABFunction3_c1.Ox[0];
        CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_i.Ox[1] -
          CLVF_B.sf_MATLABFunction3_c1.Ox[1];

        // MATLAB Function: '<S470>/MATLAB Function4'
        CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_i.Oy,
                             CLVF_B.rtb_Subtract2_l_g,
                             &CLVF_B.sf_MATLABFunction4_n);

        // Delay: '<S471>/Delay1'
        if (CLVF_DW.icLoad_dh != 0) {
          CLVF_DW.Delay1_DSTATE_i = CLVF_B.sf_MATLABFunction4_n.e_out;
        }

        // Sum: '<S471>/Sum6' incorporates:
        //   Delay: '<S471>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.sf_MATLABFunction4_n.e_out -
          CLVF_DW.Delay1_DSTATE_i;

        // If: '<S471>/if we went through a "step"' incorporates:
        //   Inport: '<S472>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S471>/Hold this value' incorporates:
          //   ActionPort: '<S472>/Action Port'

          CLVF_B.In1_h = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S471>/Hold this value'
        }

        // End of If: '<S471>/if we went through a "step"'

        // Sum: '<S467>/Sum3' incorporates:
        //   DataStoreWrite: '<S439>/RED_Tz'
        //   Gain: '<S467>/kd_tr'
        //   Gain: '<S467>/kp_tr'
        //   Gain: '<S471>/divide by delta T'

        CLVF_DW.RED_Tz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_h * CLVF_P.Kd_tr +
          CLVF_P.Kp_tr * CLVF_B.sf_MATLABFunction4_n.e_out;

        // Sum: '<S439>/Subtract' incorporates:
        //   Constant: '<S439>/Constant1'
        //   DataStoreRead: '<S439>/RED_Px'

        CLVF_B.Subtract_l = CLVF_P.home_states_RED[0] - CLVF_DW.RED_Px;

        // Delay: '<S473>/Delay1'
        if (CLVF_DW.icLoad_a2 != 0) {
          CLVF_DW.Delay1_DSTATE_h = CLVF_B.Subtract_l;
        }

        // Sum: '<S473>/Sum6' incorporates:
        //   Delay: '<S473>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract_l - CLVF_DW.Delay1_DSTATE_h;

        // If: '<S473>/if we went through a "step"' incorporates:
        //   Inport: '<S474>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S473>/Hold this value' incorporates:
          //   ActionPort: '<S474>/Action Port'

          CLVF_B.In1_b = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S473>/Hold this value'
        }

        // End of If: '<S473>/if we went through a "step"'

        // Sum: '<S468>/Sum3' incorporates:
        //   DataStoreWrite: '<S439>/RED_Fx'
        //   Gain: '<S468>/kd_xr'
        //   Gain: '<S468>/kp_xr'
        //   Gain: '<S473>/divide by delta T'

        CLVF_DW.RED_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_b * CLVF_P.Kd_xr +
          CLVF_P.Kp_xr * CLVF_B.Subtract_l;

        // DataStoreWrite: '<S439>/RED_Tz_RW1' incorporates:
        //   Constant: '<S439>/Constant2'

        CLVF_DW.RED_Tz_RW = CLVF_P.Constant2_Value_g;

        // Sum: '<S439>/Subtract1' incorporates:
        //   Constant: '<S439>/Constant3'
        //   DataStoreRead: '<S439>/RED_Py '

        CLVF_B.Subtract1_g = CLVF_P.home_states_RED[1] - CLVF_DW.RED_Py;

        // Delay: '<S475>/Delay1'
        if (CLVF_DW.icLoad_f != 0) {
          CLVF_DW.Delay1_DSTATE_l = CLVF_B.Subtract1_g;
        }

        // Sum: '<S475>/Sum6' incorporates:
        //   Delay: '<S475>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract1_g - CLVF_DW.Delay1_DSTATE_l;

        // If: '<S475>/if we went through a "step"' incorporates:
        //   Inport: '<S476>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S475>/Hold this value' incorporates:
          //   ActionPort: '<S476>/Action Port'

          CLVF_B.In1_f = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S475>/Hold this value'
        }

        // End of If: '<S475>/if we went through a "step"'

        // Sum: '<S469>/Sum3' incorporates:
        //   DataStoreWrite: '<S439>/RED_Fy'
        //   Gain: '<S469>/kd_yr'
        //   Gain: '<S469>/kp_yr'
        //   Gain: '<S475>/divide by delta T'

        CLVF_DW.RED_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_f * CLVF_P.Kd_yr +
          CLVF_P.Kp_yr * CLVF_B.Subtract1_g;

        // DataStoreWrite: '<S439>/Data Store Write3' incorporates:
        //   Constant: '<S439>/Puck State'

        CLVF_DW.Float_State = CLVF_P.PuckState_Value_od;

        // End of Outputs for SubSystem: '<S12>/Change RED Behavior'
      }

      // End of If: '<S12>/This IF block determines whether or not to run the RED sim//exp ' 
      // End of Outputs for SubSystem: '<Root>/Phase #4:  Return Home'
      break;

     case 5:
      // Outputs for IfAction SubSystem: '<Root>/Phase #5:  Hold Home' incorporates:
      //   ActionPort: '<S13>/Action Port'

      // If: '<S13>/This IF block determines whether or not to run the BLACK sim//exp' incorporates:
      //   Constant: '<S13>/Constant'
      //   Constant: '<S483>/Constant'

      CLVF_DW.ThisIFblockdetermineswhethero_b = -1;
      if ((CLVF_P.WhoAmI == 2.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhethero_b = 0;

        // Outputs for IfAction SubSystem: '<S13>/Change BLACK Behavior' incorporates:
        //   ActionPort: '<S480>/Action Port'

        // Sum: '<S480>/Subtract' incorporates:
        //   Constant: '<S480>/Constant'
        //   DataStoreRead: '<S480>/BLACK_Px'

        CLVF_B.Subtract_o = CLVF_P.home_states_BLACK[0] - CLVF_DW.BLACK_Px;

        // Delay: '<S490>/Delay1'
        if (CLVF_DW.icLoad_e != 0) {
          CLVF_DW.Delay1_DSTATE_o = CLVF_B.Subtract_o;
        }

        // Sum: '<S490>/Sum6' incorporates:
        //   Delay: '<S490>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract_o - CLVF_DW.Delay1_DSTATE_o;

        // If: '<S490>/if we went through a "step"' incorporates:
        //   Inport: '<S491>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S490>/Hold this value' incorporates:
          //   ActionPort: '<S491>/Action Port'

          CLVF_B.In1_i = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S490>/Hold this value'
        }

        // End of If: '<S490>/if we went through a "step"'

        // Sum: '<S485>/Sum3' incorporates:
        //   DataStoreWrite: '<S480>/BLACK_Fx'
        //   Gain: '<S485>/kd_xb'
        //   Gain: '<S485>/kp_xb'
        //   Gain: '<S490>/divide by delta T'

        CLVF_DW.BLACK_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_i * CLVF_P.Kd_xb
          + CLVF_P.Kp_xb * CLVF_B.Subtract_o;

        // Sum: '<S480>/Subtract1' incorporates:
        //   Constant: '<S480>/Constant2'
        //   DataStoreRead: '<S480>/BLACK_Py '

        CLVF_B.Subtract1_c = CLVF_P.home_states_BLACK[1] - CLVF_DW.BLACK_Py;

        // Delay: '<S492>/Delay1'
        if (CLVF_DW.icLoad_m != 0) {
          CLVF_DW.Delay1_DSTATE_p = CLVF_B.Subtract1_c;
        }

        // Sum: '<S492>/Sum6' incorporates:
        //   Delay: '<S492>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract1_c - CLVF_DW.Delay1_DSTATE_p;

        // If: '<S492>/if we went through a "step"' incorporates:
        //   Inport: '<S493>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S492>/Hold this value' incorporates:
          //   ActionPort: '<S493>/Action Port'

          CLVF_B.In1_c = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S492>/Hold this value'
        }

        // End of If: '<S492>/if we went through a "step"'

        // Sum: '<S486>/Sum3' incorporates:
        //   DataStoreWrite: '<S480>/BLACK_Fy'
        //   Gain: '<S486>/kd_yb'
        //   Gain: '<S486>/kp_yb'
        //   Gain: '<S492>/divide by delta T'

        CLVF_DW.BLACK_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_c * CLVF_P.Kd_yb
          + CLVF_P.Kp_yb * CLVF_B.Subtract1_c;

        // MATLAB Function: '<S487>/MATLAB Function2' incorporates:
        //   Constant: '<S480>/Constant3'

        CLVF_MATLABFunction2_o(CLVF_P.home_states_BLACK[2],
          &CLVF_B.sf_MATLABFunction2_g);

        // MATLAB Function: '<S487>/MATLAB Function3' incorporates:
        //   DataStoreRead: '<S480>/BLACK_Rz'

        CLVF_MATLABFunction3(CLVF_DW.BLACK_Rz, &CLVF_B.sf_MATLABFunction3_o);

        // Sum: '<S487>/Subtract2'
        CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_g.Ox[0] -
          CLVF_B.sf_MATLABFunction3_o.Ox[0];
        CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_g.Ox[1] -
          CLVF_B.sf_MATLABFunction3_o.Ox[1];

        // MATLAB Function: '<S487>/MATLAB Function4'
        CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_g.Oy,
                             CLVF_B.rtb_Subtract2_l_g,
                             &CLVF_B.sf_MATLABFunction4_d);

        // Delay: '<S488>/Delay1'
        if (CLVF_DW.icLoad_d != 0) {
          CLVF_DW.Delay1_DSTATE_b = CLVF_B.sf_MATLABFunction4_d.e_out;
        }

        // Sum: '<S488>/Sum6' incorporates:
        //   Delay: '<S488>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.sf_MATLABFunction4_d.e_out -
          CLVF_DW.Delay1_DSTATE_b;

        // If: '<S488>/if we went through a "step"' incorporates:
        //   Inport: '<S489>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S488>/Hold this value' incorporates:
          //   ActionPort: '<S489>/Action Port'

          CLVF_B.In1_g = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S488>/Hold this value'
        }

        // End of If: '<S488>/if we went through a "step"'

        // Sum: '<S484>/Sum3' incorporates:
        //   DataStoreWrite: '<S480>/BLACK_Tz'
        //   Gain: '<S484>/kd_tb'
        //   Gain: '<S484>/kp_tb'
        //   Gain: '<S488>/divide by delta T'

        CLVF_DW.BLACK_Tz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_g * CLVF_P.Kd_tb
          + CLVF_P.Kp_tb * CLVF_B.sf_MATLABFunction4_d.e_out;

        // DataStoreWrite: '<S480>/Data Store Write3' incorporates:
        //   Constant: '<S480>/Puck State'

        CLVF_DW.Float_State = CLVF_P.PuckState_Value_ge;

        // End of Outputs for SubSystem: '<S13>/Change BLACK Behavior'
      }

      // End of If: '<S13>/This IF block determines whether or not to run the BLACK sim//exp' 

      // If: '<S13>/This IF block determines whether or not to run the BLUE sim//exp' incorporates:
      //   Constant: '<S13>/Constant'
      //   Constant: '<S483>/Constant'

      CLVF_DW.ThisIFblockdetermineswhethero_n = -1;
      if ((CLVF_P.WhoAmI == 3.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhethero_n = 0;

        // Outputs for IfAction SubSystem: '<S13>/Change BLUE Behavior' incorporates:
        //   ActionPort: '<S481>/Action Port'

        CLVF_ChangeBLUEBehavior_g(&CLVF_B.ChangeBLUEBehavior_b,
          &CLVF_DW.ChangeBLUEBehavior_b, &CLVF_P.ChangeBLUEBehavior_b,
          &CLVF_DW.BLUE_Fx, &CLVF_DW.BLUE_Fy, &CLVF_DW.BLUE_Px, &CLVF_DW.BLUE_Py,
          &CLVF_DW.BLUE_Rz, &CLVF_DW.BLUE_Tz, &CLVF_DW.Float_State);

        // End of Outputs for SubSystem: '<S13>/Change BLUE Behavior'
      }

      // End of If: '<S13>/This IF block determines whether or not to run the BLUE sim//exp' 

      // If: '<S13>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
      //   Constant: '<S13>/Constant'
      //   Constant: '<S483>/Constant'

      CLVF_DW.ThisIFblockdetermineswhethero_j = -1;
      if ((CLVF_P.WhoAmI == 1.0) || (CLVF_P.simMode == 1.0)) {
        CLVF_DW.ThisIFblockdetermineswhethero_j = 0;

        // Outputs for IfAction SubSystem: '<S13>/Change RED Behavior' incorporates:
        //   ActionPort: '<S482>/Action Port'

        // Sum: '<S482>/Subtract' incorporates:
        //   Constant: '<S482>/Constant'
        //   DataStoreRead: '<S482>/RED_Px'

        CLVF_B.Subtract = CLVF_P.home_states_RED[0] - CLVF_DW.RED_Px;

        // Delay: '<S516>/Delay1'
        if (CLVF_DW.icLoad != 0) {
          CLVF_DW.Delay1_DSTATE = CLVF_B.Subtract;
        }

        // Sum: '<S516>/Sum6' incorporates:
        //   Delay: '<S516>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract - CLVF_DW.Delay1_DSTATE;

        // If: '<S516>/if we went through a "step"' incorporates:
        //   Inport: '<S517>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S516>/Hold this value' incorporates:
          //   ActionPort: '<S517>/Action Port'

          CLVF_B.In1_j = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S516>/Hold this value'
        }

        // End of If: '<S516>/if we went through a "step"'

        // Sum: '<S511>/Sum3' incorporates:
        //   DataStoreWrite: '<S482>/RED_Fx'
        //   Gain: '<S511>/kd_xr'
        //   Gain: '<S511>/kp_xr'
        //   Gain: '<S516>/divide by delta T'

        CLVF_DW.RED_Fx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_j * CLVF_P.Kd_xr +
          CLVF_P.Kp_xr * CLVF_B.Subtract;

        // DataStoreWrite: '<S482>/RED_Tz_RW' incorporates:
        //   Constant: '<S482>/Constant1'

        CLVF_DW.RED_Tz_RW = CLVF_P.Constant1_Value_k;

        // Sum: '<S482>/Subtract1' incorporates:
        //   Constant: '<S482>/Constant2'
        //   DataStoreRead: '<S482>/RED_Py '

        CLVF_B.Subtract1 = CLVF_P.home_states_RED[1] - CLVF_DW.RED_Py;

        // Delay: '<S518>/Delay1'
        if (CLVF_DW.icLoad_j != 0) {
          CLVF_DW.Delay1_DSTATE_m = CLVF_B.Subtract1;
        }

        // Sum: '<S518>/Sum6' incorporates:
        //   Delay: '<S518>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Subtract1 - CLVF_DW.Delay1_DSTATE_m;

        // If: '<S518>/if we went through a "step"' incorporates:
        //   Inport: '<S519>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S518>/Hold this value' incorporates:
          //   ActionPort: '<S519>/Action Port'

          CLVF_B.In1 = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S518>/Hold this value'
        }

        // End of If: '<S518>/if we went through a "step"'

        // Sum: '<S512>/Sum3' incorporates:
        //   DataStoreWrite: '<S482>/RED_Fy'
        //   Gain: '<S512>/kd_yr'
        //   Gain: '<S512>/kp_yr'
        //   Gain: '<S518>/divide by delta T'

        CLVF_DW.RED_Fy = 1.0 / CLVF_P.serverRate * CLVF_B.In1 * CLVF_P.Kd_yr +
          CLVF_P.Kp_yr * CLVF_B.Subtract1;

        // MATLAB Function: '<S513>/MATLAB Function2' incorporates:
        //   Constant: '<S482>/Constant3'

        CLVF_MATLABFunction2_o(CLVF_P.home_states_RED[2],
          &CLVF_B.sf_MATLABFunction2_mx);

        // MATLAB Function: '<S513>/MATLAB Function3' incorporates:
        //   DataStoreRead: '<S482>/RED_Rz'

        CLVF_MATLABFunction3(CLVF_DW.RED_Rz, &CLVF_B.sf_MATLABFunction3_bc);

        // Sum: '<S513>/Subtract2'
        CLVF_B.rtb_Subtract2_l_g[0] = CLVF_B.sf_MATLABFunction2_mx.Ox[0] -
          CLVF_B.sf_MATLABFunction3_bc.Ox[0];
        CLVF_B.rtb_Subtract2_l_g[1] = CLVF_B.sf_MATLABFunction2_mx.Ox[1] -
          CLVF_B.sf_MATLABFunction3_bc.Ox[1];

        // MATLAB Function: '<S513>/MATLAB Function4'
        CLVF_MATLABFunction4(CLVF_B.sf_MATLABFunction2_mx.Oy,
                             CLVF_B.rtb_Subtract2_l_g,
                             &CLVF_B.sf_MATLABFunction4_df);

        // Delay: '<S514>/Delay1'
        if (CLVF_DW.icLoad_a != 0) {
          CLVF_DW.Delay1_DSTATE_n = CLVF_B.sf_MATLABFunction4_df.e_out;
        }

        // Sum: '<S514>/Sum6' incorporates:
        //   Delay: '<S514>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.sf_MATLABFunction4_df.e_out -
          CLVF_DW.Delay1_DSTATE_n;

        // If: '<S514>/if we went through a "step"' incorporates:
        //   Inport: '<S515>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S514>/Hold this value' incorporates:
          //   ActionPort: '<S515>/Action Port'

          CLVF_B.In1_d = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S514>/Hold this value'
        }

        // End of If: '<S514>/if we went through a "step"'

        // Sum: '<S510>/Sum3' incorporates:
        //   DataStoreWrite: '<S482>/RED_Tz'
        //   Gain: '<S510>/kd_tr'
        //   Gain: '<S510>/kp_tr'
        //   Gain: '<S514>/divide by delta T'

        CLVF_DW.RED_Tz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_d * CLVF_P.Kd_tr +
          CLVF_P.Kp_tr * CLVF_B.sf_MATLABFunction4_df.e_out;

        // DataStoreWrite: '<S482>/Data Store Write3' incorporates:
        //   Constant: '<S482>/Puck State'

        CLVF_DW.Float_State = CLVF_P.PuckState_Value_e;

        // End of Outputs for SubSystem: '<S13>/Change RED Behavior'
      }

      // End of If: '<S13>/This IF block determines whether or not to run the RED sim//exp ' 
      // End of Outputs for SubSystem: '<Root>/Phase #5:  Hold Home'
      break;

     default:
      // Outputs for IfAction SubSystem: '<Root>/Phase #6:  Stop Floating and Spin Down RW' incorporates:
      //   ActionPort: '<S14>/Action Port'

      // If: '<S14>/This IF block determines whether or not to run the BLACK sim//exp' incorporates:
      //   Constant: '<S14>/Constant'
      //   Constant: '<S526>/Constant'

      if ((CLVF_P.WhoAmI == 2.0) || (CLVF_P.simMode == 1.0)) {
        // Outputs for IfAction SubSystem: '<S14>/Change BLACK Behavior' incorporates:
        //   ActionPort: '<S523>/Action Port'

        // DataStoreWrite: '<S523>/BLACK_Fx' incorporates:
        //   Constant: '<S523>/Constant3'

        CLVF_DW.BLACK_Fx = CLVF_P.Constant3_Value_d;

        // DataStoreWrite: '<S523>/BLACK_Fy' incorporates:
        //   Constant: '<S523>/Constant4'

        CLVF_DW.BLACK_Fy = CLVF_P.Constant4_Value_e;

        // DataStoreWrite: '<S523>/BLACK_Tz' incorporates:
        //   Constant: '<S523>/Constant5'

        CLVF_DW.BLACK_Tz = CLVF_P.Constant5_Value_n;

        // DataStoreWrite: '<S523>/Data Store Write3' incorporates:
        //   Constant: '<S523>/Puck State'

        CLVF_DW.Float_State = CLVF_P.PuckState_Value_gd;

        // End of Outputs for SubSystem: '<S14>/Change BLACK Behavior'
      }

      // End of If: '<S14>/This IF block determines whether or not to run the BLACK sim//exp' 

      // If: '<S14>/This IF block determines whether or not to run the BLUE sim//exp' incorporates:
      //   Constant: '<S14>/Constant'
      //   Constant: '<S526>/Constant'

      if ((CLVF_P.WhoAmI == 3.0) || (CLVF_P.simMode == 1.0)) {
        // Outputs for IfAction SubSystem: '<S14>/Change BLUE Behavior' incorporates:
        //   ActionPort: '<S524>/Action Port'

        CLVF_ChangeBLUEBehavior(&CLVF_P.ChangeBLUEBehavior_k, &CLVF_DW.BLUE_Fx,
          &CLVF_DW.BLUE_Fy, &CLVF_DW.BLUE_Tz, &CLVF_DW.Float_State);

        // End of Outputs for SubSystem: '<S14>/Change BLUE Behavior'
      }

      // End of If: '<S14>/This IF block determines whether or not to run the BLUE sim//exp' 

      // If: '<S14>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
      //   Constant: '<S14>/Constant'
      //   Constant: '<S526>/Constant'

      if ((CLVF_P.WhoAmI == 1.0) || (CLVF_P.simMode == 1.0)) {
        // Outputs for IfAction SubSystem: '<S14>/Change RED Behavior' incorporates:
        //   ActionPort: '<S525>/Action Port'

        // DataStoreWrite: '<S525>/RED_Fx' incorporates:
        //   Constant: '<S525>/Constant'

        CLVF_DW.RED_Fx = CLVF_P.Constant_Value_jq;

        // DataStoreWrite: '<S525>/RED_Fy' incorporates:
        //   Constant: '<S525>/Constant1'

        CLVF_DW.RED_Fy = CLVF_P.Constant1_Value_l;

        // DataStoreWrite: '<S525>/RED_Tz' incorporates:
        //   Constant: '<S525>/Constant2'

        CLVF_DW.RED_Tz = CLVF_P.Constant2_Value_m;

        // DataStoreWrite: '<S525>/Data Store Write3' incorporates:
        //   Constant: '<S525>/Puck State'

        CLVF_DW.Float_State = CLVF_P.PuckState_Value_hv;

        // Gain: '<S525>/Gain' incorporates:
        //   DataStoreRead: '<S525>/RED_dRz_Speed'

        CLVF_B.RED_Tz_Elbow = CLVF_P.Gain_Gain_a * CLVF_DW.RED_dRz_RW_Sat;

        // Saturate: '<S525>/Saturation'
        if (CLVF_B.RED_Tz_Elbow > CLVF_P.Saturation_UpperSat) {
          CLVF_DW.RED_Tz_RW = CLVF_P.Saturation_UpperSat;
        } else if (CLVF_B.RED_Tz_Elbow < CLVF_P.Saturation_LowerSat) {
          CLVF_DW.RED_Tz_RW = CLVF_P.Saturation_LowerSat;
        } else {
          CLVF_DW.RED_Tz_RW = CLVF_B.RED_Tz_Elbow;
        }

        // End of Saturate: '<S525>/Saturation'
        // End of Outputs for SubSystem: '<S14>/Change RED Behavior'
      }

      // End of If: '<S14>/This IF block determines whether or not to run the RED sim//exp ' 
      // End of Outputs for SubSystem: '<Root>/Phase #6:  Stop Floating and Spin Down RW' 
      break;
    }

    // End of If: '<Root>/Separate Phases'

    // If: '<Root>/If running a simulation,  grab the simulated states.' incorporates:
    //   Constant: '<S7>/Constant'
    //   DiscreteIntegrator: '<S527>/Acceleration  to Velocity'
    //   DiscreteIntegrator: '<S528>/Acceleration  to Velocity'
    //   DiscreteIntegrator: '<S532>/Acceleration  to Velocity'
    //   Inport: '<S555>/In'
    //   Inport: '<S557>/In'
    //   Inport: '<S559>/In'
    //   RandomNumber: '<S527>/Random Number'
    //   RandomNumber: '<S528>/Random Number'
    //   RandomNumber: '<S532>/Random Number'
    //   Sum: '<S527>/Sum'
    //   Sum: '<S528>/Sum'
    //   Sum: '<S532>/Sum'

    if (CLVF_P.simMode == 1.0) {
      // Outputs for IfAction SubSystem: '<Root>/Simulate Plant Dynamics' incorporates:
      //   ActionPort: '<S16>/Action Port'

      // DiscreteIntegrator: '<S527>/Acceleration  to Velocity' incorporates:
      //   DataStoreRead: '<S16>/BLACK_Fx_Sat'
      //   MATLAB Function: '<S527>/MATLAB Function'
      //   SignalConversion generated from: '<S551>/ SFunction '

      // MATLAB Function 'Simulate Plant Dynamics/BLACK Dynamics Model/MATLAB Function': '<S551>:1' 
      // '<S551>:1:5' x_ddot     = zeros(3,1);
      // '<S551>:1:9' m_BLACK    = model_param(3);
      // '<S551>:1:10' I_BLACK    = model_param(4);
      // '<S551>:1:13' Fx        = control_inputs(1);
      // '<S551>:1:14' Fy        = control_inputs(2);
      // '<S551>:1:15' Tz        = control_inputs(3);
      // '<S551>:1:18' x_ddot(1) = Fx/m_BLACK;
      // '<S551>:1:19' x_ddot(2) = Fy/m_BLACK;
      // '<S551>:1:20' x_ddot(3) = Tz/I_BLACK;
      CLVF_B.RED_Tz_Wrist = CLVF_DW.BLACK_Fx_Sat / CLVF_P.model_param[2] *
        CLVF_P.AccelerationtoVelocity_gainval +
        CLVF_DW.AccelerationtoVelocity_DSTATE[0];

      // DiscreteIntegrator: '<S527>/Velocity to Position'
      CLVF_B.TmpSignalConversionAtSFun_n[0] = CLVF_P.VelocitytoPosition_gainval *
        CLVF_B.RED_Tz_Wrist + CLVF_DW.VelocitytoPosition_DSTATE[0];
      CLVF_B.x_ddot_b[0] = CLVF_B.RED_Tz_Wrist;

      // DiscreteIntegrator: '<S527>/Acceleration  to Velocity' incorporates:
      //   DataStoreRead: '<S16>/BLACK_Fy_Sat'
      //   MATLAB Function: '<S527>/MATLAB Function'
      //   SignalConversion generated from: '<S551>/ SFunction '

      CLVF_B.RED_Tz_Wrist = CLVF_DW.BLACK_Fy_Sat / CLVF_P.model_param[2] *
        CLVF_P.AccelerationtoVelocity_gainval +
        CLVF_DW.AccelerationtoVelocity_DSTATE[1];

      // DiscreteIntegrator: '<S527>/Velocity to Position'
      CLVF_B.TmpSignalConversionAtSFun_n[1] = CLVF_P.VelocitytoPosition_gainval *
        CLVF_B.RED_Tz_Wrist + CLVF_DW.VelocitytoPosition_DSTATE[1];
      CLVF_B.x_ddot_b[1] = CLVF_B.RED_Tz_Wrist;

      // DiscreteIntegrator: '<S527>/Acceleration  to Velocity' incorporates:
      //   DataStoreRead: '<S16>/BLACK_Tz_Sat'
      //   MATLAB Function: '<S527>/MATLAB Function'
      //   SignalConversion generated from: '<S551>/ SFunction '

      CLVF_B.RED_Tz_Wrist = CLVF_DW.BLACK_Tz_Sat / CLVF_P.model_param[3] *
        CLVF_P.AccelerationtoVelocity_gainval +
        CLVF_DW.AccelerationtoVelocity_DSTATE[2];

      // DiscreteIntegrator: '<S527>/Velocity to Position'
      CLVF_B.TmpSignalConversionAtSFun_n[2] = CLVF_P.VelocitytoPosition_gainval *
        CLVF_B.RED_Tz_Wrist + CLVF_DW.VelocitytoPosition_DSTATE[2];

      // MATLAB Function: '<S533>/MATLAB Function' incorporates:
      //   Constant: '<S533>/Constant1'
      //   DataStoreRead: '<S533>/Data Store Read2'

      CLVF_MATLABFunction_f(CLVF_DW.Univ_Time, CLVF_P.serverRate,
                            &CLVF_B.sf_MATLABFunction_f);

      // Outputs for Triggered SubSystem: '<S533>/Sample and Hold1' incorporates:
      //   TriggerPort: '<S555>/Trigger'

      CLVF_B.zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
        &CLVF_PrevZCX.SampleandHold1_Trig_ZCE_g,
        (CLVF_B.sf_MATLABFunction_f.y));
      if (CLVF_B.zcEvent != NO_ZCEVENT) {
        CLVF_B.In_lb[0] = CLVF_B.TmpSignalConversionAtSFun_n[0] +
          CLVF_DW.NextOutput;
        CLVF_B.In_lb[1] = CLVF_B.TmpSignalConversionAtSFun_n[1] +
          CLVF_DW.NextOutput;
        CLVF_B.In_lb[2] = CLVF_B.TmpSignalConversionAtSFun_n[2] +
          CLVF_DW.NextOutput;
      }

      // End of Outputs for SubSystem: '<S533>/Sample and Hold1'

      // Delay: '<S545>/Delay1' incorporates:
      //   Inport: '<S555>/In'
      //   RandomNumber: '<S527>/Random Number'
      //   Sum: '<S527>/Sum'

      if (CLVF_DW.icLoad_jt != 0) {
        CLVF_DW.Delay1_DSTATE_lo = CLVF_B.In_lb[0];
      }

      // Sum: '<S545>/Sum6' incorporates:
      //   Delay: '<S545>/Delay1'

      CLVF_B.RED_Tz_Elbow = CLVF_B.In_lb[0] - CLVF_DW.Delay1_DSTATE_lo;

      // If: '<S545>/if we went through a "step"' incorporates:
      //   Inport: '<S569>/In1'

      if (CLVF_B.RED_Tz_Elbow != 0.0) {
        // Outputs for IfAction SubSystem: '<S545>/Hold this value' incorporates:
        //   ActionPort: '<S569>/Action Port'

        CLVF_B.In1_aj = CLVF_B.RED_Tz_Elbow;

        // End of Outputs for SubSystem: '<S545>/Hold this value'
      }

      // End of If: '<S545>/if we went through a "step"'

      // Gain: '<S545>/divide by delta T' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_Vx'

      CLVF_DW.BLACK_Vx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_aj;

      // Delay: '<S539>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_Vx'

      if (CLVF_DW.icLoad_fs != 0) {
        CLVF_DW.Delay1_DSTATE_k3 = CLVF_DW.BLACK_Vx;
      }

      // Sum: '<S539>/Sum6' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_Vx'
      //   Delay: '<S539>/Delay1'

      CLVF_B.RED_Tz_Elbow = CLVF_DW.BLACK_Vx - CLVF_DW.Delay1_DSTATE_k3;

      // If: '<S539>/if we went through a "step"' incorporates:
      //   Inport: '<S563>/In1'

      if (CLVF_B.RED_Tz_Elbow != 0.0) {
        // Outputs for IfAction SubSystem: '<S539>/Hold this value' incorporates:
        //   ActionPort: '<S563>/Action Port'

        CLVF_B.In1_e1 = CLVF_B.RED_Tz_Elbow;

        // End of Outputs for SubSystem: '<S539>/Hold this value'
      }

      // End of If: '<S539>/if we went through a "step"'

      // Sum: '<S16>/Sum1' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_IMU_Ax_I'
      //   Gain: '<S539>/divide by delta T'
      //   RandomNumber: '<S16>/Random Number1'

      CLVF_DW.BLACK_IMU_Ax_I = 1.0 / CLVF_P.serverRate * CLVF_B.In1_e1 +
        CLVF_DW.NextOutput_k;

      // Delay: '<S546>/Delay1'
      if (CLVF_DW.icLoad_b != 0) {
        CLVF_DW.Delay1_DSTATE_j3 = CLVF_B.In_lb[1];
      }

      // Sum: '<S546>/Sum6' incorporates:
      //   Delay: '<S546>/Delay1'

      CLVF_B.RED_Tz_Elbow = CLVF_B.In_lb[1] - CLVF_DW.Delay1_DSTATE_j3;

      // If: '<S546>/if we went through a "step"' incorporates:
      //   Inport: '<S570>/In1'

      if (CLVF_B.RED_Tz_Elbow != 0.0) {
        // Outputs for IfAction SubSystem: '<S546>/Hold this value' incorporates:
        //   ActionPort: '<S570>/Action Port'

        CLVF_B.In1_i5 = CLVF_B.RED_Tz_Elbow;

        // End of Outputs for SubSystem: '<S546>/Hold this value'
      }

      // End of If: '<S546>/if we went through a "step"'

      // Gain: '<S546>/divide by delta T' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_Vy'

      CLVF_DW.BLACK_Vy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_i5;

      // Delay: '<S540>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_Vy'

      if (CLVF_DW.icLoad_if != 0) {
        CLVF_DW.Delay1_DSTATE_gq = CLVF_DW.BLACK_Vy;
      }

      // Sum: '<S540>/Sum6' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_Vy'
      //   Delay: '<S540>/Delay1'

      CLVF_B.RED_Tz_Elbow = CLVF_DW.BLACK_Vy - CLVF_DW.Delay1_DSTATE_gq;

      // If: '<S540>/if we went through a "step"' incorporates:
      //   Inport: '<S564>/In1'

      if (CLVF_B.RED_Tz_Elbow != 0.0) {
        // Outputs for IfAction SubSystem: '<S540>/Hold this value' incorporates:
        //   ActionPort: '<S564>/Action Port'

        CLVF_B.In1_hm = CLVF_B.RED_Tz_Elbow;

        // End of Outputs for SubSystem: '<S540>/Hold this value'
      }

      // End of If: '<S540>/if we went through a "step"'

      // Sum: '<S16>/Sum2' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_IMU_Ay_I'
      //   Gain: '<S540>/divide by delta T'
      //   RandomNumber: '<S16>/Random Number'

      CLVF_DW.BLACK_IMU_Ay_I = 1.0 / CLVF_P.serverRate * CLVF_B.In1_hm +
        CLVF_DW.NextOutput_n;

      // Delay: '<S547>/Delay1'
      if (CLVF_DW.icLoad_eg != 0) {
        CLVF_DW.Delay1_DSTATE_ea = CLVF_B.In_lb[2];
      }

      // Sum: '<S547>/Sum6' incorporates:
      //   Delay: '<S547>/Delay1'

      CLVF_B.RED_Tz_Elbow = CLVF_B.In_lb[2] - CLVF_DW.Delay1_DSTATE_ea;

      // If: '<S547>/if we went through a "step"' incorporates:
      //   Inport: '<S571>/In1'

      if (CLVF_B.RED_Tz_Elbow != 0.0) {
        // Outputs for IfAction SubSystem: '<S547>/Hold this value' incorporates:
        //   ActionPort: '<S571>/Action Port'

        CLVF_B.In1_lx = CLVF_B.RED_Tz_Elbow;

        // End of Outputs for SubSystem: '<S547>/Hold this value'
      }

      // End of If: '<S547>/if we went through a "step"'

      // Gain: '<S547>/divide by delta T' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_RzD'

      CLVF_DW.BLACK_RzD = 1.0 / CLVF_P.serverRate * CLVF_B.In1_lx;

      // Sum: '<S16>/Sum3' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_IMU_R'
      //   DataStoreWrite: '<S16>/BLACK_RzD'
      //   RandomNumber: '<S16>/Random Number2'

      CLVF_DW.BLACK_IMU_R = CLVF_DW.BLACK_RzD + CLVF_DW.NextOutput_m;

      // DataStoreWrite: '<S16>/BLACK_Px'
      CLVF_DW.BLACK_Px = CLVF_B.In_lb[0];

      // DataStoreWrite: '<S16>/BLACK_Py'
      CLVF_DW.BLACK_Py = CLVF_B.In_lb[1];

      // DataStoreWrite: '<S16>/BLACK_Rz'
      CLVF_DW.BLACK_Rz = CLVF_B.In_lb[2];

      // DiscreteIntegrator: '<S528>/Acceleration  to Velocity' incorporates:
      //   DataStoreRead: '<S16>/BLUE_Fx_Sat '
      //   MATLAB Function: '<S528>/MATLAB Function'
      //   SignalConversion generated from: '<S552>/ SFunction '

      // MATLAB Function 'Simulate Plant Dynamics/BLUE  Dynamics Model/MATLAB Function': '<S552>:1' 
      // '<S552>:1:5' x_ddot     = zeros(3,1);
      // '<S552>:1:9' m_BLUE    = model_param(5);
      // '<S552>:1:10' I_BLUE    = model_param(6);
      // '<S552>:1:13' Fx        = control_inputs(1);
      // '<S552>:1:14' Fy        = control_inputs(2);
      // '<S552>:1:15' Tz        = control_inputs(3);
      // '<S552>:1:18' x_ddot(1) = Fx/m_BLUE;
      // '<S552>:1:19' x_ddot(2) = Fy/m_BLUE;
      // '<S552>:1:20' x_ddot(3) = Tz/I_BLUE;
      CLVF_B.vc_dot = CLVF_DW.BLUE_Fx_Sat / CLVF_P.model_param[4] *
        CLVF_P.AccelerationtoVelocity_gainva_a +
        CLVF_DW.AccelerationtoVelocity_DSTATE_d[0];

      // DiscreteIntegrator: '<S528>/Velocity to Position'
      CLVF_B.o_hat[0] = CLVF_P.VelocitytoPosition_gainval_a * CLVF_B.vc_dot +
        CLVF_DW.VelocitytoPosition_DSTATE_i[0];
      CLVF_B.x_ddot_g[0] = CLVF_B.vc_dot;

      // DiscreteIntegrator: '<S528>/Acceleration  to Velocity' incorporates:
      //   DataStoreRead: '<S16>/BLUE_Fy_Sat'
      //   MATLAB Function: '<S528>/MATLAB Function'
      //   SignalConversion generated from: '<S552>/ SFunction '

      CLVF_B.vc_dot = CLVF_DW.BLUE_Fy_Sat / CLVF_P.model_param[4] *
        CLVF_P.AccelerationtoVelocity_gainva_a +
        CLVF_DW.AccelerationtoVelocity_DSTATE_d[1];

      // DiscreteIntegrator: '<S528>/Velocity to Position'
      CLVF_B.o_hat[1] = CLVF_P.VelocitytoPosition_gainval_a * CLVF_B.vc_dot +
        CLVF_DW.VelocitytoPosition_DSTATE_i[1];
      CLVF_B.x_ddot_g[1] = CLVF_B.vc_dot;

      // DiscreteIntegrator: '<S528>/Acceleration  to Velocity' incorporates:
      //   DataStoreRead: '<S16>/BLUE_Tz_Sat'
      //   MATLAB Function: '<S528>/MATLAB Function'
      //   SignalConversion generated from: '<S552>/ SFunction '

      CLVF_B.vc_dot = CLVF_DW.BLUE_Tz_Sat / CLVF_P.model_param[5] *
        CLVF_P.AccelerationtoVelocity_gainva_a +
        CLVF_DW.AccelerationtoVelocity_DSTATE_d[2];

      // DiscreteIntegrator: '<S528>/Velocity to Position'
      CLVF_B.o_hat[2] = CLVF_P.VelocitytoPosition_gainval_a * CLVF_B.vc_dot +
        CLVF_DW.VelocitytoPosition_DSTATE_i[2];

      // MATLAB Function: '<S534>/MATLAB Function' incorporates:
      //   Constant: '<S534>/Constant1'
      //   DataStoreRead: '<S534>/Data Store Read2'

      CLVF_MATLABFunction_f(CLVF_DW.Univ_Time, CLVF_P.serverRate,
                            &CLVF_B.sf_MATLABFunction_he);

      // Outputs for Triggered SubSystem: '<S534>/Sample and Hold1' incorporates:
      //   TriggerPort: '<S557>/Trigger'

      CLVF_B.zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
        &CLVF_PrevZCX.SampleandHold1_Trig_ZCE,
        (CLVF_B.sf_MATLABFunction_he.y));
      if (CLVF_B.zcEvent != NO_ZCEVENT) {
        CLVF_B.In_e[0] = CLVF_B.o_hat[0] + CLVF_DW.NextOutput_h;
        CLVF_B.In_e[1] = CLVF_B.o_hat[1] + CLVF_DW.NextOutput_h;
        CLVF_B.In_e[2] = CLVF_B.o_hat[2] + CLVF_DW.NextOutput_h;
      }

      // End of Outputs for SubSystem: '<S534>/Sample and Hold1'

      // Delay: '<S548>/Delay1' incorporates:
      //   Inport: '<S557>/In'
      //   RandomNumber: '<S528>/Random Number'
      //   Sum: '<S528>/Sum'

      if (CLVF_DW.icLoad_hr != 0) {
        CLVF_DW.Delay1_DSTATE_oh = CLVF_B.In_e[0];
      }

      // Sum: '<S548>/Sum6' incorporates:
      //   Delay: '<S548>/Delay1'

      CLVF_B.RED_Tz_Elbow = CLVF_B.In_e[0] - CLVF_DW.Delay1_DSTATE_oh;

      // If: '<S548>/if we went through a "step"' incorporates:
      //   Inport: '<S572>/In1'

      if (CLVF_B.RED_Tz_Elbow != 0.0) {
        // Outputs for IfAction SubSystem: '<S548>/Hold this value' incorporates:
        //   ActionPort: '<S572>/Action Port'

        CLVF_B.In1_n5 = CLVF_B.RED_Tz_Elbow;

        // End of Outputs for SubSystem: '<S548>/Hold this value'
      }

      // End of If: '<S548>/if we went through a "step"'

      // Gain: '<S548>/divide by delta T' incorporates:
      //   DataStoreWrite: '<S16>/BLUE_Vx'

      CLVF_DW.BLUE_Vx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_n5;

      // Delay: '<S541>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/BLUE_Vx'

      if (CLVF_DW.icLoad_kp != 0) {
        CLVF_DW.Delay1_DSTATE_ko = CLVF_DW.BLUE_Vx;
      }

      // End of Delay: '<S541>/Delay1'

      // Delay: '<S549>/Delay1'
      if (CLVF_DW.icLoad_g != 0) {
        CLVF_DW.Delay1_DSTATE_il = CLVF_B.In_e[1];
      }

      // Sum: '<S549>/Sum6' incorporates:
      //   Delay: '<S549>/Delay1'

      CLVF_B.RED_Tz_Elbow = CLVF_B.In_e[1] - CLVF_DW.Delay1_DSTATE_il;

      // If: '<S549>/if we went through a "step"' incorporates:
      //   Inport: '<S573>/In1'

      if (CLVF_B.RED_Tz_Elbow != 0.0) {
        // Outputs for IfAction SubSystem: '<S549>/Hold this value' incorporates:
        //   ActionPort: '<S573>/Action Port'

        CLVF_B.In1_l4 = CLVF_B.RED_Tz_Elbow;

        // End of Outputs for SubSystem: '<S549>/Hold this value'
      }

      // End of If: '<S549>/if we went through a "step"'

      // Gain: '<S549>/divide by delta T' incorporates:
      //   DataStoreWrite: '<S16>/BLUE_Vy'

      CLVF_DW.BLUE_Vy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_l4;

      // Delay: '<S542>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/BLUE_Vy'

      if (CLVF_DW.icLoad_ge != 0) {
        CLVF_DW.Delay1_DSTATE_mf = CLVF_DW.BLUE_Vy;
      }

      // End of Delay: '<S542>/Delay1'

      // Delay: '<S550>/Delay1'
      if (CLVF_DW.icLoad_ho != 0) {
        CLVF_DW.Delay1_DSTATE_e1 = CLVF_B.In_e[2];
      }

      // Sum: '<S550>/Sum6' incorporates:
      //   Delay: '<S550>/Delay1'

      CLVF_B.RED_Tz_Elbow = CLVF_B.In_e[2] - CLVF_DW.Delay1_DSTATE_e1;

      // If: '<S550>/if we went through a "step"' incorporates:
      //   Inport: '<S574>/In1'

      if (CLVF_B.RED_Tz_Elbow != 0.0) {
        // Outputs for IfAction SubSystem: '<S550>/Hold this value' incorporates:
        //   ActionPort: '<S574>/Action Port'

        CLVF_B.In1_p = CLVF_B.RED_Tz_Elbow;

        // End of Outputs for SubSystem: '<S550>/Hold this value'
      }

      // End of If: '<S550>/if we went through a "step"'

      // Gain: '<S550>/divide by delta T' incorporates:
      //   DataStoreWrite: '<S16>/BLUE_RzD'

      CLVF_DW.BLUE_RzD = 1.0 / CLVF_P.serverRate * CLVF_B.In1_p;

      // Sum: '<S16>/Sum6' incorporates:
      //   DataStoreWrite: '<S16>/BLUE_IMU_R'
      //   DataStoreWrite: '<S16>/BLUE_RzD'
      //   RandomNumber: '<S16>/Random Number5'

      CLVF_B.RED_Tz_Elbow = CLVF_DW.BLUE_RzD + CLVF_DW.NextOutput_eo;

      // DataStoreWrite: '<S16>/BLUE_Px '
      CLVF_DW.BLUE_Px = CLVF_B.In_e[0];

      // DataStoreWrite: '<S16>/BLUE_Py'
      CLVF_DW.BLUE_Py = CLVF_B.In_e[1];

      // DataStoreWrite: '<S16>/BLUE_Rz'
      CLVF_DW.BLUE_Rz = CLVF_B.In_e[2];

      // RelationalOperator: '<S529>/Compare' incorporates:
      //   Constant: '<S529>/Constant'
      //   DataStoreRead: '<S16>/Data Store Read1'

      CLVF_B.rtb_Compare_dy = (CLVF_DW.BLUE_Rz == CLVF_P.Constant_Value_h);

      // DiscreteIntegrator: '<S16>/Discrete-Time Integrator' incorporates:
      //   DataStoreRead: '<S16>/Data Store Read1'

      if (CLVF_DW.DiscreteTimeIntegrator_IC_LOADI != 0) {
        CLVF_DW.DiscreteTimeIntegrator_DSTATE = CLVF_DW.BLUE_Rz;
      }

      if ((CLVF_B.rtb_Compare_dy && (CLVF_DW.DiscreteTimeIntegrator_PrevRese <=
            0)) || ((!CLVF_B.rtb_Compare_dy) &&
                    (CLVF_DW.DiscreteTimeIntegrator_PrevRese == 1))) {
        CLVF_DW.DiscreteTimeIntegrator_DSTATE = CLVF_DW.BLUE_Rz;
      }

      // RelationalOperator: '<S530>/Compare' incorporates:
      //   Constant: '<S530>/Constant'
      //   DataStoreRead: '<S16>/Data Store Read2'

      CLVF_B.rtb_Compare_n_d = (CLVF_DW.BLACK_Rz == CLVF_P.Constant_Value_j);

      // DiscreteIntegrator: '<S16>/Discrete-Time Integrator1' incorporates:
      //   DataStoreRead: '<S16>/Data Store Read2'
      //   DataStoreRead: '<S16>/Data Store Read4'
      //   DataStoreWrite: '<S16>/RED_Px1'

      if (CLVF_DW.DiscreteTimeIntegrator1_IC_LOAD != 0) {
        CLVF_DW.DiscreteTimeIntegrator1_DSTATE = CLVF_DW.BLACK_Rz;
      }

      if ((CLVF_B.rtb_Compare_n_d && (CLVF_DW.DiscreteTimeIntegrator1_PrevRes <=
            0)) || ((!CLVF_B.rtb_Compare_n_d) &&
                    (CLVF_DW.DiscreteTimeIntegrator1_PrevRes == 1))) {
        CLVF_DW.DiscreteTimeIntegrator1_DSTATE = CLVF_DW.BLACK_Rz;
      }

      CLVF_DW.BLACK_IMU_Psi = CLVF_P.DiscreteTimeIntegrator1_gainv_f *
        CLVF_DW.BLACK_IMU_R + CLVF_DW.DiscreteTimeIntegrator1_DSTATE;

      // End of DiscreteIntegrator: '<S16>/Discrete-Time Integrator1'

      // RelationalOperator: '<S531>/Compare' incorporates:
      //   Constant: '<S531>/Constant'
      //   DataStoreRead: '<S16>/Data Store Read5'

      CLVF_B.rtb_Compare_nd_l = (CLVF_DW.RED_Rz == CLVF_P.Constant_Value_c);

      // DiscreteIntegrator: '<S16>/Discrete-Time Integrator2' incorporates:
      //   DataStoreRead: '<S16>/Data Store Read5'
      //   DataStoreRead: '<S16>/Data Store Read6'
      //   DataStoreWrite: '<S16>/RED_Px2'

      if (CLVF_DW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
        CLVF_DW.DiscreteTimeIntegrator2_DSTATE = CLVF_DW.RED_Rz;
      }

      if ((CLVF_B.rtb_Compare_nd_l && (CLVF_DW.DiscreteTimeIntegrator2_PrevRes <=
            0)) || ((!CLVF_B.rtb_Compare_nd_l) &&
                    (CLVF_DW.DiscreteTimeIntegrator2_PrevRes == 1))) {
        CLVF_DW.DiscreteTimeIntegrator2_DSTATE = CLVF_DW.RED_Rz;
      }

      CLVF_DW.RED_IMU_Psi = CLVF_P.DiscreteTimeIntegrator2_gainval *
        CLVF_DW.RED_IMU_R + CLVF_DW.DiscreteTimeIntegrator2_DSTATE;

      // End of DiscreteIntegrator: '<S16>/Discrete-Time Integrator2'

      // DiscreteIntegrator: '<S532>/Acceleration  to Velocity' incorporates:
      //   DataStoreRead: '<S16>/RED_Fx_Sat'
      //   MATLAB Function: '<S532>/MATLAB Function'
      //   SignalConversion generated from: '<S553>/ SFunction '

      // MATLAB Function 'Simulate Plant Dynamics/RED Dynamics Model/MATLAB Function': '<S553>:1' 
      // '<S553>:1:5' x_ddot     = zeros(3,1);
      // '<S553>:1:9' m_RED    = model_param(1);
      // '<S553>:1:10' I_RED    = model_param(2);
      // '<S553>:1:13' Fx        = control_inputs(1);
      // '<S553>:1:14' Fy        = control_inputs(2);
      // '<S553>:1:15' Tz        = control_inputs(3);
      // '<S553>:1:18' x_ddot(1) = Fx/m_RED;
      // '<S553>:1:19' x_ddot(2) = Fy/m_RED;
      // '<S553>:1:20' x_ddot(3) = Tz/I_RED;
      CLVF_B.vc_p = CLVF_DW.RED_Fx_Sat / CLVF_P.model_param[0] *
        CLVF_P.AccelerationtoVelocity_gainva_i +
        CLVF_DW.AccelerationtoVelocity_DSTATE_h[0];

      // DiscreteIntegrator: '<S532>/Velocity to Position'
      CLVF_B.d_T[0] = CLVF_P.VelocitytoPosition_gainval_ad * CLVF_B.vc_p +
        CLVF_DW.VelocitytoPosition_DSTATE_f[0];
      CLVF_B.x_ddot[0] = CLVF_B.vc_p;

      // DiscreteIntegrator: '<S532>/Acceleration  to Velocity' incorporates:
      //   DataStoreRead: '<S16>/RED_Fy_Sat'
      //   MATLAB Function: '<S532>/MATLAB Function'
      //   SignalConversion generated from: '<S553>/ SFunction '

      CLVF_B.vc_p = CLVF_DW.RED_Fy_Sat / CLVF_P.model_param[0] *
        CLVF_P.AccelerationtoVelocity_gainva_i +
        CLVF_DW.AccelerationtoVelocity_DSTATE_h[1];

      // DiscreteIntegrator: '<S532>/Velocity to Position'
      CLVF_B.d_T[1] = CLVF_P.VelocitytoPosition_gainval_ad * CLVF_B.vc_p +
        CLVF_DW.VelocitytoPosition_DSTATE_f[1];
      CLVF_B.x_ddot[1] = CLVF_B.vc_p;

      // DiscreteIntegrator: '<S532>/Acceleration  to Velocity' incorporates:
      //   DataStoreRead: '<S16>/Data Store Read'
      //   DataStoreRead: '<S16>/RED_Tz_Sat'
      //   MATLAB Function: '<S532>/MATLAB Function'
      //   Sum: '<S16>/Sum'

      CLVF_B.vc_p = (CLVF_DW.RED_Tz_Sat + CLVF_DW.RED_Tz_RW_Sat) /
        CLVF_P.model_param[1] * CLVF_P.AccelerationtoVelocity_gainva_i +
        CLVF_DW.AccelerationtoVelocity_DSTATE_h[2];

      // DiscreteIntegrator: '<S532>/Velocity to Position'
      CLVF_B.d_T[2] = CLVF_P.VelocitytoPosition_gainval_ad * CLVF_B.vc_p +
        CLVF_DW.VelocitytoPosition_DSTATE_f[2];

      // MATLAB Function: '<S535>/MATLAB Function' incorporates:
      //   Constant: '<S535>/Constant'
      //   DataStoreRead: '<S535>/Data Store Read1'

      CLVF_MATLABFunction_f(CLVF_DW.Univ_Time, CLVF_P.serverRate,
                            &CLVF_B.sf_MATLABFunction_f2);

      // Outputs for Triggered SubSystem: '<S535>/Sample and Hold' incorporates:
      //   TriggerPort: '<S559>/Trigger'

      CLVF_B.zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
        &CLVF_PrevZCX.SampleandHold_Trig_ZCE_l,
        (CLVF_B.sf_MATLABFunction_f2.y));
      if (CLVF_B.zcEvent != NO_ZCEVENT) {
        CLVF_B.In_l[0] = CLVF_B.d_T[0] + CLVF_DW.NextOutput_i;
        CLVF_B.In_l[1] = CLVF_B.d_T[1] + CLVF_DW.NextOutput_i;
        CLVF_B.In_l[2] = CLVF_B.d_T[2] + CLVF_DW.NextOutput_i;
      }

      // End of Outputs for SubSystem: '<S535>/Sample and Hold'

      // Delay: '<S536>/Delay1' incorporates:
      //   Inport: '<S559>/In'
      //   RandomNumber: '<S532>/Random Number'
      //   Sum: '<S532>/Sum'

      if (CLVF_DW.icLoad_im != 0) {
        CLVF_DW.Delay1_DSTATE_i3 = CLVF_B.In_l[0];
      }

      // Sum: '<S536>/Sum6' incorporates:
      //   Delay: '<S536>/Delay1'

      CLVF_B.RED_Tz_Shoulder = CLVF_B.In_l[0] - CLVF_DW.Delay1_DSTATE_i3;

      // If: '<S536>/if we went through a "step"' incorporates:
      //   Inport: '<S560>/In1'

      if (CLVF_B.RED_Tz_Shoulder != 0.0) {
        // Outputs for IfAction SubSystem: '<S536>/Hold this value' incorporates:
        //   ActionPort: '<S560>/Action Port'

        CLVF_B.In1_al = CLVF_B.RED_Tz_Shoulder;

        // End of Outputs for SubSystem: '<S536>/Hold this value'
      }

      // End of If: '<S536>/if we went through a "step"'

      // Gain: '<S536>/divide by delta T' incorporates:
      //   DataStoreWrite: '<S16>/RED_Vx'

      CLVF_DW.RED_Vx = 1.0 / CLVF_P.serverRate * CLVF_B.In1_al;

      // Delay: '<S537>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/RED_Vx'

      if (CLVF_DW.icLoad_jx != 0) {
        CLVF_DW.Delay1_DSTATE_ji = CLVF_DW.RED_Vx;
      }

      // Sum: '<S537>/Sum6' incorporates:
      //   DataStoreWrite: '<S16>/RED_Vx'
      //   Delay: '<S537>/Delay1'

      CLVF_B.RED_Tz_Shoulder = CLVF_DW.RED_Vx - CLVF_DW.Delay1_DSTATE_ji;

      // If: '<S537>/if we went through a "step"' incorporates:
      //   Inport: '<S561>/In1'

      if (CLVF_B.RED_Tz_Shoulder != 0.0) {
        // Outputs for IfAction SubSystem: '<S537>/Hold this value' incorporates:
        //   ActionPort: '<S561>/Action Port'

        CLVF_B.In1_lo = CLVF_B.RED_Tz_Shoulder;

        // End of Outputs for SubSystem: '<S537>/Hold this value'
      }

      // End of If: '<S537>/if we went through a "step"'

      // Sum: '<S16>/Sum7' incorporates:
      //   DataStoreWrite: '<S16>/RED_IMU_Ay'
      //   Gain: '<S537>/divide by delta T'
      //   RandomNumber: '<S16>/Random Number7'

      CLVF_DW.RED_IMU_Ax_I = 1.0 / CLVF_P.serverRate * CLVF_B.In1_lo +
        CLVF_DW.NextOutput_mh;

      // Delay: '<S543>/Delay1'
      if (CLVF_DW.icLoad_mj != 0) {
        CLVF_DW.Delay1_DSTATE_mm = CLVF_B.In_l[1];
      }

      // Sum: '<S543>/Sum6' incorporates:
      //   Delay: '<S543>/Delay1'

      CLVF_B.RED_Tz_Shoulder = CLVF_B.In_l[1] - CLVF_DW.Delay1_DSTATE_mm;

      // If: '<S543>/if we went through a "step"' incorporates:
      //   Inport: '<S567>/In1'

      if (CLVF_B.RED_Tz_Shoulder != 0.0) {
        // Outputs for IfAction SubSystem: '<S543>/Hold this value' incorporates:
        //   ActionPort: '<S567>/Action Port'

        CLVF_B.In1_jz = CLVF_B.RED_Tz_Shoulder;

        // End of Outputs for SubSystem: '<S543>/Hold this value'
      }

      // End of If: '<S543>/if we went through a "step"'

      // Gain: '<S543>/divide by delta T' incorporates:
      //   DataStoreWrite: '<S16>/RED_Vy'

      CLVF_DW.RED_Vy = 1.0 / CLVF_P.serverRate * CLVF_B.In1_jz;

      // Delay: '<S538>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/RED_Vy'

      if (CLVF_DW.icLoad_nv != 0) {
        CLVF_DW.Delay1_DSTATE_d4 = CLVF_DW.RED_Vy;
      }

      // Sum: '<S538>/Sum6' incorporates:
      //   DataStoreWrite: '<S16>/RED_Vy'
      //   Delay: '<S538>/Delay1'

      CLVF_B.RED_Tz_Shoulder = CLVF_DW.RED_Vy - CLVF_DW.Delay1_DSTATE_d4;

      // If: '<S538>/if we went through a "step"' incorporates:
      //   Inport: '<S562>/In1'

      if (CLVF_B.RED_Tz_Shoulder != 0.0) {
        // Outputs for IfAction SubSystem: '<S538>/Hold this value' incorporates:
        //   ActionPort: '<S562>/Action Port'

        CLVF_B.In1_kx = CLVF_B.RED_Tz_Shoulder;

        // End of Outputs for SubSystem: '<S538>/Hold this value'
      }

      // End of If: '<S538>/if we went through a "step"'

      // Sum: '<S16>/Sum8' incorporates:
      //   DataStoreWrite: '<S16>/RED_IMU_Ay_I'
      //   Gain: '<S538>/divide by delta T'
      //   RandomNumber: '<S16>/Random Number6'

      CLVF_DW.RED_IMU_Ay_I = 1.0 / CLVF_P.serverRate * CLVF_B.In1_kx +
        CLVF_DW.NextOutput_kj;

      // Delay: '<S544>/Delay1'
      if (CLVF_DW.icLoad_nx != 0) {
        CLVF_DW.Delay1_DSTATE_cc = CLVF_B.In_l[2];
      }

      // Sum: '<S544>/Sum6' incorporates:
      //   Delay: '<S544>/Delay1'

      CLVF_B.RED_Tz_Shoulder = CLVF_B.In_l[2] - CLVF_DW.Delay1_DSTATE_cc;

      // If: '<S544>/if we went through a "step"' incorporates:
      //   Inport: '<S568>/In1'

      if (CLVF_B.RED_Tz_Shoulder != 0.0) {
        // Outputs for IfAction SubSystem: '<S544>/Hold this value' incorporates:
        //   ActionPort: '<S568>/Action Port'

        CLVF_B.In1_g0 = CLVF_B.RED_Tz_Shoulder;

        // End of Outputs for SubSystem: '<S544>/Hold this value'
      }

      // End of If: '<S544>/if we went through a "step"'

      // Gain: '<S544>/divide by delta T' incorporates:
      //   DataStoreWrite: '<S16>/RED_RzD'

      CLVF_DW.RED_RzD = 1.0 / CLVF_P.serverRate * CLVF_B.In1_g0;

      // Sum: '<S16>/Sum9' incorporates:
      //   DataStoreWrite: '<S16>/RED_IMU_R'
      //   DataStoreWrite: '<S16>/RED_RzD'
      //   RandomNumber: '<S16>/Random Number8'

      CLVF_DW.RED_IMU_R = CLVF_DW.RED_RzD + CLVF_DW.NextOutput_m2;

      // DataStoreWrite: '<S16>/RED_Px'
      CLVF_DW.RED_Px = CLVF_B.In_l[0];

      // DataStoreWrite: '<S16>/RED_Py'
      CLVF_DW.RED_Py = CLVF_B.In_l[1];

      // DataStoreWrite: '<S16>/RED_Rz'
      CLVF_DW.RED_Rz = CLVF_B.In_l[2];

      // Update for DiscreteIntegrator: '<S527>/Acceleration  to Velocity'
      CLVF_DW.AccelerationtoVelocity_DSTATE[0] = CLVF_B.x_ddot_b[0];

      // Update for DiscreteIntegrator: '<S527>/Velocity to Position'
      CLVF_DW.VelocitytoPosition_DSTATE[0] = CLVF_B.TmpSignalConversionAtSFun_n
        [0];

      // Update for DiscreteIntegrator: '<S527>/Acceleration  to Velocity'
      CLVF_DW.AccelerationtoVelocity_DSTATE[1] = CLVF_B.x_ddot_b[1];

      // Update for DiscreteIntegrator: '<S527>/Velocity to Position'
      CLVF_DW.VelocitytoPosition_DSTATE[1] = CLVF_B.TmpSignalConversionAtSFun_n
        [1];

      // Update for DiscreteIntegrator: '<S527>/Acceleration  to Velocity'
      CLVF_DW.AccelerationtoVelocity_DSTATE[2] = CLVF_B.RED_Tz_Wrist;

      // Update for DiscreteIntegrator: '<S527>/Velocity to Position'
      CLVF_DW.VelocitytoPosition_DSTATE[2] = CLVF_B.TmpSignalConversionAtSFun_n
        [2];

      // Update for RandomNumber: '<S527>/Random Number'
      CLVF_DW.NextOutput = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed) *
        sqrt(CLVF_P.noise_variance_BLACK) + CLVF_P.RandomNumber_Mean;

      // Update for Delay: '<S545>/Delay1'
      CLVF_DW.icLoad_jt = 0U;
      CLVF_DW.Delay1_DSTATE_lo = CLVF_B.In_lb[0];

      // Update for Delay: '<S539>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_Vx'

      CLVF_DW.icLoad_fs = 0U;
      CLVF_DW.Delay1_DSTATE_k3 = CLVF_DW.BLACK_Vx;

      // Update for RandomNumber: '<S16>/Random Number1'
      CLVF_DW.NextOutput_k = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_d)
        * CLVF_P.RandomNumber1_StdDev + CLVF_P.RandomNumber1_Mean;

      // Update for Delay: '<S546>/Delay1'
      CLVF_DW.icLoad_b = 0U;
      CLVF_DW.Delay1_DSTATE_j3 = CLVF_B.In_lb[1];

      // Update for Delay: '<S540>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/BLACK_Vy'

      CLVF_DW.icLoad_if = 0U;
      CLVF_DW.Delay1_DSTATE_gq = CLVF_DW.BLACK_Vy;

      // Update for RandomNumber: '<S16>/Random Number'
      CLVF_DW.NextOutput_n = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_b)
        * CLVF_P.RandomNumber_StdDev + CLVF_P.RandomNumber_Mean_o;

      // Update for Delay: '<S547>/Delay1'
      CLVF_DW.icLoad_eg = 0U;
      CLVF_DW.Delay1_DSTATE_ea = CLVF_B.In_lb[2];

      // Update for RandomNumber: '<S16>/Random Number2'
      CLVF_DW.NextOutput_m = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_a)
        * CLVF_P.RandomNumber2_StdDev + CLVF_P.RandomNumber2_Mean;

      // Update for RandomNumber: '<S528>/Random Number'
      CLVF_DW.NextOutput_h = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_h)
        * sqrt(CLVF_P.noise_variance_BLUE) + CLVF_P.RandomNumber_Mean_n;

      // Update for Delay: '<S548>/Delay1'
      CLVF_DW.icLoad_hr = 0U;
      CLVF_DW.Delay1_DSTATE_oh = CLVF_B.In_e[0];

      // Update for Delay: '<S541>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/BLUE_Vx'

      CLVF_DW.icLoad_kp = 0U;
      CLVF_DW.Delay1_DSTATE_ko = CLVF_DW.BLUE_Vx;

      // Update for RandomNumber: '<S16>/Random Number4'
      CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_i);

      // Update for Delay: '<S549>/Delay1'
      CLVF_DW.icLoad_g = 0U;
      CLVF_DW.Delay1_DSTATE_il = CLVF_B.In_e[1];

      // Update for Delay: '<S542>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/BLUE_Vy'

      CLVF_DW.icLoad_ge = 0U;
      CLVF_DW.Delay1_DSTATE_mf = CLVF_DW.BLUE_Vy;

      // Update for RandomNumber: '<S16>/Random Number3'
      CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_c);

      // Update for Delay: '<S550>/Delay1'
      CLVF_DW.icLoad_ho = 0U;
      CLVF_DW.Delay1_DSTATE_e1 = CLVF_B.In_e[2];

      // Update for RandomNumber: '<S16>/Random Number5'
      CLVF_DW.NextOutput_eo = CLVF_rt_nrand_Upu32_Yd_f_pw_snf
        (&CLVF_DW.RandSeed_n) * CLVF_P.RandomNumber5_StdDev +
        CLVF_P.RandomNumber5_Mean;

      // Update for DiscreteIntegrator: '<S16>/Discrete-Time Integrator' incorporates:
      //   DataStoreRead: '<S16>/Data Store Read3'

      CLVF_DW.DiscreteTimeIntegrator_IC_LOADI = 0U;
      CLVF_DW.DiscreteTimeIntegrator_DSTATE +=
        CLVF_P.DiscreteTimeIntegrator_gainval * CLVF_B.RED_Tz_Elbow;
      CLVF_DW.DiscreteTimeIntegrator_PrevRese = static_cast<int8_T>
        (CLVF_B.rtb_Compare_dy);

      // Update for DiscreteIntegrator: '<S16>/Discrete-Time Integrator1' incorporates:
      //   DataStoreWrite: '<S16>/RED_Px1'

      CLVF_DW.DiscreteTimeIntegrator1_IC_LOAD = 0U;
      CLVF_DW.DiscreteTimeIntegrator1_DSTATE = CLVF_DW.BLACK_IMU_Psi;
      CLVF_DW.DiscreteTimeIntegrator1_PrevRes = static_cast<int8_T>
        (CLVF_B.rtb_Compare_n_d);

      // Update for DiscreteIntegrator: '<S16>/Discrete-Time Integrator2' incorporates:
      //   DataStoreWrite: '<S16>/RED_Px2'

      CLVF_DW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
      CLVF_DW.DiscreteTimeIntegrator2_DSTATE = CLVF_DW.RED_IMU_Psi;
      CLVF_DW.DiscreteTimeIntegrator2_PrevRes = static_cast<int8_T>
        (CLVF_B.rtb_Compare_nd_l);

      // Update for DiscreteIntegrator: '<S528>/Acceleration  to Velocity'
      CLVF_DW.AccelerationtoVelocity_DSTATE_d[0] = CLVF_B.x_ddot_g[0];

      // Update for DiscreteIntegrator: '<S528>/Velocity to Position'
      CLVF_DW.VelocitytoPosition_DSTATE_i[0] = CLVF_B.o_hat[0];

      // Update for DiscreteIntegrator: '<S532>/Acceleration  to Velocity'
      CLVF_DW.AccelerationtoVelocity_DSTATE_h[0] = CLVF_B.x_ddot[0];

      // Update for DiscreteIntegrator: '<S532>/Velocity to Position'
      CLVF_DW.VelocitytoPosition_DSTATE_f[0] = CLVF_B.d_T[0];

      // Update for DiscreteIntegrator: '<S528>/Acceleration  to Velocity'
      CLVF_DW.AccelerationtoVelocity_DSTATE_d[1] = CLVF_B.x_ddot_g[1];

      // Update for DiscreteIntegrator: '<S528>/Velocity to Position'
      CLVF_DW.VelocitytoPosition_DSTATE_i[1] = CLVF_B.o_hat[1];

      // Update for DiscreteIntegrator: '<S532>/Acceleration  to Velocity'
      CLVF_DW.AccelerationtoVelocity_DSTATE_h[1] = CLVF_B.x_ddot[1];

      // Update for DiscreteIntegrator: '<S532>/Velocity to Position'
      CLVF_DW.VelocitytoPosition_DSTATE_f[1] = CLVF_B.d_T[1];

      // Update for DiscreteIntegrator: '<S528>/Acceleration  to Velocity'
      CLVF_DW.AccelerationtoVelocity_DSTATE_d[2] = CLVF_B.vc_dot;

      // Update for DiscreteIntegrator: '<S528>/Velocity to Position'
      CLVF_DW.VelocitytoPosition_DSTATE_i[2] = CLVF_B.o_hat[2];

      // Update for DiscreteIntegrator: '<S532>/Acceleration  to Velocity'
      CLVF_DW.AccelerationtoVelocity_DSTATE_h[2] = CLVF_B.vc_p;

      // Update for DiscreteIntegrator: '<S532>/Velocity to Position'
      CLVF_DW.VelocitytoPosition_DSTATE_f[2] = CLVF_B.d_T[2];

      // Update for RandomNumber: '<S532>/Random Number'
      CLVF_DW.NextOutput_i = CLVF_rt_nrand_Upu32_Yd_f_pw_snf
        (&CLVF_DW.RandSeed_ct) * sqrt(CLVF_P.noise_variance_RED) +
        CLVF_P.RandomNumber_Mean_c;

      // Update for Delay: '<S536>/Delay1'
      CLVF_DW.icLoad_im = 0U;
      CLVF_DW.Delay1_DSTATE_i3 = CLVF_B.In_l[0];

      // Update for Delay: '<S537>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/RED_Vx'

      CLVF_DW.icLoad_jx = 0U;
      CLVF_DW.Delay1_DSTATE_ji = CLVF_DW.RED_Vx;

      // Update for RandomNumber: '<S16>/Random Number7'
      CLVF_DW.NextOutput_mh = CLVF_rt_nrand_Upu32_Yd_f_pw_snf
        (&CLVF_DW.RandSeed_dd) * CLVF_P.RandomNumber7_StdDev +
        CLVF_P.RandomNumber7_Mean;

      // Update for Delay: '<S543>/Delay1'
      CLVF_DW.icLoad_mj = 0U;
      CLVF_DW.Delay1_DSTATE_mm = CLVF_B.In_l[1];

      // Update for Delay: '<S538>/Delay1' incorporates:
      //   DataStoreWrite: '<S16>/RED_Vy'

      CLVF_DW.icLoad_nv = 0U;
      CLVF_DW.Delay1_DSTATE_d4 = CLVF_DW.RED_Vy;

      // Update for RandomNumber: '<S16>/Random Number6'
      CLVF_DW.NextOutput_kj = CLVF_rt_nrand_Upu32_Yd_f_pw_snf
        (&CLVF_DW.RandSeed_ik) * CLVF_P.RandomNumber6_StdDev +
        CLVF_P.RandomNumber6_Mean;

      // Update for Delay: '<S544>/Delay1'
      CLVF_DW.icLoad_nx = 0U;
      CLVF_DW.Delay1_DSTATE_cc = CLVF_B.In_l[2];

      // Update for RandomNumber: '<S16>/Random Number8'
      CLVF_DW.NextOutput_m2 = CLVF_rt_nrand_Upu32_Yd_f_pw_snf
        (&CLVF_DW.RandSeed_bm) * CLVF_P.RandomNumber8_StdDev +
        CLVF_P.RandomNumber8_Mean;

      // End of Outputs for SubSystem: '<Root>/Simulate Plant Dynamics'
    }

    // End of If: '<Root>/If running a simulation,  grab the simulated states.'

    // SignalConversion generated from: '<S1>/To Workspace' incorporates:
    //   DataStoreRead: '<S1>/ARM_Elbow_Px'
    //   DataStoreRead: '<S1>/ARM_Elbow_Py'
    //   DataStoreRead: '<S1>/ARM_Wrist_Px'
    //   DataStoreRead: '<S1>/ARM_Wrist_Py'
    //   DataStoreRead: '<S1>/BLACK_AHRS_P'
    //   DataStoreRead: '<S1>/BLACK_AHRS_Q'
    //   DataStoreRead: '<S1>/BLACK_AHRS_R'
    //   DataStoreRead: '<S1>/BLACK_Ax'
    //   DataStoreRead: '<S1>/BLACK_Ay'
    //   DataStoreRead: '<S1>/BLACK_Fx_Sat'
    //   DataStoreRead: '<S1>/BLACK_Fy_Sat'
    //   DataStoreRead: '<S1>/BLACK_GyroX_Raw'
    //   DataStoreRead: '<S1>/BLACK_GyroY_Raw'
    //   DataStoreRead: '<S1>/BLACK_GyroZ_Raw'
    //   DataStoreRead: '<S1>/BLACK_Gyro_Attitude'
    //   DataStoreRead: '<S1>/BLACK_IMU_Ax_I'
    //   DataStoreRead: '<S1>/BLACK_IMU_Ax_b'
    //   DataStoreRead: '<S1>/BLACK_IMU_Ay_I'
    //   DataStoreRead: '<S1>/BLACK_IMU_Ay_b'
    //   DataStoreRead: '<S1>/BLACK_IMU_Az_b'
    //   DataStoreRead: '<S1>/BLACK_Px'
    //   DataStoreRead: '<S1>/BLACK_Py'
    //   DataStoreRead: '<S1>/BLACK_Rz'
    //   DataStoreRead: '<S1>/BLACK_RzD'
    //   DataStoreRead: '<S1>/BLACK_RzDD'
    //   DataStoreRead: '<S1>/BLACK_Tz_Sat'
    //   DataStoreRead: '<S1>/BLACK_Vx'
    //   DataStoreRead: '<S1>/BLACK_Vy'
    //   DataStoreRead: '<S1>/BLUE_Fx_Sat'
    //   DataStoreRead: '<S1>/BLUE_Fy_Sat'
    //   DataStoreRead: '<S1>/BLUE_Px'
    //   DataStoreRead: '<S1>/BLUE_Py'
    //   DataStoreRead: '<S1>/BLUE_Rz'
    //   DataStoreRead: '<S1>/BLUE_RzD'
    //   DataStoreRead: '<S1>/BLUE_Tz_Sat'
    //   DataStoreRead: '<S1>/BLUE_Vx'
    //   DataStoreRead: '<S1>/BLUE_Vy'
    //   DataStoreRead: '<S1>/Desired speed x'
    //   DataStoreRead: '<S1>/Desired speed y'
    //   DataStoreRead: '<S1>/RED_AHRS_P'
    //   DataStoreRead: '<S1>/RED_AHRS_Q'
    //   DataStoreRead: '<S1>/RED_AHRS_R'
    //   DataStoreRead: '<S1>/RED_Ax'
    //   DataStoreRead: '<S1>/RED_Ay'
    //   DataStoreRead: '<S1>/RED_Fx_Sat'
    //   DataStoreRead: '<S1>/RED_Fy_Sat'
    //   DataStoreRead: '<S1>/RED_GyroX_Raw'
    //   DataStoreRead: '<S1>/RED_GyroY_Raw'
    //   DataStoreRead: '<S1>/RED_GyroZ_Raw'
    //   DataStoreRead: '<S1>/RED_Gyro_Attitude'
    //   DataStoreRead: '<S1>/RED_IMU_Ax_I'
    //   DataStoreRead: '<S1>/RED_IMU_Ax_b'
    //   DataStoreRead: '<S1>/RED_IMU_Ay_I'
    //   DataStoreRead: '<S1>/RED_IMU_Ay_b'
    //   DataStoreRead: '<S1>/RED_IMU_Az_b'
    //   DataStoreRead: '<S1>/RED_Px'
    //   DataStoreRead: '<S1>/RED_Py '
    //   DataStoreRead: '<S1>/RED_Rz'
    //   DataStoreRead: '<S1>/RED_RzDD'
    //   DataStoreRead: '<S1>/RED_Tz_Sat'
    //   DataStoreRead: '<S1>/RED_Vx '
    //   DataStoreRead: '<S1>/RED_Vy'
    //   DataStoreRead: '<S1>/RED_Vz'
    //   DataStoreRead: '<S1>/Universal_Time'

    CLVF_B.TmpSignalConversionAtToWork[0] = CLVF_DW.Univ_Time;
    CLVF_B.TmpSignalConversionAtToWork[1] = CLVF_DW.RED_Fx_Sat;
    CLVF_B.TmpSignalConversionAtToWork[2] = CLVF_DW.RED_Fy_Sat;
    CLVF_B.TmpSignalConversionAtToWork[3] = CLVF_DW.RED_Tz_Sat;
    CLVF_B.TmpSignalConversionAtToWork[4] = CLVF_DW.RED_Px;
    CLVF_B.TmpSignalConversionAtToWork[5] = CLVF_DW.RED_Py;
    CLVF_B.TmpSignalConversionAtToWork[6] = CLVF_DW.RED_Rz;
    CLVF_B.TmpSignalConversionAtToWork[7] = CLVF_DW.RED_Vx;
    CLVF_B.TmpSignalConversionAtToWork[8] = CLVF_DW.RED_Vy;
    CLVF_B.TmpSignalConversionAtToWork[9] = CLVF_DW.RED_RzD;
    CLVF_B.TmpSignalConversionAtToWork[10] = CLVF_DW.RED_AHRS_Q;
    CLVF_B.TmpSignalConversionAtToWork[11] = CLVF_DW.RED_AHRS_P;
    CLVF_B.TmpSignalConversionAtToWork[12] = CLVF_DW.RED_AHRS_R;
    CLVF_B.TmpSignalConversionAtToWork[13] = CLVF_DW.RED_IMU_Ax_b;
    CLVF_B.TmpSignalConversionAtToWork[14] = CLVF_DW.RED_IMU_Ay_b;
    CLVF_B.TmpSignalConversionAtToWork[15] = CLVF_DW.RED_IMU_Az_b;
    CLVF_B.TmpSignalConversionAtToWork[16] = CLVF_DW.RED_IMU_Ax_I;
    CLVF_B.TmpSignalConversionAtToWork[17] = CLVF_DW.RED_IMU_Ay_I;
    CLVF_B.TmpSignalConversionAtToWork[18] = CLVF_DW.RED_Ax;
    CLVF_B.TmpSignalConversionAtToWork[19] = CLVF_DW.RED_Ay;
    CLVF_B.TmpSignalConversionAtToWork[20] = CLVF_DW.RED_RzDD;
    CLVF_B.TmpSignalConversionAtToWork[21] = CLVF_DW.BLACK_Fx_Sat;
    CLVF_B.TmpSignalConversionAtToWork[22] = CLVF_DW.BLACK_Fy_Sat;
    CLVF_B.TmpSignalConversionAtToWork[23] = CLVF_DW.BLACK_Tz_Sat;
    CLVF_B.TmpSignalConversionAtToWork[24] = CLVF_DW.BLACK_Px;
    CLVF_B.TmpSignalConversionAtToWork[25] = CLVF_DW.BLACK_Py;
    CLVF_B.TmpSignalConversionAtToWork[26] = CLVF_DW.BLACK_Rz;
    CLVF_B.TmpSignalConversionAtToWork[27] = CLVF_DW.BLACK_Vx;
    CLVF_B.TmpSignalConversionAtToWork[28] = CLVF_DW.BLACK_Vy;
    CLVF_B.TmpSignalConversionAtToWork[29] = CLVF_DW.BLACK_RzD;
    CLVF_B.TmpSignalConversionAtToWork[30] = CLVF_DW.BLACK_AHRS_Q;
    CLVF_B.TmpSignalConversionAtToWork[31] = CLVF_DW.BLACK_AHRS_P;
    CLVF_B.TmpSignalConversionAtToWork[32] = CLVF_DW.BLACK_AHRS_R;
    CLVF_B.TmpSignalConversionAtToWork[33] = CLVF_DW.BLACK_IMU_Ax_b;
    CLVF_B.TmpSignalConversionAtToWork[34] = CLVF_DW.BLACK_IMU_Ay_b;
    CLVF_B.TmpSignalConversionAtToWork[35] = CLVF_DW.BLACK_IMU_Az_b;
    CLVF_B.TmpSignalConversionAtToWork[36] = CLVF_DW.BLACK_IMU_Ax_I;
    CLVF_B.TmpSignalConversionAtToWork[37] = CLVF_DW.BLACK_IMU_Ay_I;
    CLVF_B.TmpSignalConversionAtToWork[38] = CLVF_DW.BLACK_Ax;
    CLVF_B.TmpSignalConversionAtToWork[39] = CLVF_DW.BLACK_Ay;
    CLVF_B.TmpSignalConversionAtToWork[40] = CLVF_DW.BLACK_RzDD;
    CLVF_B.TmpSignalConversionAtToWork[41] = CLVF_DW.BLUE_Fx_Sat;
    CLVF_B.TmpSignalConversionAtToWork[42] = CLVF_DW.BLUE_Fy_Sat;
    CLVF_B.TmpSignalConversionAtToWork[43] = CLVF_DW.BLUE_Tz_Sat;
    CLVF_B.TmpSignalConversionAtToWork[44] = CLVF_DW.BLUE_Px;
    CLVF_B.TmpSignalConversionAtToWork[45] = CLVF_DW.BLUE_Py;
    CLVF_B.TmpSignalConversionAtToWork[46] = CLVF_DW.BLUE_Rz;
    CLVF_B.TmpSignalConversionAtToWork[47] = CLVF_DW.BLUE_Vx;
    CLVF_B.TmpSignalConversionAtToWork[48] = CLVF_DW.BLUE_Vy;
    CLVF_B.TmpSignalConversionAtToWork[49] = CLVF_DW.BLUE_RzD;
    CLVF_B.TmpSignalConversionAtToWork[50] = CLVF_DW.RED_IMU_Q;
    CLVF_B.TmpSignalConversionAtToWork[51] = CLVF_DW.RED_IMU_P;
    CLVF_B.TmpSignalConversionAtToWork[52] = CLVF_DW.RED_IMU_R;
    CLVF_B.TmpSignalConversionAtToWork[53] = CLVF_DW.BLACK_IMU_Q;
    CLVF_B.TmpSignalConversionAtToWork[54] = CLVF_DW.BLACK_IMU_P;
    CLVF_B.TmpSignalConversionAtToWork[55] = CLVF_DW.BLACK_IMU_R;
    CLVF_B.TmpSignalConversionAtToWork[56] = CLVF_DW.RED_IMU_Psi;
    CLVF_B.TmpSignalConversionAtToWork[57] = CLVF_DW.BLACK_IMU_Psi;
    CLVF_B.TmpSignalConversionAtToWork[58] = CLVF_DW.ARM_Elbow_Px;
    CLVF_B.TmpSignalConversionAtToWork[59] = CLVF_DW.ARM_Elbow_Py;
    CLVF_B.TmpSignalConversionAtToWork[60] = CLVF_DW.ARM_Wrist_Px;
    CLVF_B.TmpSignalConversionAtToWork[61] = CLVF_DW.ARM_Wrist_Py;
    CLVF_B.TmpSignalConversionAtToWork[62] = CLVF_DW.SPEED_DESIRED_x;
    CLVF_B.TmpSignalConversionAtToWork[63] = CLVF_DW.SPEED_DESIRED_y;

    // ToWorkspace: '<S1>/To Workspace'
    rt_UpdateLogVar((LogVar *)(LogVar*) (CLVF_DW.ToWorkspace_PWORK.LoggedData),
                    &CLVF_B.TmpSignalConversionAtToWork[0], 0);

    // If: '<S2>/This IF block determines whether or not to run the exp code' incorporates:
    //   Constant: '<S19>/Constant'

    if (CLVF_P.simMode == 0.0) {
      // Outputs for IfAction SubSystem: '<S2>/Change Behavior' incorporates:
      //   ActionPort: '<S18>/Action Port'

      // MATLABSystem: '<S20>/Digital Write' incorporates:
      //   DataStoreRead: '<S18>/Data Store Read'

      CLVF_B.RED_Tz_Wrist = rt_roundd_snf(CLVF_DW.Magnet_State);
      if (CLVF_B.RED_Tz_Wrist < 256.0) {
        if (CLVF_B.RED_Tz_Wrist >= 0.0) {
          CLVF_B.status = static_cast<uint8_T>(CLVF_B.RED_Tz_Wrist);
        } else {
          CLVF_B.status = 0U;
        }
      } else {
        CLVF_B.status = MAX_uint8_T;
      }

      MW_gpioWrite(10U, CLVF_B.status);

      // End of MATLABSystem: '<S20>/Digital Write'

      // MATLABSystem: '<S21>/Digital Write' incorporates:
      //   DataStoreRead: '<S18>/Data Store Read1'

      CLVF_B.RED_Tz_Wrist = rt_roundd_snf(CLVF_DW.Float_State);
      if (CLVF_B.RED_Tz_Wrist < 256.0) {
        if (CLVF_B.RED_Tz_Wrist >= 0.0) {
          CLVF_B.status = static_cast<uint8_T>(CLVF_B.RED_Tz_Wrist);
        } else {
          CLVF_B.status = 0U;
        }
      } else {
        CLVF_B.status = MAX_uint8_T;
      }

      MW_gpioWrite(26U, CLVF_B.status);

      // End of MATLABSystem: '<S21>/Digital Write'
      // End of Outputs for SubSystem: '<S2>/Change Behavior'
    }

    // End of If: '<S2>/This IF block determines whether or not to run the exp code' 

    // If: '<S3>/This IF block determines whether or not to run the BLACK sim//exp' incorporates:
    //   Constant: '<S25>/Constant'
    //   Constant: '<S3>/Constant'
    //   DataStoreRead: '<S22>/BLACK_Fx'
    //   DataStoreRead: '<S22>/BLACK_Fy'
    //   Product: '<S28>/Rotate F_I to F_b'
    //   SignalConversion generated from: '<S28>/Rotate F_I to F_b'

    if ((CLVF_P.WhoAmI == 2.0) || (CLVF_P.simMode == 1.0)) {
      // Outputs for IfAction SubSystem: '<S3>/Change BLACK Behavior' incorporates:
      //   ActionPort: '<S22>/Action Port'

      // MATLAB Function: '<S32>/Create Rotation Matrix' incorporates:
      //   DataStoreRead: '<S27>/BLACK_Rz'

      CLVF_CreateRotationMatrix(CLVF_DW.BLACK_Rz,
        &CLVF_B.sf_CreateRotationMatrix);

      // MATLAB Function: '<S27>/MATLAB Function1'
      // MATLAB Function 'From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/MATLAB Function1': '<S30>:1' 
      // '<S30>:1:3' Vec1 = [ -1
      // '<S30>:1:4'          -1
      // '<S30>:1:5'           0
      // '<S30>:1:6'           0
      // '<S30>:1:7'           1
      // '<S30>:1:8'           1
      // '<S30>:1:9'           0
      // '<S30>:1:10'           0 ];
      // '<S30>:1:12' Vec2 = [  0
      // '<S30>:1:13'           0
      // '<S30>:1:14'           1
      // '<S30>:1:15'           1
      // '<S30>:1:16'           0
      // '<S30>:1:17'           0
      // '<S30>:1:18'          -1
      // '<S30>:1:19'          -1 ];
      // '<S30>:1:21' Vec3 = thruster_dist2CG_BLACK./1000;
      // '<S30>:1:23' Mat1 = [Vec1, Vec2, Vec3]';
      // '<S30>:1:25' Mat2 = diag((F_thrusters_BLACK));
      memset(&CLVF_B.Mat2[0], 0, sizeof(real_T) << 6U);

      // '<S30>:1:27' H    = Mat1*Mat2;
      // MATLAB Function 'From Force//Torque to PWM Signal/Change BLACK Behavior/Calculate Thruster  ON//OFF/MATLAB Function': '<S29>:1' 
      // '<S29>:1:3' Vec1 = [ -1
      // '<S29>:1:4'          -1
      // '<S29>:1:5'           0
      // '<S29>:1:6'           0
      // '<S29>:1:7'           1
      // '<S29>:1:8'           1
      // '<S29>:1:9'           0
      // '<S29>:1:10'           0 ];
      // '<S29>:1:12' Vec2 = [  0
      // '<S29>:1:13'           0
      // '<S29>:1:14'           1
      // '<S29>:1:15'           1
      // '<S29>:1:16'           0
      // '<S29>:1:17'           0
      // '<S29>:1:18'          -1
      // '<S29>:1:19'          -1 ];
      // '<S29>:1:21' Vec3 = thruster_dist2CG_BLACK./1000;
      // '<S29>:1:23' Mat1 = [Vec1, Vec2, Vec3]';
      // '<S29>:1:25' Mat2 = diag((F_thrusters_BLACK./2));
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        CLVF_B.Mat2[CLVF_B.uElOffset1 + (CLVF_B.uElOffset1 << 3)] =
          CLVF_P.F_thrusters_BLACK[CLVF_B.uElOffset1];

        // MATLAB Function: '<S27>/MATLAB Function'
        CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
          CLVF_P.F_thrusters_BLACK[CLVF_B.uElOffset1] / 2.0;
      }

      // MATLAB Function: '<S27>/MATLAB Function'
      memset(&CLVF_B.Mat2_m[0], 0, sizeof(real_T) << 6U);

      // '<S29>:1:27' H    = Mat1*Mat2;
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        CLVF_B.Mat2_m[CLVF_B.uElOffset1 + (CLVF_B.uElOffset1 << 3)] =
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1] = b_0[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1 + 1] = c[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1 + 2] =
          CLVF_P.thruster_dist2CG_BLACK[CLVF_B.uElOffset1] / 1000.0;
      }

      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx0 = CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1;
          CLVF_B.H_bu[CLVF_B.ntIdx0] = 0.0;
          for (CLVF_B.ntIdx1 = 0; CLVF_B.ntIdx1 < 8; CLVF_B.ntIdx1++) {
            CLVF_B.H_bu[CLVF_B.ntIdx0] += CLVF_B.b[3 * CLVF_B.ntIdx1 +
              CLVF_B.yElIdx] * CLVF_B.Mat2_m[(CLVF_B.uElOffset1 << 3) +
              CLVF_B.ntIdx1];
          }
        }
      }

      // PermuteDimensions: '<S33>/transpose'
      CLVF_B.yElIdx = 0;
      CLVF_B.uElOffset1 = 0;
      for (CLVF_B.ntIdx1 = 0; CLVF_B.ntIdx1 < 3; CLVF_B.ntIdx1++) {
        CLVF_B.uElOffset0 = CLVF_B.uElOffset1;
        for (CLVF_B.ntIdx0 = 0; CLVF_B.ntIdx0 < 8; CLVF_B.ntIdx0++) {
          CLVF_B.transpose_d[CLVF_B.yElIdx] = CLVF_B.H_bu[CLVF_B.uElOffset0];
          CLVF_B.yElIdx++;
          CLVF_B.uElOffset0 += 3;
        }

        CLVF_B.uElOffset1++;
      }

      // End of PermuteDimensions: '<S33>/transpose'

      // MATLAB Function: '<S28>/Create Rotation Matrix' incorporates:
      //   DataStoreRead: '<S22>/BLACK_Rz'

      CLVF_CreateRotationMatrix_h(CLVF_DW.BLACK_Rz,
        &CLVF_B.sf_CreateRotationMatrix_h);

      // Product: '<S33>/Product'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx0 = CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1;
          CLVF_B.C_IB[CLVF_B.ntIdx0] = 0.0;
          for (CLVF_B.ntIdx1 = 0; CLVF_B.ntIdx1 < 8; CLVF_B.ntIdx1++) {
            CLVF_B.C_IB[CLVF_B.ntIdx0] += CLVF_B.H_bu[3 * CLVF_B.ntIdx1 +
              CLVF_B.yElIdx] * CLVF_B.transpose_d[(CLVF_B.uElOffset1 << 3) +
              CLVF_B.ntIdx1];
          }
        }
      }

      // Product: '<S33>/Product1' incorporates:
      //   Product: '<S33>/Product'

      CLVF_rt_invd3x3_snf(CLVF_B.C_IB, CLVF_B.vc);
      CLVF_B.RED_Tz_Elbow = CLVF_B.sf_CreateRotationMatrix_h.C_bI[0] *
        CLVF_DW.BLACK_Fx + CLVF_B.sf_CreateRotationMatrix_h.C_bI[2] *
        CLVF_DW.BLACK_Fy;

      // Product: '<S28>/Rotate F_I to F_b' incorporates:
      //   DataStoreRead: '<S22>/BLACK_Fx'
      //   DataStoreRead: '<S22>/BLACK_Fy'
      //   SignalConversion generated from: '<S28>/Rotate F_I to F_b'

      CLVF_B.RED_Tz_Shoulder = CLVF_B.sf_CreateRotationMatrix_h.C_bI[1] *
        CLVF_DW.BLACK_Fx + CLVF_B.sf_CreateRotationMatrix_h.C_bI[3] *
        CLVF_DW.BLACK_Fy;

      // Product: '<S33>/Product2'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 8; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx1 = CLVF_B.yElIdx + (CLVF_B.uElOffset1 << 3);
          CLVF_B.b[CLVF_B.ntIdx1] = 0.0;
          CLVF_B.b[CLVF_B.ntIdx1] += CLVF_B.vc[3 * CLVF_B.uElOffset1] *
            CLVF_B.transpose_d[CLVF_B.yElIdx];
          CLVF_B.b[CLVF_B.ntIdx1] += CLVF_B.vc[3 * CLVF_B.uElOffset1 + 1] *
            CLVF_B.transpose_d[CLVF_B.yElIdx + 8];
          CLVF_B.b[CLVF_B.ntIdx1] += CLVF_B.vc[3 * CLVF_B.uElOffset1 + 2] *
            CLVF_B.transpose_d[CLVF_B.yElIdx + 16];
        }
      }

      // End of Product: '<S33>/Product2'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        CLVF_B.RED_Tz_Wrist = CLVF_B.b[CLVF_B.uElOffset1 + 16] *
          CLVF_DW.BLACK_Tz + (CLVF_B.b[CLVF_B.uElOffset1 + 8] *
                              CLVF_B.RED_Tz_Shoulder +
                              CLVF_B.b[CLVF_B.uElOffset1] * CLVF_B.RED_Tz_Elbow);

        // Saturate: '<S27>/Remove Negatives' incorporates:
        //   DataStoreRead: '<S22>/BLACK_Tz'
        //   Product: '<S27>/Product'
        //   SignalConversion generated from: '<S27>/Product'

        if (CLVF_B.RED_Tz_Wrist > CLVF_P.RemoveNegatives_UpperSat) {
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
            CLVF_P.RemoveNegatives_UpperSat;
        } else if (CLVF_B.RED_Tz_Wrist < CLVF_P.RemoveNegatives_LowerSat) {
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
            CLVF_P.RemoveNegatives_LowerSat;
        } else {
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
            CLVF_B.RED_Tz_Wrist;
        }

        // End of Saturate: '<S27>/Remove Negatives'
      }

      // MATLAB Function: '<S27>/MATLAB Function2' incorporates:
      //   DataStoreRead: '<S22>/BLACK_Tz'
      //   Product: '<S27>/Product'
      //   SignalConversion generated from: '<S27>/Product'

      CLVF_MATLABFunction2(CLVF_B.rtb_RemoveNegatives_i_p,
                           &CLVF_B.sf_MATLABFunction2);

      // MATLAB Function: '<S27>/MATLAB Function1'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        CLVF_B.b[3 * CLVF_B.uElOffset1] = b_0[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1 + 1] = c[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1 + 2] =
          CLVF_P.thruster_dist2CG_BLACK[CLVF_B.uElOffset1] / 1000.0;
      }

      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx0 = CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1;
          CLVF_B.transpose_d[CLVF_B.ntIdx0] = 0.0;
          for (CLVF_B.ntIdx1 = 0; CLVF_B.ntIdx1 < 8; CLVF_B.ntIdx1++) {
            CLVF_B.transpose_d[CLVF_B.ntIdx0] += CLVF_B.b[3 * CLVF_B.ntIdx1 +
              CLVF_B.yElIdx] * CLVF_B.Mat2[(CLVF_B.uElOffset1 << 3) +
              CLVF_B.ntIdx1];
          }
        }
      }

      // Product: '<S27>/Product1'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
        CLVF_B.x_ddot_b[CLVF_B.uElOffset1] = 0.0;
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 8; CLVF_B.yElIdx++) {
          CLVF_B.x_ddot_b[CLVF_B.uElOffset1] += CLVF_B.transpose_d[3 *
            CLVF_B.yElIdx + CLVF_B.uElOffset1] *
            CLVF_B.sf_MATLABFunction2.ThrustPer_Final[CLVF_B.yElIdx];
        }
      }

      // End of Product: '<S27>/Product1'

      // DataStoreWrite: '<S27>/BLACK_Fx_Sat' incorporates:
      //   Product: '<S32>/Rotate F_b to F_I'

      CLVF_DW.BLACK_Fx_Sat = CLVF_B.sf_CreateRotationMatrix.C_Ib[0] *
        CLVF_B.x_ddot_b[0] + CLVF_B.sf_CreateRotationMatrix.C_Ib[2] *
        CLVF_B.x_ddot_b[1];

      // DataStoreWrite: '<S27>/BLACK_Fy_Sat' incorporates:
      //   Product: '<S32>/Rotate F_b to F_I'

      CLVF_DW.BLACK_Fy_Sat = CLVF_B.sf_CreateRotationMatrix.C_Ib[1] *
        CLVF_B.x_ddot_b[0] + CLVF_B.sf_CreateRotationMatrix.C_Ib[3] *
        CLVF_B.x_ddot_b[1];

      // DataStoreWrite: '<S27>/BLACK_Tz_Sat'
      CLVF_DW.BLACK_Tz_Sat = CLVF_B.x_ddot_b[2];

      // End of Outputs for SubSystem: '<S3>/Change BLACK Behavior'
    }

    // End of If: '<S3>/This IF block determines whether or not to run the BLACK sim//exp' 

    // If: '<S3>/This IF block determines whether or not to run the BLUE sim//exp' incorporates:
    //   Constant: '<S25>/Constant'
    //   Constant: '<S3>/Constant'
    //   DataStoreRead: '<S23>/BLUE_Fx'
    //   DataStoreRead: '<S23>/BLUE_Fy'
    //   Product: '<S37>/Rotate F_I to F_b'
    //   SignalConversion generated from: '<S37>/Rotate F_I to F_b'

    if ((CLVF_P.WhoAmI == 3.0) || (CLVF_P.simMode == 1.0)) {
      // Outputs for IfAction SubSystem: '<S3>/Change BLUE Behavior' incorporates:
      //   ActionPort: '<S23>/Action Port'

      // MATLAB Function: '<S41>/Create Rotation Matrix' incorporates:
      //   DataStoreRead: '<S36>/BLUE_Rz'

      CLVF_CreateRotationMatrix(CLVF_DW.BLUE_Rz,
        &CLVF_B.sf_CreateRotationMatrix_i);

      // MATLAB Function: '<S36>/MATLAB Function1'
      CLVF_MATLABFunction(&CLVF_B.sf_MATLABFunction1_b);

      // MATLAB Function: '<S36>/MATLAB Function'
      CLVF_MATLABFunction(&CLVF_B.sf_MATLABFunction_i);

      // PermuteDimensions: '<S42>/transpose'
      CLVF_B.yElIdx = 0;
      CLVF_B.uElOffset1 = 0;
      for (CLVF_B.ntIdx1 = 0; CLVF_B.ntIdx1 < 3; CLVF_B.ntIdx1++) {
        CLVF_B.uElOffset0 = CLVF_B.uElOffset1;
        for (CLVF_B.ntIdx0 = 0; CLVF_B.ntIdx0 < 8; CLVF_B.ntIdx0++) {
          CLVF_B.transpose_p[CLVF_B.yElIdx] =
            CLVF_B.sf_MATLABFunction_i.H[CLVF_B.uElOffset0];
          CLVF_B.yElIdx++;
          CLVF_B.uElOffset0 += 3;
        }

        CLVF_B.uElOffset1++;
      }

      // End of PermuteDimensions: '<S42>/transpose'

      // MATLAB Function: '<S37>/Create Rotation Matrix' incorporates:
      //   DataStoreRead: '<S23>/BLUE_Rz'

      CLVF_CreateRotationMatrix_h(CLVF_DW.BLUE_Rz,
        &CLVF_B.sf_CreateRotationMatrix_f);

      // Product: '<S42>/Product'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx1 = CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1;
          CLVF_B.vc[CLVF_B.ntIdx1] = 0.0;
          for (CLVF_B.ntIdx0 = 0; CLVF_B.ntIdx0 < 8; CLVF_B.ntIdx0++) {
            CLVF_B.vc[CLVF_B.ntIdx1] += CLVF_B.sf_MATLABFunction_i.H[3 *
              CLVF_B.ntIdx0 + CLVF_B.yElIdx] * CLVF_B.transpose_p
              [(CLVF_B.uElOffset1 << 3) + CLVF_B.ntIdx0];
          }
        }
      }

      // Product: '<S42>/Product1' incorporates:
      //   Product: '<S42>/Product'

      CLVF_rt_invd3x3_snf(CLVF_B.vc, CLVF_B.C_IB_c);
      CLVF_B.RED_Tz_Elbow = CLVF_B.sf_CreateRotationMatrix_f.C_bI[0] *
        CLVF_DW.BLUE_Fx + CLVF_B.sf_CreateRotationMatrix_f.C_bI[2] *
        CLVF_DW.BLUE_Fy;

      // Product: '<S37>/Rotate F_I to F_b' incorporates:
      //   DataStoreRead: '<S23>/BLUE_Fx'
      //   DataStoreRead: '<S23>/BLUE_Fy'
      //   SignalConversion generated from: '<S37>/Rotate F_I to F_b'

      CLVF_B.RED_Tz_Shoulder = CLVF_B.sf_CreateRotationMatrix_f.C_bI[1] *
        CLVF_DW.BLUE_Fx + CLVF_B.sf_CreateRotationMatrix_f.C_bI[3] *
        CLVF_DW.BLUE_Fy;

      // Product: '<S42>/Product2'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 8; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx1 = CLVF_B.yElIdx + (CLVF_B.uElOffset1 << 3);
          CLVF_B.b[CLVF_B.ntIdx1] = 0.0;
          CLVF_B.b[CLVF_B.ntIdx1] += CLVF_B.C_IB_c[3 * CLVF_B.uElOffset1] *
            CLVF_B.transpose_p[CLVF_B.yElIdx];
          CLVF_B.b[CLVF_B.ntIdx1] += CLVF_B.C_IB_c[3 * CLVF_B.uElOffset1 + 1] *
            CLVF_B.transpose_p[CLVF_B.yElIdx + 8];
          CLVF_B.b[CLVF_B.ntIdx1] += CLVF_B.C_IB_c[3 * CLVF_B.uElOffset1 + 2] *
            CLVF_B.transpose_p[CLVF_B.yElIdx + 16];
        }
      }

      // End of Product: '<S42>/Product2'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        CLVF_B.RED_Tz_Wrist = CLVF_B.b[CLVF_B.uElOffset1 + 16] * CLVF_DW.BLUE_Tz
          + (CLVF_B.b[CLVF_B.uElOffset1 + 8] * CLVF_B.RED_Tz_Shoulder +
             CLVF_B.b[CLVF_B.uElOffset1] * CLVF_B.RED_Tz_Elbow);

        // Saturate: '<S36>/Remove Negatives' incorporates:
        //   DataStoreRead: '<S23>/BLUE_Tz'
        //   Product: '<S36>/Product'
        //   SignalConversion generated from: '<S36>/Product'

        if (CLVF_B.RED_Tz_Wrist > CLVF_P.RemoveNegatives_UpperSat_p) {
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
            CLVF_P.RemoveNegatives_UpperSat_p;
        } else if (CLVF_B.RED_Tz_Wrist < CLVF_P.RemoveNegatives_LowerSat_c) {
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
            CLVF_P.RemoveNegatives_LowerSat_c;
        } else {
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
            CLVF_B.RED_Tz_Wrist;
        }

        // End of Saturate: '<S36>/Remove Negatives'
      }

      // MATLAB Function: '<S36>/MATLAB Function2' incorporates:
      //   DataStoreRead: '<S23>/BLUE_Tz'
      //   Product: '<S36>/Product'
      //   SignalConversion generated from: '<S36>/Product'

      CLVF_MATLABFunction2(CLVF_B.rtb_RemoveNegatives_i_p,
                           &CLVF_B.sf_MATLABFunction2_n);

      // Product: '<S36>/Product1'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
        CLVF_B.x_ddot_b[CLVF_B.uElOffset1] = 0.0;
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 8; CLVF_B.yElIdx++) {
          CLVF_B.x_ddot_b[CLVF_B.uElOffset1] += CLVF_B.sf_MATLABFunction1_b.H[3 *
            CLVF_B.yElIdx + CLVF_B.uElOffset1] *
            CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[CLVF_B.yElIdx];
        }
      }

      // End of Product: '<S36>/Product1'

      // DataStoreWrite: '<S36>/BLUE_Fx_Sat' incorporates:
      //   Product: '<S41>/Rotate F_b to F_I'

      CLVF_DW.BLUE_Fx_Sat = CLVF_B.sf_CreateRotationMatrix_i.C_Ib[0] *
        CLVF_B.x_ddot_b[0] + CLVF_B.sf_CreateRotationMatrix_i.C_Ib[2] *
        CLVF_B.x_ddot_b[1];

      // DataStoreWrite: '<S36>/BLUE_Fy_Sat' incorporates:
      //   Product: '<S41>/Rotate F_b to F_I'

      CLVF_DW.BLUE_Fy_Sat = CLVF_B.sf_CreateRotationMatrix_i.C_Ib[1] *
        CLVF_B.x_ddot_b[0] + CLVF_B.sf_CreateRotationMatrix_i.C_Ib[3] *
        CLVF_B.x_ddot_b[1];

      // DataStoreWrite: '<S36>/BLUE_Tz_Sat'
      CLVF_DW.BLUE_Tz_Sat = CLVF_B.x_ddot_b[2];

      // End of Outputs for SubSystem: '<S3>/Change BLUE Behavior'
    }

    // End of If: '<S3>/This IF block determines whether or not to run the BLUE sim//exp' 

    // If: '<S3>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
    //   Constant: '<S25>/Constant'
    //   Constant: '<S3>/Constant'
    //   DataStoreRead: '<S24>/RED_Fx'
    //   DataStoreRead: '<S24>/RED_Fy'
    //   Product: '<S46>/Rotate F_I to F_b'
    //   SignalConversion generated from: '<S46>/Rotate F_I to F_b'

    if ((CLVF_P.WhoAmI == 1.0) || (CLVF_P.simMode == 1.0)) {
      // Outputs for IfAction SubSystem: '<S3>/Change RED Behavior' incorporates:
      //   ActionPort: '<S24>/Action Port'

      // MATLAB Function: '<S50>/Create Rotation Matrix' incorporates:
      //   DataStoreRead: '<S45>/RED_Rz'

      CLVF_CreateRotationMatrix(CLVF_DW.RED_Rz,
        &CLVF_B.sf_CreateRotationMatrix_l);

      // MATLAB Function: '<S45>/MATLAB Function1'
      // MATLAB Function 'From Force//Torque to PWM Signal/Change RED Behavior/Calculate Thruster  ON//OFF/MATLAB Function1': '<S48>:1' 
      // '<S48>:1:3' Vec1 = [ -1
      // '<S48>:1:4'          -1
      // '<S48>:1:5'           0
      // '<S48>:1:6'           0
      // '<S48>:1:7'           1
      // '<S48>:1:8'           1
      // '<S48>:1:9'           0
      // '<S48>:1:10'           0 ];
      // '<S48>:1:12' Vec2 = [  0
      // '<S48>:1:13'           0
      // '<S48>:1:14'           1
      // '<S48>:1:15'           1
      // '<S48>:1:16'           0
      // '<S48>:1:17'           0
      // '<S48>:1:18'          -1
      // '<S48>:1:19'          -1 ];
      // '<S48>:1:21' Vec3 = thruster_dist2CG_RED./1000;
      // '<S48>:1:23' Mat1 = [Vec1, Vec2, Vec3]';
      // '<S48>:1:25' Mat2 = diag((F_thrusters_RED));
      memset(&CLVF_B.Mat2[0], 0, sizeof(real_T) << 6U);

      // '<S48>:1:27' H    = Mat1*Mat2;
      // MATLAB Function 'From Force//Torque to PWM Signal/Change RED Behavior/Calculate Thruster  ON//OFF/MATLAB Function': '<S47>:1' 
      // '<S47>:1:3' Vec1 = [ -1
      // '<S47>:1:4'          -1
      // '<S47>:1:5'           0
      // '<S47>:1:6'           0
      // '<S47>:1:7'           1
      // '<S47>:1:8'           1
      // '<S47>:1:9'           0
      // '<S47>:1:10'           0 ];
      // '<S47>:1:12' Vec2 = [  0
      // '<S47>:1:13'           0
      // '<S47>:1:14'           1
      // '<S47>:1:15'           1
      // '<S47>:1:16'           0
      // '<S47>:1:17'           0
      // '<S47>:1:18'          -1
      // '<S47>:1:19'          -1 ];
      // '<S47>:1:21' Vec3 = thruster_dist2CG_RED./1000;
      // '<S47>:1:23' Mat1 = [Vec1, Vec2, Vec3]';
      // '<S47>:1:25' Mat2 = diag((F_thrusters_RED./2));
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        CLVF_B.Mat2[CLVF_B.uElOffset1 + (CLVF_B.uElOffset1 << 3)] =
          CLVF_P.F_thrusters_RED[CLVF_B.uElOffset1];

        // MATLAB Function: '<S45>/MATLAB Function'
        CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
          CLVF_P.F_thrusters_RED[CLVF_B.uElOffset1] / 2.0;
      }

      // MATLAB Function: '<S45>/MATLAB Function'
      memset(&CLVF_B.Mat2_m[0], 0, sizeof(real_T) << 6U);

      // '<S47>:1:27' H    = Mat1*Mat2;
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        CLVF_B.Mat2_m[CLVF_B.uElOffset1 + (CLVF_B.uElOffset1 << 3)] =
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1] = b_0[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1 + 1] = c[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1 + 2] =
          CLVF_P.thruster_dist2CG_RED[CLVF_B.uElOffset1] / 1000.0;
      }

      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx0 = CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1;
          CLVF_B.H_bu[CLVF_B.ntIdx0] = 0.0;
          for (CLVF_B.ntIdx1 = 0; CLVF_B.ntIdx1 < 8; CLVF_B.ntIdx1++) {
            CLVF_B.H_bu[CLVF_B.ntIdx0] += CLVF_B.b[3 * CLVF_B.ntIdx1 +
              CLVF_B.yElIdx] * CLVF_B.Mat2_m[(CLVF_B.uElOffset1 << 3) +
              CLVF_B.ntIdx1];
          }
        }
      }

      // PermuteDimensions: '<S51>/transpose'
      CLVF_B.yElIdx = 0;
      CLVF_B.uElOffset1 = 0;
      for (CLVF_B.ntIdx1 = 0; CLVF_B.ntIdx1 < 3; CLVF_B.ntIdx1++) {
        CLVF_B.uElOffset0 = CLVF_B.uElOffset1;
        for (CLVF_B.ntIdx0 = 0; CLVF_B.ntIdx0 < 8; CLVF_B.ntIdx0++) {
          CLVF_B.transpose[CLVF_B.yElIdx] = CLVF_B.H_bu[CLVF_B.uElOffset0];
          CLVF_B.yElIdx++;
          CLVF_B.uElOffset0 += 3;
        }

        CLVF_B.uElOffset1++;
      }

      // End of PermuteDimensions: '<S51>/transpose'

      // MATLAB Function: '<S46>/Create Rotation Matrix' incorporates:
      //   DataStoreRead: '<S24>/RED_Rz'

      CLVF_CreateRotationMatrix_h(CLVF_DW.RED_Rz,
        &CLVF_B.sf_CreateRotationMatrix_n);

      // Product: '<S51>/Product'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx0 = CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1;
          CLVF_B.C_IB[CLVF_B.ntIdx0] = 0.0;
          for (CLVF_B.ntIdx1 = 0; CLVF_B.ntIdx1 < 8; CLVF_B.ntIdx1++) {
            CLVF_B.C_IB[CLVF_B.ntIdx0] += CLVF_B.H_bu[3 * CLVF_B.ntIdx1 +
              CLVF_B.yElIdx] * CLVF_B.transpose[(CLVF_B.uElOffset1 << 3) +
              CLVF_B.ntIdx1];
          }
        }
      }

      // Product: '<S51>/Product1' incorporates:
      //   Product: '<S51>/Product'

      CLVF_rt_invd3x3_snf(CLVF_B.C_IB, CLVF_B.vc);
      CLVF_B.RED_Tz_Elbow = CLVF_B.sf_CreateRotationMatrix_n.C_bI[0] *
        CLVF_DW.RED_Fx + CLVF_B.sf_CreateRotationMatrix_n.C_bI[2] *
        CLVF_DW.RED_Fy;

      // Product: '<S46>/Rotate F_I to F_b' incorporates:
      //   DataStoreRead: '<S24>/RED_Fx'
      //   DataStoreRead: '<S24>/RED_Fy'
      //   SignalConversion generated from: '<S46>/Rotate F_I to F_b'

      CLVF_B.RED_Tz_Shoulder = CLVF_B.sf_CreateRotationMatrix_n.C_bI[1] *
        CLVF_DW.RED_Fx + CLVF_B.sf_CreateRotationMatrix_n.C_bI[3] *
        CLVF_DW.RED_Fy;

      // Product: '<S51>/Product2'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 8; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx1 = CLVF_B.yElIdx + (CLVF_B.uElOffset1 << 3);
          CLVF_B.transpose_p[CLVF_B.ntIdx1] = 0.0;
          CLVF_B.transpose_p[CLVF_B.ntIdx1] += CLVF_B.vc[3 * CLVF_B.uElOffset1] *
            CLVF_B.transpose[CLVF_B.yElIdx];
          CLVF_B.transpose_p[CLVF_B.ntIdx1] += CLVF_B.vc[3 * CLVF_B.uElOffset1 +
            1] * CLVF_B.transpose[CLVF_B.yElIdx + 8];
          CLVF_B.transpose_p[CLVF_B.ntIdx1] += CLVF_B.vc[3 * CLVF_B.uElOffset1 +
            2] * CLVF_B.transpose[CLVF_B.yElIdx + 16];
        }
      }

      // End of Product: '<S51>/Product2'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        CLVF_B.RED_Tz_Wrist = CLVF_B.transpose_p[CLVF_B.uElOffset1 + 16] *
          CLVF_DW.RED_Tz + (CLVF_B.transpose_p[CLVF_B.uElOffset1 + 8] *
                            CLVF_B.RED_Tz_Shoulder +
                            CLVF_B.transpose_p[CLVF_B.uElOffset1] *
                            CLVF_B.RED_Tz_Elbow);

        // Saturate: '<S45>/Remove Negatives' incorporates:
        //   DataStoreRead: '<S24>/RED_Tz'
        //   Product: '<S45>/Product'
        //   SignalConversion generated from: '<S45>/Product'

        if (CLVF_B.RED_Tz_Wrist > CLVF_P.RemoveNegatives_UpperSat_d) {
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
            CLVF_P.RemoveNegatives_UpperSat_d;
        } else if (CLVF_B.RED_Tz_Wrist < CLVF_P.RemoveNegatives_LowerSat_a) {
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
            CLVF_P.RemoveNegatives_LowerSat_a;
        } else {
          CLVF_B.rtb_RemoveNegatives_i_p[CLVF_B.uElOffset1] =
            CLVF_B.RED_Tz_Wrist;
        }

        // End of Saturate: '<S45>/Remove Negatives'
      }

      // MATLAB Function: '<S45>/MATLAB Function2' incorporates:
      //   DataStoreRead: '<S24>/RED_Tz'
      //   Product: '<S45>/Product'
      //   SignalConversion generated from: '<S45>/Product'

      CLVF_MATLABFunction2(CLVF_B.rtb_RemoveNegatives_i_p,
                           &CLVF_B.sf_MATLABFunction2_l);

      // MATLAB Function: '<S45>/MATLAB Function1'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        CLVF_B.b[3 * CLVF_B.uElOffset1] = b_0[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1 + 1] = c[CLVF_B.uElOffset1];
        CLVF_B.b[3 * CLVF_B.uElOffset1 + 2] =
          CLVF_P.thruster_dist2CG_RED[CLVF_B.uElOffset1] / 1000.0;
      }

      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 8; CLVF_B.uElOffset1++) {
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 3; CLVF_B.yElIdx++) {
          CLVF_B.ntIdx0 = CLVF_B.yElIdx + 3 * CLVF_B.uElOffset1;
          CLVF_B.transpose_d[CLVF_B.ntIdx0] = 0.0;
          for (CLVF_B.ntIdx1 = 0; CLVF_B.ntIdx1 < 8; CLVF_B.ntIdx1++) {
            CLVF_B.transpose_d[CLVF_B.ntIdx0] += CLVF_B.b[3 * CLVF_B.ntIdx1 +
              CLVF_B.yElIdx] * CLVF_B.Mat2[(CLVF_B.uElOffset1 << 3) +
              CLVF_B.ntIdx1];
          }
        }
      }

      // Product: '<S45>/Product1'
      for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
        CLVF_B.x_ddot_b[CLVF_B.uElOffset1] = 0.0;
        for (CLVF_B.yElIdx = 0; CLVF_B.yElIdx < 8; CLVF_B.yElIdx++) {
          CLVF_B.x_ddot_b[CLVF_B.uElOffset1] += CLVF_B.transpose_d[3 *
            CLVF_B.yElIdx + CLVF_B.uElOffset1] *
            CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[CLVF_B.yElIdx];
        }
      }

      // End of Product: '<S45>/Product1'

      // DataStoreWrite: '<S45>/RED_Fx_Sat' incorporates:
      //   Product: '<S50>/Rotate F_b to F_I'

      CLVF_DW.RED_Fx_Sat = CLVF_B.sf_CreateRotationMatrix_l.C_Ib[0] *
        CLVF_B.x_ddot_b[0] + CLVF_B.sf_CreateRotationMatrix_l.C_Ib[2] *
        CLVF_B.x_ddot_b[1];

      // DataStoreWrite: '<S45>/RED_Fy_Sat' incorporates:
      //   Product: '<S50>/Rotate F_b to F_I'

      CLVF_DW.RED_Fy_Sat = CLVF_B.sf_CreateRotationMatrix_l.C_Ib[1] *
        CLVF_B.x_ddot_b[0] + CLVF_B.sf_CreateRotationMatrix_l.C_Ib[3] *
        CLVF_B.x_ddot_b[1];

      // DataStoreWrite: '<S45>/RED_Tz_Sat'
      CLVF_DW.RED_Tz_Sat = CLVF_B.x_ddot_b[2];

      // End of Outputs for SubSystem: '<S3>/Change RED Behavior'
    }

    // End of If: '<S3>/This IF block determines whether or not to run the RED sim//exp ' 

    // RateTransition: '<S3>/Rate Transition' incorporates:
    //   RateTransition: '<S3>/Rate Transition1'
    //   RateTransition: '<S3>/Rate Transition3'
    //   RateTransition: '<S3>/Rate Transition4'
    //   RateTransition: '<S3>/Rate Transition5'
    //   RateTransition: '<S3>/Rate Transition6'
    //   RateTransition: '<S3>/Rate Transition7'
    //   RateTransition: '<S3>/Rate Transition8'
    //   Sum: '<S3>/Sum'
    //   Sum: '<S3>/Sum1'
    //   Sum: '<S3>/Sum2'
    //   Sum: '<S3>/Sum3'
    //   Sum: '<S3>/Sum4'
    //   Sum: '<S3>/Sum5'
    //   Sum: '<S3>/Sum6'
    //   Sum: '<S3>/Sum7'

    if (CLVF_M->Timing.RateInteraction.TID1_2) {
      CLVF_DW.RateTransition_Buffer =
        (CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[0] +
         CLVF_B.sf_MATLABFunction2.ThrustPer_Final[7]) +
        CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[7];
      CLVF_DW.RateTransition1_Buffer =
        (CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[1] +
         CLVF_B.sf_MATLABFunction2.ThrustPer_Final[2]) +
        CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[2];

      // RateTransition: '<S3>/Rate Transition2' incorporates:
      //   Sum: '<S3>/Sum'
      //   Sum: '<S3>/Sum1'

      CLVF_DW.RateTransition2_Buffer =
        CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[2];
      CLVF_DW.RateTransition3_Buffer =
        (CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[3] +
         CLVF_B.sf_MATLABFunction2.ThrustPer_Final[4]) +
        CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[4];
      CLVF_DW.RateTransition4_Buffer =
        (CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[4] +
         CLVF_B.sf_MATLABFunction2.ThrustPer_Final[3]) +
        CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[3];
      CLVF_DW.RateTransition5_Buffer =
        (CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[5] +
         CLVF_B.sf_MATLABFunction2.ThrustPer_Final[6]) +
        CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[6];
      CLVF_DW.RateTransition6_Buffer =
        (CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[6] +
         CLVF_B.sf_MATLABFunction2.ThrustPer_Final[5]) +
        CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[5];
      CLVF_DW.RateTransition7_Buffer =
        (CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[7] +
         CLVF_B.sf_MATLABFunction2.ThrustPer_Final[0]) +
        CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[0];
      CLVF_DW.RateTransition8_Buffer =
        CLVF_B.sf_MATLABFunction2.ThrustPer_Final[1] +
        CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[1];
    }

    // End of RateTransition: '<S3>/Rate Transition'

    // If: '<S5>/If performing an experiment, grab the PhaseSpace data. Otherwise, use a clock to set time in SIM.' incorporates:
    //   Constant: '<S74>/Constant'

    CLVF_B.rtPrevAction = CLVF_DW.Ifperforminganexperimentgrabthe;
    CLVF_B.rtAction = -1;
    if (CLVF_P.simMode == 0.0) {
      CLVF_B.rtAction = 0;
    } else {
      if (CLVF_P.simMode == 1.0) {
        CLVF_B.rtAction = 1;
      }
    }

    CLVF_DW.Ifperforminganexperimentgrabthe = CLVF_B.rtAction;
    if ((CLVF_B.rtPrevAction != CLVF_B.rtAction) && (CLVF_B.rtPrevAction == 0))
    {
      // Disable for If: '<S75>/Check whether both platforms are being used, and if so then use RED to send data to BLACK ' 
      switch (CLVF_DW.Checkwhetherbothplatformsarebei) {
       case 0:
        // Disable for Enabled SubSystem: '<S77>/Enabled Subsystem1'
        if (CLVF_DW.EnabledSubsystem1_MODE_d) {
          CLVF_DW.EnabledSubsystem1_MODE_d = false;
        }

        // End of Disable for SubSystem: '<S77>/Enabled Subsystem1'

        // Disable for Enabled SubSystem: '<S138>/Enabled Subsystem'
        if (CLVF_DW.EnabledSubsystem_MODE_l) {
          CLVF_DW.EnabledSubsystem_MODE_l = false;
        }

        // End of Disable for SubSystem: '<S138>/Enabled Subsystem'
        break;

       case 1:
        // Disable for If: '<S76>/This IF block determines whether or not to run the BLACK sim//exp' 
        CLVF_DW.ThisIFblockdetermineswhethero_c = -1;

        // Disable for If: '<S76>/This IF block determines whether or not to run the RED sim//exp ' 
        if (CLVF_DW.ThisIFblockdetermineswhether_cx == 0) {
          // Disable for Enabled SubSystem: '<S80>/Enabled Subsystem'
          if (CLVF_DW.EnabledSubsystem_MODE) {
            CLVF_DW.EnabledSubsystem_MODE = false;
          }

          // End of Disable for SubSystem: '<S80>/Enabled Subsystem'

          // Disable for Enabled SubSystem: '<S80>/Enabled Subsystem1'
          if (CLVF_DW.EnabledSubsystem1_MODE) {
            CLVF_DW.EnabledSubsystem1_MODE = false;
          }

          // End of Disable for SubSystem: '<S80>/Enabled Subsystem1'

          // Disable for Enabled SubSystem: '<S106>/Enabled Subsystem'
          if (CLVF_DW.EnabledSubsystem_MODE_p) {
            CLVF_DW.EnabledSubsystem_MODE_p = false;
          }

          // End of Disable for SubSystem: '<S106>/Enabled Subsystem'
        }

        CLVF_DW.ThisIFblockdetermineswhether_cx = -1;
        break;
      }

      CLVF_DW.Checkwhetherbothplatformsarebei = -1;

      // End of Disable for If: '<S75>/Check whether both platforms are being used, and if so then use RED to send data to BLACK ' 
    }

    switch (CLVF_B.rtAction) {
     case 0:
      // Outputs for IfAction SubSystem: '<S5>/Use Hardware to Obtain States' incorporates:
      //   ActionPort: '<S75>/Action Port'

      // If: '<S75>/Check whether both platforms are being used, and if so then use RED to send data to BLACK ' incorporates:
      //   Clock: '<S77>/Clock'
      //   Constant: '<S75>/Which PLATFORM is being used?'
      //   Delay: '<S153>/Delay'
      //   Inport: '<S139>/In1'
      //   Inport: '<S140>/In1'
      //   Inport: '<S154>/In1'
      //   Logic: '<S153>/Logical Operator'
      //   Logic: '<S153>/Logical Operator1'

      CLVF_B.rtPrevAction = CLVF_DW.Checkwhetherbothplatformsarebei;
      CLVF_B.rtAction = -1;
      if ((CLVF_P.platformSelection == 1.0) || (CLVF_P.platformSelection == 2.0)
          || (CLVF_P.platformSelection == 5.0)) {
        CLVF_B.rtAction = 0;
      } else {
        if ((CLVF_P.platformSelection == 3.0) || (CLVF_P.platformSelection ==
             4.0)) {
          CLVF_B.rtAction = 1;
        }
      }

      CLVF_DW.Checkwhetherbothplatformsarebei = CLVF_B.rtAction;
      if (CLVF_B.rtPrevAction != CLVF_B.rtAction) {
        switch (CLVF_B.rtPrevAction) {
         case 0:
          // Disable for Enabled SubSystem: '<S77>/Enabled Subsystem1'
          if (CLVF_DW.EnabledSubsystem1_MODE_d) {
            CLVF_DW.EnabledSubsystem1_MODE_d = false;
          }

          // End of Disable for SubSystem: '<S77>/Enabled Subsystem1'

          // Disable for Enabled SubSystem: '<S138>/Enabled Subsystem'
          if (CLVF_DW.EnabledSubsystem_MODE_l) {
            CLVF_DW.EnabledSubsystem_MODE_l = false;
          }

          // End of Disable for SubSystem: '<S138>/Enabled Subsystem'
          break;

         case 1:
          // Disable for If: '<S76>/This IF block determines whether or not to run the BLACK sim//exp' 
          CLVF_DW.ThisIFblockdetermineswhethero_c = -1;

          // Disable for If: '<S76>/This IF block determines whether or not to run the RED sim//exp ' 
          if (CLVF_DW.ThisIFblockdetermineswhether_cx == 0) {
            // Disable for Enabled SubSystem: '<S80>/Enabled Subsystem'
            if (CLVF_DW.EnabledSubsystem_MODE) {
              CLVF_DW.EnabledSubsystem_MODE = false;
            }

            // End of Disable for SubSystem: '<S80>/Enabled Subsystem'

            // Disable for Enabled SubSystem: '<S80>/Enabled Subsystem1'
            if (CLVF_DW.EnabledSubsystem1_MODE) {
              CLVF_DW.EnabledSubsystem1_MODE = false;
            }

            // End of Disable for SubSystem: '<S80>/Enabled Subsystem1'

            // Disable for Enabled SubSystem: '<S106>/Enabled Subsystem'
            if (CLVF_DW.EnabledSubsystem_MODE_p) {
              CLVF_DW.EnabledSubsystem_MODE_p = false;
            }

            // End of Disable for SubSystem: '<S106>/Enabled Subsystem'
          }

          CLVF_DW.ThisIFblockdetermineswhether_cx = -1;
          break;
        }
      }

      switch (CLVF_B.rtAction) {
       case 0:
        // Outputs for IfAction SubSystem: '<S75>/Using RED, BLACK, BLUE, or RED + ARM' incorporates:
        //   ActionPort: '<S77>/Action Port'

        // RelationalOperator: '<S137>/Compare' incorporates:
        //   Constant: '<S137>/Constant'

        for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 13; CLVF_B.uElOffset1++)
        {
          CLVF_B.Compare_e[CLVF_B.uElOffset1] = (0.0 != CLVF_P.Constant_Value);
        }

        // End of RelationalOperator: '<S137>/Compare'

        // Outputs for Enabled SubSystem: '<S77>/Enabled Subsystem1' incorporates:
        //   EnablePort: '<S140>/Enable'

        // Outputs for Enabled SubSystem: '<S77>/Enabled Subsystem' incorporates:
        //   EnablePort: '<S139>/Enable'

        CLVF_B.rtb_Compare_dy = (CLVF_B.Compare_e[0] || CLVF_B.Compare_e[1] ||
          CLVF_B.Compare_e[2] || CLVF_B.Compare_e[3] || CLVF_B.Compare_e[4] ||
          CLVF_B.Compare_e[5] || CLVF_B.Compare_e[6] || CLVF_B.Compare_e[7] ||
          CLVF_B.Compare_e[8] || CLVF_B.Compare_e[9] || CLVF_B.Compare_e[10] ||
          CLVF_B.Compare_e[11]);

        // End of Outputs for SubSystem: '<S77>/Enabled Subsystem1'
        if (CLVF_B.rtb_Compare_dy || CLVF_B.Compare_e[12]) {
          memset(&CLVF_B.In1_ak[0], 0, 13U * sizeof(real_T));
        }

        // End of Outputs for SubSystem: '<S77>/Enabled Subsystem'

        // DataStoreWrite: '<S77>/Data Store Write3' incorporates:
        //   Inport: '<S139>/In1'

        CLVF_DW.ARM_Elbow_Px = CLVF_B.In1_ak[7];

        // DataStoreWrite: '<S77>/Data Store Write4'
        CLVF_DW.ARM_Elbow_Py = CLVF_B.In1_ak[8];

        // DataStoreWrite: '<S77>/Data Store Write5'
        CLVF_DW.ARM_Wrist_Px = CLVF_B.In1_ak[9];

        // DataStoreWrite: '<S77>/Data Store Write6'
        CLVF_DW.ARM_Wrist_Py = CLVF_B.In1_ak[10];

        // DataStoreWrite: '<S77>/RED_Px1'
        CLVF_DW.RED_Px = CLVF_B.In1_ak[1];

        // Outputs for Enabled SubSystem: '<S77>/Enabled Subsystem1' incorporates:
        //   EnablePort: '<S140>/Enable'

        if (CLVF_B.rtb_Compare_dy || CLVF_B.Compare_e[12]) {
          if (!CLVF_DW.EnabledSubsystem1_MODE_d) {
            CLVF_DW.EnabledSubsystem1_MODE_d = true;
          }
        } else {
          if (CLVF_DW.EnabledSubsystem1_MODE_d) {
            CLVF_DW.EnabledSubsystem1_MODE_d = false;
          }
        }

        if (CLVF_DW.EnabledSubsystem1_MODE_d) {
          CLVF_B.In1_oz = CLVF_M->Timing.t[0];
        }

        // End of Outputs for SubSystem: '<S77>/Enabled Subsystem1'

        // DataTypeConversion: '<S153>/Data Type Conversion' incorporates:
        //   Clock: '<S77>/Clock'
        //   Inport: '<S140>/In1'

        CLVF_B.DataTypeConversion_o = (CLVF_B.In1_oz != 0.0);

        // Outputs for Enabled SubSystem: '<S138>/Enabled Subsystem' incorporates:
        //   EnablePort: '<S154>/Enable'

        if (CLVF_B.DataTypeConversion_o && (!CLVF_DW.Delay_DSTATE_m)) {
          if (!CLVF_DW.EnabledSubsystem_MODE_l) {
            CLVF_DW.EnabledSubsystem_MODE_l = true;
          }
        } else {
          if (CLVF_DW.EnabledSubsystem_MODE_l) {
            CLVF_DW.EnabledSubsystem_MODE_l = false;
          }
        }

        if (CLVF_DW.EnabledSubsystem_MODE_l) {
          CLVF_B.In1_g5 = CLVF_B.In1_oz;
        }

        // End of Outputs for SubSystem: '<S138>/Enabled Subsystem'

        // Sum: '<S138>/Subtract' incorporates:
        //   DataStoreWrite: '<S77>/RED_Px10'
        //   Delay: '<S153>/Delay'
        //   Inport: '<S154>/In1'
        //   Logic: '<S153>/Logical Operator'
        //   Logic: '<S153>/Logical Operator1'

        CLVF_DW.Univ_Time = CLVF_B.In1_oz - CLVF_B.In1_g5;

        // Delay: '<S147>/Delay1'
        if (CLVF_DW.icLoad_po != 0) {
          CLVF_DW.Delay1_DSTATE_ds = CLVF_B.In1_ak[4];
        }

        // Sum: '<S147>/Sum6' incorporates:
        //   Delay: '<S147>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.In1_ak[4] - CLVF_DW.Delay1_DSTATE_ds;

        // If: '<S147>/if we went through a "step"' incorporates:
        //   Inport: '<S163>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S147>/Hold this value' incorporates:
          //   ActionPort: '<S163>/Action Port'

          CLVF_B.In1_gi = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S147>/Hold this value'
        }

        // End of If: '<S147>/if we went through a "step"'

        // Gain: '<S147>/divide by delta T'
        CLVF_B.dividebydeltaT_d = 1.0 / CLVF_P.serverRate * CLVF_B.In1_gi;

        // Delay: '<S142>/Delay1'
        if (CLVF_DW.icLoad_nx2 != 0) {
          CLVF_DW.Delay1_DSTATE_bx = CLVF_B.dividebydeltaT_d;
        }

        // Sum: '<S142>/Sum6' incorporates:
        //   Delay: '<S142>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_d - CLVF_DW.Delay1_DSTATE_bx;

        // If: '<S142>/if we went through a "step"' incorporates:
        //   Inport: '<S158>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S142>/Hold this value' incorporates:
          //   ActionPort: '<S158>/Action Port'

          CLVF_B.In1_lu = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S142>/Hold this value'
        }

        // End of If: '<S142>/if we went through a "step"'

        // Gain: '<S142>/divide by delta T' incorporates:
        //   DataStoreWrite: '<S77>/RED_Px11'

        CLVF_DW.BLACK_Ax = 1.0 / CLVF_P.serverRate * CLVF_B.In1_lu;

        // Delay: '<S148>/Delay1'
        if (CLVF_DW.icLoad_on != 0) {
          CLVF_DW.Delay1_DSTATE_bq = CLVF_B.In1_ak[5];
        }

        // Sum: '<S148>/Sum6' incorporates:
        //   Delay: '<S148>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.In1_ak[5] - CLVF_DW.Delay1_DSTATE_bq;

        // If: '<S148>/if we went through a "step"' incorporates:
        //   Inport: '<S164>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S148>/Hold this value' incorporates:
          //   ActionPort: '<S164>/Action Port'

          CLVF_B.In1_oq = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S148>/Hold this value'
        }

        // End of If: '<S148>/if we went through a "step"'

        // Gain: '<S148>/divide by delta T'
        CLVF_B.dividebydeltaT_ka = 1.0 / CLVF_P.serverRate * CLVF_B.In1_oq;

        // Delay: '<S143>/Delay1'
        if (CLVF_DW.icLoad_lw != 0) {
          CLVF_DW.Delay1_DSTATE_n2 = CLVF_B.dividebydeltaT_ka;
        }

        // Sum: '<S143>/Sum6' incorporates:
        //   Delay: '<S143>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_ka -
          CLVF_DW.Delay1_DSTATE_n2;

        // If: '<S143>/if we went through a "step"' incorporates:
        //   Inport: '<S159>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S143>/Hold this value' incorporates:
          //   ActionPort: '<S159>/Action Port'

          CLVF_B.In1_k0 = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S143>/Hold this value'
        }

        // End of If: '<S143>/if we went through a "step"'

        // Gain: '<S143>/divide by delta T' incorporates:
        //   DataStoreWrite: '<S77>/RED_Px12'

        CLVF_DW.BLACK_Ay = 1.0 / CLVF_P.serverRate * CLVF_B.In1_k0;

        // S-Function (sdspunwrap2): '<S77>/Unwrap1'
        if (CLVF_DW.Unwrap1_FirstStep_jh) {
          CLVF_DW.Unwrap1_Prev_pv = CLVF_B.In1_ak[6];
          CLVF_DW.Unwrap1_FirstStep_jh = false;
        }

        CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap1_Cumsum_a;
        CLVF_B.RED_Tz_Shoulder = CLVF_B.In1_ak[6] - CLVF_DW.Unwrap1_Prev_pv;
        CLVF_B.RED_Tz_Wrist = CLVF_B.RED_Tz_Shoulder - floor
          ((CLVF_B.RED_Tz_Shoulder + 3.1415926535897931) / 6.2831853071795862) *
          6.2831853071795862;
        if ((CLVF_B.RED_Tz_Wrist == -3.1415926535897931) &&
            (CLVF_B.RED_Tz_Shoulder > 0.0)) {
          CLVF_B.RED_Tz_Wrist = 3.1415926535897931;
        }

        CLVF_B.RED_Tz_Shoulder = CLVF_B.RED_Tz_Wrist - CLVF_B.RED_Tz_Shoulder;
        if (fabs(CLVF_B.RED_Tz_Shoulder) > 3.1415926535897931) {
          CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap1_Cumsum_a +
            CLVF_B.RED_Tz_Shoulder;
        }

        CLVF_B.Unwrap1_pz = CLVF_B.In1_ak[6] + CLVF_B.RED_Tz_Elbow;
        CLVF_DW.Unwrap1_Prev_pv = CLVF_B.In1_ak[6];
        CLVF_DW.Unwrap1_Cumsum_a = CLVF_B.RED_Tz_Elbow;

        // End of S-Function (sdspunwrap2): '<S77>/Unwrap1'

        // Delay: '<S149>/Delay1'
        if (CLVF_DW.icLoad_dx != 0) {
          CLVF_DW.Delay1_DSTATE_az = CLVF_B.Unwrap1_pz;
        }

        // Sum: '<S149>/Sum6' incorporates:
        //   Delay: '<S149>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Unwrap1_pz - CLVF_DW.Delay1_DSTATE_az;

        // If: '<S149>/if we went through a "step"' incorporates:
        //   Inport: '<S165>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S149>/Hold this value' incorporates:
          //   ActionPort: '<S165>/Action Port'

          CLVF_B.In1_hy = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S149>/Hold this value'
        }

        // End of If: '<S149>/if we went through a "step"'

        // Gain: '<S149>/divide by delta T'
        CLVF_B.dividebydeltaT_l = 1.0 / CLVF_P.serverRate * CLVF_B.In1_hy;

        // Delay: '<S144>/Delay1'
        if (CLVF_DW.icLoad_pz != 0) {
          CLVF_DW.Delay1_DSTATE_nu = CLVF_B.dividebydeltaT_l;
        }

        // Sum: '<S144>/Sum6' incorporates:
        //   Delay: '<S144>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_l - CLVF_DW.Delay1_DSTATE_nu;

        // If: '<S144>/if we went through a "step"' incorporates:
        //   Inport: '<S160>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S144>/Hold this value' incorporates:
          //   ActionPort: '<S160>/Action Port'

          CLVF_B.In1_la = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S144>/Hold this value'
        }

        // End of If: '<S144>/if we went through a "step"'

        // Gain: '<S144>/divide by delta T' incorporates:
        //   DataStoreWrite: '<S77>/RED_Px13'

        CLVF_DW.BLACK_RzDD = 1.0 / CLVF_P.serverRate * CLVF_B.In1_la;

        // Delay: '<S141>/Delay1'
        if (CLVF_DW.icLoad_jj != 0) {
          CLVF_DW.Delay1_DSTATE_d3 = CLVF_B.In1_ak[1];
        }

        // Sum: '<S141>/Sum6' incorporates:
        //   Delay: '<S141>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.In1_ak[1] - CLVF_DW.Delay1_DSTATE_d3;

        // If: '<S141>/if we went through a "step"' incorporates:
        //   Inport: '<S157>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S141>/Hold this value' incorporates:
          //   ActionPort: '<S157>/Action Port'

          CLVF_B.In1_o2 = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S141>/Hold this value'
        }

        // End of If: '<S141>/if we went through a "step"'

        // Gain: '<S141>/divide by delta T'
        CLVF_B.dividebydeltaT_nn = 1.0 / CLVF_P.serverRate * CLVF_B.In1_o2;

        // DataStoreWrite: '<S77>/RED_Px2'
        CLVF_DW.RED_Vx = CLVF_B.dividebydeltaT_nn;

        // DataStoreWrite: '<S77>/RED_Px3'
        CLVF_DW.BLACK_Vx = CLVF_B.dividebydeltaT_d;

        // DataStoreWrite: '<S77>/RED_Px4'
        CLVF_DW.BLACK_Px = CLVF_B.In1_ak[4];

        // DataStoreWrite: '<S77>/RED_Px5'
        CLVF_DW.BLACK_Py = CLVF_B.In1_ak[5];

        // DataStoreWrite: '<S77>/RED_Px6'
        CLVF_DW.BLACK_Rz = CLVF_B.In1_ak[6];

        // Delay: '<S150>/Delay1'
        if (CLVF_DW.icLoad_fv != 0) {
          CLVF_DW.Delay1_DSTATE_hy = CLVF_B.dividebydeltaT_nn;
        }

        // Sum: '<S150>/Sum6' incorporates:
        //   Delay: '<S150>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_nn -
          CLVF_DW.Delay1_DSTATE_hy;

        // If: '<S150>/if we went through a "step"' incorporates:
        //   Inport: '<S166>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S150>/Hold this value' incorporates:
          //   ActionPort: '<S166>/Action Port'

          CLVF_B.In1_ii = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S150>/Hold this value'
        }

        // End of If: '<S150>/if we went through a "step"'

        // Gain: '<S150>/divide by delta T' incorporates:
        //   DataStoreWrite: '<S77>/RED_Px7'

        CLVF_DW.RED_Ax = 1.0 / CLVF_P.serverRate * CLVF_B.In1_ii;

        // Delay: '<S145>/Delay1'
        if (CLVF_DW.icLoad_jqu != 0) {
          CLVF_DW.Delay1_DSTATE_gb = CLVF_B.In1_ak[2];
        }

        // Sum: '<S145>/Sum6' incorporates:
        //   Delay: '<S145>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.In1_ak[2] - CLVF_DW.Delay1_DSTATE_gb;

        // If: '<S145>/if we went through a "step"' incorporates:
        //   Inport: '<S161>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S145>/Hold this value' incorporates:
          //   ActionPort: '<S161>/Action Port'

          CLVF_B.In1_mk3 = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S145>/Hold this value'
        }

        // End of If: '<S145>/if we went through a "step"'

        // Gain: '<S145>/divide by delta T'
        CLVF_B.dividebydeltaT_pf = 1.0 / CLVF_P.serverRate * CLVF_B.In1_mk3;

        // Delay: '<S151>/Delay1'
        if (CLVF_DW.icLoad_b3 != 0) {
          CLVF_DW.Delay1_DSTATE_fn = CLVF_B.dividebydeltaT_pf;
        }

        // Sum: '<S151>/Sum6' incorporates:
        //   Delay: '<S151>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_pf -
          CLVF_DW.Delay1_DSTATE_fn;

        // If: '<S151>/if we went through a "step"' incorporates:
        //   Inport: '<S167>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S151>/Hold this value' incorporates:
          //   ActionPort: '<S167>/Action Port'

          CLVF_B.In1_bw = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S151>/Hold this value'
        }

        // End of If: '<S151>/if we went through a "step"'

        // Gain: '<S151>/divide by delta T' incorporates:
        //   DataStoreWrite: '<S77>/RED_Px8'

        CLVF_DW.RED_Ay = 1.0 / CLVF_P.serverRate * CLVF_B.In1_bw;

        // S-Function (sdspunwrap2): '<S77>/Unwrap'
        if (CLVF_DW.Unwrap_FirstStep_b) {
          CLVF_DW.Unwrap_Prev_o = CLVF_B.In1_ak[3];
          CLVF_DW.Unwrap_FirstStep_b = false;
        }

        CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap_Cumsum_o;
        CLVF_B.RED_Tz_Shoulder = CLVF_B.In1_ak[3] - CLVF_DW.Unwrap_Prev_o;
        CLVF_B.RED_Tz_Wrist = CLVF_B.RED_Tz_Shoulder - floor
          ((CLVF_B.RED_Tz_Shoulder + 3.1415926535897931) / 6.2831853071795862) *
          6.2831853071795862;
        if ((CLVF_B.RED_Tz_Wrist == -3.1415926535897931) &&
            (CLVF_B.RED_Tz_Shoulder > 0.0)) {
          CLVF_B.RED_Tz_Wrist = 3.1415926535897931;
        }

        CLVF_B.RED_Tz_Shoulder = CLVF_B.RED_Tz_Wrist - CLVF_B.RED_Tz_Shoulder;
        if (fabs(CLVF_B.RED_Tz_Shoulder) > 3.1415926535897931) {
          CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap_Cumsum_o + CLVF_B.RED_Tz_Shoulder;
        }

        CLVF_B.Unwrap_d = CLVF_B.In1_ak[3] + CLVF_B.RED_Tz_Elbow;
        CLVF_DW.Unwrap_Prev_o = CLVF_B.In1_ak[3];
        CLVF_DW.Unwrap_Cumsum_o = CLVF_B.RED_Tz_Elbow;

        // End of S-Function (sdspunwrap2): '<S77>/Unwrap'

        // Delay: '<S146>/Delay1'
        if (CLVF_DW.icLoad_cw != 0) {
          CLVF_DW.Delay1_DSTATE_ek = CLVF_B.Unwrap_d;
        }

        // Sum: '<S146>/Sum6' incorporates:
        //   Delay: '<S146>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.Unwrap_d - CLVF_DW.Delay1_DSTATE_ek;

        // If: '<S146>/if we went through a "step"' incorporates:
        //   Inport: '<S162>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S146>/Hold this value' incorporates:
          //   ActionPort: '<S162>/Action Port'

          CLVF_B.In1_cp = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S146>/Hold this value'
        }

        // End of If: '<S146>/if we went through a "step"'

        // Gain: '<S146>/divide by delta T'
        CLVF_B.dividebydeltaT_lz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_cp;

        // Delay: '<S152>/Delay1'
        if (CLVF_DW.icLoad_hrx != 0) {
          CLVF_DW.Delay1_DSTATE_k1 = CLVF_B.dividebydeltaT_lz;
        }

        // Sum: '<S152>/Sum6' incorporates:
        //   Delay: '<S152>/Delay1'

        CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_lz -
          CLVF_DW.Delay1_DSTATE_k1;

        // If: '<S152>/if we went through a "step"' incorporates:
        //   Inport: '<S168>/In1'

        if (CLVF_B.RED_Tz_Elbow != 0.0) {
          // Outputs for IfAction SubSystem: '<S152>/Hold this value' incorporates:
          //   ActionPort: '<S168>/Action Port'

          CLVF_B.In1_bj = CLVF_B.RED_Tz_Elbow;

          // End of Outputs for SubSystem: '<S152>/Hold this value'
        }

        // End of If: '<S152>/if we went through a "step"'

        // Gain: '<S152>/divide by delta T' incorporates:
        //   DataStoreWrite: '<S77>/RED_Px9'

        CLVF_DW.RED_RzDD = 1.0 / CLVF_P.serverRate * CLVF_B.In1_bj;

        // DataStoreWrite: '<S77>/RED_Py1'
        CLVF_DW.RED_Py = CLVF_B.In1_ak[2];

        // DataStoreWrite: '<S77>/RED_Py2'
        CLVF_DW.RED_Vy = CLVF_B.dividebydeltaT_pf;

        // DataStoreWrite: '<S77>/RED_Py3'
        CLVF_DW.BLACK_Vy = CLVF_B.dividebydeltaT_ka;

        // DataStoreWrite: '<S77>/RED_Rz1'
        CLVF_DW.RED_Rz = CLVF_B.In1_ak[3];

        // DataStoreWrite: '<S77>/RED_Rz2'
        CLVF_DW.RED_RzD = CLVF_B.dividebydeltaT_lz;

        // DataStoreWrite: '<S77>/RED_Rz3'
        CLVF_DW.BLACK_RzD = CLVF_B.dividebydeltaT_l;

        // End of Outputs for SubSystem: '<S75>/Using RED, BLACK, BLUE, or RED + ARM' 
        break;

       case 1:
        // Outputs for IfAction SubSystem: '<S75>/Using RED+BLACK, or RED+BLACK+ARM' incorporates:
        //   ActionPort: '<S76>/Action Port'

        // If: '<S76>/This IF block determines whether or not to run the BLACK sim//exp' incorporates:
        //   Constant: '<S76>/Constant'
        //   Constant: '<S78>/Constant'

        CLVF_B.rtAction = -1;
        if ((CLVF_P.WhoAmI == 2.0) || (CLVF_P.simMode == 1.0)) {
          CLVF_B.rtAction = 0;

          // Outputs for IfAction SubSystem: '<S76>/Obtain BLACK States' incorporates:
          //   ActionPort: '<S79>/Action Port'

          // S-Function (sdspFromNetwork): '<S79>/UDP Receive'
          sErr = GetErrorBuffer(&CLVF_DW.UDPReceive_NetworkLib[0U]);
          CLVF_B.uElOffset1 = 13;
          LibOutputs_Network(&CLVF_DW.UDPReceive_NetworkLib[0U],
                             &CLVF_B.UDPReceive_o1[0U], &CLVF_B.uElOffset1);
          if (*sErr != 0) {
            rtmSetErrorStatus(CLVF_M, sErr);
            rtmSetStopRequested(CLVF_M, 1);
          }

          // End of S-Function (sdspFromNetwork): '<S79>/UDP Receive'

          // DataStoreWrite: '<S79>/Data Store Write3'
          CLVF_DW.ARM_Elbow_Px = CLVF_B.UDPReceive_o1[7];

          // DataStoreWrite: '<S79>/Data Store Write4'
          CLVF_DW.ARM_Elbow_Py = CLVF_B.UDPReceive_o1[8];

          // DataStoreWrite: '<S79>/Data Store Write5'
          CLVF_DW.ARM_Wrist_Px = CLVF_B.UDPReceive_o1[9];

          // DataStoreWrite: '<S79>/Data Store Write6'
          CLVF_DW.ARM_Wrist_Py = CLVF_B.UDPReceive_o1[10];

          // DataStoreWrite: '<S79>/RED_Px1'
          CLVF_DW.RED_Px = CLVF_B.UDPReceive_o1[1];

          // DataStoreWrite: '<S79>/RED_Px10'
          CLVF_DW.Univ_Time = CLVF_B.UDPReceive_o1[0];

          // Delay: '<S87>/Delay1'
          if (CLVF_DW.icLoad_ha != 0) {
            CLVF_DW.Delay1_DSTATE_hp = CLVF_B.UDPReceive_o1[4];
          }

          // Sum: '<S87>/Sum6' incorporates:
          //   Delay: '<S87>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.UDPReceive_o1[4] -
            CLVF_DW.Delay1_DSTATE_hp;

          // If: '<S87>/if we went through a "step"' incorporates:
          //   Inport: '<S99>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S87>/Hold this value' incorporates:
            //   ActionPort: '<S99>/Action Port'

            CLVF_B.In1_h1 = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S87>/Hold this value'
          }

          // End of If: '<S87>/if we went through a "step"'

          // Gain: '<S87>/divide by delta T'
          CLVF_B.dividebydeltaT_p = 1.0 / CLVF_P.serverRate * CLVF_B.In1_h1;

          // Delay: '<S82>/Delay1'
          if (CLVF_DW.icLoad_oh != 0) {
            CLVF_DW.Delay1_DSTATE_bi = CLVF_B.dividebydeltaT_p;
          }

          // Sum: '<S82>/Sum6' incorporates:
          //   Delay: '<S82>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_p -
            CLVF_DW.Delay1_DSTATE_bi;

          // If: '<S82>/if we went through a "step"' incorporates:
          //   Inport: '<S94>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S82>/Hold this value' incorporates:
            //   ActionPort: '<S94>/Action Port'

            CLVF_B.In1_kw = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S82>/Hold this value'
          }

          // End of If: '<S82>/if we went through a "step"'

          // Gain: '<S82>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S79>/RED_Px11'

          CLVF_DW.BLACK_Ax = 1.0 / CLVF_P.serverRate * CLVF_B.In1_kw;

          // Delay: '<S88>/Delay1'
          if (CLVF_DW.icLoad_ij != 0) {
            CLVF_DW.Delay1_DSTATE_fo = CLVF_B.UDPReceive_o1[5];
          }

          // Sum: '<S88>/Sum6' incorporates:
          //   Delay: '<S88>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.UDPReceive_o1[5] -
            CLVF_DW.Delay1_DSTATE_fo;

          // If: '<S88>/if we went through a "step"' incorporates:
          //   Inport: '<S100>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S88>/Hold this value' incorporates:
            //   ActionPort: '<S100>/Action Port'

            CLVF_B.In1_mh = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S88>/Hold this value'
          }

          // End of If: '<S88>/if we went through a "step"'

          // Gain: '<S88>/divide by delta T'
          CLVF_B.dividebydeltaT_j = 1.0 / CLVF_P.serverRate * CLVF_B.In1_mh;

          // Delay: '<S83>/Delay1'
          if (CLVF_DW.icLoad_dr != 0) {
            CLVF_DW.Delay1_DSTATE_f4 = CLVF_B.dividebydeltaT_j;
          }

          // Sum: '<S83>/Sum6' incorporates:
          //   Delay: '<S83>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_j -
            CLVF_DW.Delay1_DSTATE_f4;

          // If: '<S83>/if we went through a "step"' incorporates:
          //   Inport: '<S95>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S83>/Hold this value' incorporates:
            //   ActionPort: '<S95>/Action Port'

            CLVF_B.In1_hr = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S83>/Hold this value'
          }

          // End of If: '<S83>/if we went through a "step"'

          // Gain: '<S83>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S79>/RED_Px12'

          CLVF_DW.BLACK_Ay = 1.0 / CLVF_P.serverRate * CLVF_B.In1_hr;

          // S-Function (sdspunwrap2): '<S79>/Unwrap1'
          if (CLVF_DW.Unwrap1_FirstStep_j) {
            CLVF_DW.Unwrap1_Prev_p = CLVF_B.UDPReceive_o1[6];
            CLVF_DW.Unwrap1_FirstStep_j = false;
          }

          CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap1_Cumsum_e;
          CLVF_B.RED_Tz_Shoulder = CLVF_B.UDPReceive_o1[6] -
            CLVF_DW.Unwrap1_Prev_p;
          CLVF_B.RED_Tz_Wrist = CLVF_B.RED_Tz_Shoulder - floor
            ((CLVF_B.RED_Tz_Shoulder + 3.1415926535897931) / 6.2831853071795862)
            * 6.2831853071795862;
          if ((CLVF_B.RED_Tz_Wrist == -3.1415926535897931) &&
              (CLVF_B.RED_Tz_Shoulder > 0.0)) {
            CLVF_B.RED_Tz_Wrist = 3.1415926535897931;
          }

          CLVF_B.RED_Tz_Shoulder = CLVF_B.RED_Tz_Wrist - CLVF_B.RED_Tz_Shoulder;
          if (fabs(CLVF_B.RED_Tz_Shoulder) > 3.1415926535897931) {
            CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap1_Cumsum_e +
              CLVF_B.RED_Tz_Shoulder;
          }

          CLVF_DW.Unwrap1_Prev_p = CLVF_B.UDPReceive_o1[6];
          CLVF_B.Unwrap1_p = CLVF_B.UDPReceive_o1[6] + CLVF_B.RED_Tz_Elbow;
          CLVF_DW.Unwrap1_Cumsum_e = CLVF_B.RED_Tz_Elbow;

          // End of S-Function (sdspunwrap2): '<S79>/Unwrap1'

          // Delay: '<S89>/Delay1'
          if (CLVF_DW.icLoad_ip != 0) {
            CLVF_DW.Delay1_DSTATE_lb = CLVF_B.Unwrap1_p;
          }

          // Sum: '<S89>/Sum6' incorporates:
          //   Delay: '<S89>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.Unwrap1_p - CLVF_DW.Delay1_DSTATE_lb;

          // If: '<S89>/if we went through a "step"' incorporates:
          //   Inport: '<S101>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S89>/Hold this value' incorporates:
            //   ActionPort: '<S101>/Action Port'

            CLVF_B.In1_bc = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S89>/Hold this value'
          }

          // End of If: '<S89>/if we went through a "step"'

          // Gain: '<S89>/divide by delta T'
          CLVF_B.dividebydeltaT_k = 1.0 / CLVF_P.serverRate * CLVF_B.In1_bc;

          // Delay: '<S84>/Delay1'
          if (CLVF_DW.icLoad_lp != 0) {
            CLVF_DW.Delay1_DSTATE_i2 = CLVF_B.dividebydeltaT_k;
          }

          // Sum: '<S84>/Sum6' incorporates:
          //   Delay: '<S84>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_k -
            CLVF_DW.Delay1_DSTATE_i2;

          // If: '<S84>/if we went through a "step"' incorporates:
          //   Inport: '<S96>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S84>/Hold this value' incorporates:
            //   ActionPort: '<S96>/Action Port'

            CLVF_B.In1_fi = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S84>/Hold this value'
          }

          // End of If: '<S84>/if we went through a "step"'

          // Gain: '<S84>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S79>/RED_Px13'

          CLVF_DW.BLACK_RzDD = 1.0 / CLVF_P.serverRate * CLVF_B.In1_fi;

          // Delay: '<S81>/Delay1'
          if (CLVF_DW.icLoad_nj != 0) {
            CLVF_DW.Delay1_DSTATE_lm = CLVF_B.UDPReceive_o1[1];
          }

          // Sum: '<S81>/Sum6' incorporates:
          //   Delay: '<S81>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.UDPReceive_o1[1] -
            CLVF_DW.Delay1_DSTATE_lm;

          // If: '<S81>/if we went through a "step"' incorporates:
          //   Inport: '<S93>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S81>/Hold this value' incorporates:
            //   ActionPort: '<S93>/Action Port'

            CLVF_B.In1_it = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S81>/Hold this value'
          }

          // End of If: '<S81>/if we went through a "step"'

          // Gain: '<S81>/divide by delta T'
          CLVF_B.dividebydeltaT_iv = 1.0 / CLVF_P.serverRate * CLVF_B.In1_it;

          // DataStoreWrite: '<S79>/RED_Px2'
          CLVF_DW.RED_Vx = CLVF_B.dividebydeltaT_iv;

          // DataStoreWrite: '<S79>/RED_Px3'
          CLVF_DW.BLACK_Vx = CLVF_B.dividebydeltaT_p;

          // DataStoreWrite: '<S79>/RED_Px4'
          CLVF_DW.BLACK_Px = CLVF_B.UDPReceive_o1[4];

          // DataStoreWrite: '<S79>/RED_Px5'
          CLVF_DW.BLACK_Py = CLVF_B.UDPReceive_o1[5];

          // DataStoreWrite: '<S79>/RED_Px6'
          CLVF_DW.BLACK_Rz = CLVF_B.UDPReceive_o1[6];

          // Delay: '<S90>/Delay1'
          if (CLVF_DW.icLoad_av != 0) {
            CLVF_DW.Delay1_DSTATE_ctf = CLVF_B.dividebydeltaT_iv;
          }

          // Sum: '<S90>/Sum6' incorporates:
          //   Delay: '<S90>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_iv -
            CLVF_DW.Delay1_DSTATE_ctf;

          // If: '<S90>/if we went through a "step"' incorporates:
          //   Inport: '<S102>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S90>/Hold this value' incorporates:
            //   ActionPort: '<S102>/Action Port'

            CLVF_B.In1_jl = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S90>/Hold this value'
          }

          // End of If: '<S90>/if we went through a "step"'

          // Gain: '<S90>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S79>/RED_Px7'

          CLVF_DW.RED_Ax = 1.0 / CLVF_P.serverRate * CLVF_B.In1_jl;

          // Delay: '<S85>/Delay1'
          if (CLVF_DW.icLoad_ib != 0) {
            CLVF_DW.Delay1_DSTATE_du = CLVF_B.UDPReceive_o1[2];
          }

          // Sum: '<S85>/Sum6' incorporates:
          //   Delay: '<S85>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.UDPReceive_o1[2] -
            CLVF_DW.Delay1_DSTATE_du;

          // If: '<S85>/if we went through a "step"' incorporates:
          //   Inport: '<S97>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S85>/Hold this value' incorporates:
            //   ActionPort: '<S97>/Action Port'

            CLVF_B.In1_ix = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S85>/Hold this value'
          }

          // End of If: '<S85>/if we went through a "step"'

          // Gain: '<S85>/divide by delta T'
          CLVF_B.dividebydeltaT_g = 1.0 / CLVF_P.serverRate * CLVF_B.In1_ix;

          // Delay: '<S91>/Delay1'
          if (CLVF_DW.icLoad_hm != 0) {
            CLVF_DW.Delay1_DSTATE_gu = CLVF_B.dividebydeltaT_g;
          }

          // Sum: '<S91>/Sum6' incorporates:
          //   Delay: '<S91>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_g -
            CLVF_DW.Delay1_DSTATE_gu;

          // If: '<S91>/if we went through a "step"' incorporates:
          //   Inport: '<S103>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S91>/Hold this value' incorporates:
            //   ActionPort: '<S103>/Action Port'

            CLVF_B.In1_cv = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S91>/Hold this value'
          }

          // End of If: '<S91>/if we went through a "step"'

          // Gain: '<S91>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S79>/RED_Px8'

          CLVF_DW.RED_Ay = 1.0 / CLVF_P.serverRate * CLVF_B.In1_cv;

          // S-Function (sdspunwrap2): '<S79>/Unwrap'
          if (CLVF_DW.Unwrap_FirstStep_e) {
            CLVF_DW.Unwrap_Prev_c = CLVF_B.UDPReceive_o1[3];
            CLVF_DW.Unwrap_FirstStep_e = false;
          }

          CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap_Cumsum_m;
          CLVF_B.RED_Tz_Shoulder = CLVF_B.UDPReceive_o1[3] -
            CLVF_DW.Unwrap_Prev_c;
          CLVF_B.RED_Tz_Wrist = CLVF_B.RED_Tz_Shoulder - floor
            ((CLVF_B.RED_Tz_Shoulder + 3.1415926535897931) / 6.2831853071795862)
            * 6.2831853071795862;
          if ((CLVF_B.RED_Tz_Wrist == -3.1415926535897931) &&
              (CLVF_B.RED_Tz_Shoulder > 0.0)) {
            CLVF_B.RED_Tz_Wrist = 3.1415926535897931;
          }

          CLVF_B.RED_Tz_Shoulder = CLVF_B.RED_Tz_Wrist - CLVF_B.RED_Tz_Shoulder;
          if (fabs(CLVF_B.RED_Tz_Shoulder) > 3.1415926535897931) {
            CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap_Cumsum_m +
              CLVF_B.RED_Tz_Shoulder;
          }

          CLVF_DW.Unwrap_Prev_c = CLVF_B.UDPReceive_o1[3];
          CLVF_B.Unwrap_o = CLVF_B.UDPReceive_o1[3] + CLVF_B.RED_Tz_Elbow;
          CLVF_DW.Unwrap_Cumsum_m = CLVF_B.RED_Tz_Elbow;

          // End of S-Function (sdspunwrap2): '<S79>/Unwrap'

          // Delay: '<S86>/Delay1'
          if (CLVF_DW.icLoad_jh != 0) {
            CLVF_DW.Delay1_DSTATE_jc = CLVF_B.Unwrap_o;
          }

          // Sum: '<S86>/Sum6' incorporates:
          //   Delay: '<S86>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.Unwrap_o - CLVF_DW.Delay1_DSTATE_jc;

          // If: '<S86>/if we went through a "step"' incorporates:
          //   Inport: '<S98>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S86>/Hold this value' incorporates:
            //   ActionPort: '<S98>/Action Port'

            CLVF_B.In1_nd = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S86>/Hold this value'
          }

          // End of If: '<S86>/if we went through a "step"'

          // Gain: '<S86>/divide by delta T'
          CLVF_B.dividebydeltaT_nw = 1.0 / CLVF_P.serverRate * CLVF_B.In1_nd;

          // Delay: '<S92>/Delay1'
          if (CLVF_DW.icLoad_bb != 0) {
            CLVF_DW.Delay1_DSTATE_c4 = CLVF_B.dividebydeltaT_nw;
          }

          // Sum: '<S92>/Sum6' incorporates:
          //   Delay: '<S92>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_nw -
            CLVF_DW.Delay1_DSTATE_c4;

          // If: '<S92>/if we went through a "step"' incorporates:
          //   Inport: '<S104>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S92>/Hold this value' incorporates:
            //   ActionPort: '<S104>/Action Port'

            CLVF_B.In1_ow = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S92>/Hold this value'
          }

          // End of If: '<S92>/if we went through a "step"'

          // Gain: '<S92>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S79>/RED_Px9'

          CLVF_DW.RED_RzDD = 1.0 / CLVF_P.serverRate * CLVF_B.In1_ow;

          // DataStoreWrite: '<S79>/RED_Py1'
          CLVF_DW.RED_Py = CLVF_B.UDPReceive_o1[2];

          // DataStoreWrite: '<S79>/RED_Py2'
          CLVF_DW.RED_Vy = CLVF_B.dividebydeltaT_g;

          // DataStoreWrite: '<S79>/RED_Py3'
          CLVF_DW.BLACK_Vy = CLVF_B.dividebydeltaT_j;

          // DataStoreWrite: '<S79>/RED_Rz1'
          CLVF_DW.RED_Rz = CLVF_B.UDPReceive_o1[3];

          // DataStoreWrite: '<S79>/RED_Rz2'
          CLVF_DW.RED_RzD = CLVF_B.dividebydeltaT_nw;

          // DataStoreWrite: '<S79>/RED_Rz3'
          CLVF_DW.BLACK_RzD = CLVF_B.dividebydeltaT_k;

          // End of Outputs for SubSystem: '<S76>/Obtain BLACK States'
        }

        CLVF_DW.ThisIFblockdetermineswhethero_c = CLVF_B.rtAction;

        // End of If: '<S76>/This IF block determines whether or not to run the BLACK sim//exp' 

        // If: '<S76>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
        //   Clock: '<S80>/Clock'
        //   Constant: '<S76>/Constant'
        //   Constant: '<S78>/Constant'
        //   Delay: '<S121>/Delay'
        //   Inport: '<S107>/In1'
        //   Inport: '<S108>/In1'
        //   Inport: '<S122>/In1'
        //   Logic: '<S121>/Logical Operator'
        //   Logic: '<S121>/Logical Operator1'
        //   SignalConversion generated from: '<S108>/Enable'
        //   SignalConversion generated from: '<S107>/Enable'

        CLVF_B.rtPrevAction = CLVF_DW.ThisIFblockdetermineswhether_cx;
        CLVF_B.rtAction = -1;
        if ((CLVF_P.WhoAmI == 1.0) || (CLVF_P.simMode == 1.0)) {
          CLVF_B.rtAction = 0;
        }

        CLVF_DW.ThisIFblockdetermineswhether_cx = CLVF_B.rtAction;
        if ((CLVF_B.rtPrevAction != CLVF_B.rtAction) && (CLVF_B.rtPrevAction ==
             0)) {
          // Disable for Enabled SubSystem: '<S80>/Enabled Subsystem'
          if (CLVF_DW.EnabledSubsystem_MODE) {
            CLVF_DW.EnabledSubsystem_MODE = false;
          }

          // End of Disable for SubSystem: '<S80>/Enabled Subsystem'

          // Disable for Enabled SubSystem: '<S80>/Enabled Subsystem1'
          if (CLVF_DW.EnabledSubsystem1_MODE) {
            CLVF_DW.EnabledSubsystem1_MODE = false;
          }

          // End of Disable for SubSystem: '<S80>/Enabled Subsystem1'

          // Disable for Enabled SubSystem: '<S106>/Enabled Subsystem'
          if (CLVF_DW.EnabledSubsystem_MODE_p) {
            CLVF_DW.EnabledSubsystem_MODE_p = false;
          }

          // End of Disable for SubSystem: '<S106>/Enabled Subsystem'
        }

        if (CLVF_B.rtAction == 0) {
          // Outputs for IfAction SubSystem: '<S76>/Obtain RED States' incorporates:
          //   ActionPort: '<S80>/Action Port'

          // MATLABSystem: '<S80>/Stream PhaseSpace to Platform'
          CLVF_B.RED_Tz_Elbow = 1.0 / CLVF_P.serverRate;
          if (CLVF_DW.obj_k.platformSelection != CLVF_P.platformSelection) {
            CLVF_DW.obj_k.platformSelection = CLVF_P.platformSelection;
          }

          if (CLVF_DW.obj_k.PS_SampleRate != CLVF_B.RED_Tz_Elbow) {
            CLVF_DW.obj_k.PS_SampleRate = CLVF_B.RED_Tz_Elbow;
          }

          CLVF_B.RED_Tz_Elbow = 0.0;
          CLVF_B.RED_Tz_Shoulder = 0.0;
          CLVF_B.StreamPhaseSpacetoPlatform[3] = 0.0;
          CLVF_B.vc_p = 0.0;
          CLVF_B.RED_Tz_Wrist = 0.0;
          CLVF_B.StreamPhaseSpacetoPlatform[6] = 0.0;
          CLVF_B.y7 = 0.0;
          CLVF_B.vc_dot = 0.0;
          CLVF_B.g = 0.0;
          CLVF_B.theta_dot = 0.0;
          CLVF_B.theta_l = 0.0;
          CLVF_B.v_rel = 0.0;
          CLVF_B.dgdr = 0.0;
          stream_phasespace(&CLVF_B.RED_Tz_Elbow, &CLVF_B.RED_Tz_Shoulder,
                            &CLVF_B.StreamPhaseSpacetoPlatform[3], &CLVF_B.vc_p,
                            &CLVF_B.RED_Tz_Wrist,
                            &CLVF_B.StreamPhaseSpacetoPlatform[6], &CLVF_B.y7,
                            &CLVF_B.vc_dot, &CLVF_B.g, &CLVF_B.theta_dot,
                            &CLVF_B.theta_l, &CLVF_B.v_rel, &CLVF_B.dgdr,
                            CLVF_DW.obj_k.platformSelection);
          CLVF_B.StreamPhaseSpacetoPlatform[0] = CLVF_B.y7 /
            CLVF_DW.obj_k.PS_SampleRate;
          CLVF_B.StreamPhaseSpacetoPlatform[1] = CLVF_B.RED_Tz_Elbow / 1000.0;
          CLVF_B.StreamPhaseSpacetoPlatform[2] = CLVF_B.RED_Tz_Shoulder / 1000.0;
          CLVF_B.StreamPhaseSpacetoPlatform[4] = CLVF_B.vc_p / 1000.0;
          CLVF_B.StreamPhaseSpacetoPlatform[5] = CLVF_B.RED_Tz_Wrist / 1000.0;
          CLVF_B.StreamPhaseSpacetoPlatform[7] = CLVF_B.vc_dot / 1000.0;
          CLVF_B.StreamPhaseSpacetoPlatform[8] = CLVF_B.g / 1000.0;
          CLVF_B.StreamPhaseSpacetoPlatform[9] = CLVF_B.theta_dot / 1000.0;
          CLVF_B.StreamPhaseSpacetoPlatform[10] = CLVF_B.theta_l / 1000.0;
          CLVF_B.StreamPhaseSpacetoPlatform[11] = CLVF_B.v_rel / 1000.0;
          CLVF_B.StreamPhaseSpacetoPlatform[12] = CLVF_B.dgdr / 1000.0;

          // End of MATLABSystem: '<S80>/Stream PhaseSpace to Platform'

          // RelationalOperator: '<S105>/Compare' incorporates:
          //   Constant: '<S105>/Constant'

          for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 13; CLVF_B.uElOffset1
               ++) {
            CLVF_B.Compare_e[CLVF_B.uElOffset1] =
              (CLVF_B.StreamPhaseSpacetoPlatform[CLVF_B.uElOffset1] !=
               CLVF_P.Constant_Value_f);
          }

          // End of RelationalOperator: '<S105>/Compare'

          // Outputs for Enabled SubSystem: '<S80>/Enabled Subsystem1' incorporates:
          //   EnablePort: '<S108>/Enable'

          // Outputs for Enabled SubSystem: '<S80>/Enabled Subsystem' incorporates:
          //   EnablePort: '<S107>/Enable'

          CLVF_B.rtb_Compare_dy = (CLVF_B.Compare_e[0] || CLVF_B.Compare_e[1] ||
            CLVF_B.Compare_e[2] || CLVF_B.Compare_e[3] || CLVF_B.Compare_e[4] ||
            CLVF_B.Compare_e[5] || CLVF_B.Compare_e[6] || CLVF_B.Compare_e[7] ||
            CLVF_B.Compare_e[8] || CLVF_B.Compare_e[9] || CLVF_B.Compare_e[10] ||
            CLVF_B.Compare_e[11]);

          // End of Outputs for SubSystem: '<S80>/Enabled Subsystem1'
          if (CLVF_B.rtb_Compare_dy || CLVF_B.Compare_e[12]) {
            if (!CLVF_DW.EnabledSubsystem_MODE) {
              CLVF_DW.EnabledSubsystem_MODE = true;
            }
          } else {
            if (CLVF_DW.EnabledSubsystem_MODE) {
              CLVF_DW.EnabledSubsystem_MODE = false;
            }
          }

          if (CLVF_DW.EnabledSubsystem_MODE) {
            memcpy(&CLVF_B.In1_mk[0], &CLVF_B.StreamPhaseSpacetoPlatform[0], 13U
                   * sizeof(real_T));
          }

          // End of Outputs for SubSystem: '<S80>/Enabled Subsystem'

          // DataStoreWrite: '<S80>/Data Store Write3' incorporates:
          //   Inport: '<S107>/In1'
          //   SignalConversion generated from: '<S107>/Enable'

          CLVF_DW.ARM_Elbow_Px = CLVF_B.In1_mk[7];

          // DataStoreWrite: '<S80>/Data Store Write4'
          CLVF_DW.ARM_Elbow_Py = CLVF_B.In1_mk[8];

          // DataStoreWrite: '<S80>/Data Store Write5'
          CLVF_DW.ARM_Wrist_Px = CLVF_B.In1_mk[9];

          // DataStoreWrite: '<S80>/Data Store Write6'
          CLVF_DW.ARM_Wrist_Py = CLVF_B.In1_mk[10];

          // DataStoreWrite: '<S80>/RED_Px1'
          CLVF_DW.RED_Px = CLVF_B.In1_mk[1];

          // Outputs for Enabled SubSystem: '<S80>/Enabled Subsystem1' incorporates:
          //   EnablePort: '<S108>/Enable'

          if (CLVF_B.rtb_Compare_dy || CLVF_B.Compare_e[12]) {
            if (!CLVF_DW.EnabledSubsystem1_MODE) {
              CLVF_DW.EnabledSubsystem1_MODE = true;
            }
          } else {
            if (CLVF_DW.EnabledSubsystem1_MODE) {
              CLVF_DW.EnabledSubsystem1_MODE = false;
            }
          }

          if (CLVF_DW.EnabledSubsystem1_MODE) {
            CLVF_B.In1_lxy = CLVF_M->Timing.t[0];
          }

          // End of Outputs for SubSystem: '<S80>/Enabled Subsystem1'

          // DataTypeConversion: '<S121>/Data Type Conversion' incorporates:
          //   Clock: '<S80>/Clock'
          //   Inport: '<S108>/In1'
          //   SignalConversion generated from: '<S108>/Enable'

          CLVF_B.DataTypeConversion = (CLVF_B.In1_lxy != 0.0);

          // Outputs for Enabled SubSystem: '<S106>/Enabled Subsystem' incorporates:
          //   EnablePort: '<S122>/Enable'

          if (CLVF_B.DataTypeConversion && (!CLVF_DW.Delay_DSTATE_k2)) {
            if (!CLVF_DW.EnabledSubsystem_MODE_p) {
              CLVF_DW.EnabledSubsystem_MODE_p = true;
            }
          } else {
            if (CLVF_DW.EnabledSubsystem_MODE_p) {
              CLVF_DW.EnabledSubsystem_MODE_p = false;
            }
          }

          if (CLVF_DW.EnabledSubsystem_MODE_p) {
            CLVF_B.In1_fg = CLVF_B.In1_lxy;
          }

          // End of Outputs for SubSystem: '<S106>/Enabled Subsystem'

          // Sum: '<S106>/Subtract' incorporates:
          //   DataStoreWrite: '<S80>/RED_Px10'
          //   Delay: '<S121>/Delay'
          //   Inport: '<S122>/In1'
          //   Logic: '<S121>/Logical Operator'
          //   Logic: '<S121>/Logical Operator1'

          CLVF_DW.Univ_Time = CLVF_B.In1_lxy - CLVF_B.In1_fg;

          // Delay: '<S115>/Delay1'
          if (CLVF_DW.icLoad_bo != 0) {
            CLVF_DW.Delay1_DSTATE_mo = CLVF_B.In1_mk[4];
          }

          // Sum: '<S115>/Sum6' incorporates:
          //   Delay: '<S115>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.In1_mk[4] - CLVF_DW.Delay1_DSTATE_mo;

          // If: '<S115>/if we went through a "step"' incorporates:
          //   Inport: '<S131>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S115>/Hold this value' incorporates:
            //   ActionPort: '<S131>/Action Port'

            CLVF_B.In1_mp = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S115>/Hold this value'
          }

          // End of If: '<S115>/if we went through a "step"'

          // Gain: '<S115>/divide by delta T'
          CLVF_B.dividebydeltaT_f = 1.0 / CLVF_P.serverRate * CLVF_B.In1_mp;

          // Delay: '<S110>/Delay1'
          if (CLVF_DW.icLoad_lm != 0) {
            CLVF_DW.Delay1_DSTATE_ig = CLVF_B.dividebydeltaT_f;
          }

          // Sum: '<S110>/Sum6' incorporates:
          //   Delay: '<S110>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_f -
            CLVF_DW.Delay1_DSTATE_ig;

          // If: '<S110>/if we went through a "step"' incorporates:
          //   Inport: '<S126>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S110>/Hold this value' incorporates:
            //   ActionPort: '<S126>/Action Port'

            CLVF_B.In1_ij = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S110>/Hold this value'
          }

          // End of If: '<S110>/if we went through a "step"'

          // Gain: '<S110>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S80>/RED_Px11'

          CLVF_DW.BLACK_Ax = 1.0 / CLVF_P.serverRate * CLVF_B.In1_ij;

          // Delay: '<S116>/Delay1'
          if (CLVF_DW.icLoad_h2 != 0) {
            CLVF_DW.Delay1_DSTATE_lz = CLVF_B.In1_mk[5];
          }

          // Sum: '<S116>/Sum6' incorporates:
          //   Delay: '<S116>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.In1_mk[5] - CLVF_DW.Delay1_DSTATE_lz;

          // If: '<S116>/if we went through a "step"' incorporates:
          //   Inport: '<S132>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S116>/Hold this value' incorporates:
            //   ActionPort: '<S132>/Action Port'

            CLVF_B.In1_as = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S116>/Hold this value'
          }

          // End of If: '<S116>/if we went through a "step"'

          // Gain: '<S116>/divide by delta T'
          CLVF_B.dividebydeltaT_bz = 1.0 / CLVF_P.serverRate * CLVF_B.In1_as;

          // Delay: '<S111>/Delay1'
          if (CLVF_DW.icLoad_fc0 != 0) {
            CLVF_DW.Delay1_DSTATE_ew = CLVF_B.dividebydeltaT_bz;
          }

          // Sum: '<S111>/Sum6' incorporates:
          //   Delay: '<S111>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_bz -
            CLVF_DW.Delay1_DSTATE_ew;

          // If: '<S111>/if we went through a "step"' incorporates:
          //   Inport: '<S127>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S111>/Hold this value' incorporates:
            //   ActionPort: '<S127>/Action Port'

            CLVF_B.In1_hp = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S111>/Hold this value'
          }

          // End of If: '<S111>/if we went through a "step"'

          // Gain: '<S111>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S80>/RED_Px12'

          CLVF_DW.BLACK_Ay = 1.0 / CLVF_P.serverRate * CLVF_B.In1_hp;

          // S-Function (sdspunwrap2): '<S80>/Unwrap1'
          if (CLVF_DW.Unwrap1_FirstStep) {
            CLVF_DW.Unwrap1_Prev = CLVF_B.In1_mk[6];
            CLVF_DW.Unwrap1_FirstStep = false;
          }

          CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap1_Cumsum;
          CLVF_B.RED_Tz_Shoulder = CLVF_B.In1_mk[6] - CLVF_DW.Unwrap1_Prev;
          CLVF_B.RED_Tz_Wrist = CLVF_B.RED_Tz_Shoulder - floor
            ((CLVF_B.RED_Tz_Shoulder + 3.1415926535897931) / 6.2831853071795862)
            * 6.2831853071795862;
          if ((CLVF_B.RED_Tz_Wrist == -3.1415926535897931) &&
              (CLVF_B.RED_Tz_Shoulder > 0.0)) {
            CLVF_B.RED_Tz_Wrist = 3.1415926535897931;
          }

          CLVF_B.RED_Tz_Shoulder = CLVF_B.RED_Tz_Wrist - CLVF_B.RED_Tz_Shoulder;
          if (fabs(CLVF_B.RED_Tz_Shoulder) > 3.1415926535897931) {
            CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap1_Cumsum +
              CLVF_B.RED_Tz_Shoulder;
          }

          CLVF_B.Unwrap1 = CLVF_B.In1_mk[6] + CLVF_B.RED_Tz_Elbow;
          CLVF_DW.Unwrap1_Prev = CLVF_B.In1_mk[6];
          CLVF_DW.Unwrap1_Cumsum = CLVF_B.RED_Tz_Elbow;

          // End of S-Function (sdspunwrap2): '<S80>/Unwrap1'

          // Delay: '<S117>/Delay1'
          if (CLVF_DW.icLoad_jq != 0) {
            CLVF_DW.Delay1_DSTATE_io = CLVF_B.Unwrap1;
          }

          // Sum: '<S117>/Sum6' incorporates:
          //   Delay: '<S117>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.Unwrap1 - CLVF_DW.Delay1_DSTATE_io;

          // If: '<S117>/if we went through a "step"' incorporates:
          //   Inport: '<S133>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S117>/Hold this value' incorporates:
            //   ActionPort: '<S133>/Action Port'

            CLVF_B.In1_gn = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S117>/Hold this value'
          }

          // End of If: '<S117>/if we went through a "step"'

          // Gain: '<S117>/divide by delta T'
          CLVF_B.dividebydeltaT_e = 1.0 / CLVF_P.serverRate * CLVF_B.In1_gn;

          // Delay: '<S112>/Delay1'
          if (CLVF_DW.icLoad_ab != 0) {
            CLVF_DW.Delay1_DSTATE_bt = CLVF_B.dividebydeltaT_e;
          }

          // Sum: '<S112>/Sum6' incorporates:
          //   Delay: '<S112>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_e -
            CLVF_DW.Delay1_DSTATE_bt;

          // If: '<S112>/if we went through a "step"' incorporates:
          //   Inport: '<S128>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S112>/Hold this value' incorporates:
            //   ActionPort: '<S128>/Action Port'

            CLVF_B.In1_ls = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S112>/Hold this value'
          }

          // End of If: '<S112>/if we went through a "step"'

          // Gain: '<S112>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S80>/RED_Px13'

          CLVF_DW.BLACK_RzDD = 1.0 / CLVF_P.serverRate * CLVF_B.In1_ls;

          // Delay: '<S109>/Delay1'
          if (CLVF_DW.icLoad_ez != 0) {
            CLVF_DW.Delay1_DSTATE_lg = CLVF_B.In1_mk[1];
          }

          // Sum: '<S109>/Sum6' incorporates:
          //   Delay: '<S109>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.In1_mk[1] - CLVF_DW.Delay1_DSTATE_lg;

          // If: '<S109>/if we went through a "step"' incorporates:
          //   Inport: '<S125>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S109>/Hold this value' incorporates:
            //   ActionPort: '<S125>/Action Port'

            CLVF_B.In1_g2 = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S109>/Hold this value'
          }

          // End of If: '<S109>/if we went through a "step"'

          // Gain: '<S109>/divide by delta T'
          CLVF_B.dividebydeltaT_n = 1.0 / CLVF_P.serverRate * CLVF_B.In1_g2;

          // DataStoreWrite: '<S80>/RED_Px2'
          CLVF_DW.RED_Vx = CLVF_B.dividebydeltaT_n;

          // DataStoreWrite: '<S80>/RED_Px3'
          CLVF_DW.BLACK_Vx = CLVF_B.dividebydeltaT_f;

          // DataStoreWrite: '<S80>/RED_Px4'
          CLVF_DW.BLACK_Px = CLVF_B.In1_mk[4];

          // DataStoreWrite: '<S80>/RED_Px5'
          CLVF_DW.BLACK_Py = CLVF_B.In1_mk[5];

          // DataStoreWrite: '<S80>/RED_Px6'
          CLVF_DW.BLACK_Rz = CLVF_B.In1_mk[6];

          // Delay: '<S118>/Delay1'
          if (CLVF_DW.icLoad_n4 != 0) {
            CLVF_DW.Delay1_DSTATE_kq = CLVF_B.dividebydeltaT_n;
          }

          // Sum: '<S118>/Sum6' incorporates:
          //   Delay: '<S118>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_n -
            CLVF_DW.Delay1_DSTATE_kq;

          // If: '<S118>/if we went through a "step"' incorporates:
          //   Inport: '<S134>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S118>/Hold this value' incorporates:
            //   ActionPort: '<S134>/Action Port'

            CLVF_B.In1_eh = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S118>/Hold this value'
          }

          // End of If: '<S118>/if we went through a "step"'

          // Gain: '<S118>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S80>/RED_Px7'

          CLVF_DW.RED_Ax = 1.0 / CLVF_P.serverRate * CLVF_B.In1_eh;

          // Delay: '<S113>/Delay1'
          if (CLVF_DW.icLoad_ln != 0) {
            CLVF_DW.Delay1_DSTATE_ph = CLVF_B.In1_mk[2];
          }

          // Sum: '<S113>/Sum6' incorporates:
          //   Delay: '<S113>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.In1_mk[2] - CLVF_DW.Delay1_DSTATE_ph;

          // If: '<S113>/if we went through a "step"' incorporates:
          //   Inport: '<S129>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S113>/Hold this value' incorporates:
            //   ActionPort: '<S129>/Action Port'

            CLVF_B.In1_eo = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S113>/Hold this value'
          }

          // End of If: '<S113>/if we went through a "step"'

          // Gain: '<S113>/divide by delta T'
          CLVF_B.dividebydeltaT_i = 1.0 / CLVF_P.serverRate * CLVF_B.In1_eo;

          // Delay: '<S119>/Delay1'
          if (CLVF_DW.icLoad_dk != 0) {
            CLVF_DW.Delay1_DSTATE_mc = CLVF_B.dividebydeltaT_i;
          }

          // Sum: '<S119>/Sum6' incorporates:
          //   Delay: '<S119>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_i -
            CLVF_DW.Delay1_DSTATE_mc;

          // If: '<S119>/if we went through a "step"' incorporates:
          //   Inport: '<S135>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S119>/Hold this value' incorporates:
            //   ActionPort: '<S135>/Action Port'

            CLVF_B.In1_of = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S119>/Hold this value'
          }

          // End of If: '<S119>/if we went through a "step"'

          // Gain: '<S119>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S80>/RED_Px8'

          CLVF_DW.RED_Ay = 1.0 / CLVF_P.serverRate * CLVF_B.In1_of;

          // S-Function (sdspunwrap2): '<S80>/Unwrap'
          if (CLVF_DW.Unwrap_FirstStep) {
            CLVF_DW.Unwrap_Prev = CLVF_B.In1_mk[3];
            CLVF_DW.Unwrap_FirstStep = false;
          }

          CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap_Cumsum;
          CLVF_B.RED_Tz_Shoulder = CLVF_B.In1_mk[3] - CLVF_DW.Unwrap_Prev;
          CLVF_B.RED_Tz_Wrist = CLVF_B.RED_Tz_Shoulder - floor
            ((CLVF_B.RED_Tz_Shoulder + 3.1415926535897931) / 6.2831853071795862)
            * 6.2831853071795862;
          if ((CLVF_B.RED_Tz_Wrist == -3.1415926535897931) &&
              (CLVF_B.RED_Tz_Shoulder > 0.0)) {
            CLVF_B.RED_Tz_Wrist = 3.1415926535897931;
          }

          CLVF_B.RED_Tz_Shoulder = CLVF_B.RED_Tz_Wrist - CLVF_B.RED_Tz_Shoulder;
          if (fabs(CLVF_B.RED_Tz_Shoulder) > 3.1415926535897931) {
            CLVF_B.RED_Tz_Elbow = CLVF_DW.Unwrap_Cumsum + CLVF_B.RED_Tz_Shoulder;
          }

          CLVF_B.Unwrap = CLVF_B.In1_mk[3] + CLVF_B.RED_Tz_Elbow;
          CLVF_DW.Unwrap_Prev = CLVF_B.In1_mk[3];
          CLVF_DW.Unwrap_Cumsum = CLVF_B.RED_Tz_Elbow;

          // End of S-Function (sdspunwrap2): '<S80>/Unwrap'

          // Delay: '<S114>/Delay1'
          if (CLVF_DW.icLoad_kr != 0) {
            CLVF_DW.Delay1_DSTATE_phy = CLVF_B.Unwrap;
          }

          // Sum: '<S114>/Sum6' incorporates:
          //   Delay: '<S114>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.Unwrap - CLVF_DW.Delay1_DSTATE_phy;

          // If: '<S114>/if we went through a "step"' incorporates:
          //   Inport: '<S130>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S114>/Hold this value' incorporates:
            //   ActionPort: '<S130>/Action Port'

            CLVF_B.In1_jj = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S114>/Hold this value'
          }

          // End of If: '<S114>/if we went through a "step"'

          // Gain: '<S114>/divide by delta T'
          CLVF_B.dividebydeltaT_m = 1.0 / CLVF_P.serverRate * CLVF_B.In1_jj;

          // Delay: '<S120>/Delay1'
          if (CLVF_DW.icLoad_gt != 0) {
            CLVF_DW.Delay1_DSTATE_ol = CLVF_B.dividebydeltaT_m;
          }

          // Sum: '<S120>/Sum6' incorporates:
          //   Delay: '<S120>/Delay1'

          CLVF_B.RED_Tz_Elbow = CLVF_B.dividebydeltaT_m -
            CLVF_DW.Delay1_DSTATE_ol;

          // If: '<S120>/if we went through a "step"' incorporates:
          //   Inport: '<S136>/In1'

          if (CLVF_B.RED_Tz_Elbow != 0.0) {
            // Outputs for IfAction SubSystem: '<S120>/Hold this value' incorporates:
            //   ActionPort: '<S136>/Action Port'

            CLVF_B.In1_o4 = CLVF_B.RED_Tz_Elbow;

            // End of Outputs for SubSystem: '<S120>/Hold this value'
          }

          // End of If: '<S120>/if we went through a "step"'

          // Gain: '<S120>/divide by delta T' incorporates:
          //   DataStoreWrite: '<S80>/RED_Px9'

          CLVF_DW.RED_RzDD = 1.0 / CLVF_P.serverRate * CLVF_B.In1_o4;

          // DataStoreWrite: '<S80>/RED_Py1'
          CLVF_DW.RED_Py = CLVF_B.In1_mk[2];

          // DataStoreWrite: '<S80>/RED_Py2'
          CLVF_DW.RED_Vy = CLVF_B.dividebydeltaT_i;

          // DataStoreWrite: '<S80>/RED_Py3'
          CLVF_DW.BLACK_Vy = CLVF_B.dividebydeltaT_bz;

          // DataStoreWrite: '<S80>/RED_Rz1'
          CLVF_DW.RED_Rz = CLVF_B.In1_mk[3];

          // DataStoreWrite: '<S80>/RED_Rz2'
          CLVF_DW.RED_RzD = CLVF_B.dividebydeltaT_m;

          // DataStoreWrite: '<S80>/RED_Rz3'
          CLVF_DW.BLACK_RzD = CLVF_B.dividebydeltaT_e;

          // SignalConversion generated from: '<S80>/Send BLACK States to  BLACK Platform' incorporates:
          //   DataStoreWrite: '<S80>/RED_Px10'

          CLVF_B.TmpSignalConversionAtSendBLACKS[0] = CLVF_DW.Univ_Time;
          memcpy(&CLVF_B.TmpSignalConversionAtSendBLACKS[1], &CLVF_B.In1_mk[1],
                 12U * sizeof(real_T));

          // End of Outputs for SubSystem: '<S76>/Obtain RED States'
        }

        // End of If: '<S76>/This IF block determines whether or not to run the RED sim//exp ' 
        // End of Outputs for SubSystem: '<S75>/Using RED+BLACK, or RED+BLACK+ARM' 
        break;
      }

      // End of If: '<S75>/Check whether both platforms are being used, and if so then use RED to send data to BLACK ' 
      // End of Outputs for SubSystem: '<S5>/Use Hardware to Obtain States'
      break;

     case 1:
      // Outputs for IfAction SubSystem: '<S5>/Initialize Universal Time (Simulation)' incorporates:
      //   ActionPort: '<S73>/Action Port'

      // Clock: '<S73>/Set Universal Time (If this is a simulation)' incorporates:
      //   DataStoreWrite: '<S73>/Universal_Time'

      CLVF_DW.Univ_Time = CLVF_M->Timing.t[0];

      // End of Outputs for SubSystem: '<S5>/Initialize Universal Time (Simulation)' 
      break;
    }

    // End of If: '<S5>/If performing an experiment, grab the PhaseSpace data. Otherwise, use a clock to set time in SIM.' 

    // MATLABSystem: '<S6>/LSM9DS1 IMU Sensor'
    CLVF_B.status = 24U;
    memcpy((void *)&CLVF_B.SwappedDataBytes, (void *)&CLVF_B.status, (uint32_T)
           ((size_t)1 * sizeof(uint8_T)));
    CLVF_B.status = MW_I2C_MasterWrite(CLVF_DW.obj.i2cobj_A_G.MW_I2C_HANDLE,
      106U, &CLVF_B.SwappedDataBytes, 1U, true, false);
    if (0 == CLVF_B.status) {
      MW_I2C_MasterRead(CLVF_DW.obj.i2cobj_A_G.MW_I2C_HANDLE, 106U,
                        CLVF_B.output_raw, 6U, false, true);
      memcpy((void *)&CLVF_B.b_RegisterValue[0], (void *)&CLVF_B.output_raw[0],
             (uint32_T)((size_t)3 * sizeof(int16_T)));
    } else {
      CLVF_B.b_RegisterValue[0] = 0;
      CLVF_B.b_RegisterValue[1] = 0;
      CLVF_B.b_RegisterValue[2] = 0;
    }

    memcpy(&CLVF_B.C_IB[0], &CLVF_DW.obj.CalGyroA[0], 9U * sizeof(real_T));
    for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
      CLVF_B.x_ddot_b[CLVF_B.uElOffset1] = ((CLVF_B.C_IB[3 * CLVF_B.uElOffset1 +
        1] * static_cast<real_T>(CLVF_B.b_RegisterValue[1]) + CLVF_B.C_IB[3 *
        CLVF_B.uElOffset1] * static_cast<real_T>(CLVF_B.b_RegisterValue[0])) +
        CLVF_B.C_IB[3 * CLVF_B.uElOffset1 + 2] * static_cast<real_T>
        (CLVF_B.b_RegisterValue[2])) + CLVF_DW.obj.CalGyroB[CLVF_B.uElOffset1];
    }

    CLVF_B.status = 40U;
    memcpy((void *)&CLVF_B.SwappedDataBytes, (void *)&CLVF_B.status, (uint32_T)
           ((size_t)1 * sizeof(uint8_T)));
    CLVF_B.status = MW_I2C_MasterWrite(CLVF_DW.obj.i2cobj_A_G.MW_I2C_HANDLE,
      106U, &CLVF_B.SwappedDataBytes, 1U, true, false);
    if (0 == CLVF_B.status) {
      MW_I2C_MasterRead(CLVF_DW.obj.i2cobj_A_G.MW_I2C_HANDLE, 106U,
                        CLVF_B.output_raw, 6U, false, true);
      memcpy((void *)&CLVF_B.b_RegisterValue[0], (void *)&CLVF_B.output_raw[0],
             (uint32_T)((size_t)3 * sizeof(int16_T)));
    } else {
      CLVF_B.b_RegisterValue[0] = 0;
      CLVF_B.b_RegisterValue[1] = 0;
      CLVF_B.b_RegisterValue[2] = 0;
    }

    memcpy(&CLVF_B.C_IB[0], &CLVF_DW.obj.CalAccelA[0], 9U * sizeof(real_T));
    for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
      CLVF_B.x_ddot_g[CLVF_B.uElOffset1] = ((CLVF_B.C_IB[3 * CLVF_B.uElOffset1 +
        1] * static_cast<real_T>(CLVF_B.b_RegisterValue[1]) + CLVF_B.C_IB[3 *
        CLVF_B.uElOffset1] * static_cast<real_T>(CLVF_B.b_RegisterValue[0])) +
        CLVF_B.C_IB[3 * CLVF_B.uElOffset1 + 2] * static_cast<real_T>
        (CLVF_B.b_RegisterValue[2])) + CLVF_DW.obj.CalAccelB[CLVF_B.uElOffset1];
    }

    CLVF_B.status = 40U;
    memcpy((void *)&CLVF_B.SwappedDataBytes, (void *)&CLVF_B.status, (uint32_T)
           ((size_t)1 * sizeof(uint8_T)));
    CLVF_B.status = MW_I2C_MasterWrite(CLVF_DW.obj.i2cobj_MAG.MW_I2C_HANDLE, 28U,
      &CLVF_B.SwappedDataBytes, 1U, true, false);
    if (0 == CLVF_B.status) {
      MW_I2C_MasterRead(CLVF_DW.obj.i2cobj_MAG.MW_I2C_HANDLE, 28U,
                        CLVF_B.output_raw, 6U, false, true);
      memcpy((void *)&CLVF_B.b_RegisterValue[0], (void *)&CLVF_B.output_raw[0],
             (uint32_T)((size_t)3 * sizeof(int16_T)));
    } else {
      CLVF_B.b_RegisterValue[0] = 0;
      CLVF_B.b_RegisterValue[1] = 0;
      CLVF_B.b_RegisterValue[2] = 0;
    }

    memcpy(&CLVF_B.C_IB[0], &CLVF_DW.obj.CalMagA[0], 9U * sizeof(real_T));
    for (CLVF_B.uElOffset1 = 0; CLVF_B.uElOffset1 < 3; CLVF_B.uElOffset1++) {
      CLVF_B.vc_p = ((CLVF_B.C_IB[3 * CLVF_B.uElOffset1 + 1] *
                      static_cast<real_T>(CLVF_B.b_RegisterValue[1]) +
                      CLVF_B.C_IB[3 * CLVF_B.uElOffset1] * static_cast<real_T>
                      (CLVF_B.b_RegisterValue[0])) + CLVF_B.C_IB[3 *
                     CLVF_B.uElOffset1 + 2] * static_cast<real_T>
                     (CLVF_B.b_RegisterValue[2])) +
        CLVF_DW.obj.CalMagB[CLVF_B.uElOffset1];
      CLVF_B.LSM9DS1IMUSensor_o3[CLVF_B.uElOffset1] = CLVF_B.vc_p * 4.0 /
        32768.0;
      CLVF_B.x_ddot_b[CLVF_B.uElOffset1] = CLVF_B.x_ddot_b[CLVF_B.uElOffset1] *
        245.0 / 32768.0;
      CLVF_B.x_ddot_g[CLVF_B.uElOffset1] = CLVF_B.x_ddot_g[CLVF_B.uElOffset1] *
        2.0 / 32768.0;
    }

    // End of MATLABSystem: '<S6>/LSM9DS1 IMU Sensor'

    // If: '<S6>/This IF block determines whether or not to run the BLACK sim//exp' incorporates:
    //   Constant: '<S172>/Constant'
    //   Constant: '<S6>/Constant1'

    if ((CLVF_P.WhoAmI == 2.0) && (CLVF_P.simMode == 0.0)) {
      // Outputs for IfAction SubSystem: '<S6>/Change BLACK Behavior' incorporates:
      //   ActionPort: '<S169>/Action Port'

      // Gain: '<S169>/Gain'
      CLVF_B.Gain_n[0] = CLVF_P.Gain_Gain * CLVF_B.x_ddot_b[0];
      CLVF_B.Gain_n[1] = CLVF_P.Gain_Gain * CLVF_B.x_ddot_b[1];
      CLVF_B.Gain_n[2] = CLVF_P.Gain_Gain * CLVF_B.x_ddot_b[2];

      // If: '<S179>/If' incorporates:
      //   DataStoreRead: '<S184>/Universal_Time'
      //   Inport: '<S183>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S179>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S182>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.Gain_n[0], &CLVF_B.CalculateRunningMean,
          &CLVF_DW.CalculateRunningMean);

        // End of Outputs for SubSystem: '<S179>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S179>/Pass Current Gyro' incorporates:
        //   ActionPort: '<S183>/Action Port'

        CLVF_B.In1_mx = CLVF_B.Gain_n[0];

        // End of Outputs for SubSystem: '<S179>/Pass Current Gyro'
      }

      // End of If: '<S179>/If'

      // Sum: '<S179>/Subtract' incorporates:
      //   DataStoreWrite: '<S169>/RED_Px1'

      CLVF_DW.BLACK_IMU_Q = CLVF_B.In1_mx - CLVF_B.CalculateRunningMean.Mean;

      // S-Function (sdspbiquad): '<S175>/Digital Filter' incorporates:
      //   Gain: '<S169>/Gain1'

      CLVF_B.RED_Tz_Wrist = (CLVF_P.Gain1_Gain * CLVF_B.x_ddot_g[0] *
        0.29289321881345243 - -1.3007071811330761E-16 *
        CLVF_DW.DigitalFilter_FILT_STATES_m[0]) - 0.17157287525380996 *
        CLVF_DW.DigitalFilter_FILT_STATES_m[1];
      CLVF_B.DigitalFilter_p[0] = (2.0 * CLVF_DW.DigitalFilter_FILT_STATES_m[0]
        + CLVF_B.RED_Tz_Wrist) + CLVF_DW.DigitalFilter_FILT_STATES_m[1];
      CLVF_DW.DigitalFilter_FILT_STATES_m[1] =
        CLVF_DW.DigitalFilter_FILT_STATES_m[0];
      CLVF_DW.DigitalFilter_FILT_STATES_m[0] = CLVF_B.RED_Tz_Wrist;
      CLVF_B.RED_Tz_Wrist = (CLVF_P.Gain1_Gain * CLVF_B.x_ddot_g[1] *
        0.29289321881345243 - -1.3007071811330761E-16 *
        CLVF_DW.DigitalFilter_FILT_STATES_m[2]) - 0.17157287525380996 *
        CLVF_DW.DigitalFilter_FILT_STATES_m[3];
      CLVF_B.DigitalFilter_p[1] = (2.0 * CLVF_DW.DigitalFilter_FILT_STATES_m[2]
        + CLVF_B.RED_Tz_Wrist) + CLVF_DW.DigitalFilter_FILT_STATES_m[3];
      CLVF_DW.DigitalFilter_FILT_STATES_m[3] =
        CLVF_DW.DigitalFilter_FILT_STATES_m[2];
      CLVF_DW.DigitalFilter_FILT_STATES_m[2] = CLVF_B.RED_Tz_Wrist;
      CLVF_B.RED_Tz_Wrist = (CLVF_P.Gain1_Gain * CLVF_B.x_ddot_g[2] *
        0.29289321881345243 - -1.3007071811330761E-16 *
        CLVF_DW.DigitalFilter_FILT_STATES_m[4]) - 0.17157287525380996 *
        CLVF_DW.DigitalFilter_FILT_STATES_m[5];
      CLVF_B.DigitalFilter_p[2] = (2.0 * CLVF_DW.DigitalFilter_FILT_STATES_m[4]
        + CLVF_B.RED_Tz_Wrist) + CLVF_DW.DigitalFilter_FILT_STATES_m[5];
      CLVF_DW.DigitalFilter_FILT_STATES_m[5] =
        CLVF_DW.DigitalFilter_FILT_STATES_m[4];
      CLVF_DW.DigitalFilter_FILT_STATES_m[4] = CLVF_B.RED_Tz_Wrist;

      // If: '<S192>/If' incorporates:
      //   DataStoreRead: '<S197>/Universal_Time'
      //   Inport: '<S196>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S192>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S195>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.DigitalFilter_p[0],
          &CLVF_B.CalculateRunningMean_ch, &CLVF_DW.CalculateRunningMean_ch);

        // End of Outputs for SubSystem: '<S192>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S192>/Pass Current Accel' incorporates:
        //   ActionPort: '<S196>/Action Port'

        CLVF_B.In1_lw = CLVF_B.DigitalFilter_p[0];

        // End of Outputs for SubSystem: '<S192>/Pass Current Accel'
      }

      // End of If: '<S192>/If'

      // If: '<S193>/If' incorporates:
      //   DataStoreRead: '<S200>/Universal_Time'
      //   Inport: '<S199>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S193>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S198>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.DigitalFilter_p[1],
          &CLVF_B.CalculateRunningMean_k, &CLVF_DW.CalculateRunningMean_k);

        // End of Outputs for SubSystem: '<S193>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S193>/Pass Current Accel' incorporates:
        //   ActionPort: '<S199>/Action Port'

        CLVF_B.In1_gr = CLVF_B.DigitalFilter_p[1];

        // End of Outputs for SubSystem: '<S193>/Pass Current Accel'
      }

      // End of If: '<S193>/If'

      // If: '<S194>/If' incorporates:
      //   DataStoreRead: '<S203>/Universal_Time'
      //   Inport: '<S202>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S194>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S201>/Action Port'

        CLVF_CalculateRunningMean_b(CLVF_B.DigitalFilter_p[2],
          &CLVF_B.CalculateRunningMean_b, &CLVF_DW.CalculateRunningMean_b,
          &CLVF_P.CalculateRunningMean_b);

        // End of Outputs for SubSystem: '<S194>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S194>/Pass Current Accel' incorporates:
        //   ActionPort: '<S202>/Action Port'

        CLVF_B.In1_o0 = CLVF_B.DigitalFilter_p[2];

        // End of Outputs for SubSystem: '<S194>/Pass Current Accel'
      }

      // End of If: '<S194>/If'

      // MATLAB Function: '<S169>/ChangeOrientation' incorporates:
      //   Sum: '<S192>/Subtract'
      //   Sum: '<S193>/Subtract'
      //   Sum: '<S194>/Subtract'

      CLVF_ChangeOrientation(CLVF_B.In1_lw - CLVF_B.CalculateRunningMean_ch.Mean,
        CLVF_B.In1_gr - CLVF_B.CalculateRunningMean_k.Mean, CLVF_B.In1_o0 -
        CLVF_B.CalculateRunningMean_b.Subtract, &CLVF_B.sf_ChangeOrientation);

      // DataStoreWrite: '<S169>/RED_Px10'
      CLVF_DW.BLACK_IMU_Ay_b = CLVF_B.sf_ChangeOrientation.y[1];

      // DataStoreWrite: '<S169>/RED_Px11'
      CLVF_DW.BLACK_IMU_Az_b = CLVF_B.sf_ChangeOrientation.y[2];

      // If: '<S180>/If' incorporates:
      //   DataStoreRead: '<S187>/Universal_Time'
      //   Inport: '<S186>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S180>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S185>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.Gain_n[1],
          &CLVF_B.CalculateRunningMean_p, &CLVF_DW.CalculateRunningMean_p);

        // End of Outputs for SubSystem: '<S180>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S180>/Pass Current Gyro' incorporates:
        //   ActionPort: '<S186>/Action Port'

        CLVF_B.In1_m = CLVF_B.Gain_n[1];

        // End of Outputs for SubSystem: '<S180>/Pass Current Gyro'
      }

      // End of If: '<S180>/If'

      // Sum: '<S180>/Subtract' incorporates:
      //   DataStoreWrite: '<S169>/RED_Px2'

      CLVF_DW.BLACK_IMU_P = CLVF_B.In1_m - CLVF_B.CalculateRunningMean_p.Mean;

      // If: '<S181>/If' incorporates:
      //   DataStoreRead: '<S191>/Universal_Time'
      //   Inport: '<S190>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S181>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S188>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.Gain_n[2],
          &CLVF_B.CalculateRunningMean_c, &CLVF_DW.CalculateRunningMean_c);

        // End of Outputs for SubSystem: '<S181>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S181>/Pass Current Gyro' incorporates:
        //   ActionPort: '<S190>/Action Port'

        CLVF_B.In1_ln = CLVF_B.Gain_n[2];

        // End of Outputs for SubSystem: '<S181>/Pass Current Gyro'
      }

      // End of If: '<S181>/If'

      // Sum: '<S181>/Subtract' incorporates:
      //   DataStoreWrite: '<S169>/RED_Px3'

      CLVF_DW.BLACK_IMU_R = CLVF_B.In1_ln - CLVF_B.CalculateRunningMean_c.Mean;

      // MATLAB Function: '<S169>/Create Rotation Matrix' incorporates:
      //   DataStoreRead: '<S169>/Data Store Read'

      CLVF_CreateRotationMatrix(CLVF_DW.BLACK_Rz,
        &CLVF_B.sf_CreateRotationMatrix_hj);

      // DataStoreWrite: '<S169>/RED_Px4' incorporates:
      //   Product: '<S169>/Rotate F_b to F_I'

      CLVF_DW.BLACK_IMU_Ax_I = CLVF_B.sf_CreateRotationMatrix_hj.C_Ib[0] *
        CLVF_B.sf_ChangeOrientation.y[0] +
        CLVF_B.sf_CreateRotationMatrix_hj.C_Ib[2] *
        CLVF_B.sf_ChangeOrientation.y[1];

      // DataStoreWrite: '<S169>/RED_Px5' incorporates:
      //   Product: '<S169>/Rotate F_b to F_I'

      CLVF_DW.BLACK_IMU_Ay_I = CLVF_B.sf_CreateRotationMatrix_hj.C_Ib[1] *
        CLVF_B.sf_ChangeOrientation.y[0] +
        CLVF_B.sf_CreateRotationMatrix_hj.C_Ib[3] *
        CLVF_B.sf_ChangeOrientation.y[1];

      // DataStoreWrite: '<S169>/RED_Px6'
      CLVF_DW.BLACK_IMU_Ax_b = CLVF_B.sf_ChangeOrientation.y[0];
      CLVF_AHRS2(CLVF_B.DigitalFilter_p, CLVF_B.Gain_n,
                 CLVF_B.LSM9DS1IMUSensor_o3, &CLVF_B.AHRS2, &CLVF_DW.AHRS2,
                 &CLVF_P.AHRS2);

      // DataStoreWrite: '<S169>/RED_Px7'
      CLVF_DW.BLACK_AHRS_Q = CLVF_B.AHRS2.AHRS2_o2[0];

      // DataStoreWrite: '<S169>/RED_Px8'
      CLVF_DW.BLACK_AHRS_P = CLVF_B.AHRS2.AHRS2_o2[1];

      // DataStoreWrite: '<S169>/RED_Px9'
      CLVF_DW.BLACK_AHRS_R = CLVF_B.AHRS2.AHRS2_o2[2];

      // RelationalOperator: '<S189>/Compare' incorporates:
      //   Constant: '<S189>/Constant'

      CLVF_B.rtb_Compare_dy = (CLVF_B.In1_ln == CLVF_P.Constant_Value_i);

      // DiscreteIntegrator: '<S181>/Discrete-Time Integrator1' incorporates:
      //   DataStoreRead: '<S181>/Data Store Read1'
      //   DataStoreRead: '<S181>/Data Store Read3'
      //   DataStoreWrite: '<S181>/RED_Px4'

      if (CLVF_DW.DiscreteTimeIntegrator1_IC_LO_f != 0) {
        CLVF_DW.DiscreteTimeIntegrator1_DSTAT_m = CLVF_DW.BLACK_Rz;
      }

      if ((CLVF_B.rtb_Compare_dy && (CLVF_DW.DiscreteTimeIntegrator1_PrevR_d <=
            0)) || ((!CLVF_B.rtb_Compare_dy) &&
                    (CLVF_DW.DiscreteTimeIntegrator1_PrevR_d == 1))) {
        CLVF_DW.DiscreteTimeIntegrator1_DSTAT_m = CLVF_DW.BLACK_Rz;
      }

      CLVF_DW.BLACK_IMU_Psi = CLVF_P.DiscreteTimeIntegrator1_gainval *
        CLVF_DW.BLACK_IMU_R + CLVF_DW.DiscreteTimeIntegrator1_DSTAT_m;

      // End of DiscreteIntegrator: '<S181>/Discrete-Time Integrator1'

      // Update for DiscreteIntegrator: '<S181>/Discrete-Time Integrator1' incorporates:
      //   DataStoreWrite: '<S181>/RED_Px4'

      CLVF_DW.DiscreteTimeIntegrator1_IC_LO_f = 0U;
      CLVF_DW.DiscreteTimeIntegrator1_DSTAT_m = CLVF_DW.BLACK_IMU_Psi;
      CLVF_DW.DiscreteTimeIntegrator1_PrevR_d = static_cast<int8_T>
        (CLVF_B.rtb_Compare_dy);

      // End of Outputs for SubSystem: '<S6>/Change BLACK Behavior'
    }

    // End of If: '<S6>/This IF block determines whether or not to run the BLACK sim//exp' 

    // If: '<S6>/This IF block determines whether or not to run the BLUE sim//exp' incorporates:
    //   Constant: '<S172>/Constant'
    //   Constant: '<S6>/Constant1'

    if ((CLVF_P.WhoAmI == 3.0) && (CLVF_P.simMode == 0.0)) {
      // Outputs for IfAction SubSystem: '<S6>/Change BLUE Behavior' incorporates:
      //   ActionPort: '<S170>/Action Port'

      // Gain: '<S170>/Gain'
      CLVF_B.Gain_c[0] = CLVF_P.Gain_Gain_k * CLVF_B.x_ddot_b[0];
      CLVF_B.Gain_c[1] = CLVF_P.Gain_Gain_k * CLVF_B.x_ddot_b[1];
      CLVF_B.Gain_c[2] = CLVF_P.Gain_Gain_k * CLVF_B.x_ddot_b[2];

      // If: '<S210>/If' incorporates:
      //   DataStoreRead: '<S215>/Universal_Time'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S210>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S213>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.Gain_c[0],
          &CLVF_B.CalculateRunningMean_b2, &CLVF_DW.CalculateRunningMean_b2);

        // End of Outputs for SubSystem: '<S210>/Calculate Running Mean'
      }

      // End of If: '<S210>/If'

      // S-Function (sdspbiquad): '<S206>/Digital Filter' incorporates:
      //   Gain: '<S170>/Gain1'

      CLVF_B.RED_Tz_Wrist = (CLVF_P.Gain1_Gain_c * CLVF_B.x_ddot_g[0] *
        0.29289321881345243 - -1.3007071811330761E-16 *
        CLVF_DW.DigitalFilter_FILT_STATES_k[0]) - 0.17157287525380996 *
        CLVF_DW.DigitalFilter_FILT_STATES_k[1];
      CLVF_B.DigitalFilter_l[0] = (2.0 * CLVF_DW.DigitalFilter_FILT_STATES_k[0]
        + CLVF_B.RED_Tz_Wrist) + CLVF_DW.DigitalFilter_FILT_STATES_k[1];
      CLVF_DW.DigitalFilter_FILT_STATES_k[1] =
        CLVF_DW.DigitalFilter_FILT_STATES_k[0];
      CLVF_DW.DigitalFilter_FILT_STATES_k[0] = CLVF_B.RED_Tz_Wrist;
      CLVF_B.RED_Tz_Wrist = (CLVF_P.Gain1_Gain_c * CLVF_B.x_ddot_g[1] *
        0.29289321881345243 - -1.3007071811330761E-16 *
        CLVF_DW.DigitalFilter_FILT_STATES_k[2]) - 0.17157287525380996 *
        CLVF_DW.DigitalFilter_FILT_STATES_k[3];
      CLVF_B.DigitalFilter_l[1] = (2.0 * CLVF_DW.DigitalFilter_FILT_STATES_k[2]
        + CLVF_B.RED_Tz_Wrist) + CLVF_DW.DigitalFilter_FILT_STATES_k[3];
      CLVF_DW.DigitalFilter_FILT_STATES_k[3] =
        CLVF_DW.DigitalFilter_FILT_STATES_k[2];
      CLVF_DW.DigitalFilter_FILT_STATES_k[2] = CLVF_B.RED_Tz_Wrist;
      CLVF_B.RED_Tz_Wrist = (CLVF_P.Gain1_Gain_c * CLVF_B.x_ddot_g[2] *
        0.29289321881345243 - -1.3007071811330761E-16 *
        CLVF_DW.DigitalFilter_FILT_STATES_k[4]) - 0.17157287525380996 *
        CLVF_DW.DigitalFilter_FILT_STATES_k[5];
      CLVF_B.DigitalFilter_l[2] = (2.0 * CLVF_DW.DigitalFilter_FILT_STATES_k[4]
        + CLVF_B.RED_Tz_Wrist) + CLVF_DW.DigitalFilter_FILT_STATES_k[5];
      CLVF_DW.DigitalFilter_FILT_STATES_k[5] =
        CLVF_DW.DigitalFilter_FILT_STATES_k[4];
      CLVF_DW.DigitalFilter_FILT_STATES_k[4] = CLVF_B.RED_Tz_Wrist;

      // If: '<S223>/If' incorporates:
      //   DataStoreRead: '<S228>/Universal_Time'
      //   Inport: '<S227>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S223>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S226>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.DigitalFilter_l[0],
          &CLVF_B.CalculateRunningMean_m, &CLVF_DW.CalculateRunningMean_m);

        // End of Outputs for SubSystem: '<S223>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S223>/Pass Current Accel' incorporates:
        //   ActionPort: '<S227>/Action Port'

        CLVF_B.In1_dh = CLVF_B.DigitalFilter_l[0];

        // End of Outputs for SubSystem: '<S223>/Pass Current Accel'
      }

      // End of If: '<S223>/If'

      // If: '<S224>/If' incorporates:
      //   DataStoreRead: '<S231>/Universal_Time'
      //   Inport: '<S230>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S224>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S229>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.DigitalFilter_l[1],
          &CLVF_B.CalculateRunningMean_pa, &CLVF_DW.CalculateRunningMean_pa);

        // End of Outputs for SubSystem: '<S224>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S224>/Pass Current Accel' incorporates:
        //   ActionPort: '<S230>/Action Port'

        CLVF_B.In1_px = CLVF_B.DigitalFilter_l[1];

        // End of Outputs for SubSystem: '<S224>/Pass Current Accel'
      }

      // End of If: '<S224>/If'

      // If: '<S225>/If' incorporates:
      //   DataStoreRead: '<S234>/Universal_Time'
      //   Inport: '<S233>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S225>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S232>/Action Port'

        CLVF_CalculateRunningMean_b(CLVF_B.DigitalFilter_l[2],
          &CLVF_B.CalculateRunningMean_l, &CLVF_DW.CalculateRunningMean_l,
          &CLVF_P.CalculateRunningMean_l);

        // End of Outputs for SubSystem: '<S225>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S225>/Pass Current Accel' incorporates:
        //   ActionPort: '<S233>/Action Port'

        CLVF_B.In1_bl = CLVF_B.DigitalFilter_l[2];

        // End of Outputs for SubSystem: '<S225>/Pass Current Accel'
      }

      // End of If: '<S225>/If'

      // MATLAB Function: '<S170>/ChangeOrientation' incorporates:
      //   Sum: '<S223>/Subtract'
      //   Sum: '<S224>/Subtract'
      //   Sum: '<S225>/Subtract'

      CLVF_ChangeOrientation(CLVF_B.In1_dh - CLVF_B.CalculateRunningMean_m.Mean,
        CLVF_B.In1_px - CLVF_B.CalculateRunningMean_pa.Mean, CLVF_B.In1_bl -
        CLVF_B.CalculateRunningMean_l.Subtract, &CLVF_B.sf_ChangeOrientation_d);

      // If: '<S211>/If' incorporates:
      //   DataStoreRead: '<S218>/Universal_Time'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S211>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S216>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.Gain_c[1],
          &CLVF_B.CalculateRunningMean_a, &CLVF_DW.CalculateRunningMean_a);

        // End of Outputs for SubSystem: '<S211>/Calculate Running Mean'
      }

      // End of If: '<S211>/If'

      // If: '<S212>/If' incorporates:
      //   DataStoreRead: '<S222>/Universal_Time'
      //   Inport: '<S221>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S212>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S219>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.Gain_c[2],
          &CLVF_B.CalculateRunningMean_av, &CLVF_DW.CalculateRunningMean_av);

        // End of Outputs for SubSystem: '<S212>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S212>/Pass Current Gyro' incorporates:
        //   ActionPort: '<S221>/Action Port'

        CLVF_B.In1_df = CLVF_B.Gain_c[2];

        // End of Outputs for SubSystem: '<S212>/Pass Current Gyro'
      }

      // End of If: '<S212>/If'

      // MATLAB Function: '<S170>/Create Rotation Matrix' incorporates:
      //   DataStoreRead: '<S170>/Data Store Read'

      CLVF_CreateRotationMatrix(CLVF_DW.BLUE_Rz,
        &CLVF_B.sf_CreateRotationMatrix_nl);
      CLVF_AHRS2(CLVF_B.DigitalFilter_l, CLVF_B.Gain_c,
                 CLVF_B.LSM9DS1IMUSensor_o3, &CLVF_B.AHRS2_p, &CLVF_DW.AHRS2_p,
                 &CLVF_P.AHRS2_p);

      // RelationalOperator: '<S220>/Compare' incorporates:
      //   Constant: '<S220>/Constant'

      CLVF_B.rtb_Compare_dy = (CLVF_B.In1_df == CLVF_P.Constant_Value_m);

      // DiscreteIntegrator: '<S212>/Discrete-Time Integrator1' incorporates:
      //   DataStoreRead: '<S212>/Data Store Read1'

      if (CLVF_DW.DiscreteTimeIntegrator1_IC_LO_l != 0) {
        CLVF_DW.DiscreteTimeIntegrator1_DSTAT_a = CLVF_DW.BLUE_Rz;
      }

      if ((CLVF_B.rtb_Compare_dy && (CLVF_DW.DiscreteTimeIntegrator1_PrevR_h <=
            0)) || ((!CLVF_B.rtb_Compare_dy) &&
                    (CLVF_DW.DiscreteTimeIntegrator1_PrevR_h == 1))) {
        CLVF_DW.DiscreteTimeIntegrator1_DSTAT_a = CLVF_DW.BLUE_Rz;
      }

      // Update for DiscreteIntegrator: '<S212>/Discrete-Time Integrator1' incorporates:
      //   Sum: '<S212>/Subtract'

      CLVF_DW.DiscreteTimeIntegrator1_IC_LO_l = 0U;
      CLVF_DW.DiscreteTimeIntegrator1_DSTAT_a += (CLVF_B.In1_df -
        CLVF_B.CalculateRunningMean_av.Mean) *
        CLVF_P.DiscreteTimeIntegrator1_gainv_p;
      CLVF_DW.DiscreteTimeIntegrator1_PrevR_h = static_cast<int8_T>
        (CLVF_B.rtb_Compare_dy);

      // End of Outputs for SubSystem: '<S6>/Change BLUE Behavior'
    }

    // End of If: '<S6>/This IF block determines whether or not to run the BLUE sim//exp' 

    // If: '<S6>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
    //   Constant: '<S172>/Constant'
    //   Constant: '<S6>/Constant1'
    //   Gain: '<S171>/Gain1'

    if ((CLVF_P.WhoAmI == 1.0) && (CLVF_P.simMode == 0.0)) {
      // Outputs for IfAction SubSystem: '<S6>/Change RED Behavior' incorporates:
      //   ActionPort: '<S171>/Action Port'

      // Gain: '<S171>/Gain'
      CLVF_B.Gain_p[0] = CLVF_P.Gain_Gain_f * CLVF_B.x_ddot_b[0];
      CLVF_B.Gain_p[1] = CLVF_P.Gain_Gain_f * CLVF_B.x_ddot_b[1];
      CLVF_B.Gain_p[2] = CLVF_P.Gain_Gain_f * CLVF_B.x_ddot_b[2];

      // If: '<S241>/If' incorporates:
      //   DataStoreRead: '<S246>/Universal_Time'
      //   Inport: '<S245>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S241>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S244>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.Gain_p[0],
          &CLVF_B.CalculateRunningMean_mr, &CLVF_DW.CalculateRunningMean_mr);

        // End of Outputs for SubSystem: '<S241>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S241>/Pass Current Gyro' incorporates:
        //   ActionPort: '<S245>/Action Port'

        CLVF_B.In1_hk = CLVF_B.Gain_p[0];

        // End of Outputs for SubSystem: '<S241>/Pass Current Gyro'
      }

      // End of If: '<S241>/If'

      // Sum: '<S241>/Subtract' incorporates:
      //   DataStoreWrite: '<S171>/RED_Px1'

      CLVF_DW.RED_IMU_Q = CLVF_B.In1_hk - CLVF_B.CalculateRunningMean_mr.Mean;
      CLVF_B.x_ddot_g[0] *= CLVF_P.Gain1_Gain_b;
      CLVF_B.x_ddot_g[1] *= CLVF_P.Gain1_Gain_b;

      // S-Function (sdspbiquad): '<S237>/Digital Filter' incorporates:
      //   Gain: '<S171>/Gain1'

      CLVF_B.RED_Tz_Wrist = (0.29289321881345243 * CLVF_B.x_ddot_g[0] -
        -1.3007071811330761E-16 * CLVF_DW.DigitalFilter_FILT_STATES[0]) -
        0.17157287525380996 * CLVF_DW.DigitalFilter_FILT_STATES[1];
      CLVF_B.DigitalFilter[0] = (2.0 * CLVF_DW.DigitalFilter_FILT_STATES[0] +
        CLVF_B.RED_Tz_Wrist) + CLVF_DW.DigitalFilter_FILT_STATES[1];
      CLVF_DW.DigitalFilter_FILT_STATES[1] = CLVF_DW.DigitalFilter_FILT_STATES[0];
      CLVF_DW.DigitalFilter_FILT_STATES[0] = CLVF_B.RED_Tz_Wrist;
      CLVF_B.RED_Tz_Wrist = (0.29289321881345243 * CLVF_B.x_ddot_g[1] -
        -1.3007071811330761E-16 * CLVF_DW.DigitalFilter_FILT_STATES[2]) -
        0.17157287525380996 * CLVF_DW.DigitalFilter_FILT_STATES[3];
      CLVF_B.DigitalFilter[1] = (2.0 * CLVF_DW.DigitalFilter_FILT_STATES[2] +
        CLVF_B.RED_Tz_Wrist) + CLVF_DW.DigitalFilter_FILT_STATES[3];
      CLVF_DW.DigitalFilter_FILT_STATES[3] = CLVF_DW.DigitalFilter_FILT_STATES[2];
      CLVF_DW.DigitalFilter_FILT_STATES[2] = CLVF_B.RED_Tz_Wrist;
      CLVF_B.RED_Tz_Wrist = (CLVF_P.Gain1_Gain_b * CLVF_B.x_ddot_g[2] *
        0.29289321881345243 - -1.3007071811330761E-16 *
        CLVF_DW.DigitalFilter_FILT_STATES[4]) - 0.17157287525380996 *
        CLVF_DW.DigitalFilter_FILT_STATES[5];
      CLVF_B.DigitalFilter[2] = (2.0 * CLVF_DW.DigitalFilter_FILT_STATES[4] +
        CLVF_B.RED_Tz_Wrist) + CLVF_DW.DigitalFilter_FILT_STATES[5];
      CLVF_DW.DigitalFilter_FILT_STATES[5] = CLVF_DW.DigitalFilter_FILT_STATES[4];
      CLVF_DW.DigitalFilter_FILT_STATES[4] = CLVF_B.RED_Tz_Wrist;

      // If: '<S254>/If' incorporates:
      //   DataStoreRead: '<S259>/Universal_Time'
      //   Inport: '<S258>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S254>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S257>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.DigitalFilter[0],
          &CLVF_B.CalculateRunningMean_ck, &CLVF_DW.CalculateRunningMean_ck);

        // End of Outputs for SubSystem: '<S254>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S254>/Pass Current Accel' incorporates:
        //   ActionPort: '<S258>/Action Port'

        CLVF_B.In1_lp = CLVF_B.DigitalFilter[0];

        // End of Outputs for SubSystem: '<S254>/Pass Current Accel'
      }

      // End of If: '<S254>/If'

      // If: '<S255>/If' incorporates:
      //   DataStoreRead: '<S262>/Universal_Time'
      //   Inport: '<S261>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S255>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S260>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.DigitalFilter[1],
          &CLVF_B.CalculateRunningMean_e, &CLVF_DW.CalculateRunningMean_e);

        // End of Outputs for SubSystem: '<S255>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S255>/Pass Current Accel' incorporates:
        //   ActionPort: '<S261>/Action Port'

        CLVF_B.In1_e2 = CLVF_B.DigitalFilter[1];

        // End of Outputs for SubSystem: '<S255>/Pass Current Accel'
      }

      // End of If: '<S255>/If'

      // If: '<S256>/If' incorporates:
      //   DataStoreRead: '<S265>/Universal_Time'
      //   Inport: '<S264>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S256>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S263>/Action Port'

        CLVF_CalculateRunningMean_b(CLVF_B.DigitalFilter[2],
          &CLVF_B.CalculateRunningMean_mv, &CLVF_DW.CalculateRunningMean_mv,
          &CLVF_P.CalculateRunningMean_mv);

        // End of Outputs for SubSystem: '<S256>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S256>/Pass Current Accel' incorporates:
        //   ActionPort: '<S264>/Action Port'

        CLVF_B.In1_co = CLVF_B.DigitalFilter[2];

        // End of Outputs for SubSystem: '<S256>/Pass Current Accel'
      }

      // End of If: '<S256>/If'

      // MATLAB Function: '<S171>/ChangeOrientation' incorporates:
      //   Sum: '<S254>/Subtract'
      //   Sum: '<S255>/Subtract'
      //   Sum: '<S256>/Subtract'

      CLVF_ChangeOrientation(CLVF_B.In1_lp - CLVF_B.CalculateRunningMean_ck.Mean,
        CLVF_B.In1_e2 - CLVF_B.CalculateRunningMean_e.Mean, CLVF_B.In1_co -
        CLVF_B.CalculateRunningMean_mv.Subtract, &CLVF_B.sf_ChangeOrientation_di);

      // DataStoreWrite: '<S171>/RED_Px10'
      CLVF_DW.RED_IMU_Ax_b = CLVF_B.sf_ChangeOrientation_di.y[0];

      // DataStoreWrite: '<S171>/RED_Px11'
      CLVF_DW.RED_IMU_Ay_b = CLVF_B.sf_ChangeOrientation_di.y[1];

      // DataStoreWrite: '<S171>/RED_Px12'
      CLVF_DW.RED_IMU_Az_b = CLVF_B.sf_ChangeOrientation_di.y[2];

      // If: '<S242>/If' incorporates:
      //   DataStoreRead: '<S249>/Universal_Time'
      //   Inport: '<S248>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S242>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S247>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.Gain_p[1],
          &CLVF_B.CalculateRunningMean_bv, &CLVF_DW.CalculateRunningMean_bv);

        // End of Outputs for SubSystem: '<S242>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S242>/Pass Current Gyro' incorporates:
        //   ActionPort: '<S248>/Action Port'

        CLVF_B.In1_ajd = CLVF_B.Gain_p[1];

        // End of Outputs for SubSystem: '<S242>/Pass Current Gyro'
      }

      // End of If: '<S242>/If'

      // Sum: '<S242>/Subtract' incorporates:
      //   DataStoreWrite: '<S171>/RED_Px2'

      CLVF_DW.RED_IMU_P = CLVF_B.In1_ajd - CLVF_B.CalculateRunningMean_bv.Mean;

      // If: '<S243>/If' incorporates:
      //   DataStoreRead: '<S253>/Universal_Time'
      //   Inport: '<S252>/In1'

      if (CLVF_DW.Univ_Time < 5.0) {
        // Outputs for IfAction SubSystem: '<S243>/Calculate Running Mean' incorporates:
        //   ActionPort: '<S250>/Action Port'

        CLVF_CalculateRunningMean(CLVF_B.Gain_p[2],
          &CLVF_B.CalculateRunningMean_avn, &CLVF_DW.CalculateRunningMean_avn);

        // End of Outputs for SubSystem: '<S243>/Calculate Running Mean'
      } else {
        // Outputs for IfAction SubSystem: '<S243>/Pass Current Gyro' incorporates:
        //   ActionPort: '<S252>/Action Port'

        CLVF_B.In1_eu = CLVF_B.Gain_p[2];

        // End of Outputs for SubSystem: '<S243>/Pass Current Gyro'
      }

      // End of If: '<S243>/If'

      // Sum: '<S243>/Subtract' incorporates:
      //   DataStoreWrite: '<S171>/RED_Px3'

      CLVF_DW.RED_IMU_R = CLVF_B.In1_eu - CLVF_B.CalculateRunningMean_avn.Mean;

      // MATLAB Function: '<S171>/Create Rotation Matrix' incorporates:
      //   DataStoreRead: '<S171>/Data Store Read'

      CLVF_CreateRotationMatrix(CLVF_DW.RED_Rz,
        &CLVF_B.sf_CreateRotationMatrix_c);

      // DataStoreWrite: '<S171>/RED_Px4' incorporates:
      //   Product: '<S171>/Rotate F_b to F_I'

      CLVF_DW.RED_IMU_Ax_I = CLVF_B.sf_CreateRotationMatrix_c.C_Ib[0] *
        CLVF_B.sf_ChangeOrientation_di.y[0] +
        CLVF_B.sf_CreateRotationMatrix_c.C_Ib[2] *
        CLVF_B.sf_ChangeOrientation_di.y[1];

      // DataStoreWrite: '<S171>/RED_Px5' incorporates:
      //   Product: '<S171>/Rotate F_b to F_I'

      CLVF_DW.RED_IMU_Ay_I = CLVF_B.sf_CreateRotationMatrix_c.C_Ib[1] *
        CLVF_B.sf_ChangeOrientation_di.y[0] +
        CLVF_B.sf_CreateRotationMatrix_c.C_Ib[3] *
        CLVF_B.sf_ChangeOrientation_di.y[1];
      CLVF_AHRS2(CLVF_B.DigitalFilter, CLVF_B.Gain_p, CLVF_B.LSM9DS1IMUSensor_o3,
                 &CLVF_B.AHRS2_pn, &CLVF_DW.AHRS2_pn, &CLVF_P.AHRS2_pn);

      // DataStoreWrite: '<S171>/RED_Px7'
      CLVF_DW.RED_AHRS_P = CLVF_B.AHRS2_pn.AHRS2_o2[1];

      // DataStoreWrite: '<S171>/RED_Px8'
      CLVF_DW.RED_AHRS_Q = CLVF_B.AHRS2_pn.AHRS2_o2[0];

      // DataStoreWrite: '<S171>/RED_Px9'
      CLVF_DW.RED_AHRS_R = CLVF_B.AHRS2_pn.AHRS2_o2[2];

      // RelationalOperator: '<S251>/Compare' incorporates:
      //   Constant: '<S251>/Constant'

      CLVF_B.rtb_Compare_dy = (CLVF_B.In1_eu == CLVF_P.Constant_Value_fd);

      // DiscreteIntegrator: '<S243>/Discrete-Time Integrator1' incorporates:
      //   DataStoreRead: '<S243>/Data Store Read1'
      //   DataStoreRead: '<S243>/Data Store Read3'
      //   DataStoreWrite: '<S243>/RED_Px4'

      if (CLVF_DW.DiscreteTimeIntegrator1_IC_LO_b != 0) {
        CLVF_DW.DiscreteTimeIntegrator1_DSTAT_i = CLVF_DW.RED_Rz;
      }

      if ((CLVF_B.rtb_Compare_dy && (CLVF_DW.DiscreteTimeIntegrator1_PrevR_m <=
            0)) || ((!CLVF_B.rtb_Compare_dy) &&
                    (CLVF_DW.DiscreteTimeIntegrator1_PrevR_m == 1))) {
        CLVF_DW.DiscreteTimeIntegrator1_DSTAT_i = CLVF_DW.RED_Rz;
      }

      CLVF_DW.RED_IMU_Psi = CLVF_P.DiscreteTimeIntegrator1_gainv_g *
        CLVF_DW.RED_IMU_R + CLVF_DW.DiscreteTimeIntegrator1_DSTAT_i;

      // End of DiscreteIntegrator: '<S243>/Discrete-Time Integrator1'

      // Update for DiscreteIntegrator: '<S243>/Discrete-Time Integrator1' incorporates:
      //   DataStoreWrite: '<S243>/RED_Px4'

      CLVF_DW.DiscreteTimeIntegrator1_IC_LO_b = 0U;
      CLVF_DW.DiscreteTimeIntegrator1_DSTAT_i = CLVF_DW.RED_IMU_Psi;
      CLVF_DW.DiscreteTimeIntegrator1_PrevR_m = static_cast<int8_T>
        (CLVF_B.rtb_Compare_dy);

      // End of Outputs for SubSystem: '<S6>/Change RED Behavior'
    }

    // End of If: '<S6>/This IF block determines whether or not to run the RED sim//exp ' 

    // Clock: '<S15>/Set Universal Time (If this is a simulation)'
    CLVF_B.SetUniversalTimeIfthisisasimula = CLVF_M->Timing.t[0];
  }

  // Matfile logging
  rt_UpdateTXYLogVars(CLVF_M->rtwLogInfo, (CLVF_M->Timing.t));

  {
    char_T *sErr;
    real_T (*lastU)[3];

    // Update for If: '<Root>/Separate Phases'
    switch (CLVF_DW.SeparatePhases_ActiveSubsystem) {
     case 0:
     case 1:
     case 6:
      break;

     case 2:
      // Update for IfAction SubSystem: '<Root>/Phase #2:  Move to  Initial Position' incorporates:
      //   ActionPort: '<S10>/Action Port'

      // Update for If: '<S10>/This IF block determines whether or not to run the BLACK sim//exp' 
      if (CLVF_DW.ThisIFblockdetermineswhether_dy == 0) {
        // Update for IfAction SubSystem: '<S10>/Change BLACK Behavior' incorporates:
        //   ActionPort: '<S274>/Action Port'

        // Update for Delay: '<S284>/Delay1'
        CLVF_DW.icLoad_ns = 0U;
        CLVF_DW.Delay1_DSTATE_k = CLVF_B.Subtract_ou;

        // Update for Delay: '<S286>/Delay1'
        CLVF_DW.icLoad_k1 = 0U;
        CLVF_DW.Delay1_DSTATE_dv = CLVF_B.Subtract1_m;

        // Update for Delay: '<S282>/Delay1'
        CLVF_DW.icLoad_ep = 0U;
        CLVF_DW.Delay1_DSTATE_j = CLVF_B.sf_MATLABFunction4.e_out;

        // End of Update for SubSystem: '<S10>/Change BLACK Behavior'
      }

      // End of Update for If: '<S10>/This IF block determines whether or not to run the BLACK sim//exp' 

      // Update for If: '<S10>/This IF block determines whether or not to run the BLUE sim//exp' 
      if (CLVF_DW.ThisIFblockdetermineswhether_jm == 0) {
        // Update for IfAction SubSystem: '<S10>/Change BLUE Behavior' incorporates:
        //   ActionPort: '<S275>/Action Port'

        // Update for Delay: '<S297>/Delay1'
        CLVF_DW.icLoad_dy = 0U;
        CLVF_DW.Delay1_DSTATE_fb = CLVF_B.Subtract_j;

        // Update for Delay: '<S299>/Delay1'
        CLVF_DW.icLoad_hf = 0U;
        CLVF_DW.Delay1_DSTATE_n1 = CLVF_B.Subtract1_o;

        // Update for Delay: '<S295>/Delay1'
        CLVF_DW.icLoad_k = 0U;
        CLVF_DW.Delay1_DSTATE_cq = CLVF_B.sf_MATLABFunction4_c.e_out;

        // End of Update for SubSystem: '<S10>/Change BLUE Behavior'
      }

      // End of Update for If: '<S10>/This IF block determines whether or not to run the BLUE sim//exp' 

      // Update for If: '<S10>/This IF block determines whether or not to run the RED sim//exp ' 
      if (CLVF_DW.ThisIFblockdetermineswhether_aj == 0) {
        // Update for IfAction SubSystem: '<S10>/Change RED Behavior' incorporates:
        //   ActionPort: '<S276>/Action Port'

        // Update for Delay: '<S308>/Delay1'
        CLVF_DW.icLoad_ji = 0U;
        CLVF_DW.Delay1_DSTATE_ct = CLVF_B.sf_MATLABFunction4_b.e_out;

        // Update for Delay: '<S310>/Delay1'
        CLVF_DW.icLoad_a1 = 0U;
        CLVF_DW.Delay1_DSTATE_bm = CLVF_B.Subtract_f;

        // Update for Delay: '<S312>/Delay1'
        CLVF_DW.icLoad_o0 = 0U;
        CLVF_DW.Delay1_DSTATE_pr = CLVF_B.Subtract1_dr;

        // End of Update for SubSystem: '<S10>/Change RED Behavior'
      }

      // End of Update for If: '<S10>/This IF block determines whether or not to run the RED sim//exp ' 
      // End of Update for SubSystem: '<Root>/Phase #2:  Move to  Initial Position' 
      break;

     case 3:
      // Update for IfAction SubSystem: '<Root>/Phase #3: Experiment' incorporates:
      //   ActionPort: '<S11>/Action Port'

      // Update for If: '<S11>/This IF block determines whether or not to run the BLACK sim//exp' 
      if (CLVF_DW.ThisIFblockdetermineswhethero_i == 0) {
        // Update for IfAction SubSystem: '<S11>/Change BLACK Behavior' incorporates:
        //   ActionPort: '<S317>/Action Port'

        // Update for If: '<S317>/Experiment Sub-Phases'
        switch (CLVF_DW.ExperimentSubPhases_ActiveSub_p) {
         case 0:
          // Update for IfAction SubSystem: '<S317>/Sub-Phase #1' incorporates:
          //   ActionPort: '<S321>/Action Port'

          // Update for Delay: '<S332>/Delay1'
          CLVF_DW.icLoad_iz = 0U;
          CLVF_DW.Delay1_DSTATE_g = CLVF_B.Subtract_n;

          // Update for Delay: '<S334>/Delay1'
          CLVF_DW.icLoad_p = 0U;
          CLVF_DW.Delay1_DSTATE_g2 = CLVF_B.Subtract1_n;

          // Update for Delay: '<S330>/Delay1'
          CLVF_DW.icLoad_dm = 0U;
          CLVF_DW.Delay1_DSTATE_d = CLVF_B.sf_MATLABFunction4_f.e_out;

          // End of Update for SubSystem: '<S317>/Sub-Phase #1'
          break;

         case 1:
         case 2:
          break;

         case 3:
          // Update for IfAction SubSystem: '<S317>/Sub-Phase #4' incorporates:
          //   ActionPort: '<S324>/Action Port'

          // Update for Delay: '<S354>/Delay'
          CLVF_DW.Delay_DSTATE = CLVF_B.newCnt;

          // Update for Delay: '<S344>/Delay1'
          CLVF_DW.icLoad_en = 0U;
          CLVF_DW.Delay1_DSTATE_e = CLVF_B.sf_MATLABFunction4_i.e_out;

          // Update for DiscreteTransferFcn: '<S339>/1Hz LP Filter'
          CLVF_DW.uHzLPFilter_states[1] = CLVF_DW.uHzLPFilter_states[0];
          CLVF_DW.uHzLPFilter_states[0] = CLVF_DW.uHzLPFilter_tmp;

          // Update for DiscreteTransferFcn: '<S342>/1Hz LP Filter1'
          CLVF_DW.uHzLPFilter1_states_l[1] = CLVF_DW.uHzLPFilter1_states_l[0];
          CLVF_DW.uHzLPFilter1_states_l[0] = CLVF_DW.uHzLPFilter1_tmp_i;

          // Update for DiscreteTransferFcn: '<S342>/1Hz LP Filter'
          CLVF_DW.uHzLPFilter_states_m[1] = CLVF_DW.uHzLPFilter_states_m[0];
          CLVF_DW.uHzLPFilter_states_m[0] = CLVF_DW.uHzLPFilter_tmp_g;

          // Update for Delay: '<S363>/Delay1'
          CLVF_DW.icLoad_jp = 0U;
          CLVF_DW.Delay1_DSTATE_hm = CLVF_B.DataStoreRead2;

          // Update for If: '<S349>/If'
          switch (CLVF_DW.If_ActiveSubsystem) {
           case 0:
            // Update for IfAction SubSystem: '<S349>/CLVF' incorporates:
            //   ActionPort: '<S351>/Action Port'

            // Update for Derivative: '<S351>/Derivative'
            if (CLVF_DW.TimeStampA_i == (rtInf)) {
              CLVF_DW.TimeStampA_i = CLVF_M->Timing.t[0];
              lastU = &CLVF_DW.LastUAtTimeA_i;
            } else if (CLVF_DW.TimeStampB_p == (rtInf)) {
              CLVF_DW.TimeStampB_p = CLVF_M->Timing.t[0];
              lastU = &CLVF_DW.LastUAtTimeB_k;
            } else if (CLVF_DW.TimeStampA_i < CLVF_DW.TimeStampB_p) {
              CLVF_DW.TimeStampA_i = CLVF_M->Timing.t[0];
              lastU = &CLVF_DW.LastUAtTimeA_i;
            } else {
              CLVF_DW.TimeStampB_p = CLVF_M->Timing.t[0];
              lastU = &CLVF_DW.LastUAtTimeB_k;
            }

            (*lastU)[0] = CLVF_B.h[0];
            (*lastU)[1] = CLVF_B.h[1];
            (*lastU)[2] = CLVF_B.h[2];

            // End of Update for Derivative: '<S351>/Derivative'
            // End of Update for SubSystem: '<S349>/CLVF'
            break;

           case 1:
            // Update for IfAction SubSystem: '<S349>/Lyapunov' incorporates:
            //   ActionPort: '<S353>/Action Port'

            // Update for Derivative: '<S353>/Derivative'
            if (CLVF_DW.TimeStampA == (rtInf)) {
              CLVF_DW.TimeStampA = CLVF_M->Timing.t[0];
              lastU = &CLVF_DW.LastUAtTimeA;
            } else if (CLVF_DW.TimeStampB == (rtInf)) {
              CLVF_DW.TimeStampB = CLVF_M->Timing.t[0];
              lastU = &CLVF_DW.LastUAtTimeB;
            } else if (CLVF_DW.TimeStampA < CLVF_DW.TimeStampB) {
              CLVF_DW.TimeStampA = CLVF_M->Timing.t[0];
              lastU = &CLVF_DW.LastUAtTimeA;
            } else {
              CLVF_DW.TimeStampB = CLVF_M->Timing.t[0];
              lastU = &CLVF_DW.LastUAtTimeB;
            }

            (*lastU)[0] = CLVF_B.hC_T[0];
            (*lastU)[1] = CLVF_B.hC_T[1];
            (*lastU)[2] = CLVF_B.hC_T[2];

            // End of Update for Derivative: '<S353>/Derivative'
            // End of Update for SubSystem: '<S349>/Lyapunov'
            break;
          }

          // End of Update for If: '<S349>/If'

          // Update for Delay: '<S364>/Delay1'
          CLVF_DW.icLoad_of = 0U;

          // Update for Delay: '<S365>/Delay1'
          CLVF_DW.icLoad_i = 0U;

          // Update for Delay: '<S366>/Delay1'
          CLVF_DW.icLoad_ay = 0U;

          // End of Update for SubSystem: '<S317>/Sub-Phase #4'
          break;
        }

        // End of Update for If: '<S317>/Experiment Sub-Phases'
        // End of Update for SubSystem: '<S11>/Change BLACK Behavior'
      }

      // End of Update for If: '<S11>/This IF block determines whether or not to run the BLACK sim//exp' 

      // Update for If: '<S11>/This IF block determines whether or not to run the BLUE sim//exp' 
      if (CLVF_DW.ThisIFblockdetermineswhether_aa == 0) {
        // Update for IfAction SubSystem: '<S11>/Change BLUE Behavior' incorporates:
        //   ActionPort: '<S318>/Action Port'

        // Update for If: '<S318>/Experiment Sub-Phases'
        switch (CLVF_DW.ExperimentSubPhases_ActiveSub_h) {
         case 0:
          // Update for IfAction SubSystem: '<S318>/Sub-Phase #1' incorporates:
          //   ActionPort: '<S374>/Action Port'

          CLVF_SubPhase1_Update(&CLVF_B.SubPhase1_g, &CLVF_DW.SubPhase1_g);

          // End of Update for SubSystem: '<S318>/Sub-Phase #1'
          break;

         case 1:
         case 2:
          break;

         case 3:
          // Update for IfAction SubSystem: '<S318>/Sub-Phase #4' incorporates:
          //   ActionPort: '<S377>/Action Port'

          CLVF_SubPhase1_Update(&CLVF_B.SubPhase4_o, &CLVF_DW.SubPhase4_o);

          // End of Update for SubSystem: '<S318>/Sub-Phase #4'
          break;
        }

        // End of Update for If: '<S318>/Experiment Sub-Phases'
        // End of Update for SubSystem: '<S11>/Change BLUE Behavior'
      }

      // End of Update for If: '<S11>/This IF block determines whether or not to run the BLUE sim//exp' 

      // Update for If: '<S11>/This IF block determines whether or not to run the RED sim//exp ' 
      if (CLVF_DW.ThisIFblockdetermineswhethero_f == 0) {
        // Update for IfAction SubSystem: '<S11>/Change RED Behavior' incorporates:
        //   ActionPort: '<S319>/Action Port'

        // Update for If: '<S319>/Experiment Sub-Phases'
        switch (CLVF_DW.ExperimentSubPhases_ActiveSubsy) {
         case 0:
          // Update for IfAction SubSystem: '<S319>/Sub-Phase #1' incorporates:
          //   ActionPort: '<S405>/Action Port'

          // Update for Delay: '<S414>/Delay1'
          CLVF_DW.icLoad_c = 0U;
          CLVF_DW.Delay1_DSTATE_le = CLVF_B.sf_MATLABFunction4_p.e_out;

          // Update for Delay: '<S416>/Delay1'
          CLVF_DW.icLoad_l = 0U;
          CLVF_DW.Delay1_DSTATE_nl = CLVF_B.Subtract_p;

          // Update for Delay: '<S418>/Delay1'
          CLVF_DW.icLoad_jz = 0U;
          CLVF_DW.Delay1_DSTATE_h2 = CLVF_B.Subtract1_k;

          // End of Update for SubSystem: '<S319>/Sub-Phase #1'
          break;

         case 1:
         case 2:
          break;

         case 3:
          // Update for IfAction SubSystem: '<S319>/Sub-Phase #4' incorporates:
          //   ActionPort: '<S408>/Action Port'

          // Update for Delay: '<S428>/Delay1'
          CLVF_DW.icLoad_jk = 0U;
          CLVF_DW.Delay1_DSTATE_a = CLVF_B.sf_MATLABFunction4_e.e_out;

          // Update for DiscreteTransferFcn: '<S423>/1Hz LP Filter1'
          CLVF_DW.uHzLPFilter1_states[1] = CLVF_DW.uHzLPFilter1_states[0];
          CLVF_DW.uHzLPFilter1_states[0] = CLVF_DW.uHzLPFilter1_tmp;

          // Update for Delay: '<S430>/Delay1'
          CLVF_DW.icLoad_fc = 0U;
          CLVF_DW.Delay1_DSTATE_nv = CLVF_B.Subtract_k;

          // Update for Delay: '<S432>/Delay1'
          CLVF_DW.icLoad_n = 0U;
          CLVF_DW.Delay1_DSTATE_m2 = CLVF_B.Subtract1_a;

          // End of Update for SubSystem: '<S319>/Sub-Phase #4'
          break;
        }

        // End of Update for If: '<S319>/Experiment Sub-Phases'
        // End of Update for SubSystem: '<S11>/Change RED Behavior'
      }

      // End of Update for If: '<S11>/This IF block determines whether or not to run the RED sim//exp ' 
      // End of Update for SubSystem: '<Root>/Phase #3: Experiment'
      break;

     case 4:
      // Update for IfAction SubSystem: '<Root>/Phase #4:  Return Home' incorporates:
      //   ActionPort: '<S12>/Action Port'

      // Update for If: '<S12>/This IF block determines whether or not to run the BLACK sim//exp' 
      if (CLVF_DW.ThisIFblockdetermineswhethero_d == 0) {
        // Update for IfAction SubSystem: '<S12>/Change BLACK Behavior' incorporates:
        //   ActionPort: '<S437>/Action Port'

        // Update for Delay: '<S447>/Delay1'
        CLVF_DW.icLoad_dq = 0U;
        CLVF_DW.Delay1_DSTATE_lt = CLVF_B.Subtract_d;

        // Update for Delay: '<S449>/Delay1'
        CLVF_DW.icLoad_h = 0U;
        CLVF_DW.Delay1_DSTATE_im = CLVF_B.Subtract1_d;

        // Update for Delay: '<S445>/Delay1'
        CLVF_DW.icLoad_o = 0U;
        CLVF_DW.Delay1_DSTATE_f = CLVF_B.sf_MATLABFunction4_l.e_out;

        // End of Update for SubSystem: '<S12>/Change BLACK Behavior'
      }

      // End of Update for If: '<S12>/This IF block determines whether or not to run the BLACK sim//exp' 

      // Update for If: '<S12>/This IF block determines whether or not to run the BLUE sim//exp' 
      if (CLVF_DW.ThisIFblockdetermineswhether_jc == 0) {
        // Update for IfAction SubSystem: '<S12>/Change BLUE Behavior' incorporates:
        //   ActionPort: '<S438>/Action Port'

        C_ChangeBLUEBehavior_Update(&CLVF_B.ChangeBLUEBehavior_gj,
          &CLVF_DW.ChangeBLUEBehavior_gj);

        // End of Update for SubSystem: '<S12>/Change BLUE Behavior'
      }

      // End of Update for If: '<S12>/This IF block determines whether or not to run the BLUE sim//exp' 

      // Update for If: '<S12>/This IF block determines whether or not to run the RED sim//exp ' 
      if (CLVF_DW.ThisIFblockdetermineswhether_a4 == 0) {
        // Update for IfAction SubSystem: '<S12>/Change RED Behavior' incorporates:
        //   ActionPort: '<S439>/Action Port'

        // Update for Delay: '<S471>/Delay1'
        CLVF_DW.icLoad_dh = 0U;
        CLVF_DW.Delay1_DSTATE_i = CLVF_B.sf_MATLABFunction4_n.e_out;

        // Update for Delay: '<S473>/Delay1'
        CLVF_DW.icLoad_a2 = 0U;
        CLVF_DW.Delay1_DSTATE_h = CLVF_B.Subtract_l;

        // Update for Delay: '<S475>/Delay1'
        CLVF_DW.icLoad_f = 0U;
        CLVF_DW.Delay1_DSTATE_l = CLVF_B.Subtract1_g;

        // End of Update for SubSystem: '<S12>/Change RED Behavior'
      }

      // End of Update for If: '<S12>/This IF block determines whether or not to run the RED sim//exp ' 
      // End of Update for SubSystem: '<Root>/Phase #4:  Return Home'
      break;

     case 5:
      // Update for IfAction SubSystem: '<Root>/Phase #5:  Hold Home' incorporates:
      //   ActionPort: '<S13>/Action Port'

      // Update for If: '<S13>/This IF block determines whether or not to run the BLACK sim//exp' 
      if (CLVF_DW.ThisIFblockdetermineswhethero_b == 0) {
        // Update for IfAction SubSystem: '<S13>/Change BLACK Behavior' incorporates:
        //   ActionPort: '<S480>/Action Port'

        // Update for Delay: '<S490>/Delay1'
        CLVF_DW.icLoad_e = 0U;
        CLVF_DW.Delay1_DSTATE_o = CLVF_B.Subtract_o;

        // Update for Delay: '<S492>/Delay1'
        CLVF_DW.icLoad_m = 0U;
        CLVF_DW.Delay1_DSTATE_p = CLVF_B.Subtract1_c;

        // Update for Delay: '<S488>/Delay1'
        CLVF_DW.icLoad_d = 0U;
        CLVF_DW.Delay1_DSTATE_b = CLVF_B.sf_MATLABFunction4_d.e_out;

        // End of Update for SubSystem: '<S13>/Change BLACK Behavior'
      }

      // End of Update for If: '<S13>/This IF block determines whether or not to run the BLACK sim//exp' 

      // Update for If: '<S13>/This IF block determines whether or not to run the BLUE sim//exp' 
      if (CLVF_DW.ThisIFblockdetermineswhethero_n == 0) {
        // Update for IfAction SubSystem: '<S13>/Change BLUE Behavior' incorporates:
        //   ActionPort: '<S481>/Action Port'

        C_ChangeBLUEBehavior_Update(&CLVF_B.ChangeBLUEBehavior_b,
          &CLVF_DW.ChangeBLUEBehavior_b);

        // End of Update for SubSystem: '<S13>/Change BLUE Behavior'
      }

      // End of Update for If: '<S13>/This IF block determines whether or not to run the BLUE sim//exp' 

      // Update for If: '<S13>/This IF block determines whether or not to run the RED sim//exp ' 
      if (CLVF_DW.ThisIFblockdetermineswhethero_j == 0) {
        // Update for IfAction SubSystem: '<S13>/Change RED Behavior' incorporates:
        //   ActionPort: '<S482>/Action Port'

        // Update for Delay: '<S516>/Delay1'
        CLVF_DW.icLoad = 0U;
        CLVF_DW.Delay1_DSTATE = CLVF_B.Subtract;

        // Update for Delay: '<S518>/Delay1'
        CLVF_DW.icLoad_j = 0U;
        CLVF_DW.Delay1_DSTATE_m = CLVF_B.Subtract1;

        // Update for Delay: '<S514>/Delay1'
        CLVF_DW.icLoad_a = 0U;
        CLVF_DW.Delay1_DSTATE_n = CLVF_B.sf_MATLABFunction4_df.e_out;

        // End of Update for SubSystem: '<S13>/Change RED Behavior'
      }

      // End of Update for If: '<S13>/This IF block determines whether or not to run the RED sim//exp ' 
      // End of Update for SubSystem: '<Root>/Phase #5:  Hold Home'
      break;
    }

    // End of Update for If: '<Root>/Separate Phases'

    // Update for If: '<S5>/If performing an experiment, grab the PhaseSpace data. Otherwise, use a clock to set time in SIM.' 
    if (CLVF_DW.Ifperforminganexperimentgrabthe == 0) {
      // Update for IfAction SubSystem: '<S5>/Use Hardware to Obtain States' incorporates:
      //   ActionPort: '<S75>/Action Port'

      // Update for If: '<S75>/Check whether both platforms are being used, and if so then use RED to send data to BLACK ' 
      switch (CLVF_DW.Checkwhetherbothplatformsarebei) {
       case 0:
        // Update for IfAction SubSystem: '<S75>/Using RED, BLACK, BLUE, or RED + ARM' incorporates:
        //   ActionPort: '<S77>/Action Port'

        // Update for Delay: '<S153>/Delay'
        CLVF_DW.Delay_DSTATE_m = CLVF_B.DataTypeConversion_o;

        // Update for Delay: '<S147>/Delay1'
        CLVF_DW.icLoad_po = 0U;
        CLVF_DW.Delay1_DSTATE_ds = CLVF_B.In1_ak[4];

        // Update for Delay: '<S142>/Delay1'
        CLVF_DW.icLoad_nx2 = 0U;
        CLVF_DW.Delay1_DSTATE_bx = CLVF_B.dividebydeltaT_d;

        // Update for Delay: '<S148>/Delay1'
        CLVF_DW.icLoad_on = 0U;
        CLVF_DW.Delay1_DSTATE_bq = CLVF_B.In1_ak[5];

        // Update for Delay: '<S143>/Delay1'
        CLVF_DW.icLoad_lw = 0U;
        CLVF_DW.Delay1_DSTATE_n2 = CLVF_B.dividebydeltaT_ka;

        // Update for Delay: '<S149>/Delay1'
        CLVF_DW.icLoad_dx = 0U;
        CLVF_DW.Delay1_DSTATE_az = CLVF_B.Unwrap1_pz;

        // Update for Delay: '<S144>/Delay1'
        CLVF_DW.icLoad_pz = 0U;
        CLVF_DW.Delay1_DSTATE_nu = CLVF_B.dividebydeltaT_l;

        // Update for Delay: '<S141>/Delay1'
        CLVF_DW.icLoad_jj = 0U;
        CLVF_DW.Delay1_DSTATE_d3 = CLVF_B.In1_ak[1];

        // Update for Delay: '<S150>/Delay1'
        CLVF_DW.icLoad_fv = 0U;
        CLVF_DW.Delay1_DSTATE_hy = CLVF_B.dividebydeltaT_nn;

        // Update for Delay: '<S145>/Delay1'
        CLVF_DW.icLoad_jqu = 0U;
        CLVF_DW.Delay1_DSTATE_gb = CLVF_B.In1_ak[2];

        // Update for Delay: '<S151>/Delay1'
        CLVF_DW.icLoad_b3 = 0U;
        CLVF_DW.Delay1_DSTATE_fn = CLVF_B.dividebydeltaT_pf;

        // Update for Delay: '<S146>/Delay1'
        CLVF_DW.icLoad_cw = 0U;
        CLVF_DW.Delay1_DSTATE_ek = CLVF_B.Unwrap_d;

        // Update for Delay: '<S152>/Delay1'
        CLVF_DW.icLoad_hrx = 0U;
        CLVF_DW.Delay1_DSTATE_k1 = CLVF_B.dividebydeltaT_lz;

        // End of Update for SubSystem: '<S75>/Using RED, BLACK, BLUE, or RED + ARM' 
        break;

       case 1:
        // Update for IfAction SubSystem: '<S75>/Using RED+BLACK, or RED+BLACK+ARM' incorporates:
        //   ActionPort: '<S76>/Action Port'

        // Update for If: '<S76>/This IF block determines whether or not to run the BLACK sim//exp' 
        if (CLVF_DW.ThisIFblockdetermineswhethero_c == 0) {
          // Update for IfAction SubSystem: '<S76>/Obtain BLACK States' incorporates:
          //   ActionPort: '<S79>/Action Port'

          // Update for Delay: '<S87>/Delay1'
          CLVF_DW.icLoad_ha = 0U;
          CLVF_DW.Delay1_DSTATE_hp = CLVF_B.UDPReceive_o1[4];

          // Update for Delay: '<S82>/Delay1'
          CLVF_DW.icLoad_oh = 0U;
          CLVF_DW.Delay1_DSTATE_bi = CLVF_B.dividebydeltaT_p;

          // Update for Delay: '<S88>/Delay1'
          CLVF_DW.icLoad_ij = 0U;
          CLVF_DW.Delay1_DSTATE_fo = CLVF_B.UDPReceive_o1[5];

          // Update for Delay: '<S83>/Delay1'
          CLVF_DW.icLoad_dr = 0U;
          CLVF_DW.Delay1_DSTATE_f4 = CLVF_B.dividebydeltaT_j;

          // Update for Delay: '<S89>/Delay1'
          CLVF_DW.icLoad_ip = 0U;
          CLVF_DW.Delay1_DSTATE_lb = CLVF_B.Unwrap1_p;

          // Update for Delay: '<S84>/Delay1'
          CLVF_DW.icLoad_lp = 0U;
          CLVF_DW.Delay1_DSTATE_i2 = CLVF_B.dividebydeltaT_k;

          // Update for Delay: '<S81>/Delay1'
          CLVF_DW.icLoad_nj = 0U;
          CLVF_DW.Delay1_DSTATE_lm = CLVF_B.UDPReceive_o1[1];

          // Update for Delay: '<S90>/Delay1'
          CLVF_DW.icLoad_av = 0U;
          CLVF_DW.Delay1_DSTATE_ctf = CLVF_B.dividebydeltaT_iv;

          // Update for Delay: '<S85>/Delay1'
          CLVF_DW.icLoad_ib = 0U;
          CLVF_DW.Delay1_DSTATE_du = CLVF_B.UDPReceive_o1[2];

          // Update for Delay: '<S91>/Delay1'
          CLVF_DW.icLoad_hm = 0U;
          CLVF_DW.Delay1_DSTATE_gu = CLVF_B.dividebydeltaT_g;

          // Update for Delay: '<S86>/Delay1'
          CLVF_DW.icLoad_jh = 0U;
          CLVF_DW.Delay1_DSTATE_jc = CLVF_B.Unwrap_o;

          // Update for Delay: '<S92>/Delay1'
          CLVF_DW.icLoad_bb = 0U;
          CLVF_DW.Delay1_DSTATE_c4 = CLVF_B.dividebydeltaT_nw;

          // End of Update for SubSystem: '<S76>/Obtain BLACK States'
        }

        // End of Update for If: '<S76>/This IF block determines whether or not to run the BLACK sim//exp' 

        // Update for If: '<S76>/This IF block determines whether or not to run the RED sim//exp ' 
        if (CLVF_DW.ThisIFblockdetermineswhether_cx == 0) {
          // Update for IfAction SubSystem: '<S76>/Obtain RED States' incorporates:
          //   ActionPort: '<S80>/Action Port'

          // Update for Delay: '<S121>/Delay'
          CLVF_DW.Delay_DSTATE_k2 = CLVF_B.DataTypeConversion;

          // Update for Delay: '<S115>/Delay1'
          CLVF_DW.icLoad_bo = 0U;
          CLVF_DW.Delay1_DSTATE_mo = CLVF_B.In1_mk[4];

          // Update for Delay: '<S110>/Delay1'
          CLVF_DW.icLoad_lm = 0U;
          CLVF_DW.Delay1_DSTATE_ig = CLVF_B.dividebydeltaT_f;

          // Update for Delay: '<S116>/Delay1'
          CLVF_DW.icLoad_h2 = 0U;
          CLVF_DW.Delay1_DSTATE_lz = CLVF_B.In1_mk[5];

          // Update for Delay: '<S111>/Delay1'
          CLVF_DW.icLoad_fc0 = 0U;
          CLVF_DW.Delay1_DSTATE_ew = CLVF_B.dividebydeltaT_bz;

          // Update for Delay: '<S117>/Delay1'
          CLVF_DW.icLoad_jq = 0U;
          CLVF_DW.Delay1_DSTATE_io = CLVF_B.Unwrap1;

          // Update for Delay: '<S112>/Delay1'
          CLVF_DW.icLoad_ab = 0U;
          CLVF_DW.Delay1_DSTATE_bt = CLVF_B.dividebydeltaT_e;

          // Update for Delay: '<S109>/Delay1'
          CLVF_DW.icLoad_ez = 0U;
          CLVF_DW.Delay1_DSTATE_lg = CLVF_B.In1_mk[1];

          // Update for Delay: '<S118>/Delay1'
          CLVF_DW.icLoad_n4 = 0U;
          CLVF_DW.Delay1_DSTATE_kq = CLVF_B.dividebydeltaT_n;

          // Update for Delay: '<S113>/Delay1'
          CLVF_DW.icLoad_ln = 0U;
          CLVF_DW.Delay1_DSTATE_ph = CLVF_B.In1_mk[2];

          // Update for Delay: '<S119>/Delay1'
          CLVF_DW.icLoad_dk = 0U;
          CLVF_DW.Delay1_DSTATE_mc = CLVF_B.dividebydeltaT_i;

          // Update for Delay: '<S114>/Delay1'
          CLVF_DW.icLoad_kr = 0U;
          CLVF_DW.Delay1_DSTATE_phy = CLVF_B.Unwrap;

          // Update for Delay: '<S120>/Delay1'
          CLVF_DW.icLoad_gt = 0U;
          CLVF_DW.Delay1_DSTATE_ol = CLVF_B.dividebydeltaT_m;

          // Update for S-Function (sdspToNetwork): '<S80>/Send BLACK States to  BLACK Platform' 
          sErr = GetErrorBuffer(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U]);
          LibUpdate_Network(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U],
                            &CLVF_B.TmpSignalConversionAtSendBLACKS[0U], 13);
          if (*sErr != 0) {
            rtmSetErrorStatus(CLVF_M, sErr);
            rtmSetStopRequested(CLVF_M, 1);
          }

          // End of Update for S-Function (sdspToNetwork): '<S80>/Send BLACK States to  BLACK Platform' 
          // End of Update for SubSystem: '<S76>/Obtain RED States'
        }

        // End of Update for If: '<S76>/This IF block determines whether or not to run the RED sim//exp ' 
        // End of Update for SubSystem: '<S75>/Using RED+BLACK, or RED+BLACK+ARM' 
        break;
      }

      // End of Update for If: '<S75>/Check whether both platforms are being used, and if so then use RED to send data to BLACK ' 
      // End of Update for SubSystem: '<S5>/Use Hardware to Obtain States'
    }

    // End of Update for If: '<S5>/If performing an experiment, grab the PhaseSpace data. Otherwise, use a clock to set time in SIM.' 

    // Update for S-Function (sdspToNetwork): '<S15>/UDP Send'
    sErr = GetErrorBuffer(&CLVF_DW.UDPSend_NetworkLib[0U]);
    LibUpdate_Network(&CLVF_DW.UDPSend_NetworkLib[0U],
                      &CLVF_B.SetUniversalTimeIfthisisasimula, 1);
    if (*sErr != 0) {
      rtmSetErrorStatus(CLVF_M, sErr);
      rtmSetStopRequested(CLVF_M, 1);
    }

    // End of Update for S-Function (sdspToNetwork): '<S15>/UDP Send'
  }

  // signal main to stop simulation
  {                                    // Sample time: [0.0s, 0.0s]
    if ((rtmGetTFinal(CLVF_M)!=-1) &&
        !((rtmGetTFinal(CLVF_M)-CLVF_M->Timing.t[0]) > CLVF_M->Timing.t[0] *
          (DBL_EPSILON))) {
      rtmSetErrorStatus(CLVF_M, "Simulation finished");
    }
  }

  // Update absolute time
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  CLVF_M->Timing.t[0] =
    ((time_T)(++CLVF_M->Timing.clockTick0)) * CLVF_M->Timing.stepSize0;

  // Update absolute time
  // The "clockTick1" counts the number of times the code of this task has
  //  been executed. The resolution of this integer timer is 0.05, which is the step size
  //  of the task. Size of "clockTick1" ensures timer will not overflow during the
  //  application lifespan selected.

  CLVF_M->Timing.clockTick1++;
}

// Model step function for TID2
void CLVF_step2(void)                  // Sample time: [0.1s, 0.0s]
{
  boolean_T rtb_DigitalRead_0;
  real_T sampleTime;
  int32_T rtb_EnableSig;
  int32_T rtb_WheelDirection;
  real_T rtb_SaturateMotor;

  // Outputs for Atomic SubSystem: '<S3>/Send Commands to PWM Blocks'
  // MATLABSystem: '<S26>/RED T1 - BLACK T8' incorporates:
  //   RateTransition: '<S3>/Rate Transition'

  if (CLVF_DW.RateTransition_Buffer < 1.0) {
    rtb_SaturateMotor = CLVF_DW.RateTransition_Buffer;
  } else {
    rtb_SaturateMotor = 1.0;
  }

  if (!(rtb_SaturateMotor > 0.0)) {
    rtb_SaturateMotor = 0.0;
  }

  EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_m.PinNumber, rtb_SaturateMotor);

  // End of MATLABSystem: '<S26>/RED T1 - BLACK T8'

  // MATLABSystem: '<S26>/RED T2 - BLACK T3' incorporates:
  //   RateTransition: '<S3>/Rate Transition1'

  if (CLVF_DW.RateTransition1_Buffer < 1.0) {
    rtb_SaturateMotor = CLVF_DW.RateTransition1_Buffer;
  } else {
    rtb_SaturateMotor = 1.0;
  }

  if (!(rtb_SaturateMotor > 0.0)) {
    rtb_SaturateMotor = 0.0;
  }

  EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_ma.PinNumber, rtb_SaturateMotor);

  // End of MATLABSystem: '<S26>/RED T2 - BLACK T3'

  // MATLABSystem: '<S26>/RED T3' incorporates:
  //   RateTransition: '<S3>/Rate Transition2'

  if (CLVF_DW.RateTransition2_Buffer < 1.0) {
    rtb_SaturateMotor = CLVF_DW.RateTransition2_Buffer;
  } else {
    rtb_SaturateMotor = 1.0;
  }

  if (!(rtb_SaturateMotor > 0.0)) {
    rtb_SaturateMotor = 0.0;
  }

  EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_h.PinNumber, rtb_SaturateMotor);

  // End of MATLABSystem: '<S26>/RED T3'

  // MATLABSystem: '<S26>/RED T4 - BLACK T5' incorporates:
  //   RateTransition: '<S3>/Rate Transition3'

  if (CLVF_DW.RateTransition3_Buffer < 1.0) {
    rtb_SaturateMotor = CLVF_DW.RateTransition3_Buffer;
  } else {
    rtb_SaturateMotor = 1.0;
  }

  if (!(rtb_SaturateMotor > 0.0)) {
    rtb_SaturateMotor = 0.0;
  }

  EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_kw.PinNumber, rtb_SaturateMotor);

  // End of MATLABSystem: '<S26>/RED T4 - BLACK T5'

  // MATLABSystem: '<S26>/RED T5 - BLACK T4' incorporates:
  //   RateTransition: '<S3>/Rate Transition4'

  if (CLVF_DW.RateTransition4_Buffer < 1.0) {
    rtb_SaturateMotor = CLVF_DW.RateTransition4_Buffer;
  } else {
    rtb_SaturateMotor = 1.0;
  }

  if (!(rtb_SaturateMotor > 0.0)) {
    rtb_SaturateMotor = 0.0;
  }

  EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_hw.PinNumber, rtb_SaturateMotor);

  // End of MATLABSystem: '<S26>/RED T5 - BLACK T4'

  // MATLABSystem: '<S26>/RED T6 - BLACK T7' incorporates:
  //   RateTransition: '<S3>/Rate Transition5'

  if (CLVF_DW.RateTransition5_Buffer < 1.0) {
    rtb_SaturateMotor = CLVF_DW.RateTransition5_Buffer;
  } else {
    rtb_SaturateMotor = 1.0;
  }

  if (!(rtb_SaturateMotor > 0.0)) {
    rtb_SaturateMotor = 0.0;
  }

  EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_hf.PinNumber, rtb_SaturateMotor);

  // End of MATLABSystem: '<S26>/RED T6 - BLACK T7'

  // MATLABSystem: '<S26>/RED T7 - BLACK T6' incorporates:
  //   RateTransition: '<S3>/Rate Transition6'

  if (CLVF_DW.RateTransition6_Buffer < 1.0) {
    rtb_SaturateMotor = CLVF_DW.RateTransition6_Buffer;
  } else {
    rtb_SaturateMotor = 1.0;
  }

  if (!(rtb_SaturateMotor > 0.0)) {
    rtb_SaturateMotor = 0.0;
  }

  EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_c.PinNumber, rtb_SaturateMotor);

  // End of MATLABSystem: '<S26>/RED T7 - BLACK T6'

  // MATLABSystem: '<S26>/RED T8 - BLACK T1' incorporates:
  //   RateTransition: '<S3>/Rate Transition7'

  if (CLVF_DW.RateTransition7_Buffer < 1.0) {
    rtb_SaturateMotor = CLVF_DW.RateTransition7_Buffer;
  } else {
    rtb_SaturateMotor = 1.0;
  }

  if (!(rtb_SaturateMotor > 0.0)) {
    rtb_SaturateMotor = 0.0;
  }

  EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_fo.PinNumber, rtb_SaturateMotor);

  // End of MATLABSystem: '<S26>/RED T8 - BLACK T1'

  // MATLABSystem: '<S26>/BLACK T2' incorporates:
  //   RateTransition: '<S3>/Rate Transition8'

  if (CLVF_DW.RateTransition8_Buffer < 1.0) {
    rtb_SaturateMotor = CLVF_DW.RateTransition8_Buffer;
  } else {
    rtb_SaturateMotor = 1.0;
  }

  if (!(rtb_SaturateMotor > 0.0)) {
    rtb_SaturateMotor = 0.0;
  }

  EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_ci.PinNumber, rtb_SaturateMotor);

  // End of MATLABSystem: '<S26>/BLACK T2'
  // End of Outputs for SubSystem: '<S3>/Send Commands to PWM Blocks'

  // If: '<S4>/This IF block determines whether or not to run the RED sim//exp ' incorporates:
  //   Constant: '<S4>/Constant'
  //   Constant: '<S57>/Constant'

  if ((CLVF_P.WhoAmI == 1.0) || (CLVF_P.simMode == 1.0)) {
    // Outputs for IfAction SubSystem: '<S4>/Change RED Behavior' incorporates:
    //   ActionPort: '<S56>/Action Port'

    // Saturate: '<S58>/Saturate Torque'
    if (CLVF_DW.RED_Tz_RW > CLVF_P.SaturateTorque_UpperSat) {
      CLVF_DW.RED_Tz_RW_Sat = CLVF_P.SaturateTorque_UpperSat;
    } else if (CLVF_DW.RED_Tz_RW < CLVF_P.SaturateTorque_LowerSat) {
      CLVF_DW.RED_Tz_RW_Sat = CLVF_P.SaturateTorque_LowerSat;
    } else {
      CLVF_DW.RED_Tz_RW_Sat = CLVF_DW.RED_Tz_RW;
    }

    // End of Saturate: '<S58>/Saturate Torque'

    // Outputs for Atomic SubSystem: '<S59>/Obtain RW Status'
    // MATLABSystem: '<S70>/Digital Read'
    if (CLVF_DW.obj_f.SampleTime != CLVF_P.DigitalRead_SampleTime) {
      if (((!rtIsInf(CLVF_P.DigitalRead_SampleTime)) && (!rtIsNaN
            (CLVF_P.DigitalRead_SampleTime))) || rtIsInf
          (CLVF_P.DigitalRead_SampleTime)) {
        sampleTime = CLVF_P.DigitalRead_SampleTime;
      }

      CLVF_DW.obj_f.SampleTime = sampleTime;
    }

    rtb_DigitalRead_0 = MW_gpioRead(24U);

    // End of Outputs for SubSystem: '<S59>/Obtain RW Status'

    // Sum: '<S58>/Sum' incorporates:
    //   DataStoreWrite: '<S56>/RED_Tz_RW_Sat'
    //   Delay: '<S58>/Delay'
    //   MATLAB Function: '<S58>/Calculate RW RPM Increment'

    // MATLAB Function 'From Torque, Command RW/Change RED Behavior/Calculate Saturated RPM Signal/Calculate RW RPM Increment': '<S60>:1' 
    // '<S60>:1:3' dRPM_RW =  (Tz_Sat/0.007244720)*(60/2/pi)*0.1;
    sampleTime = CLVF_DW.RED_Tz_RW_Sat / 0.00724472 * 9.5492965855137211 * 0.1 +
      CLVF_DW.Delay_DSTATE_k;

    // Outputs for Atomic SubSystem: '<S59>/Obtain RW Status'
    // MATLAB Function: '<S59>/Check if RW is Ready' incorporates:
    //   DataStoreWrite: '<S59>/RED_RW_Speed'
    //   Delay: '<S63>/Delay1'
    //   Delay: '<S63>/Delay2'
    //   Delay: '<S63>/Delay3'
    //   Delay: '<S63>/Delay4'
    //   Delay: '<S63>/Delay5'
    //   MATLABSystem: '<S70>/Digital Read'
    //   MinMax: '<S63>/Mostly zeros?'

    // MATLAB Function 'From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Check if RW is Ready': '<S61>:1' 
    // '<S61>:1:3' if Status == 0
    if ((!rtb_DigitalRead_0) || (!CLVF_DW.Delay1_DSTATE_kt) ||
        (!CLVF_DW.Delay2_DSTATE[0U]) || (!CLVF_DW.Delay5_DSTATE[0U]) ||
        (!CLVF_DW.Delay3_DSTATE[0U]) || (!CLVF_DW.Delay4_DSTATE[0U])) {
      // '<S61>:1:4' EnableSig    = 1;
      rtb_EnableSig = 1;

      // '<S61>:1:5' WheelRPM_Out = WheelRPM_In;
      CLVF_DW.RED_dRz_RW_Sat = sampleTime;
    } else {
      // '<S61>:1:6' else
      // '<S61>:1:7' EnableSig    = 0;
      rtb_EnableSig = 0;

      // '<S61>:1:8' WheelRPM_Out = 0;
      CLVF_DW.RED_dRz_RW_Sat = 0.0;
    }

    // End of MATLAB Function: '<S59>/Check if RW is Ready'
    // End of Outputs for SubSystem: '<S59>/Obtain RW Status'

    // Product: '<S62>/Divide' incorporates:
    //   Constant: '<S62>/Gearbox  Ratio'
    //   DataStoreWrite: '<S59>/RED_RW_Speed'

    rtb_SaturateMotor = CLVF_DW.RED_dRz_RW_Sat * CLVF_P.GearboxRatio_Value;

    // Saturate: '<S59>/Saturate Motor'
    if (rtb_SaturateMotor > CLVF_P.SaturateMotor_UpperSat) {
      rtb_SaturateMotor = CLVF_P.SaturateMotor_UpperSat;
    } else {
      if (rtb_SaturateMotor < CLVF_P.SaturateMotor_LowerSat) {
        rtb_SaturateMotor = CLVF_P.SaturateMotor_LowerSat;
      }
    }

    // End of Saturate: '<S59>/Saturate Motor'

    // MATLAB Function: '<S59>/Get RW Direction'
    // MATLAB Function 'From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/Get RW Direction': '<S64>:1' 
    // '<S64>:1:3' if MotorRPM_Sat > 0
    if (rtb_SaturateMotor > 0.0) {
      // '<S64>:1:4' WheelDirection = 0;
      rtb_WheelDirection = 0;
    } else {
      // '<S64>:1:5' else
      // '<S64>:1:6' WheelDirection = 1;
      rtb_WheelDirection = 1;
    }

    // End of MATLAB Function: '<S59>/Get RW Direction'

    // Outputs for Atomic SubSystem: '<S59>/Send Direction to Motor Controller'
    // MATLABSystem: '<S71>/Digital Write'
    MW_gpioWrite(21U, static_cast<uint8_T>(rtb_WheelDirection));

    // End of Outputs for SubSystem: '<S59>/Send Direction to Motor Controller'

    // MATLAB Function: '<S59>/RPM to PWM Value' incorporates:
    //   Abs: '<S59>/Abs'

    // MATLAB Function 'From Torque, Command RW/Change RED Behavior/Wheel RPM to PWM/RPM to PWM Value': '<S66>:1' 
    // '<S66>:1:3' PWMRange = 0.9-0.1;
    // '<S66>:1:5' MotorPWM = ((MotorRPM_Sat*PWMRange)/7000) + 0.1;
    rtb_SaturateMotor = fabs(rtb_SaturateMotor) * 0.8 / 7000.0 + 0.1;

    // Outputs for Atomic SubSystem: '<S59>/Send PWM to Motor Controller'
    // MATLABSystem: '<S68>/PWM1'
    if (!(rtb_SaturateMotor < 1.0)) {
      rtb_SaturateMotor = 1.0;
    }

    EXT_PWMBlock_setDutyCycle(CLVF_DW.obj_d.PinNumber, rtb_SaturateMotor);

    // End of MATLABSystem: '<S68>/PWM1'
    // End of Outputs for SubSystem: '<S59>/Send PWM to Motor Controller'

    // Outputs for Atomic SubSystem: '<S59>/Turn on Motor'
    // MATLABSystem: '<S72>/Digital Write'
    MW_gpioWrite(25U, static_cast<uint8_T>(rtb_EnableSig));

    // End of Outputs for SubSystem: '<S59>/Turn on Motor'

    // Outputs for Atomic SubSystem: '<S59>/Obtain RW Status'
    // Update for Delay: '<S63>/Delay1' incorporates:
    //   MATLABSystem: '<S70>/Digital Read'

    CLVF_DW.Delay1_DSTATE_kt = rtb_DigitalRead_0;

    // End of Outputs for SubSystem: '<S59>/Obtain RW Status'

    // Update for Delay: '<S63>/Delay2' incorporates:
    //   MATLABSystem: '<S70>/Digital Read'

    CLVF_DW.Delay2_DSTATE[0] = CLVF_DW.Delay2_DSTATE[1];

    // Outputs for Atomic SubSystem: '<S59>/Obtain RW Status'
    CLVF_DW.Delay2_DSTATE[1] = rtb_DigitalRead_0;

    // End of Outputs for SubSystem: '<S59>/Obtain RW Status'

    // Update for Delay: '<S63>/Delay5' incorporates:
    //   MATLABSystem: '<S70>/Digital Read'

    CLVF_DW.Delay5_DSTATE[0] = CLVF_DW.Delay5_DSTATE[1];
    CLVF_DW.Delay5_DSTATE[1] = CLVF_DW.Delay5_DSTATE[2];

    // Outputs for Atomic SubSystem: '<S59>/Obtain RW Status'
    CLVF_DW.Delay5_DSTATE[2] = rtb_DigitalRead_0;

    // End of Outputs for SubSystem: '<S59>/Obtain RW Status'

    // Update for Delay: '<S63>/Delay3' incorporates:
    //   MATLABSystem: '<S70>/Digital Read'

    CLVF_DW.Delay3_DSTATE[0] = CLVF_DW.Delay3_DSTATE[1];
    CLVF_DW.Delay3_DSTATE[1] = CLVF_DW.Delay3_DSTATE[2];
    CLVF_DW.Delay3_DSTATE[2] = CLVF_DW.Delay3_DSTATE[3];

    // Outputs for Atomic SubSystem: '<S59>/Obtain RW Status'
    CLVF_DW.Delay3_DSTATE[3] = rtb_DigitalRead_0;

    // End of Outputs for SubSystem: '<S59>/Obtain RW Status'

    // Update for Delay: '<S63>/Delay4' incorporates:
    //   MATLABSystem: '<S70>/Digital Read'

    CLVF_DW.Delay4_DSTATE[0] = CLVF_DW.Delay4_DSTATE[1];
    CLVF_DW.Delay4_DSTATE[1] = CLVF_DW.Delay4_DSTATE[2];
    CLVF_DW.Delay4_DSTATE[2] = CLVF_DW.Delay4_DSTATE[3];
    CLVF_DW.Delay4_DSTATE[3] = CLVF_DW.Delay4_DSTATE[4];

    // Outputs for Atomic SubSystem: '<S59>/Obtain RW Status'
    CLVF_DW.Delay4_DSTATE[4] = rtb_DigitalRead_0;

    // End of Outputs for SubSystem: '<S59>/Obtain RW Status'

    // Saturate: '<S58>/Saturate RPM'
    if (sampleTime > CLVF_P.SaturateRPM_UpperSat) {
      // Update for Delay: '<S58>/Delay'
      CLVF_DW.Delay_DSTATE_k = CLVF_P.SaturateRPM_UpperSat;
    } else if (sampleTime < CLVF_P.SaturateRPM_LowerSat) {
      // Update for Delay: '<S58>/Delay'
      CLVF_DW.Delay_DSTATE_k = CLVF_P.SaturateRPM_LowerSat;
    } else {
      // Update for Delay: '<S58>/Delay'
      CLVF_DW.Delay_DSTATE_k = sampleTime;
    }

    // End of Saturate: '<S58>/Saturate RPM'
    // End of Outputs for SubSystem: '<S4>/Change RED Behavior'
  }

  // End of If: '<S4>/This IF block determines whether or not to run the RED sim//exp ' 
}

// Model step wrapper function for compatibility with a static main program
void CLVF_step(int_T tid)
{
  switch (tid) {
   case 0 :
    CLVF_step0();
    break;

   case 2 :
    CLVF_step2();
    break;

   default :
    break;
  }
}

// Model initialize function
void CLVF_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&CLVF_M->solverInfo, &CLVF_M->Timing.simTimeStep);
    rtsiSetTPtr(&CLVF_M->solverInfo, &rtmGetTPtr(CLVF_M));
    rtsiSetStepSizePtr(&CLVF_M->solverInfo, &CLVF_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&CLVF_M->solverInfo, (&rtmGetErrorStatus(CLVF_M)));
    rtsiSetRTModelPtr(&CLVF_M->solverInfo, CLVF_M);
  }

  rtsiSetSimTimeStep(&CLVF_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&CLVF_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(CLVF_M, &CLVF_M->Timing.tArray[0]);
  rtmSetTFinal(CLVF_M, 376.0);
  CLVF_M->Timing.stepSize0 = 0.05;

  // Setup for data logging
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    CLVF_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  // Setup for data logging
  {
    rtliSetLogXSignalInfo(CLVF_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(CLVF_M->rtwLogInfo, (NULL));
    rtliSetLogT(CLVF_M->rtwLogInfo, "tout");
    rtliSetLogX(CLVF_M->rtwLogInfo, "");
    rtliSetLogXFinal(CLVF_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(CLVF_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(CLVF_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(CLVF_M->rtwLogInfo, 0);
    rtliSetLogDecimation(CLVF_M->rtwLogInfo, 1);
    rtliSetLogY(CLVF_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(CLVF_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(CLVF_M->rtwLogInfo, (NULL));
  }

  // Matfile logging
  rt_StartDataLoggingWithStartTime(CLVF_M->rtwLogInfo, 0.0, rtmGetTFinal(CLVF_M),
    CLVF_M->Timing.stepSize0, (&rtmGetErrorStatus(CLVF_M)));

  {
    char_T *sErr;
    int32_T ret;
    uint32_T tseed;
    int32_T t;
    static const char_T tmp[45] =
      "Unable to configure pin %u for PWM output.\\n";

    // SetupRuntimeResources for ToWorkspace: '<S1>/To Workspace'
    {
      int_T dimensions[1] = { 64 };

      CLVF_DW.ToWorkspace_PWORK.LoggedData = rt_CreateLogVar(
        CLVF_M->rtwLogInfo,
        0.0,
        rtmGetTFinal(CLVF_M),
        CLVF_M->Timing.stepSize0,
        (&rtmGetErrorStatus(CLVF_M)),
        "dataPacket",
        SS_DOUBLE,
        0,
        0,
        0,
        64,
        1,
        dimensions,
        NO_LOGVALDIMS,
        (NULL),
        (NULL),
        0,
        1,
        0.05,
        1);
      if (CLVF_DW.ToWorkspace_PWORK.LoggedData == (NULL))
        return;
    }

    // Start for If: '<Root>/Separate Phases'
    CLVF_DW.SeparatePhases_ActiveSubsystem = -1;

    // Start for If: '<S5>/If performing an experiment, grab the PhaseSpace data. Otherwise, use a clock to set time in SIM.' 
    CLVF_DW.Ifperforminganexperimentgrabthe = -1;

    // Start for S-Function (sdspToNetwork): '<S15>/UDP Send'
    sErr = GetErrorBuffer(&CLVF_DW.UDPSend_NetworkLib[0U]);
    CreateUDPInterface(&CLVF_DW.UDPSend_NetworkLib[0U]);
    if (*sErr == 0) {
      LibCreate_Network(&CLVF_DW.UDPSend_NetworkLib[0U], 1, "0.0.0.0", -1,
                        "192.168.0.105", CLVF_P.UDPSend_Port, 8192, 8, 0);
    }

    if (*sErr == 0) {
      LibStart(&CLVF_DW.UDPSend_NetworkLib[0U]);
    }

    if (*sErr != 0) {
      DestroyUDPInterface(&CLVF_DW.UDPSend_NetworkLib[0U]);
      if (*sErr != 0) {
        rtmSetErrorStatus(CLVF_M, sErr);
        rtmSetStopRequested(CLVF_M, 1);
      }
    }

    // End of Start for S-Function (sdspToNetwork): '<S15>/UDP Send'

    // Start for DataStoreMemory: '<Root>/BLACK_Fx'
    CLVF_DW.BLACK_Fx = CLVF_P.BLACK_Fx_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Fx1'
    CLVF_DW.BLUE_Fx = CLVF_P.BLACK_Fx1_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Fx_Sat'
    CLVF_DW.BLACK_Fx_Sat = CLVF_P.BLACK_Fx_Sat_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Fx_Sat1'
    CLVF_DW.ARM_Elbow_Py = CLVF_P.BLACK_Fx_Sat1_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Fx_Sat2'
    CLVF_DW.BLUE_Fx_Sat = CLVF_P.BLACK_Fx_Sat2_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Fy'
    CLVF_DW.BLACK_Fy = CLVF_P.BLACK_Fy_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Fy1'
    CLVF_DW.BLUE_Fy = CLVF_P.BLACK_Fy1_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Fy_Sat'
    CLVF_DW.BLACK_Fy_Sat = CLVF_P.BLACK_Fy_Sat_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Fy_Sat1'
    CLVF_DW.ARM_Wrist_Px = CLVF_P.BLACK_Fy_Sat1_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Fy_Sat2'
    CLVF_DW.BLUE_Fy_Sat = CLVF_P.BLACK_Fy_Sat2_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Px'
    CLVF_DW.BLACK_Px = CLVF_P.BLACK_Px_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Px1'
    CLVF_DW.BLUE_Px = CLVF_P.BLACK_Px1_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Py'
    CLVF_DW.BLACK_Py = CLVF_P.BLACK_Py_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Py1'
    CLVF_DW.BLUE_Py = CLVF_P.BLACK_Py1_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Rz'
    CLVF_DW.BLACK_Rz = CLVF_P.BLACK_Rz_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Rz1'
    CLVF_DW.BLUE_Rz = CLVF_P.BLACK_Rz1_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Tz'
    CLVF_DW.BLACK_Tz = CLVF_P.BLACK_Tz_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Tz1'
    CLVF_DW.BLUE_Tz = CLVF_P.BLACK_Tz1_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Tz_Sat'
    CLVF_DW.BLACK_Tz_Sat = CLVF_P.BLACK_Tz_Sat_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Tz_Sat1'
    CLVF_DW.ARM_Wrist_Py = CLVF_P.BLACK_Tz_Sat1_InitialValue;

    // Start for DataStoreMemory: '<Root>/BLACK_Tz_Sat2'
    CLVF_DW.BLUE_Tz_Sat = CLVF_P.BLACK_Tz_Sat2_InitialValue;

    // Start for DataStoreMemory: '<Root>/Data Store Memory'
    CLVF_DW.SPEED_DESIRED_x = CLVF_P.DataStoreMemory_InitialValue;

    // Start for DataStoreMemory: '<Root>/Data Store Memory1'
    CLVF_DW.SPEED_DESIRED_y = CLVF_P.DataStoreMemory1_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Fx'
    CLVF_DW.RED_Fx = CLVF_P.RED_Fx_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Fx_Sat'
    CLVF_DW.RED_Fx_Sat = CLVF_P.RED_Fx_Sat_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Fy'
    CLVF_DW.RED_Fy = CLVF_P.RED_Fy_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Fy_Sat'
    CLVF_DW.RED_Fy_Sat = CLVF_P.RED_Fy_Sat_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px'
    CLVF_DW.RED_Px = CLVF_P.RED_Px_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px1'
    CLVF_DW.RED_AHRS_Q = CLVF_P.RED_Px1_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px10'
    CLVF_DW.BLACK_IMU_Ax_b = CLVF_P.RED_Px10_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px11'
    CLVF_DW.BLACK_IMU_Ay_b = CLVF_P.RED_Px11_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px12'
    CLVF_DW.BLACK_IMU_Az_b = CLVF_P.RED_Px12_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px19'
    CLVF_DW.RED_IMU_Q = CLVF_P.RED_Px19_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px2'
    CLVF_DW.RED_AHRS_P = CLVF_P.RED_Px2_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px20'
    CLVF_DW.RED_IMU_P = CLVF_P.RED_Px20_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px21'
    CLVF_DW.RED_IMU_R = CLVF_P.RED_Px21_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px22'
    CLVF_DW.BLACK_IMU_Q = CLVF_P.RED_Px22_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px23'
    CLVF_DW.BLACK_IMU_P = CLVF_P.RED_Px23_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px24'
    CLVF_DW.BLACK_IMU_R = CLVF_P.RED_Px24_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px25'
    CLVF_DW.RED_IMU_Psi = CLVF_P.RED_Px25_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px26'
    CLVF_DW.BLACK_IMU_Psi = CLVF_P.RED_Px26_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px3'
    CLVF_DW.RED_AHRS_R = CLVF_P.RED_Px3_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px31'
    CLVF_DW.RED_IMU_Ax_I = CLVF_P.RED_Px31_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px32'
    CLVF_DW.RED_IMU_Ay_I = CLVF_P.RED_Px32_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px37'
    CLVF_DW.BLACK_IMU_Ax_I = CLVF_P.RED_Px37_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px38'
    CLVF_DW.BLACK_IMU_Ay_I = CLVF_P.RED_Px38_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px4'
    CLVF_DW.BLACK_AHRS_Q = CLVF_P.RED_Px4_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px5'
    CLVF_DW.BLACK_AHRS_P = CLVF_P.RED_Px5_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px6'
    CLVF_DW.BLACK_AHRS_R = CLVF_P.RED_Px6_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px7'
    CLVF_DW.RED_IMU_Ax_b = CLVF_P.RED_Px7_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px8'
    CLVF_DW.RED_IMU_Ay_b = CLVF_P.RED_Px8_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Px9'
    CLVF_DW.RED_IMU_Az_b = CLVF_P.RED_Px9_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Py'
    CLVF_DW.RED_Py = CLVF_P.RED_Py_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Rz'
    CLVF_DW.RED_Rz = CLVF_P.RED_Rz_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Tz'
    CLVF_DW.RED_Tz = CLVF_P.RED_Tz_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Tz7'
    CLVF_DW.Float_State = CLVF_P.RED_Tz7_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Tz8'
    CLVF_DW.Magnet_State = CLVF_P.RED_Tz8_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Tz_RW'
    CLVF_DW.RED_Tz_RW = CLVF_P.RED_Tz_RW_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Tz_RW Sat'
    CLVF_DW.RED_Tz_RW_Sat = CLVF_P.RED_Tz_RWSat_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Tz_Sat'
    CLVF_DW.RED_Tz_Sat = CLVF_P.RED_Tz_Sat_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_Tz_Sat1'
    CLVF_DW.ARM_Elbow_Px = CLVF_P.RED_Tz_Sat1_InitialValue;

    // Start for DataStoreMemory: '<Root>/RED_dRz_RW Sat'
    CLVF_DW.RED_dRz_RW_Sat = CLVF_P.RED_dRz_RWSat_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time'
    CLVF_DW.Univ_Time = CLVF_P.Universal_Time_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time1'
    CLVF_DW.RED_Vx = CLVF_P.Universal_Time1_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time10'
    CLVF_DW.RED_Ax = CLVF_P.Universal_Time10_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time11'
    CLVF_DW.RED_Ay = CLVF_P.Universal_Time11_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time12'
    CLVF_DW.BLACK_Ax = CLVF_P.Universal_Time12_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time13'
    CLVF_DW.BLACK_Ay = CLVF_P.Universal_Time13_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time14'
    CLVF_DW.BLACK_RzDD = CLVF_P.Universal_Time14_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time15'
    CLVF_DW.RED_RzDD = CLVF_P.Universal_Time15_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time2'
    CLVF_DW.RED_Vy = CLVF_P.Universal_Time2_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time3'
    CLVF_DW.RED_RzD = CLVF_P.Universal_Time3_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time4'
    CLVF_DW.BLACK_Vx = CLVF_P.Universal_Time4_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time5'
    CLVF_DW.BLACK_Vy = CLVF_P.Universal_Time5_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time6'
    CLVF_DW.BLACK_RzD = CLVF_P.Universal_Time6_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time7'
    CLVF_DW.BLUE_Vx = CLVF_P.Universal_Time7_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time8'
    CLVF_DW.BLUE_Vy = CLVF_P.Universal_Time8_InitialValue;

    // Start for DataStoreMemory: '<Root>/Universal_Time9'
    CLVF_DW.BLUE_RzD = CLVF_P.Universal_Time9_InitialValue;
    CLVF_PrevZCX.SampleandHold1_Trig_ZCE_g = UNINITIALIZED_ZCSIG;
    CLVF_PrevZCX.SampleandHold1_Trig_ZCE = UNINITIALIZED_ZCSIG;
    CLVF_PrevZCX.SampleandHold_Trig_ZCE_l = UNINITIALIZED_ZCSIG;
    CLVF_PrevZCX.SampleandHold_Trig_ZCE = UNINITIALIZED_ZCSIG;

    // SystemInitialize for IfAction SubSystem: '<Root>/Phase #0:  Wait for Synchronization' 
    Phase0WaitforSynchroni_Init(&CLVF_DW.Phase0WaitforSynchronization);

    // End of SystemInitialize for SubSystem: '<Root>/Phase #0:  Wait for Synchronization' 

    // SystemInitialize for IfAction SubSystem: '<Root>/Phase #1:  Start Floating ' 
    Phase0WaitforSynchroni_Init(&CLVF_DW.Phase1StartFloating);

    // End of SystemInitialize for SubSystem: '<Root>/Phase #1:  Start Floating ' 

    // SystemInitialize for IfAction SubSystem: '<Root>/Phase #2:  Move to  Initial Position' 
    // Start for If: '<S10>/This IF block determines whether or not to run the BLACK sim//exp' 
    CLVF_DW.ThisIFblockdetermineswhether_dy = -1;

    // Start for If: '<S10>/This IF block determines whether or not to run the BLUE sim//exp' 
    CLVF_DW.ThisIFblockdetermineswhether_jm = -1;

    // Start for If: '<S10>/This IF block determines whether or not to run the RED sim//exp ' 
    CLVF_DW.ThisIFblockdetermineswhether_aj = -1;

    // SystemInitialize for IfAction SubSystem: '<S10>/Change BLACK Behavior'
    // InitializeConditions for Delay: '<S284>/Delay1'
    CLVF_DW.icLoad_ns = 1U;

    // InitializeConditions for Delay: '<S286>/Delay1'
    CLVF_DW.icLoad_k1 = 1U;

    // InitializeConditions for Delay: '<S282>/Delay1'
    CLVF_DW.icLoad_ep = 1U;

    // SystemInitialize for IfAction SubSystem: '<S284>/Hold this value'
    // SystemInitialize for Outport: '<S285>/Out1'
    CLVF_B.In1_ej = CLVF_P.Out1_Y0_av;

    // End of SystemInitialize for SubSystem: '<S284>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S286>/Hold this value'
    // SystemInitialize for Outport: '<S287>/Out1'
    CLVF_B.In1_gc = CLVF_P.Out1_Y0_g1;

    // End of SystemInitialize for SubSystem: '<S286>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S282>/Hold this value'
    // SystemInitialize for Outport: '<S283>/Out1'
    CLVF_B.In1_fa = CLVF_P.Out1_Y0_mp;

    // End of SystemInitialize for SubSystem: '<S282>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S10>/Change BLACK Behavior'

    // SystemInitialize for IfAction SubSystem: '<S10>/Change BLUE Behavior'
    // InitializeConditions for Delay: '<S297>/Delay1'
    CLVF_DW.icLoad_dy = 1U;

    // InitializeConditions for Delay: '<S299>/Delay1'
    CLVF_DW.icLoad_hf = 1U;

    // InitializeConditions for Delay: '<S295>/Delay1'
    CLVF_DW.icLoad_k = 1U;

    // SystemInitialize for IfAction SubSystem: '<S297>/Hold this value'
    // SystemInitialize for Outport: '<S298>/Out1'
    CLVF_B.In1_lc = CLVF_P.Out1_Y0_a4;

    // End of SystemInitialize for SubSystem: '<S297>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S299>/Hold this value'
    // SystemInitialize for Outport: '<S300>/Out1'
    CLVF_B.In1_jw = CLVF_P.Out1_Y0_nv;

    // End of SystemInitialize for SubSystem: '<S299>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S295>/Hold this value'
    // SystemInitialize for Outport: '<S296>/Out1'
    CLVF_B.In1_o = CLVF_P.Out1_Y0_ov;

    // End of SystemInitialize for SubSystem: '<S295>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S10>/Change BLUE Behavior'

    // SystemInitialize for IfAction SubSystem: '<S10>/Change RED Behavior'
    // InitializeConditions for Delay: '<S308>/Delay1'
    CLVF_DW.icLoad_ji = 1U;

    // InitializeConditions for Delay: '<S310>/Delay1'
    CLVF_DW.icLoad_a1 = 1U;

    // InitializeConditions for Delay: '<S312>/Delay1'
    CLVF_DW.icLoad_o0 = 1U;

    // SystemInitialize for IfAction SubSystem: '<S308>/Hold this value'
    // SystemInitialize for Outport: '<S309>/Out1'
    CLVF_B.In1_k = CLVF_P.Out1_Y0_fr;

    // End of SystemInitialize for SubSystem: '<S308>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S310>/Hold this value'
    // SystemInitialize for Outport: '<S311>/Out1'
    CLVF_B.In1_cr = CLVF_P.Out1_Y0_bk;

    // End of SystemInitialize for SubSystem: '<S310>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S312>/Hold this value'
    // SystemInitialize for Outport: '<S313>/Out1'
    CLVF_B.In1_if = CLVF_P.Out1_Y0_j2;

    // End of SystemInitialize for SubSystem: '<S312>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S10>/Change RED Behavior'
    // End of SystemInitialize for SubSystem: '<Root>/Phase #2:  Move to  Initial Position' 

    // SystemInitialize for IfAction SubSystem: '<Root>/Phase #3: Experiment'
    // Start for If: '<S11>/This IF block determines whether or not to run the BLACK sim//exp' 
    CLVF_DW.ThisIFblockdetermineswhethero_i = -1;

    // Start for If: '<S11>/This IF block determines whether or not to run the BLUE sim//exp' 
    CLVF_DW.ThisIFblockdetermineswhether_aa = -1;

    // Start for If: '<S11>/This IF block determines whether or not to run the RED sim//exp ' 
    CLVF_DW.ThisIFblockdetermineswhethero_f = -1;

    // SystemInitialize for IfAction SubSystem: '<S11>/Change BLACK Behavior'
    // Start for If: '<S317>/Experiment Sub-Phases'
    CLVF_DW.ExperimentSubPhases_ActiveSub_p = -1;

    // SystemInitialize for IfAction SubSystem: '<S317>/Sub-Phase #1'
    // InitializeConditions for Delay: '<S332>/Delay1'
    CLVF_DW.icLoad_iz = 1U;

    // InitializeConditions for Delay: '<S334>/Delay1'
    CLVF_DW.icLoad_p = 1U;

    // InitializeConditions for Delay: '<S330>/Delay1'
    CLVF_DW.icLoad_dm = 1U;

    // SystemInitialize for IfAction SubSystem: '<S332>/Hold this value'
    // SystemInitialize for Outport: '<S333>/Out1'
    CLVF_B.In1_gy = CLVF_P.Out1_Y0_hu;

    // End of SystemInitialize for SubSystem: '<S332>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S334>/Hold this value'
    // SystemInitialize for Outport: '<S335>/Out1'
    CLVF_B.In1_ft = CLVF_P.Out1_Y0_efk;

    // End of SystemInitialize for SubSystem: '<S334>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S330>/Hold this value'
    // SystemInitialize for Outport: '<S331>/Out1'
    CLVF_B.In1_l = CLVF_P.Out1_Y0_gc;

    // End of SystemInitialize for SubSystem: '<S330>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S317>/Sub-Phase #1'

    // SystemInitialize for IfAction SubSystem: '<S317>/Sub-Phase #4'
    // SetupRuntimeResources for ToWorkspace: '<S349>/To Workspace'
    {
      int_T dimensions[1] = { 1 };

      CLVF_DW.ToWorkspace_PWORK_h.LoggedData = rt_CreateLogVar(
        CLVF_M->rtwLogInfo,
        0.0,
        rtmGetTFinal(CLVF_M),
        CLVF_M->Timing.stepSize0,
        (&rtmGetErrorStatus(CLVF_M)),
        "whichFieldToAnimate",
        SS_DOUBLE,
        0,
        0,
        0,
        1,
        1,
        dimensions,
        NO_LOGVALDIMS,
        (NULL),
        (NULL),
        0,
        1,
        0.05,
        1);
      if (CLVF_DW.ToWorkspace_PWORK_h.LoggedData == (NULL))
        return;
    }

    // SetupRuntimeResources for ToWorkspace: '<S349>/To Workspace2'
    {
      int_T dimensions[1] = { 3 };

      CLVF_DW.ToWorkspace2_PWORK.LoggedData = rt_CreateLogVar(
        CLVF_M->rtwLogInfo,
        0.0,
        rtmGetTFinal(CLVF_M),
        CLVF_M->Timing.stepSize0,
        (&rtmGetErrorStatus(CLVF_M)),
        "h",
        SS_DOUBLE,
        0,
        0,
        0,
        3,
        1,
        dimensions,
        NO_LOGVALDIMS,
        (NULL),
        (NULL),
        0,
        1,
        0.05,
        1);
      if (CLVF_DW.ToWorkspace2_PWORK.LoggedData == (NULL))
        return;
    }

    // SetupRuntimeResources for ToWorkspace: '<S342>/To Workspace'
    {
      int_T dimensions[1] = { 1 };

      CLVF_DW.ToWorkspace_PWORK_k.LoggedData = rt_CreateLogVar(
        CLVF_M->rtwLogInfo,
        0.0,
        rtmGetTFinal(CLVF_M),
        CLVF_M->Timing.stepSize0,
        (&rtmGetErrorStatus(CLVF_M)),
        "r",
        SS_DOUBLE,
        0,
        0,
        0,
        1,
        1,
        dimensions,
        NO_LOGVALDIMS,
        (NULL),
        (NULL),
        0,
        1,
        0.05,
        1);
      if (CLVF_DW.ToWorkspace_PWORK_k.LoggedData == (NULL))
        return;
    }

    // SetupRuntimeResources for ToWorkspace: '<S342>/To Workspace1'
    {
      int_T dimensions[1] = { 1 };

      CLVF_DW.ToWorkspace1_PWORK.LoggedData = rt_CreateLogVar(
        CLVF_M->rtwLogInfo,
        0.0,
        rtmGetTFinal(CLVF_M),
        CLVF_M->Timing.stepSize0,
        (&rtmGetErrorStatus(CLVF_M)),
        "r_prime",
        SS_DOUBLE,
        0,
        0,
        0,
        1,
        1,
        dimensions,
        NO_LOGVALDIMS,
        (NULL),
        (NULL),
        0,
        1,
        0.05,
        1);
      if (CLVF_DW.ToWorkspace1_PWORK.LoggedData == (NULL))
        return;
    }

    // SetupRuntimeResources for ToWorkspace: '<S342>/To Workspace2'
    {
      int_T dimensions[1] = { 1 };

      CLVF_DW.ToWorkspace2_PWORK_i.LoggedData = rt_CreateLogVar(
        CLVF_M->rtwLogInfo,
        0.0,
        rtmGetTFinal(CLVF_M),
        CLVF_M->Timing.stepSize0,
        (&rtmGetErrorStatus(CLVF_M)),
        "theta",
        SS_DOUBLE,
        0,
        0,
        0,
        1,
        1,
        dimensions,
        NO_LOGVALDIMS,
        (NULL),
        (NULL),
        0,
        1,
        0.05,
        1);
      if (CLVF_DW.ToWorkspace2_PWORK_i.LoggedData == (NULL))
        return;
    }

    // SetupRuntimeResources for ToWorkspace: '<S342>/To Workspace3'
    {
      int_T dimensions[1] = { 1 };

      CLVF_DW.ToWorkspace3_PWORK.LoggedData = rt_CreateLogVar(
        CLVF_M->rtwLogInfo,
        0.0,
        rtmGetTFinal(CLVF_M),
        CLVF_M->Timing.stepSize0,
        (&rtmGetErrorStatus(CLVF_M)),
        "theta_prime",
        SS_DOUBLE,
        0,
        0,
        0,
        1,
        1,
        dimensions,
        NO_LOGVALDIMS,
        (NULL),
        (NULL),
        0,
        1,
        0.05,
        1);
      if (CLVF_DW.ToWorkspace3_PWORK.LoggedData == (NULL))
        return;
    }

    // Start for If: '<S349>/If'
    CLVF_DW.If_ActiveSubsystem = -1;

    // InitializeConditions for Delay: '<S354>/Delay'
    CLVF_DW.Delay_DSTATE = CLVF_P.Delay_InitialCondition_h;

    // InitializeConditions for Delay: '<S344>/Delay1'
    CLVF_DW.icLoad_en = 1U;

    // InitializeConditions for DiscreteTransferFcn: '<S339>/1Hz LP Filter'
    CLVF_DW.uHzLPFilter_states[0] = CLVF_P.uHzLPFilter_InitialStates;

    // InitializeConditions for DiscreteTransferFcn: '<S342>/1Hz LP Filter1'
    CLVF_DW.uHzLPFilter1_states_l[0] = CLVF_P.uHzLPFilter1_InitialStates;

    // InitializeConditions for DiscreteTransferFcn: '<S342>/1Hz LP Filter'
    CLVF_DW.uHzLPFilter_states_m[0] = CLVF_P.uHzLPFilter_InitialStates_k;

    // InitializeConditions for DiscreteTransferFcn: '<S339>/1Hz LP Filter'
    CLVF_DW.uHzLPFilter_states[1] = CLVF_P.uHzLPFilter_InitialStates;

    // InitializeConditions for DiscreteTransferFcn: '<S342>/1Hz LP Filter1'
    CLVF_DW.uHzLPFilter1_states_l[1] = CLVF_P.uHzLPFilter1_InitialStates;

    // InitializeConditions for DiscreteTransferFcn: '<S342>/1Hz LP Filter'
    CLVF_DW.uHzLPFilter_states_m[1] = CLVF_P.uHzLPFilter_InitialStates_k;

    // InitializeConditions for Delay: '<S363>/Delay1'
    CLVF_DW.icLoad_jp = 1U;

    // InitializeConditions for Delay: '<S364>/Delay1'
    CLVF_DW.icLoad_of = 1U;

    // InitializeConditions for Delay: '<S365>/Delay1'
    CLVF_DW.icLoad_i = 1U;

    // InitializeConditions for Delay: '<S366>/Delay1'
    CLVF_DW.icLoad_ay = 1U;

    // InitializeConditions for Derivative: '<S351>/Derivative'
    CLVF_DW.TimeStampA_i = (rtInf);
    CLVF_DW.TimeStampB_p = (rtInf);

    // End of SystemInitialize for SubSystem: '<S349>/CLVF'
    // SetupRuntimeResources for ToWorkspace: '<S353>/To Workspace'
    {
      int_T dimensions[1] = { 3 };

      CLVF_DW.ToWorkspace_PWORK_d.LoggedData = rt_CreateLogVar(
        CLVF_M->rtwLogInfo,
        0.0,
        rtmGetTFinal(CLVF_M),
        CLVF_M->Timing.stepSize0,
        (&rtmGetErrorStatus(CLVF_M)),
        "acc_ff_LVF",
        SS_DOUBLE,
        0,
        0,
        0,
        3,
        1,
        dimensions,
        NO_LOGVALDIMS,
        (NULL),
        (NULL),
        0,
        1,
        0.05,
        1);
      if (CLVF_DW.ToWorkspace_PWORK_d.LoggedData == (NULL))
        return;
    }

    // InitializeConditions for Derivative: '<S353>/Derivative'
    CLVF_DW.TimeStampA = (rtInf);
    CLVF_DW.TimeStampB = (rtInf);

    // End of SystemInitialize for SubSystem: '<S349>/Lyapunov'

    // SystemInitialize for Triggered SubSystem: '<S354>/Sample and Hold'
    // SystemInitialize for Outport: '<S358>/ '
    CLVF_B.In = CLVF_P._Y0_e;

    // End of SystemInitialize for SubSystem: '<S354>/Sample and Hold'

    // SystemInitialize for IfAction SubSystem: '<S363>/Hold this value'
    // SystemInitialize for Outport: '<S370>/Out1'
    CLVF_B.In1_a = CLVF_P.Out1_Y0_ni;

    // End of SystemInitialize for SubSystem: '<S363>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S344>/Hold this value'
    // SystemInitialize for Outport: '<S345>/Out1'
    CLVF_B.In1_jg = CLVF_P.Out1_Y0_bx;

    // End of SystemInitialize for SubSystem: '<S344>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S317>/Sub-Phase #4'
    // End of SystemInitialize for SubSystem: '<S11>/Change BLACK Behavior'

    // SystemInitialize for IfAction SubSystem: '<S11>/Change BLUE Behavior'
    // Start for If: '<S318>/Experiment Sub-Phases'
    CLVF_DW.ExperimentSubPhases_ActiveSub_h = -1;

    // SystemInitialize for IfAction SubSystem: '<S318>/Sub-Phase #1'
    CLVF_SubPhase1_Init(&CLVF_B.SubPhase1_g, &CLVF_DW.SubPhase1_g,
                        &CLVF_P.SubPhase1_g);

    // End of SystemInitialize for SubSystem: '<S318>/Sub-Phase #1'

    // SystemInitialize for IfAction SubSystem: '<S318>/Sub-Phase #4'
    CLVF_SubPhase1_Init(&CLVF_B.SubPhase4_o, &CLVF_DW.SubPhase4_o,
                        &CLVF_P.SubPhase4_o);

    // End of SystemInitialize for SubSystem: '<S318>/Sub-Phase #4'
    // End of SystemInitialize for SubSystem: '<S11>/Change BLUE Behavior'

    // SystemInitialize for IfAction SubSystem: '<S11>/Change RED Behavior'
    // Start for If: '<S319>/Experiment Sub-Phases'
    CLVF_DW.ExperimentSubPhases_ActiveSubsy = -1;

    // SystemInitialize for IfAction SubSystem: '<S319>/Sub-Phase #1'
    // InitializeConditions for Delay: '<S414>/Delay1'
    CLVF_DW.icLoad_c = 1U;

    // InitializeConditions for Delay: '<S416>/Delay1'
    CLVF_DW.icLoad_l = 1U;

    // InitializeConditions for Delay: '<S418>/Delay1'
    CLVF_DW.icLoad_jz = 1U;

    // SystemInitialize for IfAction SubSystem: '<S414>/Hold this value'
    // SystemInitialize for Outport: '<S415>/Out1'
    CLVF_B.In1_he = CLVF_P.Out1_Y0_na;

    // End of SystemInitialize for SubSystem: '<S414>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S416>/Hold this value'
    // SystemInitialize for Outport: '<S417>/Out1'
    CLVF_B.In1_n = CLVF_P.Out1_Y0_mn;

    // End of SystemInitialize for SubSystem: '<S416>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S418>/Hold this value'
    // SystemInitialize for Outport: '<S419>/Out1'
    CLVF_B.In1_e0 = CLVF_P.Out1_Y0_lx;

    // End of SystemInitialize for SubSystem: '<S418>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S319>/Sub-Phase #1'

    // SystemInitialize for IfAction SubSystem: '<S319>/Sub-Phase #4'
    // InitializeConditions for Delay: '<S428>/Delay1'
    CLVF_DW.icLoad_jk = 1U;

    // InitializeConditions for DiscreteTransferFcn: '<S423>/1Hz LP Filter1'
    CLVF_DW.uHzLPFilter1_states[0] = CLVF_P.uHzLPFilter1_InitialStates_o;
    CLVF_DW.uHzLPFilter1_states[1] = CLVF_P.uHzLPFilter1_InitialStates_o;

    // InitializeConditions for Delay: '<S430>/Delay1'
    CLVF_DW.icLoad_fc = 1U;

    // InitializeConditions for Delay: '<S432>/Delay1'
    CLVF_DW.icLoad_n = 1U;

    // SystemInitialize for IfAction SubSystem: '<S428>/Hold this value'
    // SystemInitialize for Outport: '<S429>/Out1'
    CLVF_B.In1_c5 = CLVF_P.Out1_Y0_bj;

    // End of SystemInitialize for SubSystem: '<S428>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S430>/Hold this value'
    // SystemInitialize for Outport: '<S431>/Out1'
    CLVF_B.In1_cy = CLVF_P.Out1_Y0_ki;

    // End of SystemInitialize for SubSystem: '<S430>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S432>/Hold this value'
    // SystemInitialize for Outport: '<S433>/Out1'
    CLVF_B.In1_e = CLVF_P.Out1_Y0_ky;

    // End of SystemInitialize for SubSystem: '<S432>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S319>/Sub-Phase #4'
    // End of SystemInitialize for SubSystem: '<S11>/Change RED Behavior'
    // End of SystemInitialize for SubSystem: '<Root>/Phase #3: Experiment'

    // SystemInitialize for IfAction SubSystem: '<Root>/Phase #4:  Return Home'
    // Start for If: '<S12>/This IF block determines whether or not to run the BLACK sim//exp' 
    CLVF_DW.ThisIFblockdetermineswhethero_d = -1;

    // Start for If: '<S12>/This IF block determines whether or not to run the BLUE sim//exp' 
    CLVF_DW.ThisIFblockdetermineswhether_jc = -1;

    // Start for If: '<S12>/This IF block determines whether or not to run the RED sim//exp ' 
    CLVF_DW.ThisIFblockdetermineswhether_a4 = -1;

    // SystemInitialize for IfAction SubSystem: '<S12>/Change BLACK Behavior'
    // InitializeConditions for Delay: '<S447>/Delay1'
    CLVF_DW.icLoad_dq = 1U;

    // InitializeConditions for Delay: '<S449>/Delay1'
    CLVF_DW.icLoad_h = 1U;

    // InitializeConditions for Delay: '<S445>/Delay1'
    CLVF_DW.icLoad_o = 1U;

    // SystemInitialize for IfAction SubSystem: '<S447>/Hold this value'
    // SystemInitialize for Outport: '<S448>/Out1'
    CLVF_B.In1_fx = CLVF_P.Out1_Y0_mrw;

    // End of SystemInitialize for SubSystem: '<S447>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S449>/Hold this value'
    // SystemInitialize for Outport: '<S450>/Out1'
    CLVF_B.In1_d0 = CLVF_P.Out1_Y0_i0;

    // End of SystemInitialize for SubSystem: '<S449>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S445>/Hold this value'
    // SystemInitialize for Outport: '<S446>/Out1'
    CLVF_B.In1_fk = CLVF_P.Out1_Y0_jr;

    // End of SystemInitialize for SubSystem: '<S445>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S12>/Change BLACK Behavior'

    // SystemInitialize for IfAction SubSystem: '<S12>/Change BLUE Behavior'
    C_ChangeBLUEBehavior_a_Init(&CLVF_B.ChangeBLUEBehavior_gj,
      &CLVF_DW.ChangeBLUEBehavior_gj, &CLVF_P.ChangeBLUEBehavior_gj);

    // End of SystemInitialize for SubSystem: '<S12>/Change BLUE Behavior'

    // SystemInitialize for IfAction SubSystem: '<S12>/Change RED Behavior'
    // InitializeConditions for Delay: '<S471>/Delay1'
    CLVF_DW.icLoad_dh = 1U;

    // InitializeConditions for Delay: '<S473>/Delay1'
    CLVF_DW.icLoad_a2 = 1U;

    // InitializeConditions for Delay: '<S475>/Delay1'
    CLVF_DW.icLoad_f = 1U;

    // SystemInitialize for IfAction SubSystem: '<S471>/Hold this value'
    // SystemInitialize for Outport: '<S472>/Out1'
    CLVF_B.In1_h = CLVF_P.Out1_Y0_nf;

    // End of SystemInitialize for SubSystem: '<S471>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S473>/Hold this value'
    // SystemInitialize for Outport: '<S474>/Out1'
    CLVF_B.In1_b = CLVF_P.Out1_Y0_gb;

    // End of SystemInitialize for SubSystem: '<S473>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S475>/Hold this value'
    // SystemInitialize for Outport: '<S476>/Out1'
    CLVF_B.In1_f = CLVF_P.Out1_Y0_da;

    // End of SystemInitialize for SubSystem: '<S475>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S12>/Change RED Behavior'
    // End of SystemInitialize for SubSystem: '<Root>/Phase #4:  Return Home'

    // SystemInitialize for IfAction SubSystem: '<Root>/Phase #5:  Hold Home'
    // Start for If: '<S13>/This IF block determines whether or not to run the BLACK sim//exp' 
    CLVF_DW.ThisIFblockdetermineswhethero_b = -1;

    // Start for If: '<S13>/This IF block determines whether or not to run the BLUE sim//exp' 
    CLVF_DW.ThisIFblockdetermineswhethero_n = -1;

    // Start for If: '<S13>/This IF block determines whether or not to run the RED sim//exp ' 
    CLVF_DW.ThisIFblockdetermineswhethero_j = -1;

    // SystemInitialize for IfAction SubSystem: '<S13>/Change BLACK Behavior'
    // InitializeConditions for Delay: '<S490>/Delay1'
    CLVF_DW.icLoad_e = 1U;

    // InitializeConditions for Delay: '<S492>/Delay1'
    CLVF_DW.icLoad_m = 1U;

    // InitializeConditions for Delay: '<S488>/Delay1'
    CLVF_DW.icLoad_d = 1U;

    // SystemInitialize for IfAction SubSystem: '<S490>/Hold this value'
    // SystemInitialize for Outport: '<S491>/Out1'
    CLVF_B.In1_i = CLVF_P.Out1_Y0_fj;

    // End of SystemInitialize for SubSystem: '<S490>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S492>/Hold this value'
    // SystemInitialize for Outport: '<S493>/Out1'
    CLVF_B.In1_c = CLVF_P.Out1_Y0_bm;

    // End of SystemInitialize for SubSystem: '<S492>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S488>/Hold this value'
    // SystemInitialize for Outport: '<S489>/Out1'
    CLVF_B.In1_g = CLVF_P.Out1_Y0_gx;

    // End of SystemInitialize for SubSystem: '<S488>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S13>/Change BLACK Behavior'

    // SystemInitialize for IfAction SubSystem: '<S13>/Change BLUE Behavior'
    C_ChangeBLUEBehavior_a_Init(&CLVF_B.ChangeBLUEBehavior_b,
      &CLVF_DW.ChangeBLUEBehavior_b, &CLVF_P.ChangeBLUEBehavior_b);

    // End of SystemInitialize for SubSystem: '<S13>/Change BLUE Behavior'

    // SystemInitialize for IfAction SubSystem: '<S13>/Change RED Behavior'
    // InitializeConditions for Delay: '<S516>/Delay1'
    CLVF_DW.icLoad = 1U;

    // InitializeConditions for Delay: '<S518>/Delay1'
    CLVF_DW.icLoad_j = 1U;

    // InitializeConditions for Delay: '<S514>/Delay1'
    CLVF_DW.icLoad_a = 1U;

    // SystemInitialize for IfAction SubSystem: '<S516>/Hold this value'
    // SystemInitialize for Outport: '<S517>/Out1'
    CLVF_B.In1_j = CLVF_P.Out1_Y0_bs;

    // End of SystemInitialize for SubSystem: '<S516>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S518>/Hold this value'
    // SystemInitialize for Outport: '<S519>/Out1'
    CLVF_B.In1 = CLVF_P.Out1_Y0_c5;

    // End of SystemInitialize for SubSystem: '<S518>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S514>/Hold this value'
    // SystemInitialize for Outport: '<S515>/Out1'
    CLVF_B.In1_d = CLVF_P.Out1_Y0_i3j;

    // End of SystemInitialize for SubSystem: '<S514>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S13>/Change RED Behavior'
    // End of SystemInitialize for SubSystem: '<Root>/Phase #5:  Hold Home'

    // SystemInitialize for IfAction SubSystem: '<Root>/Simulate Plant Dynamics' 
    // InitializeConditions for DiscreteIntegrator: '<S527>/Acceleration  to Velocity' 
    CLVF_DW.AccelerationtoVelocity_DSTATE[0] = CLVF_P.AccelerationtoVelocity_IC;

    // InitializeConditions for DiscreteIntegrator: '<S527>/Velocity to Position' 
    CLVF_DW.VelocitytoPosition_DSTATE[0] = CLVF_P.drop_states_BLACK[0];

    // InitializeConditions for DiscreteIntegrator: '<S527>/Acceleration  to Velocity' 
    CLVF_DW.AccelerationtoVelocity_DSTATE[1] = CLVF_P.AccelerationtoVelocity_IC;

    // InitializeConditions for DiscreteIntegrator: '<S527>/Velocity to Position' 
    CLVF_DW.VelocitytoPosition_DSTATE[1] = CLVF_P.drop_states_BLACK[1];

    // InitializeConditions for DiscreteIntegrator: '<S527>/Acceleration  to Velocity' 
    CLVF_DW.AccelerationtoVelocity_DSTATE[2] = CLVF_P.AccelerationtoVelocity_IC;

    // InitializeConditions for DiscreteIntegrator: '<S527>/Velocity to Position' 
    CLVF_DW.VelocitytoPosition_DSTATE[2] = CLVF_P.drop_states_BLACK[2];

    // InitializeConditions for RandomNumber: '<S527>/Random Number'
    CLVF_B.d = floor(CLVF_P.RandomNumber_Seed);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed = tseed;
    CLVF_DW.NextOutput = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed) *
      sqrt(CLVF_P.noise_variance_BLACK) + CLVF_P.RandomNumber_Mean;

    // End of InitializeConditions for RandomNumber: '<S527>/Random Number'

    // InitializeConditions for Delay: '<S545>/Delay1'
    CLVF_DW.icLoad_jt = 1U;

    // InitializeConditions for Delay: '<S539>/Delay1'
    CLVF_DW.icLoad_fs = 1U;

    // InitializeConditions for RandomNumber: '<S16>/Random Number1'
    CLVF_B.d = floor(CLVF_P.RandomNumber1_Seed);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_d = tseed;
    CLVF_DW.NextOutput_k = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_d) *
      CLVF_P.RandomNumber1_StdDev + CLVF_P.RandomNumber1_Mean;

    // End of InitializeConditions for RandomNumber: '<S16>/Random Number1'

    // InitializeConditions for Delay: '<S546>/Delay1'
    CLVF_DW.icLoad_b = 1U;

    // InitializeConditions for Delay: '<S540>/Delay1'
    CLVF_DW.icLoad_if = 1U;

    // InitializeConditions for RandomNumber: '<S16>/Random Number'
    CLVF_B.d = floor(CLVF_P.RandomNumber_Seed_f);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_b = tseed;
    CLVF_DW.NextOutput_n = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_b) *
      CLVF_P.RandomNumber_StdDev + CLVF_P.RandomNumber_Mean_o;

    // End of InitializeConditions for RandomNumber: '<S16>/Random Number'

    // InitializeConditions for Delay: '<S547>/Delay1'
    CLVF_DW.icLoad_eg = 1U;

    // InitializeConditions for RandomNumber: '<S16>/Random Number2'
    CLVF_B.d = floor(CLVF_P.RandomNumber2_Seed);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_a = tseed;
    CLVF_DW.NextOutput_m = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_a) *
      CLVF_P.RandomNumber2_StdDev + CLVF_P.RandomNumber2_Mean;

    // End of InitializeConditions for RandomNumber: '<S16>/Random Number2'

    // InitializeConditions for DiscreteIntegrator: '<S528>/Acceleration  to Velocity' 
    CLVF_DW.AccelerationtoVelocity_DSTATE_d[0] =
      CLVF_P.AccelerationtoVelocity_IC_n;

    // InitializeConditions for DiscreteIntegrator: '<S528>/Velocity to Position' 
    CLVF_DW.VelocitytoPosition_DSTATE_i[0] = CLVF_P.drop_states_BLUE[0];

    // InitializeConditions for DiscreteIntegrator: '<S528>/Acceleration  to Velocity' 
    CLVF_DW.AccelerationtoVelocity_DSTATE_d[1] =
      CLVF_P.AccelerationtoVelocity_IC_n;

    // InitializeConditions for DiscreteIntegrator: '<S528>/Velocity to Position' 
    CLVF_DW.VelocitytoPosition_DSTATE_i[1] = CLVF_P.drop_states_BLUE[1];

    // InitializeConditions for DiscreteIntegrator: '<S528>/Acceleration  to Velocity' 
    CLVF_DW.AccelerationtoVelocity_DSTATE_d[2] =
      CLVF_P.AccelerationtoVelocity_IC_n;

    // InitializeConditions for DiscreteIntegrator: '<S528>/Velocity to Position' 
    CLVF_DW.VelocitytoPosition_DSTATE_i[2] = CLVF_P.drop_states_BLUE[2];

    // InitializeConditions for RandomNumber: '<S528>/Random Number'
    CLVF_B.d = floor(CLVF_P.RandomNumber_Seed_g);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_h = tseed;
    CLVF_DW.NextOutput_h = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_h) *
      sqrt(CLVF_P.noise_variance_BLUE) + CLVF_P.RandomNumber_Mean_n;

    // End of InitializeConditions for RandomNumber: '<S528>/Random Number'

    // InitializeConditions for Delay: '<S548>/Delay1'
    CLVF_DW.icLoad_hr = 1U;

    // InitializeConditions for Delay: '<S541>/Delay1'
    CLVF_DW.icLoad_kp = 1U;

    // InitializeConditions for RandomNumber: '<S16>/Random Number4'
    CLVF_B.d = floor(CLVF_P.RandomNumber4_Seed);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_i = tseed;
    CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_i);

    // End of InitializeConditions for RandomNumber: '<S16>/Random Number4'

    // InitializeConditions for Delay: '<S549>/Delay1'
    CLVF_DW.icLoad_g = 1U;

    // InitializeConditions for Delay: '<S542>/Delay1'
    CLVF_DW.icLoad_ge = 1U;

    // InitializeConditions for RandomNumber: '<S16>/Random Number3'
    CLVF_B.d = floor(CLVF_P.RandomNumber3_Seed);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_c = tseed;
    CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_c);

    // End of InitializeConditions for RandomNumber: '<S16>/Random Number3'

    // InitializeConditions for Delay: '<S550>/Delay1'
    CLVF_DW.icLoad_ho = 1U;

    // InitializeConditions for RandomNumber: '<S16>/Random Number5'
    CLVF_B.d = floor(CLVF_P.RandomNumber5_Seed);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_n = tseed;
    CLVF_DW.NextOutput_eo = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_n)
      * CLVF_P.RandomNumber5_StdDev + CLVF_P.RandomNumber5_Mean;

    // End of InitializeConditions for RandomNumber: '<S16>/Random Number5'

    // InitializeConditions for DiscreteIntegrator: '<S16>/Discrete-Time Integrator' 
    CLVF_DW.DiscreteTimeIntegrator_PrevRese = 2;
    CLVF_DW.DiscreteTimeIntegrator_IC_LOADI = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S16>/Discrete-Time Integrator1' 
    CLVF_DW.DiscreteTimeIntegrator1_PrevRes = 2;
    CLVF_DW.DiscreteTimeIntegrator1_IC_LOAD = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S16>/Discrete-Time Integrator2' 
    CLVF_DW.DiscreteTimeIntegrator2_PrevRes = 2;
    CLVF_DW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S532>/Acceleration  to Velocity' 
    CLVF_DW.AccelerationtoVelocity_DSTATE_h[0] =
      CLVF_P.AccelerationtoVelocity_IC_a;

    // InitializeConditions for DiscreteIntegrator: '<S532>/Velocity to Position' 
    CLVF_DW.VelocitytoPosition_DSTATE_f[0] = CLVF_P.drop_states_RED[0];

    // InitializeConditions for DiscreteIntegrator: '<S532>/Acceleration  to Velocity' 
    CLVF_DW.AccelerationtoVelocity_DSTATE_h[1] =
      CLVF_P.AccelerationtoVelocity_IC_a;

    // InitializeConditions for DiscreteIntegrator: '<S532>/Velocity to Position' 
    CLVF_DW.VelocitytoPosition_DSTATE_f[1] = CLVF_P.drop_states_RED[1];

    // InitializeConditions for DiscreteIntegrator: '<S532>/Acceleration  to Velocity' 
    CLVF_DW.AccelerationtoVelocity_DSTATE_h[2] =
      CLVF_P.AccelerationtoVelocity_IC_a;

    // InitializeConditions for DiscreteIntegrator: '<S532>/Velocity to Position' 
    CLVF_DW.VelocitytoPosition_DSTATE_f[2] = CLVF_P.drop_states_RED[2];

    // InitializeConditions for RandomNumber: '<S532>/Random Number'
    CLVF_B.d = floor(CLVF_P.RandomNumber_Seed_p);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_ct = tseed;
    CLVF_DW.NextOutput_i = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_ct)
      * sqrt(CLVF_P.noise_variance_RED) + CLVF_P.RandomNumber_Mean_c;

    // End of InitializeConditions for RandomNumber: '<S532>/Random Number'

    // InitializeConditions for Delay: '<S536>/Delay1'
    CLVF_DW.icLoad_im = 1U;

    // InitializeConditions for Delay: '<S537>/Delay1'
    CLVF_DW.icLoad_jx = 1U;

    // InitializeConditions for RandomNumber: '<S16>/Random Number7'
    CLVF_B.d = floor(CLVF_P.RandomNumber7_Seed);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_dd = tseed;
    CLVF_DW.NextOutput_mh = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_dd)
      * CLVF_P.RandomNumber7_StdDev + CLVF_P.RandomNumber7_Mean;

    // End of InitializeConditions for RandomNumber: '<S16>/Random Number7'

    // InitializeConditions for Delay: '<S543>/Delay1'
    CLVF_DW.icLoad_mj = 1U;

    // InitializeConditions for Delay: '<S538>/Delay1'
    CLVF_DW.icLoad_nv = 1U;

    // InitializeConditions for RandomNumber: '<S16>/Random Number6'
    CLVF_B.d = floor(CLVF_P.RandomNumber6_Seed);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_ik = tseed;
    CLVF_DW.NextOutput_kj = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_ik)
      * CLVF_P.RandomNumber6_StdDev + CLVF_P.RandomNumber6_Mean;

    // End of InitializeConditions for RandomNumber: '<S16>/Random Number6'

    // InitializeConditions for Delay: '<S544>/Delay1'
    CLVF_DW.icLoad_nx = 1U;

    // InitializeConditions for RandomNumber: '<S16>/Random Number8'
    CLVF_B.d = floor(CLVF_P.RandomNumber8_Seed);
    if (rtIsNaN(CLVF_B.d) || rtIsInf(CLVF_B.d)) {
      CLVF_B.d = 0.0;
    } else {
      CLVF_B.d = fmod(CLVF_B.d, 4.294967296E+9);
    }

    tseed = CLVF_B.d < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(
      static_cast<uint32_T>(-CLVF_B.d))) : static_cast<uint32_T>(CLVF_B.d);
    ret = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(ret) << 16U)) + t) << 16U) + t) +
      ret;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    CLVF_DW.RandSeed_bm = tseed;
    CLVF_DW.NextOutput_m2 = CLVF_rt_nrand_Upu32_Yd_f_pw_snf(&CLVF_DW.RandSeed_bm)
      * CLVF_P.RandomNumber8_StdDev + CLVF_P.RandomNumber8_Mean;

    // End of InitializeConditions for RandomNumber: '<S16>/Random Number8'

    // SystemInitialize for IfAction SubSystem: '<S539>/Hold this value'
    // SystemInitialize for Outport: '<S563>/Out1'
    CLVF_B.In1_e1 = CLVF_P.Out1_Y0_ko;

    // End of SystemInitialize for SubSystem: '<S539>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S540>/Hold this value'
    // SystemInitialize for Outport: '<S564>/Out1'
    CLVF_B.In1_hm = CLVF_P.Out1_Y0_ar;

    // End of SystemInitialize for SubSystem: '<S540>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S545>/Hold this value'
    // SystemInitialize for Outport: '<S569>/Out1'
    CLVF_B.In1_aj = CLVF_P.Out1_Y0_lj;

    // End of SystemInitialize for SubSystem: '<S545>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S546>/Hold this value'
    // SystemInitialize for Outport: '<S570>/Out1'
    CLVF_B.In1_i5 = CLVF_P.Out1_Y0_h2;

    // End of SystemInitialize for SubSystem: '<S546>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S547>/Hold this value'
    // SystemInitialize for Outport: '<S571>/Out1'
    CLVF_B.In1_lx = CLVF_P.Out1_Y0_i3;

    // End of SystemInitialize for SubSystem: '<S547>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S548>/Hold this value'
    // SystemInitialize for Outport: '<S572>/Out1'
    CLVF_B.In1_n5 = CLVF_P.Out1_Y0_mh;

    // End of SystemInitialize for SubSystem: '<S548>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S549>/Hold this value'
    // SystemInitialize for Outport: '<S573>/Out1'
    CLVF_B.In1_l4 = CLVF_P.Out1_Y0_cb;

    // End of SystemInitialize for SubSystem: '<S549>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S550>/Hold this value'
    // SystemInitialize for Outport: '<S574>/Out1'
    CLVF_B.In1_p = CLVF_P.Out1_Y0_d1;

    // End of SystemInitialize for SubSystem: '<S550>/Hold this value'

    // SystemInitialize for Triggered SubSystem: '<S533>/Sample and Hold1'
    // SystemInitialize for Outport: '<S555>/ '
    CLVF_B.In_lb[0] = CLVF_P._Y0[0];

    // End of SystemInitialize for SubSystem: '<S533>/Sample and Hold1'

    // SystemInitialize for Triggered SubSystem: '<S534>/Sample and Hold1'
    // SystemInitialize for Outport: '<S557>/ '
    CLVF_B.In_e[0] = CLVF_P._Y0_m[0];

    // End of SystemInitialize for SubSystem: '<S534>/Sample and Hold1'

    // SystemInitialize for Triggered SubSystem: '<S535>/Sample and Hold'
    // SystemInitialize for Outport: '<S559>/ '
    CLVF_B.In_l[0] = CLVF_P._Y0_n[0];

    // End of SystemInitialize for SubSystem: '<S535>/Sample and Hold'

    // SystemInitialize for Triggered SubSystem: '<S533>/Sample and Hold1'
    // SystemInitialize for Outport: '<S555>/ '
    CLVF_B.In_lb[1] = CLVF_P._Y0[1];

    // End of SystemInitialize for SubSystem: '<S533>/Sample and Hold1'

    // SystemInitialize for Triggered SubSystem: '<S534>/Sample and Hold1'
    // SystemInitialize for Outport: '<S557>/ '
    CLVF_B.In_e[1] = CLVF_P._Y0_m[1];

    // End of SystemInitialize for SubSystem: '<S534>/Sample and Hold1'

    // SystemInitialize for Triggered SubSystem: '<S535>/Sample and Hold'
    // SystemInitialize for Outport: '<S559>/ '
    CLVF_B.In_l[1] = CLVF_P._Y0_n[1];

    // End of SystemInitialize for SubSystem: '<S535>/Sample and Hold'

    // SystemInitialize for Triggered SubSystem: '<S533>/Sample and Hold1'
    // SystemInitialize for Outport: '<S555>/ '
    CLVF_B.In_lb[2] = CLVF_P._Y0[2];

    // End of SystemInitialize for SubSystem: '<S533>/Sample and Hold1'

    // SystemInitialize for Triggered SubSystem: '<S534>/Sample and Hold1'
    // SystemInitialize for Outport: '<S557>/ '
    CLVF_B.In_e[2] = CLVF_P._Y0_m[2];

    // End of SystemInitialize for SubSystem: '<S534>/Sample and Hold1'

    // SystemInitialize for Triggered SubSystem: '<S535>/Sample and Hold'
    // SystemInitialize for Outport: '<S559>/ '
    CLVF_B.In_l[2] = CLVF_P._Y0_n[2];

    // End of SystemInitialize for SubSystem: '<S535>/Sample and Hold'

    // SystemInitialize for IfAction SubSystem: '<S536>/Hold this value'
    // SystemInitialize for Outport: '<S560>/Out1'
    CLVF_B.In1_al = CLVF_P.Out1_Y0_eq;

    // End of SystemInitialize for SubSystem: '<S536>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S537>/Hold this value'
    // SystemInitialize for Outport: '<S561>/Out1'
    CLVF_B.In1_lo = CLVF_P.Out1_Y0_ir;

    // End of SystemInitialize for SubSystem: '<S537>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S538>/Hold this value'
    // SystemInitialize for Outport: '<S562>/Out1'
    CLVF_B.In1_kx = CLVF_P.Out1_Y0_ny;

    // End of SystemInitialize for SubSystem: '<S538>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S543>/Hold this value'
    // SystemInitialize for Outport: '<S567>/Out1'
    CLVF_B.In1_jz = CLVF_P.Out1_Y0_ep;

    // End of SystemInitialize for SubSystem: '<S543>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S544>/Hold this value'
    // SystemInitialize for Outport: '<S568>/Out1'
    CLVF_B.In1_g0 = CLVF_P.Out1_Y0_ln;

    // End of SystemInitialize for SubSystem: '<S544>/Hold this value'
    // End of SystemInitialize for SubSystem: '<Root>/Simulate Plant Dynamics'

    // SystemInitialize for IfAction SubSystem: '<S2>/Change Behavior'
    // Start for MATLABSystem: '<S20>/Digital Write'
    CLVF_DW.obj_i.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_i.isInitialized = 0;
    CLVF_DW.obj_i.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_i.isSetupComplete = false;
    CLVF_DW.obj_i.isInitialized = 1;
    MW_gpioInit(10U, true);
    CLVF_DW.obj_i.isSetupComplete = true;

    // Start for MATLABSystem: '<S21>/Digital Write'
    CLVF_DW.obj_a.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_a.isInitialized = 0;
    CLVF_DW.obj_a.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_a.isSetupComplete = false;
    CLVF_DW.obj_a.isInitialized = 1;
    MW_gpioInit(26U, true);
    CLVF_DW.obj_a.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<S2>/Change Behavior'

    // SystemInitialize for Atomic SubSystem: '<S3>/Send Commands to PWM Blocks' 
    // Start for MATLABSystem: '<S26>/RED T1 - BLACK T8'
    CLVF_DW.obj_m.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_m.isInitialized = 0;
    CLVF_DW.obj_m.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_m.isSetupComplete = false;
    CLVF_DW.obj_m.isInitialized = 1;
    CLVF_DW.obj_m.PinNumber = 27U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_m.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_m.PinNumber);
    }

    CLVF_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/RED T1 - BLACK T8'

    // Start for MATLABSystem: '<S26>/RED T2 - BLACK T3'
    CLVF_DW.obj_ma.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_ma.isInitialized = 0;
    CLVF_DW.obj_ma.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_ma.isSetupComplete = false;
    CLVF_DW.obj_ma.isInitialized = 1;
    CLVF_DW.obj_ma.PinNumber = 19U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_ma.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_ma.PinNumber);
    }

    CLVF_DW.obj_ma.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/RED T2 - BLACK T3'

    // Start for MATLABSystem: '<S26>/RED T3'
    CLVF_DW.obj_h.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_h.isInitialized = 0;
    CLVF_DW.obj_h.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_h.isSetupComplete = false;
    CLVF_DW.obj_h.isInitialized = 1;
    CLVF_DW.obj_h.PinNumber = 4U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_h.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_h.PinNumber);
    }

    CLVF_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/RED T3'

    // Start for MATLABSystem: '<S26>/RED T4 - BLACK T5'
    CLVF_DW.obj_kw.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_kw.isInitialized = 0;
    CLVF_DW.obj_kw.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_kw.isSetupComplete = false;
    CLVF_DW.obj_kw.isInitialized = 1;
    CLVF_DW.obj_kw.PinNumber = 22U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_kw.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_kw.PinNumber);
    }

    CLVF_DW.obj_kw.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/RED T4 - BLACK T5'

    // Start for MATLABSystem: '<S26>/RED T5 - BLACK T4'
    CLVF_DW.obj_hw.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_hw.isInitialized = 0;
    CLVF_DW.obj_hw.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_hw.isSetupComplete = false;
    CLVF_DW.obj_hw.isInitialized = 1;
    CLVF_DW.obj_hw.PinNumber = 5U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_hw.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_hw.PinNumber);
    }

    CLVF_DW.obj_hw.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/RED T5 - BLACK T4'

    // Start for MATLABSystem: '<S26>/RED T6 - BLACK T7'
    CLVF_DW.obj_hf.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_hf.isInitialized = 0;
    CLVF_DW.obj_hf.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_hf.isSetupComplete = false;
    CLVF_DW.obj_hf.isInitialized = 1;
    CLVF_DW.obj_hf.PinNumber = 6U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_hf.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_hf.PinNumber);
    }

    CLVF_DW.obj_hf.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/RED T6 - BLACK T7'

    // Start for MATLABSystem: '<S26>/RED T7 - BLACK T6'
    CLVF_DW.obj_c.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_c.isInitialized = 0;
    CLVF_DW.obj_c.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_c.isSetupComplete = false;
    CLVF_DW.obj_c.isInitialized = 1;
    CLVF_DW.obj_c.PinNumber = 13U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_c.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_c.PinNumber);
    }

    CLVF_DW.obj_c.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/RED T7 - BLACK T6'

    // Start for MATLABSystem: '<S26>/RED T8 - BLACK T1'
    CLVF_DW.obj_fo.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_fo.isInitialized = 0;
    CLVF_DW.obj_fo.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_fo.isSetupComplete = false;
    CLVF_DW.obj_fo.isInitialized = 1;
    CLVF_DW.obj_fo.PinNumber = 17U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_fo.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_fo.PinNumber);
    }

    CLVF_DW.obj_fo.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/RED T8 - BLACK T1'

    // Start for MATLABSystem: '<S26>/BLACK T2'
    CLVF_DW.obj_ci.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_ci.isInitialized = 0;
    CLVF_DW.obj_ci.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_ci.isSetupComplete = false;
    CLVF_DW.obj_ci.isInitialized = 1;
    CLVF_DW.obj_ci.PinNumber = 9U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_ci.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_ci.PinNumber);
    }

    CLVF_DW.obj_ci.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S26>/BLACK T2'
    // End of SystemInitialize for SubSystem: '<S3>/Send Commands to PWM Blocks' 

    // SystemInitialize for IfAction SubSystem: '<S3>/Change BLACK Behavior'
    // SystemInitialize for Outport: '<S22>/BLACK PWM'
    for (ret = 0; ret < 8; ret++) {
      CLVF_B.sf_MATLABFunction2.ThrustPer_Final[ret] = CLVF_P.BLACKPWM_Y0;
    }

    // End of SystemInitialize for Outport: '<S22>/BLACK PWM'
    // End of SystemInitialize for SubSystem: '<S3>/Change BLACK Behavior'

    // SystemInitialize for IfAction SubSystem: '<S3>/Change BLUE Behavior'
    // SystemInitialize for Outport: '<S23>/BLUE PWM'
    for (ret = 0; ret < 8; ret++) {
      CLVF_B.sf_MATLABFunction2_n.ThrustPer_Final[ret] = CLVF_P.BLUEPWM_Y0;
    }

    // End of SystemInitialize for Outport: '<S23>/BLUE PWM'
    // End of SystemInitialize for SubSystem: '<S3>/Change BLUE Behavior'

    // SystemInitialize for IfAction SubSystem: '<S3>/Change RED Behavior'
    // SystemInitialize for Outport: '<S24>/RED PWM'
    for (ret = 0; ret < 8; ret++) {
      CLVF_B.sf_MATLABFunction2_l.ThrustPer_Final[ret] = CLVF_P.REDPWM_Y0;
    }

    // End of SystemInitialize for Outport: '<S24>/RED PWM'
    // End of SystemInitialize for SubSystem: '<S3>/Change RED Behavior'

    // SystemInitialize for IfAction SubSystem: '<S4>/Change RED Behavior'
    // InitializeConditions for Delay: '<S63>/Delay1'
    CLVF_DW.Delay1_DSTATE_kt = CLVF_P.Delay1_InitialCondition;

    // InitializeConditions for Delay: '<S63>/Delay2'
    CLVF_DW.Delay2_DSTATE[0] = CLVF_P.Delay2_InitialCondition;
    CLVF_DW.Delay2_DSTATE[1] = CLVF_P.Delay2_InitialCondition;

    // InitializeConditions for Delay: '<S63>/Delay5'
    CLVF_DW.Delay5_DSTATE[0] = CLVF_P.Delay5_InitialCondition;
    CLVF_DW.Delay5_DSTATE[1] = CLVF_P.Delay5_InitialCondition;
    CLVF_DW.Delay5_DSTATE[2] = CLVF_P.Delay5_InitialCondition;

    // InitializeConditions for Delay: '<S63>/Delay3'
    CLVF_DW.Delay3_DSTATE[0] = CLVF_P.Delay3_InitialCondition;
    CLVF_DW.Delay3_DSTATE[1] = CLVF_P.Delay3_InitialCondition;
    CLVF_DW.Delay3_DSTATE[2] = CLVF_P.Delay3_InitialCondition;
    CLVF_DW.Delay3_DSTATE[3] = CLVF_P.Delay3_InitialCondition;

    // InitializeConditions for Delay: '<S63>/Delay4'
    for (ret = 0; ret < 5; ret++) {
      CLVF_DW.Delay4_DSTATE[ret] = CLVF_P.Delay4_InitialCondition;
    }

    // End of InitializeConditions for Delay: '<S63>/Delay4'

    // InitializeConditions for Delay: '<S58>/Delay'
    CLVF_DW.Delay_DSTATE_k = CLVF_P.Delay_InitialCondition;

    // SystemInitialize for Atomic SubSystem: '<S59>/Send Direction to Motor Controller' 
    // Start for MATLABSystem: '<S71>/Digital Write'
    CLVF_DW.obj_b.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_b.isInitialized = 0;
    CLVF_DW.obj_b.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_b.isSetupComplete = false;
    CLVF_DW.obj_b.isInitialized = 1;
    MW_gpioInit(21U, true);
    CLVF_DW.obj_b.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<S59>/Send Direction to Motor Controller' 

    // SystemInitialize for Atomic SubSystem: '<S59>/Send PWM to Motor Controller' 
    // Start for MATLABSystem: '<S68>/PWM1'
    CLVF_DW.obj_d.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_d.isInitialized = 0;
    CLVF_DW.obj_d.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_d.isSetupComplete = false;
    CLVF_DW.obj_d.isInitialized = 1;
    CLVF_DW.obj_d.PinNumber = 18U;
    ret = EXT_PWMBlock_init(CLVF_DW.obj_d.PinNumber, 10U, 0.0);
    if (ret != 0) {
      for (ret = 0; ret < 45; ret++) {
        CLVF_B.cv[ret] = tmp[ret];
      }

      printf(CLVF_B.cv, CLVF_DW.obj_d.PinNumber);
    }

    CLVF_DW.obj_d.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S68>/PWM1'
    // End of SystemInitialize for SubSystem: '<S59>/Send PWM to Motor Controller' 

    // SystemInitialize for Atomic SubSystem: '<S59>/Turn on Motor'
    // Start for MATLABSystem: '<S72>/Digital Write'
    CLVF_DW.obj_cp.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_cp.isInitialized = 0;
    CLVF_DW.obj_cp.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_cp.isSetupComplete = false;
    CLVF_DW.obj_cp.isInitialized = 1;
    MW_gpioInit(25U, true);
    CLVF_DW.obj_cp.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<S59>/Turn on Motor'

    // SystemInitialize for Atomic SubSystem: '<S59>/Obtain RW Status'
    // Start for MATLABSystem: '<S70>/Digital Read'
    CLVF_DW.obj_f.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_f.isInitialized = 0;
    CLVF_DW.obj_f.matlabCodegenIsDeleted = false;
    if (((!rtIsInf(CLVF_P.DigitalRead_SampleTime)) && (!rtIsNaN
          (CLVF_P.DigitalRead_SampleTime))) || rtIsInf
        (CLVF_P.DigitalRead_SampleTime)) {
      CLVF_B.sampleTime = CLVF_P.DigitalRead_SampleTime;
    }

    CLVF_DW.obj_f.SampleTime = CLVF_B.sampleTime;
    CLVF_DW.obj_f.isSetupComplete = false;
    CLVF_DW.obj_f.isInitialized = 1;
    MW_gpioInit(24U, false);
    CLVF_DW.obj_f.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S70>/Digital Read'
    // End of SystemInitialize for SubSystem: '<S59>/Obtain RW Status'
    // End of SystemInitialize for SubSystem: '<S4>/Change RED Behavior'

    // SystemInitialize for IfAction SubSystem: '<S5>/Use Hardware to Obtain States' 
    // Start for If: '<S75>/Check whether both platforms are being used, and if so then use RED to send data to BLACK ' 
    CLVF_DW.Checkwhetherbothplatformsarebei = -1;

    // SystemInitialize for IfAction SubSystem: '<S75>/Using RED+BLACK, or RED+BLACK+ARM' 
    // Start for If: '<S76>/This IF block determines whether or not to run the BLACK sim//exp' 
    CLVF_DW.ThisIFblockdetermineswhethero_c = -1;

    // Start for If: '<S76>/This IF block determines whether or not to run the RED sim//exp ' 
    CLVF_DW.ThisIFblockdetermineswhether_cx = -1;

    // SystemInitialize for IfAction SubSystem: '<S76>/Obtain BLACK States'
    // Start for S-Function (sdspFromNetwork): '<S79>/UDP Receive'
    sErr = GetErrorBuffer(&CLVF_DW.UDPReceive_NetworkLib[0U]);
    CreateUDPInterface(&CLVF_DW.UDPReceive_NetworkLib[0U]);
    if (*sErr == 0) {
      LibCreate_Network(&CLVF_DW.UDPReceive_NetworkLib[0U], 0, "0.0.0.0",
                        CLVF_P.UDPReceive_Port, "0.0.0.0", -1, 8192, 8, 0);
    }

    if (*sErr == 0) {
      LibStart(&CLVF_DW.UDPReceive_NetworkLib[0U]);
    }

    if (*sErr != 0) {
      DestroyUDPInterface(&CLVF_DW.UDPReceive_NetworkLib[0U]);
      if (*sErr != 0) {
        rtmSetErrorStatus(CLVF_M, sErr);
        rtmSetStopRequested(CLVF_M, 1);
      }
    }

    // End of Start for S-Function (sdspFromNetwork): '<S79>/UDP Receive'

    // InitializeConditions for Delay: '<S87>/Delay1'
    CLVF_DW.icLoad_ha = 1U;

    // InitializeConditions for Delay: '<S82>/Delay1'
    CLVF_DW.icLoad_oh = 1U;

    // InitializeConditions for Delay: '<S88>/Delay1'
    CLVF_DW.icLoad_ij = 1U;

    // InitializeConditions for Delay: '<S83>/Delay1'
    CLVF_DW.icLoad_dr = 1U;

    // InitializeConditions for S-Function (sdspunwrap2): '<S79>/Unwrap1'
    CLVF_DW.Unwrap1_FirstStep_j = true;

    // InitializeConditions for Delay: '<S89>/Delay1'
    CLVF_DW.icLoad_ip = 1U;

    // InitializeConditions for Delay: '<S84>/Delay1'
    CLVF_DW.icLoad_lp = 1U;

    // InitializeConditions for Delay: '<S81>/Delay1'
    CLVF_DW.icLoad_nj = 1U;

    // InitializeConditions for Delay: '<S90>/Delay1'
    CLVF_DW.icLoad_av = 1U;

    // InitializeConditions for Delay: '<S85>/Delay1'
    CLVF_DW.icLoad_ib = 1U;

    // InitializeConditions for Delay: '<S91>/Delay1'
    CLVF_DW.icLoad_hm = 1U;

    // InitializeConditions for S-Function (sdspunwrap2): '<S79>/Unwrap'
    CLVF_DW.Unwrap_FirstStep_e = true;

    // InitializeConditions for Delay: '<S86>/Delay1'
    CLVF_DW.icLoad_jh = 1U;

    // InitializeConditions for Delay: '<S92>/Delay1'
    CLVF_DW.icLoad_bb = 1U;

    // SystemInitialize for IfAction SubSystem: '<S81>/Hold this value'
    // SystemInitialize for Outport: '<S93>/Out1'
    CLVF_B.In1_it = CLVF_P.Out1_Y0_k4;

    // End of SystemInitialize for SubSystem: '<S81>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S82>/Hold this value'
    // SystemInitialize for Outport: '<S94>/Out1'
    CLVF_B.In1_kw = CLVF_P.Out1_Y0_c;

    // End of SystemInitialize for SubSystem: '<S82>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S83>/Hold this value'
    // SystemInitialize for Outport: '<S95>/Out1'
    CLVF_B.In1_hr = CLVF_P.Out1_Y0_l0;

    // End of SystemInitialize for SubSystem: '<S83>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S84>/Hold this value'
    // SystemInitialize for Outport: '<S96>/Out1'
    CLVF_B.In1_fi = CLVF_P.Out1_Y0_e;

    // End of SystemInitialize for SubSystem: '<S84>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S85>/Hold this value'
    // SystemInitialize for Outport: '<S97>/Out1'
    CLVF_B.In1_ix = CLVF_P.Out1_Y0_d;

    // End of SystemInitialize for SubSystem: '<S85>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S86>/Hold this value'
    // SystemInitialize for Outport: '<S98>/Out1'
    CLVF_B.In1_nd = CLVF_P.Out1_Y0_g;

    // End of SystemInitialize for SubSystem: '<S86>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S87>/Hold this value'
    // SystemInitialize for Outport: '<S99>/Out1'
    CLVF_B.In1_h1 = CLVF_P.Out1_Y0_ft;

    // End of SystemInitialize for SubSystem: '<S87>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S88>/Hold this value'
    // SystemInitialize for Outport: '<S100>/Out1'
    CLVF_B.In1_mh = CLVF_P.Out1_Y0_j1;

    // End of SystemInitialize for SubSystem: '<S88>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S89>/Hold this value'
    // SystemInitialize for Outport: '<S101>/Out1'
    CLVF_B.In1_bc = CLVF_P.Out1_Y0_ii;

    // End of SystemInitialize for SubSystem: '<S89>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S90>/Hold this value'
    // SystemInitialize for Outport: '<S102>/Out1'
    CLVF_B.In1_jl = CLVF_P.Out1_Y0_i1;

    // End of SystemInitialize for SubSystem: '<S90>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S91>/Hold this value'
    // SystemInitialize for Outport: '<S103>/Out1'
    CLVF_B.In1_cv = CLVF_P.Out1_Y0_pa;

    // End of SystemInitialize for SubSystem: '<S91>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S92>/Hold this value'
    // SystemInitialize for Outport: '<S104>/Out1'
    CLVF_B.In1_ow = CLVF_P.Out1_Y0_h;

    // End of SystemInitialize for SubSystem: '<S92>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S76>/Obtain BLACK States'

    // SystemInitialize for IfAction SubSystem: '<S76>/Obtain RED States'
    // Start for S-Function (sdspToNetwork): '<S80>/Send BLACK States to  BLACK Platform' 
    sErr = GetErrorBuffer(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U]);
    CreateUDPInterface(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U]);
    if (*sErr == 0) {
      LibCreate_Network(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U], 1,
                        "255.255.255.255", -1, "192.168.0.101",
                        CLVF_P.SendBLACKStatestoBLACKPlatform_, 8192, 8, 0);
    }

    if (*sErr == 0) {
      LibStart(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U]);
    }

    if (*sErr != 0) {
      DestroyUDPInterface(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U]);
      if (*sErr != 0) {
        rtmSetErrorStatus(CLVF_M, sErr);
        rtmSetStopRequested(CLVF_M, 1);
      }
    }

    // End of Start for S-Function (sdspToNetwork): '<S80>/Send BLACK States to  BLACK Platform' 

    // InitializeConditions for Delay: '<S121>/Delay'
    CLVF_DW.Delay_DSTATE_k2 = CLVF_P.Delay_InitialCondition_g;

    // InitializeConditions for Delay: '<S115>/Delay1'
    CLVF_DW.icLoad_bo = 1U;

    // InitializeConditions for Delay: '<S110>/Delay1'
    CLVF_DW.icLoad_lm = 1U;

    // InitializeConditions for Delay: '<S116>/Delay1'
    CLVF_DW.icLoad_h2 = 1U;

    // InitializeConditions for Delay: '<S111>/Delay1'
    CLVF_DW.icLoad_fc0 = 1U;

    // InitializeConditions for S-Function (sdspunwrap2): '<S80>/Unwrap1'
    CLVF_DW.Unwrap1_FirstStep = true;

    // InitializeConditions for Delay: '<S117>/Delay1'
    CLVF_DW.icLoad_jq = 1U;

    // InitializeConditions for Delay: '<S112>/Delay1'
    CLVF_DW.icLoad_ab = 1U;

    // InitializeConditions for Delay: '<S109>/Delay1'
    CLVF_DW.icLoad_ez = 1U;

    // InitializeConditions for Delay: '<S118>/Delay1'
    CLVF_DW.icLoad_n4 = 1U;

    // InitializeConditions for Delay: '<S113>/Delay1'
    CLVF_DW.icLoad_ln = 1U;

    // InitializeConditions for Delay: '<S119>/Delay1'
    CLVF_DW.icLoad_dk = 1U;

    // InitializeConditions for S-Function (sdspunwrap2): '<S80>/Unwrap'
    CLVF_DW.Unwrap_FirstStep = true;

    // InitializeConditions for Delay: '<S114>/Delay1'
    CLVF_DW.icLoad_kr = 1U;

    // InitializeConditions for Delay: '<S120>/Delay1'
    CLVF_DW.icLoad_gt = 1U;

    // SystemInitialize for Enabled SubSystem: '<S106>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S122>/Out1'
    CLVF_B.In1_fg = CLVF_P.Out1_Y0_ee;

    // End of SystemInitialize for SubSystem: '<S106>/Enabled Subsystem'

    // SystemInitialize for Enabled SubSystem: '<S80>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S107>/Out1'
    for (ret = 0; ret < 13; ret++) {
      CLVF_B.In1_mk[ret] = CLVF_P.Out1_Y0_ob;
    }

    // End of SystemInitialize for Outport: '<S107>/Out1'
    // End of SystemInitialize for SubSystem: '<S80>/Enabled Subsystem'

    // SystemInitialize for Enabled SubSystem: '<S80>/Enabled Subsystem1'
    // SystemInitialize for Outport: '<S108>/Actual Time'
    CLVF_B.In1_lxy = CLVF_P.ActualTime_Y0_f;

    // End of SystemInitialize for SubSystem: '<S80>/Enabled Subsystem1'

    // SystemInitialize for IfAction SubSystem: '<S109>/Hold this value'
    // SystemInitialize for Outport: '<S125>/Out1'
    CLVF_B.In1_g2 = CLVF_P.Out1_Y0_oy;

    // End of SystemInitialize for SubSystem: '<S109>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S110>/Hold this value'
    // SystemInitialize for Outport: '<S126>/Out1'
    CLVF_B.In1_ij = CLVF_P.Out1_Y0_l2;

    // End of SystemInitialize for SubSystem: '<S110>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S111>/Hold this value'
    // SystemInitialize for Outport: '<S127>/Out1'
    CLVF_B.In1_hp = CLVF_P.Out1_Y0_ow;

    // End of SystemInitialize for SubSystem: '<S111>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S112>/Hold this value'
    // SystemInitialize for Outport: '<S128>/Out1'
    CLVF_B.In1_ls = CLVF_P.Out1_Y0_mx;

    // End of SystemInitialize for SubSystem: '<S112>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S113>/Hold this value'
    // SystemInitialize for Outport: '<S129>/Out1'
    CLVF_B.In1_eo = CLVF_P.Out1_Y0_gm;

    // End of SystemInitialize for SubSystem: '<S113>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S114>/Hold this value'
    // SystemInitialize for Outport: '<S130>/Out1'
    CLVF_B.In1_jj = CLVF_P.Out1_Y0_h4;

    // End of SystemInitialize for SubSystem: '<S114>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S115>/Hold this value'
    // SystemInitialize for Outport: '<S131>/Out1'
    CLVF_B.In1_mp = CLVF_P.Out1_Y0_kh;

    // End of SystemInitialize for SubSystem: '<S115>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S116>/Hold this value'
    // SystemInitialize for Outport: '<S132>/Out1'
    CLVF_B.In1_as = CLVF_P.Out1_Y0_fm;

    // End of SystemInitialize for SubSystem: '<S116>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S117>/Hold this value'
    // SystemInitialize for Outport: '<S133>/Out1'
    CLVF_B.In1_gn = CLVF_P.Out1_Y0_er;

    // End of SystemInitialize for SubSystem: '<S117>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S118>/Hold this value'
    // SystemInitialize for Outport: '<S134>/Out1'
    CLVF_B.In1_eh = CLVF_P.Out1_Y0_il;

    // End of SystemInitialize for SubSystem: '<S118>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S119>/Hold this value'
    // SystemInitialize for Outport: '<S135>/Out1'
    CLVF_B.In1_of = CLVF_P.Out1_Y0_mr;

    // End of SystemInitialize for SubSystem: '<S119>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S120>/Hold this value'
    // SystemInitialize for Outport: '<S136>/Out1'
    CLVF_B.In1_o4 = CLVF_P.Out1_Y0_jf;

    // End of SystemInitialize for SubSystem: '<S120>/Hold this value'

    // Start for MATLABSystem: '<S80>/Stream PhaseSpace to Platform'
    CLVF_DW.obj_k.matlabCodegenIsDeleted = true;
    CLVF_DW.obj_k.isInitialized = 0;
    CLVF_DW.obj_k.matlabCodegenIsDeleted = false;
    CLVF_DW.obj_k.platformSelection = CLVF_P.platformSelection;
    CLVF_DW.obj_k.PS_SampleRate = 1.0 / CLVF_P.serverRate;
    CLVF_DW.obj_k.isSetupComplete = false;
    CLVF_DW.obj_k.isInitialized = 1;
    initialize_phasespace(CLVF_DW.obj_k.platformSelection,
                          CLVF_DW.obj_k.PS_SampleRate);
    CLVF_DW.obj_k.isSetupComplete = true;

    // End of SystemInitialize for SubSystem: '<S76>/Obtain RED States'
    // End of SystemInitialize for SubSystem: '<S75>/Using RED+BLACK, or RED+BLACK+ARM' 

    // SystemInitialize for IfAction SubSystem: '<S75>/Using RED, BLACK, BLUE, or RED + ARM' 
    // InitializeConditions for Delay: '<S153>/Delay'
    CLVF_DW.Delay_DSTATE_m = CLVF_P.Delay_InitialCondition_m;

    // InitializeConditions for Delay: '<S147>/Delay1'
    CLVF_DW.icLoad_po = 1U;

    // InitializeConditions for Delay: '<S142>/Delay1'
    CLVF_DW.icLoad_nx2 = 1U;

    // InitializeConditions for Delay: '<S148>/Delay1'
    CLVF_DW.icLoad_on = 1U;

    // InitializeConditions for Delay: '<S143>/Delay1'
    CLVF_DW.icLoad_lw = 1U;

    // InitializeConditions for S-Function (sdspunwrap2): '<S77>/Unwrap1'
    CLVF_DW.Unwrap1_FirstStep_jh = true;

    // InitializeConditions for Delay: '<S149>/Delay1'
    CLVF_DW.icLoad_dx = 1U;

    // InitializeConditions for Delay: '<S144>/Delay1'
    CLVF_DW.icLoad_pz = 1U;

    // InitializeConditions for Delay: '<S141>/Delay1'
    CLVF_DW.icLoad_jj = 1U;

    // InitializeConditions for Delay: '<S150>/Delay1'
    CLVF_DW.icLoad_fv = 1U;

    // InitializeConditions for Delay: '<S145>/Delay1'
    CLVF_DW.icLoad_jqu = 1U;

    // InitializeConditions for Delay: '<S151>/Delay1'
    CLVF_DW.icLoad_b3 = 1U;

    // InitializeConditions for S-Function (sdspunwrap2): '<S77>/Unwrap'
    CLVF_DW.Unwrap_FirstStep_b = true;

    // InitializeConditions for Delay: '<S146>/Delay1'
    CLVF_DW.icLoad_cw = 1U;

    // InitializeConditions for Delay: '<S152>/Delay1'
    CLVF_DW.icLoad_hrx = 1U;

    // SystemInitialize for Enabled SubSystem: '<S138>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S154>/Out1'
    CLVF_B.In1_g5 = CLVF_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S138>/Enabled Subsystem'

    // SystemInitialize for Enabled SubSystem: '<S77>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S139>/Out1'
    for (ret = 0; ret < 13; ret++) {
      CLVF_B.In1_ak[ret] = CLVF_P.Out1_Y0_b;
    }

    // End of SystemInitialize for Outport: '<S139>/Out1'
    // End of SystemInitialize for SubSystem: '<S77>/Enabled Subsystem'

    // SystemInitialize for Enabled SubSystem: '<S77>/Enabled Subsystem1'
    // SystemInitialize for Outport: '<S140>/Actual Time'
    CLVF_B.In1_oz = CLVF_P.ActualTime_Y0;

    // End of SystemInitialize for SubSystem: '<S77>/Enabled Subsystem1'

    // SystemInitialize for IfAction SubSystem: '<S141>/Hold this value'
    // SystemInitialize for Outport: '<S157>/Out1'
    CLVF_B.In1_o2 = CLVF_P.Out1_Y0_o;

    // End of SystemInitialize for SubSystem: '<S141>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S142>/Hold this value'
    // SystemInitialize for Outport: '<S158>/Out1'
    CLVF_B.In1_lu = CLVF_P.Out1_Y0_k;

    // End of SystemInitialize for SubSystem: '<S142>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S143>/Hold this value'
    // SystemInitialize for Outport: '<S159>/Out1'
    CLVF_B.In1_k0 = CLVF_P.Out1_Y0_m;

    // End of SystemInitialize for SubSystem: '<S143>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S144>/Hold this value'
    // SystemInitialize for Outport: '<S160>/Out1'
    CLVF_B.In1_la = CLVF_P.Out1_Y0_j;

    // End of SystemInitialize for SubSystem: '<S144>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S145>/Hold this value'
    // SystemInitialize for Outport: '<S161>/Out1'
    CLVF_B.In1_mk3 = CLVF_P.Out1_Y0_f;

    // End of SystemInitialize for SubSystem: '<S145>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S146>/Hold this value'
    // SystemInitialize for Outport: '<S162>/Out1'
    CLVF_B.In1_cp = CLVF_P.Out1_Y0_p;

    // End of SystemInitialize for SubSystem: '<S146>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S147>/Hold this value'
    // SystemInitialize for Outport: '<S163>/Out1'
    CLVF_B.In1_gi = CLVF_P.Out1_Y0_i;

    // End of SystemInitialize for SubSystem: '<S147>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S148>/Hold this value'
    // SystemInitialize for Outport: '<S164>/Out1'
    CLVF_B.In1_oq = CLVF_P.Out1_Y0_l;

    // End of SystemInitialize for SubSystem: '<S148>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S149>/Hold this value'
    // SystemInitialize for Outport: '<S165>/Out1'
    CLVF_B.In1_hy = CLVF_P.Out1_Y0_mc;

    // End of SystemInitialize for SubSystem: '<S149>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S150>/Hold this value'
    // SystemInitialize for Outport: '<S166>/Out1'
    CLVF_B.In1_ii = CLVF_P.Out1_Y0_ie;

    // End of SystemInitialize for SubSystem: '<S150>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S151>/Hold this value'
    // SystemInitialize for Outport: '<S167>/Out1'
    CLVF_B.In1_bw = CLVF_P.Out1_Y0_a;

    // End of SystemInitialize for SubSystem: '<S151>/Hold this value'

    // SystemInitialize for IfAction SubSystem: '<S152>/Hold this value'
    // SystemInitialize for Outport: '<S168>/Out1'
    CLVF_B.In1_bj = CLVF_P.Out1_Y0_ief;

    // End of SystemInitialize for SubSystem: '<S152>/Hold this value'
    // End of SystemInitialize for SubSystem: '<S75>/Using RED, BLACK, BLUE, or RED + ARM' 
    // End of SystemInitialize for SubSystem: '<S5>/Use Hardware to Obtain States' 

    // SystemInitialize for IfAction SubSystem: '<S6>/Change BLACK Behavior'
    // InitializeConditions for DiscreteIntegrator: '<S181>/Discrete-Time Integrator1' 
    CLVF_DW.DiscreteTimeIntegrator1_PrevR_d = 2;
    CLVF_DW.DiscreteTimeIntegrator1_IC_LO_f = 1U;

    // SystemInitialize for IfAction SubSystem: '<S181>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_c,
      &CLVF_DW.CalculateRunningMean_c, &CLVF_P.CalculateRunningMean_c);

    // End of SystemInitialize for SubSystem: '<S181>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S181>/Pass Current Gyro'
    // SystemInitialize for Outport: '<S190>/Out1'
    CLVF_B.In1_ln = CLVF_P.Out1_Y0_n3;

    // End of SystemInitialize for SubSystem: '<S181>/Pass Current Gyro'

    // SystemInitialize for IfAction SubSystem: '<S179>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean,
      &CLVF_DW.CalculateRunningMean, &CLVF_P.CalculateRunningMean);

    // End of SystemInitialize for SubSystem: '<S179>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S179>/Pass Current Gyro'
    // SystemInitialize for Outport: '<S183>/Out1'
    CLVF_B.In1_mx = CLVF_P.Out1_Y0_n;

    // End of SystemInitialize for SubSystem: '<S179>/Pass Current Gyro'

    // SystemInitialize for IfAction SubSystem: '<S180>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_p,
      &CLVF_DW.CalculateRunningMean_p, &CLVF_P.CalculateRunningMean_p);

    // End of SystemInitialize for SubSystem: '<S180>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S180>/Pass Current Gyro'
    // SystemInitialize for Outport: '<S186>/Out1'
    CLVF_B.In1_m = CLVF_P.Out1_Y0_kc;

    // End of SystemInitialize for SubSystem: '<S180>/Pass Current Gyro'

    // SystemInitialize for IfAction SubSystem: '<S192>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_ch,
      &CLVF_DW.CalculateRunningMean_ch, &CLVF_P.CalculateRunningMean_ch);

    // End of SystemInitialize for SubSystem: '<S192>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S192>/Pass Current Accel'
    // SystemInitialize for Outport: '<S196>/Out1'
    CLVF_B.In1_lw = CLVF_P.Out1_Y0_hs;

    // End of SystemInitialize for SubSystem: '<S192>/Pass Current Accel'

    // SystemInitialize for IfAction SubSystem: '<S193>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_k,
      &CLVF_DW.CalculateRunningMean_k, &CLVF_P.CalculateRunningMean_k);

    // End of SystemInitialize for SubSystem: '<S193>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S193>/Pass Current Accel'
    // SystemInitialize for Outport: '<S199>/Out1'
    CLVF_B.In1_gr = CLVF_P.Out1_Y0_gz;

    // End of SystemInitialize for SubSystem: '<S193>/Pass Current Accel'

    // SystemInitialize for IfAction SubSystem: '<S194>/Calculate Running Mean'
    CalculateRunningMean_b_Init(&CLVF_B.CalculateRunningMean_b,
      &CLVF_DW.CalculateRunningMean_b, &CLVF_P.CalculateRunningMean_b);

    // End of SystemInitialize for SubSystem: '<S194>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S194>/Pass Current Accel'
    // SystemInitialize for Outport: '<S202>/Out1'
    CLVF_B.In1_o0 = CLVF_P.Out1_Y0_cg;

    // End of SystemInitialize for SubSystem: '<S194>/Pass Current Accel'
    CLVF_AHRS2_Init(CLVF_B.DigitalFilter_p, &CLVF_DW.AHRS2, &CLVF_P.AHRS2);

    // End of SystemInitialize for SubSystem: '<S6>/Change BLACK Behavior'

    // SystemInitialize for IfAction SubSystem: '<S6>/Change BLUE Behavior'
    // InitializeConditions for DiscreteIntegrator: '<S212>/Discrete-Time Integrator1' 
    CLVF_DW.DiscreteTimeIntegrator1_PrevR_h = 2;
    CLVF_DW.DiscreteTimeIntegrator1_IC_LO_l = 1U;

    // SystemInitialize for IfAction SubSystem: '<S212>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_av,
      &CLVF_DW.CalculateRunningMean_av, &CLVF_P.CalculateRunningMean_av);

    // End of SystemInitialize for SubSystem: '<S212>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S212>/Pass Current Gyro'
    // SystemInitialize for Outport: '<S221>/Out1'
    CLVF_B.In1_df = CLVF_P.Out1_Y0_dr;

    // End of SystemInitialize for SubSystem: '<S212>/Pass Current Gyro'

    // SystemInitialize for IfAction SubSystem: '<S210>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_b2,
      &CLVF_DW.CalculateRunningMean_b2, &CLVF_P.CalculateRunningMean_b2);

    // End of SystemInitialize for SubSystem: '<S210>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S211>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_a,
      &CLVF_DW.CalculateRunningMean_a, &CLVF_P.CalculateRunningMean_a);

    // End of SystemInitialize for SubSystem: '<S211>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S223>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_m,
      &CLVF_DW.CalculateRunningMean_m, &CLVF_P.CalculateRunningMean_m);

    // End of SystemInitialize for SubSystem: '<S223>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S223>/Pass Current Accel'
    // SystemInitialize for Outport: '<S227>/Out1'
    CLVF_B.In1_dh = CLVF_P.Out1_Y0_kcv;

    // End of SystemInitialize for SubSystem: '<S223>/Pass Current Accel'

    // SystemInitialize for IfAction SubSystem: '<S224>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_pa,
      &CLVF_DW.CalculateRunningMean_pa, &CLVF_P.CalculateRunningMean_pa);

    // End of SystemInitialize for SubSystem: '<S224>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S224>/Pass Current Accel'
    // SystemInitialize for Outport: '<S230>/Out1'
    CLVF_B.In1_px = CLVF_P.Out1_Y0_ie0;

    // End of SystemInitialize for SubSystem: '<S224>/Pass Current Accel'

    // SystemInitialize for IfAction SubSystem: '<S225>/Calculate Running Mean'
    CalculateRunningMean_b_Init(&CLVF_B.CalculateRunningMean_l,
      &CLVF_DW.CalculateRunningMean_l, &CLVF_P.CalculateRunningMean_l);

    // End of SystemInitialize for SubSystem: '<S225>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S225>/Pass Current Accel'
    // SystemInitialize for Outport: '<S233>/Out1'
    CLVF_B.In1_bl = CLVF_P.Out1_Y0_gi;

    // End of SystemInitialize for SubSystem: '<S225>/Pass Current Accel'
    CLVF_AHRS2_Init(CLVF_B.DigitalFilter_l, &CLVF_DW.AHRS2_p, &CLVF_P.AHRS2_p);

    // End of SystemInitialize for SubSystem: '<S6>/Change BLUE Behavior'

    // SystemInitialize for IfAction SubSystem: '<S6>/Change RED Behavior'
    // InitializeConditions for DiscreteIntegrator: '<S243>/Discrete-Time Integrator1' 
    CLVF_DW.DiscreteTimeIntegrator1_PrevR_m = 2;
    CLVF_DW.DiscreteTimeIntegrator1_IC_LO_b = 1U;

    // SystemInitialize for IfAction SubSystem: '<S243>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_avn,
      &CLVF_DW.CalculateRunningMean_avn, &CLVF_P.CalculateRunningMean_avn);

    // End of SystemInitialize for SubSystem: '<S243>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S243>/Pass Current Gyro'
    // SystemInitialize for Outport: '<S252>/Out1'
    CLVF_B.In1_eu = CLVF_P.Out1_Y0_m0;

    // End of SystemInitialize for SubSystem: '<S243>/Pass Current Gyro'

    // SystemInitialize for IfAction SubSystem: '<S241>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_mr,
      &CLVF_DW.CalculateRunningMean_mr, &CLVF_P.CalculateRunningMean_mr);

    // End of SystemInitialize for SubSystem: '<S241>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S241>/Pass Current Gyro'
    // SystemInitialize for Outport: '<S245>/Out1'
    CLVF_B.In1_hk = CLVF_P.Out1_Y0_az;

    // End of SystemInitialize for SubSystem: '<S241>/Pass Current Gyro'

    // SystemInitialize for IfAction SubSystem: '<S242>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_bv,
      &CLVF_DW.CalculateRunningMean_bv, &CLVF_P.CalculateRunningMean_bv);

    // End of SystemInitialize for SubSystem: '<S242>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S242>/Pass Current Gyro'
    // SystemInitialize for Outport: '<S248>/Out1'
    CLVF_B.In1_ajd = CLVF_P.Out1_Y0_cc;

    // End of SystemInitialize for SubSystem: '<S242>/Pass Current Gyro'

    // SystemInitialize for IfAction SubSystem: '<S254>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_ck,
      &CLVF_DW.CalculateRunningMean_ck, &CLVF_P.CalculateRunningMean_ck);

    // End of SystemInitialize for SubSystem: '<S254>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S254>/Pass Current Accel'
    // SystemInitialize for Outport: '<S258>/Out1'
    CLVF_B.In1_lp = CLVF_P.Out1_Y0_cz;

    // End of SystemInitialize for SubSystem: '<S254>/Pass Current Accel'

    // SystemInitialize for IfAction SubSystem: '<S255>/Calculate Running Mean'
    C_CalculateRunningMean_Init(&CLVF_B.CalculateRunningMean_e,
      &CLVF_DW.CalculateRunningMean_e, &CLVF_P.CalculateRunningMean_e);

    // End of SystemInitialize for SubSystem: '<S255>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S255>/Pass Current Accel'
    // SystemInitialize for Outport: '<S261>/Out1'
    CLVF_B.In1_e2 = CLVF_P.Out1_Y0_ai;

    // End of SystemInitialize for SubSystem: '<S255>/Pass Current Accel'

    // SystemInitialize for IfAction SubSystem: '<S256>/Calculate Running Mean'
    CalculateRunningMean_b_Init(&CLVF_B.CalculateRunningMean_mv,
      &CLVF_DW.CalculateRunningMean_mv, &CLVF_P.CalculateRunningMean_mv);

    // End of SystemInitialize for SubSystem: '<S256>/Calculate Running Mean'

    // SystemInitialize for IfAction SubSystem: '<S256>/Pass Current Accel'
    // SystemInitialize for Outport: '<S264>/Out1'
    CLVF_B.In1_co = CLVF_P.Out1_Y0_ef;

    // End of SystemInitialize for SubSystem: '<S256>/Pass Current Accel'
    CLVF_AHRS2_Init(CLVF_B.DigitalFilter, &CLVF_DW.AHRS2_pn, &CLVF_P.AHRS2_pn);

    // End of SystemInitialize for SubSystem: '<S6>/Change RED Behavior'

    // Start for MATLABSystem: '<S6>/LSM9DS1 IMU Sensor'
    CLVF_DW.obj.i2cobj_A_G.matlabCodegenIsDeleted = true;
    CLVF_DW.obj.i2cobj_MAG.matlabCodegenIsDeleted = true;
    CLVF_DW.obj.matlabCodegenIsDeleted = true;
    CLVF_lsm9ds1Block_lsm9ds1Block(&CLVF_DW.obj);
    CLVF_SystemCore_setup(&CLVF_DW.obj);
  }
}

// Model terminate function
void CLVF_terminate(void)
{
  char_T *sErr;

  // Terminate for IfAction SubSystem: '<S2>/Change Behavior'
  // Terminate for MATLABSystem: '<S20>/Digital Write'
  matlabCodegenHandle_matlabCodeg(&CLVF_DW.obj_i);

  // Terminate for MATLABSystem: '<S21>/Digital Write'
  matlabCodegenHandle_matlabCod_g(&CLVF_DW.obj_a);

  // End of Terminate for SubSystem: '<S2>/Change Behavior'

  // Terminate for IfAction SubSystem: '<S5>/Use Hardware to Obtain States'
  // Terminate for IfAction SubSystem: '<S75>/Using RED+BLACK, or RED+BLACK+ARM' 
  // Terminate for IfAction SubSystem: '<S76>/Obtain BLACK States'
  // Terminate for S-Function (sdspFromNetwork): '<S79>/UDP Receive'
  sErr = GetErrorBuffer(&CLVF_DW.UDPReceive_NetworkLib[0U]);
  LibTerminate(&CLVF_DW.UDPReceive_NetworkLib[0U]);
  if (*sErr != 0) {
    rtmSetErrorStatus(CLVF_M, sErr);
    rtmSetStopRequested(CLVF_M, 1);
  }

  LibDestroy(&CLVF_DW.UDPReceive_NetworkLib[0U], 0);
  DestroyUDPInterface(&CLVF_DW.UDPReceive_NetworkLib[0U]);

  // End of Terminate for S-Function (sdspFromNetwork): '<S79>/UDP Receive'
  // End of Terminate for SubSystem: '<S76>/Obtain BLACK States'

  // Terminate for IfAction SubSystem: '<S76>/Obtain RED States'
  // Terminate for MATLABSystem: '<S80>/Stream PhaseSpace to Platform'
  matlabCodegenHa_gffipocpdxml04g(&CLVF_DW.obj_k);

  // Terminate for S-Function (sdspToNetwork): '<S80>/Send BLACK States to  BLACK Platform' 
  sErr = GetErrorBuffer(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U]);
  LibTerminate(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U]);
  if (*sErr != 0) {
    rtmSetErrorStatus(CLVF_M, sErr);
    rtmSetStopRequested(CLVF_M, 1);
  }

  LibDestroy(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U], 1);
  DestroyUDPInterface(&CLVF_DW.SendBLACKStatestoBLACKPlatform_[0U]);

  // End of Terminate for S-Function (sdspToNetwork): '<S80>/Send BLACK States to  BLACK Platform' 
  // End of Terminate for SubSystem: '<S76>/Obtain RED States'
  // End of Terminate for SubSystem: '<S75>/Using RED+BLACK, or RED+BLACK+ARM'
  // End of Terminate for SubSystem: '<S5>/Use Hardware to Obtain States'

  // Terminate for MATLABSystem: '<S6>/LSM9DS1 IMU Sensor'
  CLVF_matlabCodegenHa_a(&CLVF_DW.obj);
  CLVF_matlabCodegenHa_b(&CLVF_DW.obj.i2cobj_MAG);
  CLVF_matlabCodegenHa_f(&CLVF_DW.obj.i2cobj_A_G);

  // Terminate for S-Function (sdspToNetwork): '<S15>/UDP Send'
  sErr = GetErrorBuffer(&CLVF_DW.UDPSend_NetworkLib[0U]);
  LibTerminate(&CLVF_DW.UDPSend_NetworkLib[0U]);
  if (*sErr != 0) {
    rtmSetErrorStatus(CLVF_M, sErr);
    rtmSetStopRequested(CLVF_M, 1);
  }

  LibDestroy(&CLVF_DW.UDPSend_NetworkLib[0U], 1);
  DestroyUDPInterface(&CLVF_DW.UDPSend_NetworkLib[0U]);

  // End of Terminate for S-Function (sdspToNetwork): '<S15>/UDP Send'

  // Terminate for Atomic SubSystem: '<S3>/Send Commands to PWM Blocks'
  // Terminate for MATLABSystem: '<S26>/RED T1 - BLACK T8'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_m);

  // Terminate for MATLABSystem: '<S26>/RED T2 - BLACK T3'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_ma);

  // Terminate for MATLABSystem: '<S26>/RED T3'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_h);

  // Terminate for MATLABSystem: '<S26>/RED T4 - BLACK T5'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_kw);

  // Terminate for MATLABSystem: '<S26>/RED T5 - BLACK T4'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_hw);

  // Terminate for MATLABSystem: '<S26>/RED T6 - BLACK T7'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_hf);

  // Terminate for MATLABSystem: '<S26>/RED T7 - BLACK T6'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_c);

  // Terminate for MATLABSystem: '<S26>/RED T8 - BLACK T1'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_fo);

  // Terminate for MATLABSystem: '<S26>/BLACK T2'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_ci);

  // End of Terminate for SubSystem: '<S3>/Send Commands to PWM Blocks'

  // Terminate for IfAction SubSystem: '<S4>/Change RED Behavior'
  // Terminate for Atomic SubSystem: '<S59>/Obtain RW Status'
  // Terminate for MATLABSystem: '<S70>/Digital Read'
  matlabCodegenHandle_gffipocpdxm(&CLVF_DW.obj_f);

  // End of Terminate for SubSystem: '<S59>/Obtain RW Status'

  // Terminate for Atomic SubSystem: '<S59>/Send Direction to Motor Controller'
  // Terminate for MATLABSystem: '<S71>/Digital Write'
  matlabCodegenHandl_gffipocpdxml(&CLVF_DW.obj_b);

  // End of Terminate for SubSystem: '<S59>/Send Direction to Motor Controller'

  // Terminate for Atomic SubSystem: '<S59>/Send PWM to Motor Controller'
  // Terminate for MATLABSystem: '<S68>/PWM1'
  matlabCodegenHandle_matlabCo_gf(&CLVF_DW.obj_d);

  // End of Terminate for SubSystem: '<S59>/Send PWM to Motor Controller'

  // Terminate for Atomic SubSystem: '<S59>/Turn on Motor'
  // Terminate for MATLABSystem: '<S72>/Digital Write'
  matlabCodegenHan_gffipocpdxml04(&CLVF_DW.obj_cp);

  // End of Terminate for SubSystem: '<S59>/Turn on Motor'
  // End of Terminate for SubSystem: '<S4>/Change RED Behavior'
}
