//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: CLVF_data.cpp
//
// Code generated for Simulink model 'CLVF'.
//
// Model version                  : 1.279
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Wed Apr  6 21:29:06 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objective: Execution efficiency
// Validation result: Not run
//
#include "CLVF.h"
#include "CLVF_private.h"

// Block parameters (default storage)
P_CLVF_T CLVF_P = {
  // Variable: BLACKMass
  //  Referenced by: '<S352>/Constant'

  12.3341,

  // Variable: F_thrusters_BLACK
  //  Referenced by:
  //    '<S27>/MATLAB Function'
  //    '<S27>/MATLAB Function1'

  { 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25 },

  // Variable: F_thrusters_BLUE
  //  Referenced by:
  //    '<S36>/MATLAB Function'
  //    '<S36>/MATLAB Function1'

  { 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25 },

  // Variable: F_thrusters_RED
  //  Referenced by:
  //    '<S45>/MATLAB Function'
  //    '<S45>/MATLAB Function1'

  { 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25 },

  // Variable: Kd_tb
  //  Referenced by:
  //    '<S278>/kd_tb'
  //    '<S441>/kd_tb'
  //    '<S484>/kd_tb'
  //    '<S326>/kd_tb'
  //    '<S339>/kd_tb'

  0.10631149539747861,

  // Variable: Kd_tblue
  //  Referenced by:
  //    '<S291>/kd_tb'
  //    '<S454>/kd_tb'
  //    '<S497>/kd_tb'
  //    '<S379>/kd_tb'
  //    '<S392>/kd_tb'

  0.4,

  // Variable: Kd_tr
  //  Referenced by:
  //    '<S304>/kd_tr'
  //    '<S467>/kd_tr'
  //    '<S510>/kd_tr'
  //    '<S410>/kd_tr'
  //    '<S423>/kd_tr'

  0.15319034097434547,

  // Variable: Kd_xb
  //  Referenced by:
  //    '<S279>/kd_xb'
  //    '<S442>/kd_xb'
  //    '<S485>/kd_xb'
  //    '<S327>/kd_xb'

  5.0,

  // Variable: Kd_xblue
  //  Referenced by:
  //    '<S292>/kd_xb'
  //    '<S455>/kd_xb'
  //    '<S498>/kd_xb'
  //    '<S380>/kd_xb'
  //    '<S393>/kd_xb'

  5.0,

  // Variable: Kd_xr
  //  Referenced by:
  //    '<S305>/kd_xr'
  //    '<S468>/kd_xr'
  //    '<S511>/kd_xr'
  //    '<S411>/kd_xr'
  //    '<S424>/kd_xr'

  5.0,

  // Variable: Kd_yb
  //  Referenced by:
  //    '<S280>/kd_yb'
  //    '<S443>/kd_yb'
  //    '<S486>/kd_yb'
  //    '<S328>/kd_yb'

  5.0,

  // Variable: Kd_yblue
  //  Referenced by:
  //    '<S293>/kd_yb'
  //    '<S456>/kd_yb'
  //    '<S499>/kd_yb'
  //    '<S381>/kd_yb'
  //    '<S394>/kd_yb'

  5.0,

  // Variable: Kd_yr
  //  Referenced by:
  //    '<S306>/kd_yr'
  //    '<S469>/kd_yr'
  //    '<S512>/kd_yr'
  //    '<S412>/kd_yr'
  //    '<S425>/kd_yr'

  5.0,

  // Variable: Kp_tb
  //  Referenced by:
  //    '<S278>/kp_tb'
  //    '<S441>/kp_tb'
  //    '<S484>/kp_tb'
  //    '<S326>/kp_tb'
  //    '<S339>/kp_tb'

  0.018554856274047991,

  // Variable: Kp_tblue
  //  Referenced by:
  //    '<S291>/kp_tb'
  //    '<S454>/kp_tb'
  //    '<S497>/kp_tb'
  //    '<S379>/kp_tb'
  //    '<S392>/kp_tb'

  0.1,

  // Variable: Kp_tr
  //  Referenced by:
  //    '<S304>/kp_tr'
  //    '<S467>/kp_tr'
  //    '<S510>/kp_tr'
  //    '<S410>/kp_tr'
  //    '<S423>/kp_tr'

  0.026736758322551065,

  // Variable: Kp_xb
  //  Referenced by:
  //    '<S279>/kp_xb'
  //    '<S442>/kp_xb'
  //    '<S485>/kp_xb'
  //    '<S327>/kp_xb'

  2.0,

  // Variable: Kp_xblue
  //  Referenced by:
  //    '<S292>/kp_xb'
  //    '<S455>/kp_xb'
  //    '<S498>/kp_xb'
  //    '<S380>/kp_xb'
  //    '<S393>/kp_xb'

  2.0,

  // Variable: Kp_xr
  //  Referenced by:
  //    '<S305>/kp_xr'
  //    '<S468>/kp_xr'
  //    '<S511>/kp_xr'
  //    '<S411>/kp_xr'
  //    '<S424>/kp_xr'

  2.0,

  // Variable: Kp_yb
  //  Referenced by:
  //    '<S280>/kp_yb'
  //    '<S443>/kp_yb'
  //    '<S486>/kp_yb'
  //    '<S328>/kp_yb'

  2.0,

  // Variable: Kp_yblue
  //  Referenced by:
  //    '<S293>/kp_yb'
  //    '<S456>/kp_yb'
  //    '<S499>/kp_yb'
  //    '<S381>/kp_yb'
  //    '<S394>/kp_yb'

  2.0,

  // Variable: Kp_yr
  //  Referenced by:
  //    '<S306>/kp_yr'
  //    '<S469>/kp_yr'
  //    '<S512>/kp_yr'
  //    '<S412>/kp_yr'
  //    '<S425>/kp_yr'

  2.0,

  // Variable: Phase0_End
  //  Referenced by: '<Root>/Constant4'

  10.0,

  // Variable: Phase1_End
  //  Referenced by: '<Root>/Constant'

  15.0,

  // Variable: Phase2_End
  //  Referenced by: '<Root>/Constant1'

  35.0,

  // Variable: Phase3_End
  //  Referenced by: '<Root>/Constant2'

  351.0,

  // Variable: Phase3_SubPhase1_End
  //  Referenced by:
  //    '<S317>/Constant4'
  //    '<S318>/Constant4'
  //    '<S319>/Constant4'

  40.0,

  // Variable: Phase3_SubPhase2_End
  //  Referenced by:
  //    '<S317>/Constant1'
  //    '<S318>/Constant1'
  //    '<S319>/Constant1'

  45.0,

  // Variable: Phase3_SubPhase3_End_BLACK
  //  Referenced by:
  //    '<S317>/Constant2'
  //    '<S318>/Constant2'

  51.0,

  // Variable: Phase3_SubPhase3_End_RED
  //  Referenced by:
  //    '<S319>/Constant2'
  //    '<S427>/Constant1'

  46.0,

  // Variable: Phase3_SubPhase4_End
  //  Referenced by:
  //    '<S317>/Constant3'
  //    '<S318>/Constant3'
  //    '<S319>/Constant3'

  351.0,

  // Variable: Phase4_End
  //  Referenced by: '<Root>/Constant3'

  371.0,

  // Variable: Phase5_End
  //  Referenced by: '<Root>/Constant6'

  376.0,

  // Variable: WhoAmI
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

  2.0,

  // Variable: a
  //  Referenced by: '<S351>/Constant1'

  0.75905467523756143,

  // Variable: aTimesOVec
  //  Referenced by: '<S354>/Constant'

  { 0.16, 0.742, 0.0 },

  // Variable: a_prime
  //  Referenced by: '<S353>/Constant4'

  0.2,

  // Variable: acceptableRadius
  //  Referenced by: '<S354>/Constant1'

  0.05,

  // Variable: b
  //  Referenced by: '<S351>/Constant2'

  0.64957986332963258,

  // Variable: cntThreshold
  //  Referenced by: '<S354>/Constant2'

  100.0,

  // Variable: d
  //  Referenced by:
  //    '<S342>/MATLAB Function'
  //    '<S353>/Constant1'

  { 0.16, 0.542, 0.0 },

  // Variable: den_d
  //  Referenced by:
  //    '<S339>/1Hz LP Filter'
  //    '<S342>/1Hz LP Filter'
  //    '<S342>/1Hz LP Filter1'
  //    '<S423>/1Hz LP Filter1'

  { 1.0, -0.28094573786148697, 0.18556053673605119 },

  // Variable: drop_states_BLACK
  //  Referenced by: '<S527>/Velocity to Position'

  { 3.1603950000000003, 0.70967500000000006, 1.5707963267948966 },

  // Variable: drop_states_BLUE
  //  Referenced by: '<S528>/Velocity to Position'

  { 2.655775, 1.709675, 0.0 },

  // Variable: drop_states_RED
  //  Referenced by: '<S532>/Velocity to Position'

  { 1.209675, 1.209675, 5.026548245743669 },

  // Variable: finalAngle
  //  Referenced by: '<S353>/Constant5'

  2.9845130209103035,

  // Variable: home_states_BLACK
  //  Referenced by:
  //    '<S437>/Desired Attitude (BLACK)'
  //    '<S437>/Desired Px (BLACK)'
  //    '<S437>/Desired Py (BLACK)'
  //    '<S480>/Constant'
  //    '<S480>/Constant2'
  //    '<S480>/Constant3'

  { 1.355775, 1.209675, 0.0 },

  // Variable: home_states_BLUE
  //  Referenced by:
  //    '<S438>/Desired Attitude (BLUE)'
  //    '<S438>/Desired Px (BLUE)'
  //    '<S438>/Desired Py (BLUE)'
  //    '<S481>/Desired Attitude (BLUE)'
  //    '<S481>/Desired Px (BLUE)'
  //    '<S481>/Desired Py (BLUE)'

  { 0.85577500000000006, 1.709675, 0.0 },

  // Variable: home_states_RED
  //  Referenced by:
  //    '<S439>/Constant'
  //    '<S439>/Constant1'
  //    '<S439>/Constant3'
  //    '<S482>/Constant'
  //    '<S482>/Constant2'
  //    '<S482>/Constant3'

  { 2.155775, 1.209675, 0.0 },

  // Variable: init_states_BLACK
  //  Referenced by:
  //    '<S274>/Desired Attitude (BLACK)'
  //    '<S274>/Desired Px (BLACK)'
  //    '<S274>/Desired Py (BLACK)'
  //    '<S321>/Desired Attitude (BLACK)'
  //    '<S321>/Desired X-Position (BLACK)'
  //    '<S321>/Desired Y-Position (BLACK)'

  { 3.1603950000000003, 0.70967500000000006, 1.5707963267948966 },

  // Variable: init_states_BLUE
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

  { 2.655775, 1.709675, 0.0 },

  // Variable: init_states_RED
  //  Referenced by:
  //    '<S276>/Constant'
  //    '<S276>/Constant1'
  //    '<S276>/Constant3'
  //    '<S405>/Desired Attitude (RED)'
  //    '<S405>/Desired X-Position (RED)'
  //    '<S405>/Desired Y-Position (RED)'
  //    '<S408>/Desired X-Position (RED)'
  //    '<S408>/Desired Y-Position (RED)'

  { 1.209675, 1.209675, 5.026548245743669 },

  // Variable: ka
  //  Referenced by: '<S351>/Constant3'

  0.021264737843828367,

  // Variable: kc
  //  Referenced by: '<S351>/Constant4'

  0.0784425037227523,

  // Variable: kd
  //  Referenced by: '<S352>/Constant1'

  15.0,

  // Variable: model_param
  //  Referenced by:
  //    '<S527>/MATLAB Function'
  //    '<S528>/MATLAB Function'
  //    '<S532>/MATLAB Function'

  { 16.9478, 0.2709, 12.3341, 0.188, 12.7621, 0.193 },

  // Variable: noise_variance_BLACK
  //  Referenced by: '<S527>/Random Number'

  1.0E-5,

  // Variable: noise_variance_BLUE
  //  Referenced by: '<S528>/Random Number'

  0.0,

  // Variable: noise_variance_RED
  //  Referenced by: '<S532>/Random Number'

  1.0E-5,

  // Variable: num_d
  //  Referenced by:
  //    '<S339>/1Hz LP Filter'
  //    '<S342>/1Hz LP Filter'
  //    '<S342>/1Hz LP Filter1'
  //    '<S423>/1Hz LP Filter1'

  { 0.22615369971864102, 0.45230739943728204, 0.22615369971864102 },

  // Variable: o_hat_B
  //  Referenced by:
  //    '<S342>/Norm'
  //    '<S351>/Constant'

  { 0.21078850472783767, 0.97753169067534718, 0.0 },

  // Variable: o_hat_prime
  //  Referenced by:
  //    '<S342>/Norm1'
  //    '<S353>/Constant2'

  { 0.0, 1.0, 0.0 },

  // Variable: platformSelection
  //  Referenced by:
  //    '<S75>/Which PLATFORM is being used?'
  //    '<S80>/Stream PhaseSpace to Platform'

  3.0,

  // Variable: rT_I0
  //  Referenced by: '<S408>/Constant'

  { 1.209675, 1.209675, 5.026548245743669 },

  // Variable: serverRate
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

  0.1,

  // Variable: simMode
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

  0.0,

  // Variable: theta_d
  //  Referenced by: '<S353>/Constant3'

  0.52359877559829882,

  // Variable: thruster_dist2CG_BLACK
  //  Referenced by:
  //    '<S27>/MATLAB Function'
  //    '<S27>/MATLAB Function1'

  { 83.42, -52.58, 55.94, -60.05, 54.08, -53.92, 77.06, -55.94 },

  // Variable: thruster_dist2CG_BLUE
  //  Referenced by:
  //    '<S36>/MATLAB Function'
  //    '<S36>/MATLAB Function1'

  { 83.42, -52.58, 55.94, -60.05, 54.08, -53.92, 77.06, -55.94 },

  // Variable: thruster_dist2CG_RED
  //  Referenced by:
  //    '<S45>/MATLAB Function'
  //    '<S45>/MATLAB Function1'

  { 49.92, -78.08, 70.46, -63.54, 81.08, -50.42, 57.44, -75.96 },

  // Variable: v_max
  //  Referenced by: '<S353>/Constant'

  0.00856260913530072,

  // Variable: w_body
  //  Referenced by: '<S408>/Gain'

  0.017453292519943295,

  // Computed Parameter: BLACKPWM_Y0
  //  Referenced by: '<S22>/BLACK PWM'

  0.0,

  // Expression: 1000
  //  Referenced by: '<S27>/Remove Negatives'

  1000.0,

  // Expression: 0
  //  Referenced by: '<S27>/Remove Negatives'

  0.0,

  // Computed Parameter: BLUEPWM_Y0
  //  Referenced by: '<S23>/BLUE PWM'

  0.0,

  // Expression: 1000
  //  Referenced by: '<S36>/Remove Negatives'

  1000.0,

  // Expression: 0
  //  Referenced by: '<S36>/Remove Negatives'

  0.0,

  // Computed Parameter: REDPWM_Y0
  //  Referenced by: '<S24>/RED PWM'

  0.0,

  // Expression: 1000
  //  Referenced by: '<S45>/Remove Negatives'

  1000.0,

  // Expression: 0
  //  Referenced by: '<S45>/Remove Negatives'

  0.0,

  // Expression: sampleTime
  //  Referenced by: '<S70>/Digital Read'

  0.1,

  // Expression: 0.1
  //  Referenced by: '<S58>/Saturate Torque'

  0.1,

  // Expression: -0.1
  //  Referenced by: '<S58>/Saturate Torque'

  -0.1,

  // Expression: 0.0
  //  Referenced by: '<S58>/Delay'

  0.0,

  // Expression: 3375/64
  //  Referenced by: '<S62>/Gearbox  Ratio'

  52.734375,

  // Expression: 7000
  //  Referenced by: '<S59>/Saturate Motor'

  7000.0,

  // Expression: -7000
  //  Referenced by: '<S59>/Saturate Motor'

  -7000.0,

  // Expression: (7000*64/3375)
  //  Referenced by: '<S58>/Saturate RPM'

  132.74074074074073,

  // Expression: -(7000*64/3375)
  //  Referenced by: '<S58>/Saturate RPM'

  -132.74074074074073,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S154>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_b
  //  Referenced by: '<S139>/Out1'

  0.0,

  // Computed Parameter: ActualTime_Y0
  //  Referenced by: '<S140>/Actual Time'

  0.0,

  // Computed Parameter: Out1_Y0_o
  //  Referenced by: '<S157>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_k
  //  Referenced by: '<S158>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_m
  //  Referenced by: '<S159>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_j
  //  Referenced by: '<S160>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_f
  //  Referenced by: '<S161>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_p
  //  Referenced by: '<S162>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_i
  //  Referenced by: '<S163>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_l
  //  Referenced by: '<S164>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_mc
  //  Referenced by: '<S165>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ie
  //  Referenced by: '<S166>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_a
  //  Referenced by: '<S167>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ief
  //  Referenced by: '<S168>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S137>/Constant'

  0.0,

  // Computed Parameter: Out1_Y0_k4
  //  Referenced by: '<S93>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_c
  //  Referenced by: '<S94>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_l0
  //  Referenced by: '<S95>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_e
  //  Referenced by: '<S96>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_d
  //  Referenced by: '<S97>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_g
  //  Referenced by: '<S98>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ft
  //  Referenced by: '<S99>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_j1
  //  Referenced by: '<S100>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ii
  //  Referenced by: '<S101>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_i1
  //  Referenced by: '<S102>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_pa
  //  Referenced by: '<S103>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_h
  //  Referenced by: '<S104>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ee
  //  Referenced by: '<S122>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ob
  //  Referenced by: '<S107>/Out1'

  0.0,

  // Computed Parameter: ActualTime_Y0_f
  //  Referenced by: '<S108>/Actual Time'

  0.0,

  // Computed Parameter: Out1_Y0_oy
  //  Referenced by: '<S125>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_l2
  //  Referenced by: '<S126>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ow
  //  Referenced by: '<S127>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_mx
  //  Referenced by: '<S128>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_gm
  //  Referenced by: '<S129>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_h4
  //  Referenced by: '<S130>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_kh
  //  Referenced by: '<S131>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_fm
  //  Referenced by: '<S132>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_er
  //  Referenced by: '<S133>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_il
  //  Referenced by: '<S134>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_mr
  //  Referenced by: '<S135>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_jf
  //  Referenced by: '<S136>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S105>/Constant'

  0.0,

  // Computed Parameter: Out1_Y0_n
  //  Referenced by: '<S183>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_kc
  //  Referenced by: '<S186>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_n3
  //  Referenced by: '<S190>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_hs
  //  Referenced by: '<S196>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_gz
  //  Referenced by: '<S199>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_cg
  //  Referenced by: '<S202>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S189>/Constant'

  0.0,

  // Expression: pi/180
  //  Referenced by: '<S169>/Gain'

  0.017453292519943295,

  // Expression: 9.81
  //  Referenced by: '<S169>/Gain1'

  9.81,

  // Computed Parameter: DiscreteTimeIntegrator1_gainval
  //  Referenced by: '<S181>/Discrete-Time Integrator1'

  0.05,

  // Computed Parameter: Out1_Y0_o0
  //  Referenced by: '<S214>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_jb
  //  Referenced by: '<S217>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_dr
  //  Referenced by: '<S221>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_kcv
  //  Referenced by: '<S227>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ie0
  //  Referenced by: '<S230>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_gi
  //  Referenced by: '<S233>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S220>/Constant'

  0.0,

  // Expression: pi/180
  //  Referenced by: '<S170>/Gain'

  0.017453292519943295,

  // Expression: 9.81
  //  Referenced by: '<S170>/Gain1'

  9.81,

  // Computed Parameter: DiscreteTimeIntegrator1_gainv_p
  //  Referenced by: '<S212>/Discrete-Time Integrator1'

  0.05,

  // Computed Parameter: Out1_Y0_az
  //  Referenced by: '<S245>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_cc
  //  Referenced by: '<S248>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_m0
  //  Referenced by: '<S252>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_cz
  //  Referenced by: '<S258>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ai
  //  Referenced by: '<S261>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ef
  //  Referenced by: '<S264>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S251>/Constant'

  0.0,

  // Expression: pi/180
  //  Referenced by: '<S171>/Gain'

  0.017453292519943295,

  // Expression: 9.81
  //  Referenced by: '<S171>/Gain1'

  9.81,

  // Computed Parameter: DiscreteTimeIntegrator1_gainv_g
  //  Referenced by: '<S243>/Discrete-Time Integrator1'

  0.05,

  // Expression: initCond
  //  Referenced by: '<S555>/ '

  { 3.1603950000000003, 0.70967500000000006, 1.5707963267948966 },

  // Expression: initCond
  //  Referenced by: '<S557>/ '

  { 2.655775, 1.709675, 0.0 },

  // Expression: initCond
  //  Referenced by: '<S559>/ '

  { 1.209675, 1.209675, 5.026548245743669 },

  // Computed Parameter: Out1_Y0_eq
  //  Referenced by: '<S560>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ir
  //  Referenced by: '<S561>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ny
  //  Referenced by: '<S562>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ko
  //  Referenced by: '<S563>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ar
  //  Referenced by: '<S564>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_lp
  //  Referenced by: '<S565>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_l2n
  //  Referenced by: '<S566>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ep
  //  Referenced by: '<S567>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ln
  //  Referenced by: '<S568>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_lj
  //  Referenced by: '<S569>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_h2
  //  Referenced by: '<S570>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_i3
  //  Referenced by: '<S571>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_mh
  //  Referenced by: '<S572>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_cb
  //  Referenced by: '<S573>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_d1
  //  Referenced by: '<S574>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S529>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S530>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S531>/Constant'

  0.0,

  // Computed Parameter: AccelerationtoVelocity_gainval
  //  Referenced by: '<S527>/Acceleration  to Velocity'

  0.05,

  // Expression: 0
  //  Referenced by: '<S527>/Acceleration  to Velocity'

  0.0,

  // Computed Parameter: VelocitytoPosition_gainval
  //  Referenced by: '<S527>/Velocity to Position'

  0.05,

  // Expression: 0
  //  Referenced by: '<S527>/Random Number'

  0.0,

  // Expression: 0
  //  Referenced by: '<S527>/Random Number'

  0.0,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number1'

  0.0,

  // Computed Parameter: RandomNumber1_StdDev
  //  Referenced by: '<S16>/Random Number1'

  0.0012247448713915891,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number'

  0.0,

  // Computed Parameter: RandomNumber_StdDev
  //  Referenced by: '<S16>/Random Number'

  0.0012247448713915891,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number'

  0.0,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number2'

  0.0,

  // Computed Parameter: RandomNumber2_StdDev
  //  Referenced by: '<S16>/Random Number2'

  0.0012247448713915891,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number2'

  0.0,

  // Computed Parameter: AccelerationtoVelocity_gainva_a
  //  Referenced by: '<S528>/Acceleration  to Velocity'

  0.05,

  // Expression: 0
  //  Referenced by: '<S528>/Acceleration  to Velocity'

  0.0,

  // Computed Parameter: VelocitytoPosition_gainval_a
  //  Referenced by: '<S528>/Velocity to Position'

  0.05,

  // Expression: 0
  //  Referenced by: '<S528>/Random Number'

  0.0,

  // Expression: 0
  //  Referenced by: '<S528>/Random Number'

  0.0,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number4'

  0.0,

  // Computed Parameter: RandomNumber4_StdDev
  //  Referenced by: '<S16>/Random Number4'

  0.0012247448713915891,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number3'

  0.0,

  // Computed Parameter: RandomNumber3_StdDev
  //  Referenced by: '<S16>/Random Number3'

  0.0012247448713915891,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number3'

  0.0,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number5'

  0.0,

  // Computed Parameter: RandomNumber5_StdDev
  //  Referenced by: '<S16>/Random Number5'

  0.0012247448713915891,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number5'

  0.0,

  // Computed Parameter: DiscreteTimeIntegrator_gainval
  //  Referenced by: '<S16>/Discrete-Time Integrator'

  0.05,

  // Computed Parameter: DiscreteTimeIntegrator1_gainv_f
  //  Referenced by: '<S16>/Discrete-Time Integrator1'

  0.05,

  // Computed Parameter: DiscreteTimeIntegrator2_gainval
  //  Referenced by: '<S16>/Discrete-Time Integrator2'

  0.05,

  // Computed Parameter: AccelerationtoVelocity_gainva_i
  //  Referenced by: '<S532>/Acceleration  to Velocity'

  0.05,

  // Expression: 0
  //  Referenced by: '<S532>/Acceleration  to Velocity'

  0.0,

  // Computed Parameter: VelocitytoPosition_gainval_ad
  //  Referenced by: '<S532>/Velocity to Position'

  0.05,

  // Expression: 0
  //  Referenced by: '<S532>/Random Number'

  0.0,

  // Expression: 0
  //  Referenced by: '<S532>/Random Number'

  0.0,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number7'

  0.0,

  // Computed Parameter: RandomNumber7_StdDev
  //  Referenced by: '<S16>/Random Number7'

  0.0012247448713915891,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number7'

  0.0,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number6'

  0.0,

  // Computed Parameter: RandomNumber6_StdDev
  //  Referenced by: '<S16>/Random Number6'

  0.0012247448713915891,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number6'

  0.0,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number8'

  0.0,

  // Computed Parameter: RandomNumber8_StdDev
  //  Referenced by: '<S16>/Random Number8'

  0.0012247448713915891,

  // Expression: 0
  //  Referenced by: '<S16>/Random Number8'

  0.0,

  // Computed Parameter: Out1_Y0_mp
  //  Referenced by: '<S283>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_av
  //  Referenced by: '<S285>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_g1
  //  Referenced by: '<S287>/Out1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S274>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_ov
  //  Referenced by: '<S296>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_a4
  //  Referenced by: '<S298>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_nv
  //  Referenced by: '<S300>/Out1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S275>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_fr
  //  Referenced by: '<S309>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_bk
  //  Referenced by: '<S311>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_j2
  //  Referenced by: '<S313>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S276>/Constant2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S276>/Constant4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S276>/Constant5'

  0.0,

  // Expression: 0
  //  Referenced by: '<S276>/Constant6'

  0.0,

  // Expression: 1
  //  Referenced by: '<S276>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_gc
  //  Referenced by: '<S331>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_hu
  //  Referenced by: '<S333>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_efk
  //  Referenced by: '<S335>/Out1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S321>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_bx
  //  Referenced by: '<S345>/Out1'

  0.0,

  // Expression: initCond
  //  Referenced by: '<S358>/ '

  0.0,

  // Computed Parameter: Out1_Y0_fl
  //  Referenced by: '<S367>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_j3
  //  Referenced by: '<S368>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_px
  //  Referenced by: '<S369>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ni
  //  Referenced by: '<S370>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S342>/Constant2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S342>/Constant3'

  0.0,

  // Expression: 0
  //  Referenced by: '<S354>/Delay'

  0.0,

  // Expression: 0
  //  Referenced by: '<S339>/1Hz LP Filter'

  0.0,

  // Expression: 0
  //  Referenced by: '<S342>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S342>/1Hz LP Filter1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S342>/1Hz LP Filter'

  0.0,

  // Expression: [0;0;0]
  //  Referenced by: '<S349>/Constant'

  { 0.0, 0.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<S350>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S350>/Constant'

  0.0,

  // Expression: [0;0;0]
  //  Referenced by: '<S341>/Constant'

  { 0.0, 0.0, 0.0 },

  // Expression: 1
  //  Referenced by: '<S324>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_na
  //  Referenced by: '<S415>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_mn
  //  Referenced by: '<S417>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_lx
  //  Referenced by: '<S419>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S405>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S405>/Constant4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S405>/Constant5'

  0.0,

  // Expression: 0
  //  Referenced by: '<S405>/Constant6'

  0.0,

  // Expression: 1
  //  Referenced by: '<S405>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_bj
  //  Referenced by: '<S429>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ki
  //  Referenced by: '<S431>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_ky
  //  Referenced by: '<S433>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S423>/1Hz LP Filter1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S408>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S408>/Constant4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S408>/Constant5'

  0.0,

  // Expression: 0
  //  Referenced by: '<S408>/Constant6'

  0.0,

  // Expression: 1
  //  Referenced by: '<S408>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_jr
  //  Referenced by: '<S446>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_mrw
  //  Referenced by: '<S448>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_i0
  //  Referenced by: '<S450>/Out1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S437>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_nf
  //  Referenced by: '<S472>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_gb
  //  Referenced by: '<S474>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_da
  //  Referenced by: '<S476>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S439>/Constant2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S439>/Constant4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S439>/Constant5'

  0.0,

  // Expression: 0
  //  Referenced by: '<S439>/Constant6'

  0.0,

  // Expression: 1
  //  Referenced by: '<S439>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_gx
  //  Referenced by: '<S489>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_fj
  //  Referenced by: '<S491>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_bm
  //  Referenced by: '<S493>/Out1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S480>/Puck State'

  1.0,

  // Computed Parameter: Out1_Y0_i3j
  //  Referenced by: '<S515>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_bs
  //  Referenced by: '<S517>/Out1'

  0.0,

  // Computed Parameter: Out1_Y0_c5
  //  Referenced by: '<S519>/Out1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S482>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S482>/Constant4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S482>/Constant5'

  0.0,

  // Expression: 0
  //  Referenced by: '<S482>/Constant6'

  0.0,

  // Expression: 1
  //  Referenced by: '<S482>/Puck State'

  1.0,

  // Expression: 0
  //  Referenced by: '<S523>/Constant3'

  0.0,

  // Expression: 0
  //  Referenced by: '<S523>/Constant4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S523>/Constant5'

  0.0,

  // Expression: 0
  //  Referenced by: '<S523>/Puck State'

  0.0,

  // Expression: 0
  //  Referenced by: '<S525>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S525>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S525>/Constant2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S525>/Constant4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S525>/Constant5'

  0.0,

  // Expression: 0
  //  Referenced by: '<S525>/Constant6'

  0.0,

  // Expression: 0
  //  Referenced by: '<S525>/Puck State'

  0.0,

  // Expression: -1
  //  Referenced by: '<S525>/Gain'

  -1.0,

  // Expression: 0.005
  //  Referenced by: '<S525>/Saturation'

  0.005,

  // Expression: -0.005
  //  Referenced by: '<S525>/Saturation'

  -0.005,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px33'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px36'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px39'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz10'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz11'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz4'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz5'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz6'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz9'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fx'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fx1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fx_Sat'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fx_Sat1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fx_Sat2'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fy'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fy1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fy_Sat'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fy_Sat1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Fy_Sat2'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Px'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Px1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Py'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Py1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Rz'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Rz1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Tz'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Tz1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Tz_Sat'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Tz_Sat1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/BLACK_Tz_Sat2'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Data Store Memory'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Data Store Memory1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Fx'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Fx_Sat'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Fy'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Fy_Sat'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px10'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px11'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px12'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px13'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px14'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px15'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px16'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px17'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px18'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px19'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px2'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px20'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px21'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px22'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px23'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px24'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px25'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px26'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px27'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px28'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px29'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px3'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px30'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px31'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px32'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px34'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px35'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px37'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px38'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px4'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px5'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px6'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px7'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px8'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Px9'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Py'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Rz'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz2'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz3'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz7'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz8'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz_RW'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz_RW Sat'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz_RW Sat1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz_RW1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz_Sat'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_Tz_Sat1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/RED_dRz_RW Sat'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time1'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time10'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time11'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time12'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time13'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time14'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time15'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time2'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time3'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time4'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time5'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time6'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time7'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time8'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Universal_Time9'

  0.0,

  // Computed Parameter: UDPReceive_Port
  //  Referenced by: '<S79>/UDP Receive'

  25000,

  // Computed Parameter: SendBLACKStatestoBLACKPlatform_
  //  Referenced by: '<S80>/Send BLACK States to  BLACK Platform'

  25000,

  // Computed Parameter: UDPSend_Port
  //  Referenced by: '<S15>/UDP Send'

  26000,

  // Computed Parameter: Delay1_InitialCondition
  //  Referenced by: '<S63>/Delay1'

  0,

  // Computed Parameter: Delay2_InitialCondition
  //  Referenced by: '<S63>/Delay2'

  0,

  // Computed Parameter: Delay5_InitialCondition
  //  Referenced by: '<S63>/Delay5'

  0,

  // Computed Parameter: Delay3_InitialCondition
  //  Referenced by: '<S63>/Delay3'

  0,

  // Computed Parameter: Delay4_InitialCondition
  //  Referenced by: '<S63>/Delay4'

  0,

  // Computed Parameter: Delay_InitialCondition_m
  //  Referenced by: '<S153>/Delay'

  1,

  // Computed Parameter: Delay_InitialCondition_g
  //  Referenced by: '<S121>/Delay'

  1,

  // Start of '<S14>/Change BLUE Behavior'
  {
    // Expression: 0
    //  Referenced by: '<S524>/Constant3'

    0.0,

    // Expression: 0
    //  Referenced by: '<S524>/Constant4'

    0.0,

    // Expression: 0
    //  Referenced by: '<S524>/Constant5'

    0.0,

    // Expression: 0
    //  Referenced by: '<S524>/Puck State'

    0.0
  }
  ,

  // End of '<S14>/Change BLUE Behavior'

  // Start of '<S13>/Change BLUE Behavior'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S502>/Out1'

    0.0,

    // Computed Parameter: Out1_Y0_k
    //  Referenced by: '<S504>/Out1'

    0.0,

    // Computed Parameter: Out1_Y0_n
    //  Referenced by: '<S506>/Out1'

    0.0,

    // Expression: 1
    //  Referenced by: '<S481>/Puck State'

    1.0
  }
  ,

  // End of '<S13>/Change BLUE Behavior'

  // Start of '<S12>/Change BLUE Behavior'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S459>/Out1'

    0.0,

    // Computed Parameter: Out1_Y0_k
    //  Referenced by: '<S461>/Out1'

    0.0,

    // Computed Parameter: Out1_Y0_n
    //  Referenced by: '<S463>/Out1'

    0.0,

    // Expression: 1
    //  Referenced by: '<S438>/Puck State'

    1.0
  }
  ,

  // End of '<S12>/Change BLUE Behavior'

  // Start of '<S319>/Sub-Phase #3 '
  {
    // Expression: 0
    //  Referenced by: '<S407>/Constant'

    0.0,

    // Expression: 0
    //  Referenced by: '<S407>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S407>/Constant4'

    0.0,

    // Expression: 0
    //  Referenced by: '<S407>/Constant5'

    0.0,

    // Expression: 0
    //  Referenced by: '<S407>/Constant6'

    0.0,

    // Expression: 1
    //  Referenced by: '<S407>/Puck State'

    1.0
  }
  ,

  // End of '<S319>/Sub-Phase #3 '

  // Start of '<S319>/Sub-Phase #2 '
  {
    // Expression: 0
    //  Referenced by: '<S406>/Constant'

    0.0,

    // Expression: 0
    //  Referenced by: '<S406>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S406>/Constant4'

    0.0,

    // Expression: 0
    //  Referenced by: '<S406>/Constant5'

    0.0,

    // Expression: 0
    //  Referenced by: '<S406>/Constant6'

    0.0,

    // Expression: 0
    //  Referenced by: '<S406>/Puck State'

    0.0
  }
  ,

  // End of '<S319>/Sub-Phase #2 '

  // Start of '<S318>/Sub-Phase #4'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S397>/Out1'

    0.0,

    // Computed Parameter: Out1_Y0_l
    //  Referenced by: '<S399>/Out1'

    0.0,

    // Computed Parameter: Out1_Y0_e
    //  Referenced by: '<S401>/Out1'

    0.0,

    // Expression: 1
    //  Referenced by: '<S377>/Puck State'

    1.0
  }
  ,

  // End of '<S318>/Sub-Phase #4'

  // Start of '<S318>/Sub-Phase #3 '
  {
    // Expression: 0
    //  Referenced by: '<S376>/Constant'

    0.0,

    // Expression: 1
    //  Referenced by: '<S376>/Puck State'

    1.0
  }
  ,

  // End of '<S318>/Sub-Phase #3 '

  // Start of '<S318>/Sub-Phase #2 '
  {
    // Expression: 0
    //  Referenced by: '<S375>/Constant'

    0.0,

    // Expression: 0
    //  Referenced by: '<S375>/Puck State'

    0.0
  }
  ,

  // End of '<S318>/Sub-Phase #2 '

  // Start of '<S318>/Sub-Phase #1'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S384>/Out1'

    0.0,

    // Computed Parameter: Out1_Y0_l
    //  Referenced by: '<S386>/Out1'

    0.0,

    // Computed Parameter: Out1_Y0_e
    //  Referenced by: '<S388>/Out1'

    0.0,

    // Expression: 1
    //  Referenced by: '<S374>/Puck State'

    1.0
  }
  ,

  // End of '<S318>/Sub-Phase #1'

  // Start of '<S317>/Sub-Phase #3 '
  {
    // Expression: 0
    //  Referenced by: '<S323>/Constant'

    0.0,

    // Expression: 1
    //  Referenced by: '<S323>/Puck State'

    1.0
  }
  ,

  // End of '<S317>/Sub-Phase #3 '

  // Start of '<S317>/Sub-Phase #2 '
  {
    // Expression: 0
    //  Referenced by: '<S322>/Constant'

    0.0,

    // Expression: 0
    //  Referenced by: '<S322>/Puck State'

    0.0
  }
  ,

  // End of '<S317>/Sub-Phase #2 '

  // Start of '<Root>/Phase #1:  Start Floating '
  {
    // Expression: 0
    //  Referenced by: '<S270>/Constant3'

    0.0,

    // Expression: 0
    //  Referenced by: '<S270>/Constant4'

    0.0,

    // Expression: 0
    //  Referenced by: '<S270>/Constant5'

    0.0,

    // Expression: 1
    //  Referenced by: '<S270>/Puck State'

    1.0,

    // Expression: 0
    //  Referenced by: '<S272>/Constant'

    0.0,

    // Expression: 0
    //  Referenced by: '<S272>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S272>/Constant2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S272>/Constant3'

    0.0,

    // Expression: 0
    //  Referenced by: '<S272>/Constant4'

    0.0,

    // Expression: 0
    //  Referenced by: '<S272>/Constant5'

    0.0,

    // Expression: 0
    //  Referenced by: '<S272>/Constant6'

    0.0,

    // Expression: 1
    //  Referenced by: '<S272>/Puck State'

    1.0,

    // Start of '<S8>/Change BLUE Behavior'
    {
      // Expression: 0
      //  Referenced by: '<S271>/Constant3'

      0.0,

      // Expression: 0
      //  Referenced by: '<S271>/Constant4'

      0.0,

      // Expression: 0
      //  Referenced by: '<S271>/Constant5'

      0.0,

      // Expression: 1
      //  Referenced by: '<S271>/Puck State'

      1.0
    }
    // End of '<S8>/Change BLUE Behavior'
  }
  ,

  // End of '<Root>/Phase #1:  Start Floating '

  // Start of '<Root>/Phase #0:  Wait for Synchronization'
  {
    // Expression: 0
    //  Referenced by: '<S266>/Constant3'

    0.0,

    // Expression: 0
    //  Referenced by: '<S266>/Constant4'

    0.0,

    // Expression: 0
    //  Referenced by: '<S266>/Constant5'

    0.0,

    // Expression: 0
    //  Referenced by: '<S266>/Puck State'

    0.0,

    // Expression: 0
    //  Referenced by: '<S268>/Constant'

    0.0,

    // Expression: 0
    //  Referenced by: '<S268>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S268>/Constant2'

    0.0,

    // Expression: 0
    //  Referenced by: '<S268>/Constant3'

    0.0,

    // Expression: 0
    //  Referenced by: '<S268>/Constant4'

    0.0,

    // Expression: 0
    //  Referenced by: '<S268>/Constant5'

    0.0,

    // Expression: 0
    //  Referenced by: '<S268>/Constant6'

    0.0,

    // Expression: 0
    //  Referenced by: '<S268>/Puck State'

    0.0,

    // Start of '<S8>/Change BLUE Behavior'
    {
      // Expression: 0
      //  Referenced by: '<S267>/Constant3'

      0.0,

      // Expression: 0
      //  Referenced by: '<S267>/Constant4'

      0.0,

      // Expression: 0
      //  Referenced by: '<S267>/Constant5'

      0.0,

      // Expression: 0
      //  Referenced by: '<S267>/Puck State'

      0.0
    }
    // End of '<S8>/Change BLUE Behavior'
  }
  ,

  // End of '<Root>/Phase #0:  Wait for Synchronization'

  // Start of '<S256>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S263>/Out1'

    0.0,

    // Expression: 9.81
    //  Referenced by: '<S263>/Constant'

    9.81,

    // Expression: 0.0
    //  Referenced by: '<S263>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S263>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S263>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S263>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S263>/Delay4'

    0.0
  }
  ,

  // End of '<S256>/Calculate Running Mean'

  // Start of '<S255>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S260>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S260>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S260>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S260>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S260>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S260>/Delay4'

    0.0
  }
  ,

  // End of '<S255>/Calculate Running Mean'

  // Start of '<S254>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S257>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S257>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S257>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S257>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S257>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S257>/Delay4'

    0.0
  }
  ,

  // End of '<S254>/Calculate Running Mean'

  // Start of '<S243>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S250>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S250>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S250>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S250>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S250>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S250>/Delay4'

    0.0
  }
  ,

  // End of '<S243>/Calculate Running Mean'

  // Start of '<S242>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S247>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S247>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S247>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S247>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S247>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S247>/Delay4'

    0.0
  }
  ,

  // End of '<S242>/Calculate Running Mean'

  // Start of '<S241>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S244>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S244>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S244>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S244>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S244>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S244>/Delay4'

    0.0
  }
  ,

  // End of '<S241>/Calculate Running Mean'

  // Start of '<S171>/AHRS2'
  {
    // Expression: 0.0001924722
    //  Referenced by: '<S171>/AHRS2'

    0.0001924722,

    // Expression: 9.1385e-5
    //  Referenced by: '<S171>/AHRS2'

    9.1385E-5,

    // Expression: 0.1
    //  Referenced by: '<S171>/AHRS2'

    0.1,

    // Expression: 3.0462e-13
    //  Referenced by: '<S171>/AHRS2'

    3.0462E-13,

    // Expression: 0.0096236100000000012
    //  Referenced by: '<S171>/AHRS2'

    0.0096236100000000012,

    // Expression: 0.5
    //  Referenced by: '<S171>/AHRS2'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S171>/AHRS2'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S171>/AHRS2'

    0.5,

    // Expression: 33
    //  Referenced by: '<S171>/AHRS2'

    33.0
  }
  ,

  // End of '<S171>/AHRS2'

  // Start of '<S225>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S232>/Out1'

    0.0,

    // Expression: 9.81
    //  Referenced by: '<S232>/Constant'

    9.81,

    // Expression: 0.0
    //  Referenced by: '<S232>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S232>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S232>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S232>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S232>/Delay4'

    0.0
  }
  ,

  // End of '<S225>/Calculate Running Mean'

  // Start of '<S224>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S229>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S229>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S229>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S229>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S229>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S229>/Delay4'

    0.0
  }
  ,

  // End of '<S224>/Calculate Running Mean'

  // Start of '<S223>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S226>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S226>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S226>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S226>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S226>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S226>/Delay4'

    0.0
  }
  ,

  // End of '<S223>/Calculate Running Mean'

  // Start of '<S212>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S219>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S219>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S219>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S219>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S219>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S219>/Delay4'

    0.0
  }
  ,

  // End of '<S212>/Calculate Running Mean'

  // Start of '<S211>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S216>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S216>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S216>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S216>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S216>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S216>/Delay4'

    0.0
  }
  ,

  // End of '<S211>/Calculate Running Mean'

  // Start of '<S210>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S213>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S213>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S213>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S213>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S213>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S213>/Delay4'

    0.0
  }
  ,

  // End of '<S210>/Calculate Running Mean'

  // Start of '<S170>/AHRS2'
  {
    // Expression: 0.0001924722
    //  Referenced by: '<S170>/AHRS2'

    0.0001924722,

    // Expression: 9.1385e-5
    //  Referenced by: '<S170>/AHRS2'

    9.1385E-5,

    // Expression: 0.1
    //  Referenced by: '<S170>/AHRS2'

    0.1,

    // Expression: 3.0462e-13
    //  Referenced by: '<S170>/AHRS2'

    3.0462E-13,

    // Expression: 0.0096236100000000012
    //  Referenced by: '<S170>/AHRS2'

    0.0096236100000000012,

    // Expression: 0.5
    //  Referenced by: '<S170>/AHRS2'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S170>/AHRS2'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S170>/AHRS2'

    0.5,

    // Expression: 33
    //  Referenced by: '<S170>/AHRS2'

    33.0
  }
  ,

  // End of '<S170>/AHRS2'

  // Start of '<S194>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S201>/Out1'

    0.0,

    // Expression: 9.81
    //  Referenced by: '<S201>/Constant'

    9.81,

    // Expression: 0.0
    //  Referenced by: '<S201>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S201>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S201>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S201>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S201>/Delay4'

    0.0
  }
  ,

  // End of '<S194>/Calculate Running Mean'

  // Start of '<S193>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S198>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S198>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S198>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S198>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S198>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S198>/Delay4'

    0.0
  }
  ,

  // End of '<S193>/Calculate Running Mean'

  // Start of '<S192>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S195>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S195>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S195>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S195>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S195>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S195>/Delay4'

    0.0
  }
  ,

  // End of '<S192>/Calculate Running Mean'

  // Start of '<S181>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S188>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S188>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S188>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S188>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S188>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S188>/Delay4'

    0.0
  }
  ,

  // End of '<S181>/Calculate Running Mean'

  // Start of '<S180>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S185>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S185>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S185>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S185>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S185>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S185>/Delay4'

    0.0
  }
  ,

  // End of '<S180>/Calculate Running Mean'

  // Start of '<S179>/Calculate Running Mean'
  {
    // Computed Parameter: Out1_Y0
    //  Referenced by: '<S182>/Out1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S182>/Delay'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S182>/Delay1'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S182>/Delay2'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S182>/Delay3'

    0.0,

    // Expression: 0.0
    //  Referenced by: '<S182>/Delay4'

    0.0
  }
  ,

  // End of '<S179>/Calculate Running Mean'

  // Start of '<S169>/AHRS2'
  {
    // Expression: 0.0001924722
    //  Referenced by: '<S169>/AHRS2'

    0.0001924722,

    // Expression: 9.1385e-5
    //  Referenced by: '<S169>/AHRS2'

    9.1385E-5,

    // Expression: 0.1
    //  Referenced by: '<S169>/AHRS2'

    0.1,

    // Expression: 3.0462e-13
    //  Referenced by: '<S169>/AHRS2'

    3.0462E-13,

    // Expression: 0.0096236100000000012
    //  Referenced by: '<S169>/AHRS2'

    0.0096236100000000012,

    // Expression: 0.5
    //  Referenced by: '<S169>/AHRS2'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S169>/AHRS2'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S169>/AHRS2'

    0.5,

    // Expression: 33
    //  Referenced by: '<S169>/AHRS2'

    33.0
  }
  // End of '<S169>/AHRS2'
};

//
// File trailer for generated code.
//
// [EOF]
//
