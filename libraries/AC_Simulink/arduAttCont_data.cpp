/*
 * arduAttCont_data.cpp
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "arduAttCont".
 *
 * Model version              : 2.13
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C++ source code generated on : Sat Nov 18 11:42:48 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "arduAttCont.h"
#include "arduAttCont_private.h"

/* Block parameters (default storage) */
P_arduAttCont_T arduAttCont::arduAttCont_P{
  /* Variable: ANG_PIT_P
   * Referenced by: '<Root>/Gain1'
   */
  4.05F,

  /* Variable: ANG_RLL_P
   * Referenced by: '<Root>/Gain'
   */
  4.05F,

  /* Variable: ANG_YAW_P
   * Referenced by: '<Root>/Gain2'
   */
  4.05F,

  /* Variable: RAT_PIT_D
   * Referenced by: '<S29>/Derivative Gain'
   */
  0.00324F,

  /* Variable: RAT_PIT_I
   * Referenced by: '<S32>/Integral Gain'
   */
  0.1215F,

  /* Variable: RAT_PIT_P
   * Referenced by: '<S40>/Proportional Gain'
   */
  0.1215F,

  /* Variable: RAT_RLL_D
   * Referenced by: '<S77>/Derivative Gain'
   */
  0.00324F,

  /* Variable: RAT_RLL_I
   * Referenced by: '<S80>/Integral Gain'
   */
  0.1215F,

  /* Variable: RAT_RLL_P
   * Referenced by: '<S88>/Proportional Gain'
   */
  0.1215F,

  /* Variable: RAT_YAW_D
   * Referenced by: '<S125>/Derivative Gain'
   */
  0.0F,

  /* Variable: RAT_YAW_I
   * Referenced by: '<S128>/Integral Gain'
   */
  0.0162F,

  /* Variable: RAT_YAW_P
   * Referenced by: '<S136>/Proportional Gain'
   */
  0.162F,

  /* Mask Parameter: rollratePIDController_InitialCo
   * Referenced by: '<S78>/Filter'
   */
  0.0F,

  /* Mask Parameter: pitchratePIDController_InitialC
   * Referenced by: '<S30>/Filter'
   */
  0.0F,

  /* Mask Parameter: yawratePIDController_InitialCon
   * Referenced by: '<S126>/Filter'
   */
  0.0F,

  /* Mask Parameter: rollratePIDController_Initial_o
   * Referenced by: '<S83>/Integrator'
   */
  0.0F,

  /* Mask Parameter: pitchratePIDController_Initia_d
   * Referenced by: '<S35>/Integrator'
   */
  0.0F,

  /* Mask Parameter: yawratePIDController_InitialC_n
   * Referenced by: '<S131>/Integrator'
   */
  0.0F,

  /* Mask Parameter: rollratePIDController_N
   * Referenced by: '<S86>/Filter Coefficient'
   */
  50.2654839F,

  /* Mask Parameter: pitchratePIDController_N
   * Referenced by: '<S38>/Filter Coefficient'
   */
  50.2654839F,

  /* Mask Parameter: yawratePIDController_N
   * Referenced by: '<S134>/Filter Coefficient'
   */
  0.0F,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S83>/Integrator'
   */
  0.01F,

  /* Computed Parameter: Filter_gainval
   * Referenced by: '<S78>/Filter'
   */
  0.01F,

  /* Computed Parameter: Gain7_Gain
   * Referenced by: '<Root>/Gain7'
   */
  0.0F,

  /* Computed Parameter: Integrator_gainval_d
   * Referenced by: '<S35>/Integrator'
   */
  0.01F,

  /* Computed Parameter: Filter_gainval_c
   * Referenced by: '<S30>/Filter'
   */
  0.01F,

  /* Computed Parameter: Gain8_Gain
   * Referenced by: '<Root>/Gain8'
   */
  0.0F,

  /* Computed Parameter: Gain4_Gain
   * Referenced by: '<Root>/Gain4'
   */
  -1.0F,

  /* Computed Parameter: Gain3_Gain
   * Referenced by: '<Root>/Gain3'
   */
  0.0F,

  /* Computed Parameter: Gain5_Gain
   * Referenced by: '<Root>/Gain5'
   */
  1.0F,

  /* Computed Parameter: Integrator_gainval_o
   * Referenced by: '<S131>/Integrator'
   */
  0.01F,

  /* Computed Parameter: Filter_gainval_n
   * Referenced by: '<S126>/Filter'
   */
  0.01F
};
