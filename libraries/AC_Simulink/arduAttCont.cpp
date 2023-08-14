/*
 * arduAttCont.cpp
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "arduAttCont".
 *
 * Model version              : 1.4
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C++ source code generated on : Sat Jul 15 10:42:20 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "arduAttCont.h"
//#include "rtwtypes.h"
#include "arduAttCont_private.h"

/* extern "C" {

#include "rt_nonfinite.h"

} */
/* Model step function */
 /*  void arduAttCont::step()
{ */
void arduAttCont::step(real32_T arg_attiude_error[3], real32_T
  arg_rate_ff[3], real32_T arg_rate_meas[3], real32_T (&arg_Out1)[3])
{
  real32_T rtb_FilterCoefficient;
  real32_T rtb_FilterCoefficient_g;
  real32_T rtb_FilterCoefficient_p;
  real32_T rtb_Sum2;
  real32_T rtb_Sum4;
  real32_T rtb_Sum6;

  /* Sum: '<Root>/Sum2' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/attiude_error'
   *  Inport: '<Root>/rate_ff'
   *  Inport: '<Root>/rate_meas'
   *  Sum: '<Root>/Sum1'
   */
  rtb_Sum2 = (arduAttCont_P.ANG_RLL_P * arduAttCont_U.attiude_error[0] +
              arduAttCont_U.rate_ff[0]) - arduAttCont_U.rate_meas[0];

  /* Gain: '<S86>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S78>/Filter'
   *  Gain: '<S77>/Derivative Gain'
   *  Sum: '<S78>/SumD'
   */
  rtb_FilterCoefficient = (arduAttCont_P.RAT_RLL_D * rtb_Sum2 -
    arduAttCont_DW.Filter_DSTATE) * arduAttCont_P.rollratePIDController_N;

  /* Sum: '<Root>/Sum4' incorporates:
   *  Gain: '<Root>/Gain1'
   *  Inport: '<Root>/attiude_error'
   *  Inport: '<Root>/rate_ff'
   *  Inport: '<Root>/rate_meas'
   *  Sum: '<Root>/Sum3'
   */
  rtb_Sum4 = (arduAttCont_P.ANG_PIT_P * arduAttCont_U.attiude_error[1] +
              arduAttCont_U.rate_ff[1]) - arduAttCont_U.rate_meas[1];

  /* Gain: '<S38>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S30>/Filter'
   *  Gain: '<S29>/Derivative Gain'
   *  Sum: '<S30>/SumD'
   */
  rtb_FilterCoefficient_g = (arduAttCont_P.RAT_PIT_D * rtb_Sum4 -
    arduAttCont_DW.Filter_DSTATE_j) * arduAttCont_P.pitchratePIDController_N;

  /* Sum: '<Root>/Sum6' incorporates:
   *  Gain: '<Root>/Gain2'
   *  Inport: '<Root>/attiude_error'
   *  Inport: '<Root>/rate_ff'
   *  Inport: '<Root>/rate_meas'
   *  Sum: '<Root>/Sum5'
   */
  rtb_Sum6 = (arduAttCont_P.ANG_YAW_P * arduAttCont_U.attiude_error[2] +
              arduAttCont_U.rate_ff[2]) - arduAttCont_U.rate_meas[2];

  /* Gain: '<S134>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S126>/Filter'
   *  Gain: '<S125>/Derivative Gain'
   *  Sum: '<S126>/SumD'
   */
  rtb_FilterCoefficient_p = (arduAttCont_P.RAT_YAW_D * rtb_Sum6 -
    arduAttCont_DW.Filter_DSTATE_b) * arduAttCont_P.yawratePIDController_N;

  /* Outport: '<Root>/Out1' incorporates:
   *  DiscreteIntegrator: '<S131>/Integrator'
   *  DiscreteIntegrator: '<S35>/Integrator'
   *  DiscreteIntegrator: '<S83>/Integrator'
   *  Gain: '<S136>/Proportional Gain'
   *  Gain: '<S40>/Proportional Gain'
   *  Gain: '<S88>/Proportional Gain'
   *  Sum: '<S140>/Sum'
   *  Sum: '<S44>/Sum'
   *  Sum: '<S92>/Sum'
   */
  arg_Out1[0] = (arduAttCont_P.RAT_RLL_P * rtb_Sum2 +
    arduAttCont_DW.Integrator_DSTATE) + rtb_FilterCoefficient;
  arg_Out1[1] = (arduAttCont_P.RAT_PIT_P * rtb_Sum4 +
    arduAttCont_DW.Integrator_DSTATE_k) + rtb_FilterCoefficient_g;
  arg_Out1[2] = (arduAttCont_P.RAT_YAW_P * rtb_Sum6 +
    arduAttCont_DW.Integrator_DSTATE_a) + rtb_FilterCoefficient_p;

  /* Update for DiscreteIntegrator: '<S83>/Integrator' incorporates:
   *  Gain: '<S80>/Integral Gain'
   */
  arduAttCont_DW.Integrator_DSTATE += arduAttCont_P.RAT_RLL_I * rtb_Sum2 *
    arduAttCont_P.Integrator_gainval;

  /* Update for DiscreteIntegrator: '<S78>/Filter' */
  arduAttCont_DW.Filter_DSTATE += arduAttCont_P.Filter_gainval *
    rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S35>/Integrator' incorporates:
   *  Gain: '<S32>/Integral Gain'
   */
  arduAttCont_DW.Integrator_DSTATE_k += arduAttCont_P.RAT_PIT_I * rtb_Sum4 *
    arduAttCont_P.Integrator_gainval_d;

  /* Update for DiscreteIntegrator: '<S30>/Filter' */
  arduAttCont_DW.Filter_DSTATE_j += arduAttCont_P.Filter_gainval_c *
    rtb_FilterCoefficient_g;

  /* Update for DiscreteIntegrator: '<S131>/Integrator' incorporates:
   *  Gain: '<S128>/Integral Gain'
   */
  arduAttCont_DW.Integrator_DSTATE_a += arduAttCont_P.RAT_YAW_I * rtb_Sum6 *
    arduAttCont_P.Integrator_gainval_o;

  /* Update for DiscreteIntegrator: '<S126>/Filter' */
  arduAttCont_DW.Filter_DSTATE_b += arduAttCont_P.Filter_gainval_n *
    rtb_FilterCoefficient_p;

/* Matfile logging */
/*   rt_UpdateTXYLogVars((&arduAttCont_M)->rtwLogInfo, (&(&arduAttCont_M)
    ->Timing.taskTime0));
 */
/* signal main to stop simulation */
/* Sample time: [0.01s, 0.0s] */
  /*if ((rtmGetTFinal((&arduAttCont_M))!=-1) &&
        !((rtmGetTFinal((&arduAttCont_M))-(&arduAttCont_M)->Timing.taskTime0) >
          (&arduAttCont_M)->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus((&arduAttCont_M), "Simulation finished");
    }
  */


  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
/*   if (!(++(&arduAttCont_M)->Timing.clockTick0)) {
    ++(&arduAttCont_M)->Timing.clockTickH0;
  }

  (&arduAttCont_M)->Timing.taskTime0 = (&arduAttCont_M)->Timing.clockTick0 *
    (&arduAttCont_M)->Timing.stepSize0 + (&arduAttCont_M)->Timing.clockTickH0 *
    (&arduAttCont_M)->Timing.stepSize0 * 4294967296.0;  */

}

/* Model initialize function */
void arduAttCont::initialize()
{
  /* Registration code */

  /* initialize non-finites */
/*   rt_InitInfAndNaN(sizeof(real_T));
  rtmSetTFinal((&arduAttCont_M), 10.0);
  (&arduAttCont_M)->Timing.stepSize0 = 0.01; */

  /* Setup for data logging */
/*   {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (nullptr);
    (&arduAttCont_M)->rtwLogInfo = &rt_DataLoggingInfo;
  } */

  /* Setup for data logging */
/*   {
    rtliSetLogXSignalInfo((&arduAttCont_M)->rtwLogInfo, (nullptr));
    rtliSetLogXSignalPtrs((&arduAttCont_M)->rtwLogInfo, (nullptr));
    rtliSetLogT((&arduAttCont_M)->rtwLogInfo, "tout");
    rtliSetLogX((&arduAttCont_M)->rtwLogInfo, "");
    rtliSetLogXFinal((&arduAttCont_M)->rtwLogInfo, "");
    rtliSetLogVarNameModifier((&arduAttCont_M)->rtwLogInfo, "rt_");
    rtliSetLogFormat((&arduAttCont_M)->rtwLogInfo, 4);
    rtliSetLogMaxRows((&arduAttCont_M)->rtwLogInfo, 0);
    rtliSetLogDecimation((&arduAttCont_M)->rtwLogInfo, 1);
    rtliSetLogY((&arduAttCont_M)->rtwLogInfo, "");
    rtliSetLogYSignalInfo((&arduAttCont_M)->rtwLogInfo, (nullptr));
    rtliSetLogYSignalPtrs((&arduAttCont_M)->rtwLogInfo, (nullptr));
  } */

  /* Matfile logging */
/*   rt_StartDataLoggingWithStartTime((&arduAttCont_M)->rtwLogInfo, 0.0,
    rtmGetTFinal((&arduAttCont_M)), (&arduAttCont_M)->Timing.stepSize0,
    (&rtmGetErrorStatus((&arduAttCont_M)))); */

  /* InitializeConditions for DiscreteIntegrator: '<S83>/Integrator' */
  arduAttCont_DW.Integrator_DSTATE =
    arduAttCont_P.rollratePIDController_Initial_o;

  /* InitializeConditions for DiscreteIntegrator: '<S78>/Filter' */
  arduAttCont_DW.Filter_DSTATE = arduAttCont_P.rollratePIDController_InitialCo;

  /* InitializeConditions for DiscreteIntegrator: '<S35>/Integrator' */
  arduAttCont_DW.Integrator_DSTATE_k =
    arduAttCont_P.pitchratePIDController_Initia_d;

  /* InitializeConditions for DiscreteIntegrator: '<S30>/Filter' */
  arduAttCont_DW.Filter_DSTATE_j = arduAttCont_P.pitchratePIDController_InitialC;

  /* InitializeConditions for DiscreteIntegrator: '<S131>/Integrator' */
  arduAttCont_DW.Integrator_DSTATE_a =
    arduAttCont_P.yawratePIDController_InitialC_n;

  /* InitializeConditions for DiscreteIntegrator: '<S126>/Filter' */
  arduAttCont_DW.Filter_DSTATE_b = arduAttCont_P.yawratePIDController_InitialCon;
}

/* Model terminate function */
void arduAttCont::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
arduAttCont::arduAttCont() :
  arduAttCont_U(),
  arduAttCont_Y(),
  arduAttCont_DW(),
  arduAttCont_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
arduAttCont::~arduAttCont()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_arduAttCont_T * arduAttCont::getRTM()
{
  return (&arduAttCont_M);
}
