/*
 * model4_pi_controller_hw.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "model4_pi_controller_hw".
 *
 * Model version              : 18.1
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Sat Nov 14 17:05:14 2026
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "model4_pi_controller_hw.h"
#include "rtwtypes.h"
#include "model4_pi_controller_hw_private.h"
#include <string.h>
#include "model4_pi_controller_hw_dt.h"

/* Block signals (default storage) */
B_model4_pi_controller_hw_T model4_pi_controller_hw_B;

/* Block states (default storage) */
DW_model4_pi_controller_hw_T model4_pi_controller_hw_DW;

/* Real-time model */
static RT_MODEL_model4_pi_controller_T model4_pi_controller_hw_M_;
RT_MODEL_model4_pi_controller_T *const model4_pi_controller_hw_M =
  &model4_pi_controller_hw_M_;

/*
 * Writes out MAT-file header.  Returns success or failure.
 * Returns:
 * 0 - success
 * 1 - failure
 */
int_T rt_WriteMat4FileHeader(FILE *fp, int32_T m, int32_T n, const char *name)
{
  typedef enum { ELITTLE_ENDIAN, EBIG_ENDIAN } ByteOrder;

  int16_T one = 1;
  ByteOrder byteOrder = (*((int8_T *)&one)==1) ? ELITTLE_ENDIAN : EBIG_ENDIAN;
  int32_T type = (byteOrder == ELITTLE_ENDIAN) ? 0: 1000;
  int32_T imagf = 0;
  int32_T name_len = (int32_T)strlen(name) + 1;
  if ((fwrite(&type, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&m, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&n, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&imagf, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&name_len, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(name, sizeof(char), name_len, fp) == 0)) {
    return(1);
  } else {
    return(0);
  }
}                                      /* end rt_WriteMat4FileHeader */

/* Model output function */
void model4_pi_controller_hw_output(void)
{
  real_T Speed_Ref_tmp;
  real_T rtb_IntegralGain;
  int8_T tmp;
  int8_T tmp_0;
  boolean_T rtb_RelationalOperator;
  boolean_T rtb_fixforDTpropagationissue;

  /* Step: '<Root>/Speed_Ref' incorporates:
   *  Step: '<Root>/Disturbance'
   */
  Speed_Ref_tmp = model4_pi_controller_hw_M->Timing.t[0];
  if (Speed_Ref_tmp < model4_pi_controller_hw_P.Speed_Ref_Time) {
    /* Step: '<Root>/Speed_Ref' */
    model4_pi_controller_hw_B.Speed_Ref = model4_pi_controller_hw_P.Speed_Ref_Y0;
  } else {
    /* Step: '<Root>/Speed_Ref' */
    model4_pi_controller_hw_B.Speed_Ref =
      model4_pi_controller_hw_P.Speed_Ref_YFinal;
  }

  /* End of Step: '<Root>/Speed_Ref' */

  /* S-Function (hil_read_encoder_block): '<Root>/HIL Read Encoder' */

  /* S-Function Block: model4_pi_controller_hw/HIL Read Encoder (hil_read_encoder_block) */
  {
    t_error result = hil_read_encoder
      (model4_pi_controller_hw_DW.HILInitialize_Card,
       &model4_pi_controller_hw_P.HILReadEncoder_channels, 1,
       &model4_pi_controller_hw_DW.HILReadEncoder_Buffer);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
    } else {
      model4_pi_controller_hw_B.HILReadEncoder =
        model4_pi_controller_hw_DW.HILReadEncoder_Buffer;
    }
  }

  /* Gain: '<Root>/Enc_to_RadPerSec' incorporates:
   *  Delay: '<Root>/Count_Delay'
   *  Sum: '<Root>/Count_Diff'
   */
  model4_pi_controller_hw_B.Enc_to_RadPerSec =
    (model4_pi_controller_hw_B.HILReadEncoder -
     model4_pi_controller_hw_DW.Count_Delay_DSTATE[0]) *
    model4_pi_controller_hw_P.Enc_to_RadPerSec_Gain;

  /* DiscreteFilter: '<Root>/Speed_Filter' */
  model4_pi_controller_hw_DW.Speed_Filter_tmp =
    (model4_pi_controller_hw_B.Enc_to_RadPerSec -
     model4_pi_controller_hw_P.Speed_Filter_DenCoef[1] *
     model4_pi_controller_hw_DW.Speed_Filter_states) /
    model4_pi_controller_hw_P.Speed_Filter_DenCoef[0];

  /* DiscreteFilter: '<Root>/Speed_Filter' */
  model4_pi_controller_hw_B.Speed_Filter =
    model4_pi_controller_hw_P.Speed_Filter_NumCoef *
    model4_pi_controller_hw_DW.Speed_Filter_tmp;

  /* Sum: '<Root>/Error_Sum' */
  model4_pi_controller_hw_B.Error_Sum = model4_pi_controller_hw_B.Speed_Ref -
    model4_pi_controller_hw_B.Speed_Filter;

  /* Sum: '<S49>/Sum' incorporates:
   *  DiscreteIntegrator: '<S40>/Integrator'
   *  Gain: '<S45>/Proportional Gain'
   */
  rtb_IntegralGain = model4_pi_controller_hw_P.Kp *
    model4_pi_controller_hw_B.Error_Sum +
    model4_pi_controller_hw_DW.Integrator_DSTATE;

  /* Saturate: '<S47>/Saturation' */
  if (rtb_IntegralGain >
      model4_pi_controller_hw_P.Discrete_PI_UpperSaturationLimi) {
    /* Saturate: '<S47>/Saturation' */
    model4_pi_controller_hw_B.Saturation =
      model4_pi_controller_hw_P.Discrete_PI_UpperSaturationLimi;
  } else if (rtb_IntegralGain <
             model4_pi_controller_hw_P.Discrete_PI_LowerSaturationLimi) {
    /* Saturate: '<S47>/Saturation' */
    model4_pi_controller_hw_B.Saturation =
      model4_pi_controller_hw_P.Discrete_PI_LowerSaturationLimi;
  } else {
    /* Saturate: '<S47>/Saturation' */
    model4_pi_controller_hw_B.Saturation = rtb_IntegralGain;
  }

  /* End of Saturate: '<S47>/Saturation' */

  /* Step: '<Root>/Disturbance' */
  if (Speed_Ref_tmp < model4_pi_controller_hw_P.Disturbance_Time) {
    Speed_Ref_tmp = model4_pi_controller_hw_P.Disturbance_Y0;
  } else {
    Speed_Ref_tmp = model4_pi_controller_hw_P.Disturbance_YFinal;
  }

  /* Sum: '<Root>/Dist_Sum' incorporates:
   *  Step: '<Root>/Disturbance'
   */
  model4_pi_controller_hw_B.Dist_Sum = model4_pi_controller_hw_B.Saturation +
    Speed_Ref_tmp;

  /* Saturate: '<Root>/Voltage_Sat' */
  if (model4_pi_controller_hw_B.Dist_Sum >
      model4_pi_controller_hw_P.Voltage_Sat_UpperSat) {
    /* Saturate: '<Root>/Voltage_Sat' */
    model4_pi_controller_hw_B.Voltage_Sat =
      model4_pi_controller_hw_P.Voltage_Sat_UpperSat;
  } else if (model4_pi_controller_hw_B.Dist_Sum <
             model4_pi_controller_hw_P.Voltage_Sat_LowerSat) {
    /* Saturate: '<Root>/Voltage_Sat' */
    model4_pi_controller_hw_B.Voltage_Sat =
      model4_pi_controller_hw_P.Voltage_Sat_LowerSat;
  } else {
    /* Saturate: '<Root>/Voltage_Sat' */
    model4_pi_controller_hw_B.Voltage_Sat = model4_pi_controller_hw_B.Dist_Sum;
  }

  /* End of Saturate: '<Root>/Voltage_Sat' */
  /* S-Function (hil_write_analog_block): '<Root>/HIL Write Analog' */

  /* S-Function Block: model4_pi_controller_hw/HIL Write Analog (hil_write_analog_block) */
  {
    t_error result;
    result = hil_write_analog(model4_pi_controller_hw_DW.HILInitialize_Card,
      &model4_pi_controller_hw_P.HILWriteAnalog_channels, 1,
      &model4_pi_controller_hw_B.Voltage_Sat);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
    }
  }

  /* ToFile: '<Root>/Log_Cmd' */
  {
    if (!(++model4_pi_controller_hw_DW.Log_Cmd_IWORK.Decimation % 1) &&
        (model4_pi_controller_hw_DW.Log_Cmd_IWORK.Count * (1 + 1)) + 1 <
        100000000 ) {
      FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_Cmd_PWORK.FilePtr;
      if (fp != (NULL)) {
        real_T u[1 + 1];
        model4_pi_controller_hw_DW.Log_Cmd_IWORK.Decimation = 0;
        u[0] = model4_pi_controller_hw_M->Timing.t[1];
        u[1] = model4_pi_controller_hw_B.Voltage_Sat;
        if (fwrite(u, sizeof(real_T), 1 + 1, fp) != 1 + 1) {
          rtmSetErrorStatus(model4_pi_controller_hw_M,
                            "Error writing to MAT-file PI_cmd.mat");
          return;
        }

        if (((++model4_pi_controller_hw_DW.Log_Cmd_IWORK.Count) * (1 + 1))+1 >=
            100000000) {
          (void)fprintf(stdout,
                        "*** The ToFile block will stop logging data before\n"
                        "    the simulation has ended, because it has reached\n"
                        "    the maximum number of elements (100000000)\n"
                        "    allowed in MAT-file PI_cmd.mat.\n");
        }
      }
    }
  }

  /* ToFile: '<Root>/Log_CmdRaw' */
  {
    if (!(++model4_pi_controller_hw_DW.Log_CmdRaw_IWORK.Decimation % 1) &&
        (model4_pi_controller_hw_DW.Log_CmdRaw_IWORK.Count * (1 + 1)) + 1 <
        100000000 ) {
      FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_CmdRaw_PWORK.FilePtr;
      if (fp != (NULL)) {
        real_T u[1 + 1];
        model4_pi_controller_hw_DW.Log_CmdRaw_IWORK.Decimation = 0;
        u[0] = model4_pi_controller_hw_M->Timing.t[1];
        u[1] = model4_pi_controller_hw_B.Dist_Sum;
        if (fwrite(u, sizeof(real_T), 1 + 1, fp) != 1 + 1) {
          rtmSetErrorStatus(model4_pi_controller_hw_M,
                            "Error writing to MAT-file PI_cmd_raw.mat");
          return;
        }

        if (((++model4_pi_controller_hw_DW.Log_CmdRaw_IWORK.Count) * (1 + 1))+1 >=
            100000000) {
          (void)fprintf(stdout,
                        "*** The ToFile block will stop logging data before\n"
                        "    the simulation has ended, because it has reached\n"
                        "    the maximum number of elements (100000000)\n"
                        "    allowed in MAT-file PI_cmd_raw.mat.\n");
        }
      }
    }
  }

  /* ToFile: '<Root>/Log_Error' */
  {
    if (!(++model4_pi_controller_hw_DW.Log_Error_IWORK.Decimation % 1) &&
        (model4_pi_controller_hw_DW.Log_Error_IWORK.Count * (1 + 1)) + 1 <
        100000000 ) {
      FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_Error_PWORK.FilePtr;
      if (fp != (NULL)) {
        real_T u[1 + 1];
        model4_pi_controller_hw_DW.Log_Error_IWORK.Decimation = 0;
        u[0] = model4_pi_controller_hw_M->Timing.t[1];
        u[1] = model4_pi_controller_hw_B.Error_Sum;
        if (fwrite(u, sizeof(real_T), 1 + 1, fp) != 1 + 1) {
          rtmSetErrorStatus(model4_pi_controller_hw_M,
                            "Error writing to MAT-file PI_error.mat");
          return;
        }

        if (((++model4_pi_controller_hw_DW.Log_Error_IWORK.Count) * (1 + 1))+1 >=
            100000000) {
          (void)fprintf(stdout,
                        "*** The ToFile block will stop logging data before\n"
                        "    the simulation has ended, because it has reached\n"
                        "    the maximum number of elements (100000000)\n"
                        "    allowed in MAT-file PI_error.mat.\n");
        }
      }
    }
  }

  /* ToFile: '<Root>/Log_Ref' */
  {
    if (!(++model4_pi_controller_hw_DW.Log_Ref_IWORK.Decimation % 1) &&
        (model4_pi_controller_hw_DW.Log_Ref_IWORK.Count * (1 + 1)) + 1 <
        100000000 ) {
      FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_Ref_PWORK.FilePtr;
      if (fp != (NULL)) {
        real_T u[1 + 1];
        model4_pi_controller_hw_DW.Log_Ref_IWORK.Decimation = 0;
        u[0] = model4_pi_controller_hw_M->Timing.t[1];
        u[1] = model4_pi_controller_hw_B.Speed_Ref;
        if (fwrite(u, sizeof(real_T), 1 + 1, fp) != 1 + 1) {
          rtmSetErrorStatus(model4_pi_controller_hw_M,
                            "Error writing to MAT-file PI_ref.mat");
          return;
        }

        if (((++model4_pi_controller_hw_DW.Log_Ref_IWORK.Count) * (1 + 1))+1 >=
            100000000) {
          (void)fprintf(stdout,
                        "*** The ToFile block will stop logging data before\n"
                        "    the simulation has ended, because it has reached\n"
                        "    the maximum number of elements (100000000)\n"
                        "    allowed in MAT-file PI_ref.mat.\n");
        }
      }
    }
  }

  /* ToFile: '<Root>/Log_SpeedFilt' */
  {
    if (!(++model4_pi_controller_hw_DW.Log_SpeedFilt_IWORK.Decimation % 1) &&
        (model4_pi_controller_hw_DW.Log_SpeedFilt_IWORK.Count * (1 + 1)) + 1 <
        100000000 ) {
      FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_SpeedFilt_PWORK.FilePtr;
      if (fp != (NULL)) {
        real_T u[1 + 1];
        model4_pi_controller_hw_DW.Log_SpeedFilt_IWORK.Decimation = 0;
        u[0] = model4_pi_controller_hw_M->Timing.t[1];
        u[1] = model4_pi_controller_hw_B.Speed_Filter;
        if (fwrite(u, sizeof(real_T), 1 + 1, fp) != 1 + 1) {
          rtmSetErrorStatus(model4_pi_controller_hw_M,
                            "Error writing to MAT-file PI_speed_filt.mat");
          return;
        }

        if (((++model4_pi_controller_hw_DW.Log_SpeedFilt_IWORK.Count) * (1 + 1))
            +1 >= 100000000) {
          (void)fprintf(stdout,
                        "*** The ToFile block will stop logging data before\n"
                        "    the simulation has ended, because it has reached\n"
                        "    the maximum number of elements (100000000)\n"
                        "    allowed in MAT-file PI_speed_filt.mat.\n");
        }
      }
    }
  }

  /* ToFile: '<Root>/Log_SpeedRaw' */
  {
    if (!(++model4_pi_controller_hw_DW.Log_SpeedRaw_IWORK.Decimation % 1) &&
        (model4_pi_controller_hw_DW.Log_SpeedRaw_IWORK.Count * (1 + 1)) + 1 <
        100000000 ) {
      FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_SpeedRaw_PWORK.FilePtr;
      if (fp != (NULL)) {
        real_T u[1 + 1];
        model4_pi_controller_hw_DW.Log_SpeedRaw_IWORK.Decimation = 0;
        u[0] = model4_pi_controller_hw_M->Timing.t[1];
        u[1] = model4_pi_controller_hw_B.Enc_to_RadPerSec;
        if (fwrite(u, sizeof(real_T), 1 + 1, fp) != 1 + 1) {
          rtmSetErrorStatus(model4_pi_controller_hw_M,
                            "Error writing to MAT-file PI_speed_raw.mat");
          return;
        }

        if (((++model4_pi_controller_hw_DW.Log_SpeedRaw_IWORK.Count) * (1 + 1))+
            1 >= 100000000) {
          (void)fprintf(stdout,
                        "*** The ToFile block will stop logging data before\n"
                        "    the simulation has ended, because it has reached\n"
                        "    the maximum number of elements (100000000)\n"
                        "    allowed in MAT-file PI_speed_raw.mat.\n");
        }
      }
    }
  }

  /* DeadZone: '<S32>/DeadZone' */
  if (rtb_IntegralGain >
      model4_pi_controller_hw_P.Discrete_PI_UpperSaturationLimi) {
    rtb_IntegralGain -=
      model4_pi_controller_hw_P.Discrete_PI_UpperSaturationLimi;
  } else if (rtb_IntegralGain >=
             model4_pi_controller_hw_P.Discrete_PI_LowerSaturationLimi) {
    rtb_IntegralGain = 0.0;
  } else {
    rtb_IntegralGain -=
      model4_pi_controller_hw_P.Discrete_PI_LowerSaturationLimi;
  }

  /* End of DeadZone: '<S32>/DeadZone' */

  /* RelationalOperator: '<S30>/Relational Operator' incorporates:
   *  Constant: '<S30>/Clamping_zero'
   */
  rtb_RelationalOperator = (model4_pi_controller_hw_P.Clamping_zero_Value !=
    rtb_IntegralGain);

  /* RelationalOperator: '<S30>/fix for DT propagation issue' incorporates:
   *  Constant: '<S30>/Clamping_zero'
   */
  rtb_fixforDTpropagationissue = (rtb_IntegralGain >
    model4_pi_controller_hw_P.Clamping_zero_Value);

  /* Gain: '<S37>/Integral Gain' */
  rtb_IntegralGain = model4_pi_controller_hw_P.Ki *
    model4_pi_controller_hw_B.Error_Sum;

  /* Switch: '<S30>/Switch1' incorporates:
   *  Constant: '<S30>/Constant'
   *  Constant: '<S30>/Constant2'
   */
  if (rtb_fixforDTpropagationissue) {
    tmp = model4_pi_controller_hw_P.Constant_Value;
  } else {
    tmp = model4_pi_controller_hw_P.Constant2_Value;
  }

  /* Switch: '<S30>/Switch2' incorporates:
   *  Constant: '<S30>/Clamping_zero'
   *  Constant: '<S30>/Constant3'
   *  Constant: '<S30>/Constant4'
   *  RelationalOperator: '<S30>/fix for DT propagation issue1'
   */
  if (rtb_IntegralGain > model4_pi_controller_hw_P.Clamping_zero_Value) {
    tmp_0 = model4_pi_controller_hw_P.Constant3_Value;
  } else {
    tmp_0 = model4_pi_controller_hw_P.Constant4_Value;
  }

  /* Switch: '<S30>/Switch' incorporates:
   *  Logic: '<S30>/AND3'
   *  RelationalOperator: '<S30>/Equal1'
   *  Switch: '<S30>/Switch1'
   *  Switch: '<S30>/Switch2'
   */
  if (rtb_RelationalOperator && (tmp == tmp_0)) {
    /* Switch: '<S30>/Switch' incorporates:
     *  Constant: '<S30>/Constant1'
     */
    model4_pi_controller_hw_B.Switch = model4_pi_controller_hw_P.Constant1_Value;
  } else {
    /* Switch: '<S30>/Switch' */
    model4_pi_controller_hw_B.Switch = rtb_IntegralGain;
  }

  /* End of Switch: '<S30>/Switch' */
}

/* Model update function */
void model4_pi_controller_hw_update(void)
{
  /* Update for Delay: '<Root>/Count_Delay' */
  model4_pi_controller_hw_DW.Count_Delay_DSTATE[0] =
    model4_pi_controller_hw_DW.Count_Delay_DSTATE[1];
  model4_pi_controller_hw_DW.Count_Delay_DSTATE[1] =
    model4_pi_controller_hw_DW.Count_Delay_DSTATE[2];
  model4_pi_controller_hw_DW.Count_Delay_DSTATE[2] =
    model4_pi_controller_hw_DW.Count_Delay_DSTATE[3];
  model4_pi_controller_hw_DW.Count_Delay_DSTATE[3] =
    model4_pi_controller_hw_B.HILReadEncoder;

  /* Update for DiscreteFilter: '<Root>/Speed_Filter' */
  model4_pi_controller_hw_DW.Speed_Filter_states =
    model4_pi_controller_hw_DW.Speed_Filter_tmp;

  /* Update for DiscreteIntegrator: '<S40>/Integrator' */
  model4_pi_controller_hw_DW.Integrator_DSTATE +=
    model4_pi_controller_hw_P.Integrator_gainval *
    model4_pi_controller_hw_B.Switch;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++model4_pi_controller_hw_M->Timing.clockTick0)) {
    ++model4_pi_controller_hw_M->Timing.clockTickH0;
  }

  model4_pi_controller_hw_M->Timing.t[0] =
    model4_pi_controller_hw_M->Timing.clockTick0 *
    model4_pi_controller_hw_M->Timing.stepSize0 +
    model4_pi_controller_hw_M->Timing.clockTickH0 *
    model4_pi_controller_hw_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.002s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++model4_pi_controller_hw_M->Timing.clockTick1)) {
      ++model4_pi_controller_hw_M->Timing.clockTickH1;
    }

    model4_pi_controller_hw_M->Timing.t[1] =
      model4_pi_controller_hw_M->Timing.clockTick1 *
      model4_pi_controller_hw_M->Timing.stepSize1 +
      model4_pi_controller_hw_M->Timing.clockTickH1 *
      model4_pi_controller_hw_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Model initialize function */
void model4_pi_controller_hw_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: model4_pi_controller_hw/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("qube_servo3_usb", "0",
                      &model4_pi_controller_hw_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options
      (model4_pi_controller_hw_DW.HILInitialize_Card, " ", 2);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(model4_pi_controller_hw_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
      return;
    }

    if ((model4_pi_controller_hw_P.HILInitialize_AIPStart && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_AIPEnter && is_switching)) {
      result = hil_set_analog_input_ranges
        (model4_pi_controller_hw_DW.HILInitialize_Card,
         &model4_pi_controller_hw_P.HILInitialize_AIChannels, 1U,
         &model4_pi_controller_hw_P.HILInitialize_AILow,
         &model4_pi_controller_hw_P.HILInitialize_AIHigh);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }

    if ((model4_pi_controller_hw_P.HILInitialize_AOPStart && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_AOPEnter && is_switching)) {
      result = hil_set_analog_output_ranges
        (model4_pi_controller_hw_DW.HILInitialize_Card,
         &model4_pi_controller_hw_P.HILInitialize_AOChannels, 1U,
         &model4_pi_controller_hw_P.HILInitialize_AOLow,
         &model4_pi_controller_hw_P.HILInitialize_AOHigh);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }

    if ((model4_pi_controller_hw_P.HILInitialize_AOStart && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_AOEnter && is_switching)) {
      result = hil_write_analog(model4_pi_controller_hw_DW.HILInitialize_Card,
        &model4_pi_controller_hw_P.HILInitialize_AOChannels, 1U,
        &model4_pi_controller_hw_P.HILInitialize_AOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }

    if (model4_pi_controller_hw_P.HILInitialize_AOReset) {
      result = hil_watchdog_set_analog_expiration_state
        (model4_pi_controller_hw_DW.HILInitialize_Card,
         &model4_pi_controller_hw_P.HILInitialize_AOChannels, 1U,
         &model4_pi_controller_hw_P.HILInitialize_AOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }

    result = hil_set_digital_directions
      (model4_pi_controller_hw_DW.HILInitialize_Card, NULL, 0U,
       &model4_pi_controller_hw_P.HILInitialize_DOChannels, 1U);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
      return;
    }

    if ((model4_pi_controller_hw_P.HILInitialize_DOStart && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_DOEnter && is_switching)) {
      result = hil_write_digital(model4_pi_controller_hw_DW.HILInitialize_Card,
        &model4_pi_controller_hw_P.HILInitialize_DOChannels, 1U, (t_boolean *)
        &model4_pi_controller_hw_P.HILInitialize_DOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }

    if (model4_pi_controller_hw_P.HILInitialize_DOReset) {
      result = hil_watchdog_set_digital_expiration_state
        (model4_pi_controller_hw_DW.HILInitialize_Card,
         &model4_pi_controller_hw_P.HILInitialize_DOChannels, 1U, (const
          t_digital_state *) &model4_pi_controller_hw_P.HILInitialize_DOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }

    if ((model4_pi_controller_hw_P.HILInitialize_EIPStart && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_EIPEnter && is_switching)) {
      model4_pi_controller_hw_DW.HILInitialize_QuadratureModes[0] =
        model4_pi_controller_hw_P.HILInitialize_EIQuadrature;
      model4_pi_controller_hw_DW.HILInitialize_QuadratureModes[1] =
        model4_pi_controller_hw_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode
        (model4_pi_controller_hw_DW.HILInitialize_Card,
         model4_pi_controller_hw_P.HILInitialize_EIChannels, 2U,
         (t_encoder_quadrature_mode *)
         &model4_pi_controller_hw_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }

    if ((model4_pi_controller_hw_P.HILInitialize_EIStart && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_EIEnter && is_switching)) {
      model4_pi_controller_hw_DW.HILInitialize_InitialEICounts[0] =
        model4_pi_controller_hw_P.HILInitialize_EIInitial;
      model4_pi_controller_hw_DW.HILInitialize_InitialEICounts[1] =
        model4_pi_controller_hw_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts
        (model4_pi_controller_hw_DW.HILInitialize_Card,
         model4_pi_controller_hw_P.HILInitialize_EIChannels, 2U,
         &model4_pi_controller_hw_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }

    if ((model4_pi_controller_hw_P.HILInitialize_OOStart && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_OOEnter && is_switching)) {
      result = hil_write_other(model4_pi_controller_hw_DW.HILInitialize_Card,
        model4_pi_controller_hw_P.HILInitialize_OOChannels, 3U,
        model4_pi_controller_hw_P.HILInitialize_OOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }

    if (model4_pi_controller_hw_P.HILInitialize_OOReset) {
      result = hil_watchdog_set_other_expiration_state
        (model4_pi_controller_hw_DW.HILInitialize_Card,
         model4_pi_controller_hw_P.HILInitialize_OOChannels, 3U,
         model4_pi_controller_hw_P.HILInitialize_OOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for ToFile: '<Root>/Log_Cmd' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "PI_cmd.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error creating .mat file PI_cmd.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 1 + 1, 0, "PI_cmd")) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error writing mat file header to file PI_cmd.mat");
      return;
    }

    model4_pi_controller_hw_DW.Log_Cmd_IWORK.Count = 0;
    model4_pi_controller_hw_DW.Log_Cmd_IWORK.Decimation = -1;
    model4_pi_controller_hw_DW.Log_Cmd_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/Log_CmdRaw' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "PI_cmd_raw.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error creating .mat file PI_cmd_raw.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 1 + 1, 0, "PI_cmd_raw")) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error writing mat file header to file PI_cmd_raw.mat");
      return;
    }

    model4_pi_controller_hw_DW.Log_CmdRaw_IWORK.Count = 0;
    model4_pi_controller_hw_DW.Log_CmdRaw_IWORK.Decimation = -1;
    model4_pi_controller_hw_DW.Log_CmdRaw_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/Log_Error' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "PI_error.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error creating .mat file PI_error.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 1 + 1, 0, "PI_error")) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error writing mat file header to file PI_error.mat");
      return;
    }

    model4_pi_controller_hw_DW.Log_Error_IWORK.Count = 0;
    model4_pi_controller_hw_DW.Log_Error_IWORK.Decimation = -1;
    model4_pi_controller_hw_DW.Log_Error_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/Log_Ref' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "PI_ref.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error creating .mat file PI_ref.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 1 + 1, 0, "PI_ref")) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error writing mat file header to file PI_ref.mat");
      return;
    }

    model4_pi_controller_hw_DW.Log_Ref_IWORK.Count = 0;
    model4_pi_controller_hw_DW.Log_Ref_IWORK.Decimation = -1;
    model4_pi_controller_hw_DW.Log_Ref_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/Log_SpeedFilt' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "PI_speed_filt.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error creating .mat file PI_speed_filt.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 1 + 1, 0, "PI_speed_filt")) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error writing mat file header to file PI_speed_filt.mat");
      return;
    }

    model4_pi_controller_hw_DW.Log_SpeedFilt_IWORK.Count = 0;
    model4_pi_controller_hw_DW.Log_SpeedFilt_IWORK.Decimation = -1;
    model4_pi_controller_hw_DW.Log_SpeedFilt_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/Log_SpeedRaw' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "PI_speed_raw.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error creating .mat file PI_speed_raw.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 1 + 1, 0, "PI_speed_raw")) {
      rtmSetErrorStatus(model4_pi_controller_hw_M,
                        "Error writing mat file header to file PI_speed_raw.mat");
      return;
    }

    model4_pi_controller_hw_DW.Log_SpeedRaw_IWORK.Count = 0;
    model4_pi_controller_hw_DW.Log_SpeedRaw_IWORK.Decimation = -1;
    model4_pi_controller_hw_DW.Log_SpeedRaw_PWORK.FilePtr = fp;
  }

  /* InitializeConditions for Delay: '<Root>/Count_Delay' */
  model4_pi_controller_hw_DW.Count_Delay_DSTATE[0] =
    model4_pi_controller_hw_P.Count_Delay_InitialCondition;
  model4_pi_controller_hw_DW.Count_Delay_DSTATE[1] =
    model4_pi_controller_hw_P.Count_Delay_InitialCondition;
  model4_pi_controller_hw_DW.Count_Delay_DSTATE[2] =
    model4_pi_controller_hw_P.Count_Delay_InitialCondition;
  model4_pi_controller_hw_DW.Count_Delay_DSTATE[3] =
    model4_pi_controller_hw_P.Count_Delay_InitialCondition;

  /* InitializeConditions for DiscreteFilter: '<Root>/Speed_Filter' */
  model4_pi_controller_hw_DW.Speed_Filter_states =
    model4_pi_controller_hw_P.Speed_Filter_InitialStates;

  /* InitializeConditions for DiscreteIntegrator: '<S40>/Integrator' */
  model4_pi_controller_hw_DW.Integrator_DSTATE =
    model4_pi_controller_hw_P.Discrete_PI_InitialConditionFor;
}

/* Model terminate function */
void model4_pi_controller_hw_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: model4_pi_controller_hw/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_digital_outputs = 0;
    t_uint32 num_final_other_outputs = 0;
    hil_task_stop_all(model4_pi_controller_hw_DW.HILInitialize_Card);
    hil_monitor_stop_all(model4_pi_controller_hw_DW.HILInitialize_Card);
    is_switching = false;
    if ((model4_pi_controller_hw_P.HILInitialize_AOTerminate && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_AOExit && is_switching)) {
      num_final_analog_outputs = 1U;
    } else {
      num_final_analog_outputs = 0;
    }

    if ((model4_pi_controller_hw_P.HILInitialize_DOTerminate && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_DOExit && is_switching)) {
      num_final_digital_outputs = 1U;
    } else {
      num_final_digital_outputs = 0;
    }

    if ((model4_pi_controller_hw_P.HILInitialize_OOTerminate && !is_switching) ||
        (model4_pi_controller_hw_P.HILInitialize_OOExit && is_switching)) {
      num_final_other_outputs = 3U;
    } else {
      num_final_other_outputs = 0;
    }

    if (0
        || num_final_analog_outputs > 0
        || num_final_digital_outputs > 0
        || num_final_other_outputs > 0
        ) {
      /* Attempt to write the final outputs atomically (due to firmware issue in old Q2-USB). Otherwise write channels individually */
      result = hil_write(model4_pi_controller_hw_DW.HILInitialize_Card
                         , &model4_pi_controller_hw_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , NULL, 0
                         , &model4_pi_controller_hw_P.HILInitialize_DOChannels,
                         num_final_digital_outputs
                         , model4_pi_controller_hw_P.HILInitialize_OOChannels,
                         num_final_other_outputs
                         , &model4_pi_controller_hw_P.HILInitialize_AOFinal
                         , NULL
                         , (t_boolean *)
                         &model4_pi_controller_hw_P.HILInitialize_DOFinal
                         , model4_pi_controller_hw_P.HILInitialize_OOFinal
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog
            (model4_pi_controller_hw_DW.HILInitialize_Card,
             &model4_pi_controller_hw_P.HILInitialize_AOChannels,
             num_final_analog_outputs,
             &model4_pi_controller_hw_P.HILInitialize_AOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_digital_outputs > 0) {
          local_result = hil_write_digital
            (model4_pi_controller_hw_DW.HILInitialize_Card,
             &model4_pi_controller_hw_P.HILInitialize_DOChannels,
             num_final_digital_outputs, (t_boolean *)
             &model4_pi_controller_hw_P.HILInitialize_DOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_other_outputs > 0) {
          local_result = hil_write_other
            (model4_pi_controller_hw_DW.HILInitialize_Card,
             model4_pi_controller_hw_P.HILInitialize_OOChannels,
             num_final_other_outputs,
             model4_pi_controller_hw_P.HILInitialize_OOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(model4_pi_controller_hw_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(model4_pi_controller_hw_DW.HILInitialize_Card);
    hil_monitor_delete_all(model4_pi_controller_hw_DW.HILInitialize_Card);
    hil_close(model4_pi_controller_hw_DW.HILInitialize_Card);
    model4_pi_controller_hw_DW.HILInitialize_Card = NULL;
  }

  /* Terminate for ToFile: '<Root>/Log_Cmd' */
  {
    FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_Cmd_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "PI_cmd.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_cmd.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error reopening MAT-file PI_cmd.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 1 + 1,
           model4_pi_controller_hw_DW.Log_Cmd_IWORK.Count, "PI_cmd")) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error writing header for PI_cmd to MAT-file PI_cmd.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_cmd.mat");
        return;
      }

      model4_pi_controller_hw_DW.Log_Cmd_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/Log_CmdRaw' */
  {
    FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_CmdRaw_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "PI_cmd_raw.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_cmd_raw.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error reopening MAT-file PI_cmd_raw.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 1 + 1,
           model4_pi_controller_hw_DW.Log_CmdRaw_IWORK.Count, "PI_cmd_raw")) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error writing header for PI_cmd_raw to MAT-file PI_cmd_raw.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_cmd_raw.mat");
        return;
      }

      model4_pi_controller_hw_DW.Log_CmdRaw_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/Log_Error' */
  {
    FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_Error_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "PI_error.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_error.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error reopening MAT-file PI_error.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 1 + 1,
           model4_pi_controller_hw_DW.Log_Error_IWORK.Count, "PI_error")) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error writing header for PI_error to MAT-file PI_error.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_error.mat");
        return;
      }

      model4_pi_controller_hw_DW.Log_Error_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/Log_Ref' */
  {
    FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_Ref_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "PI_ref.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_ref.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error reopening MAT-file PI_ref.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 1 + 1,
           model4_pi_controller_hw_DW.Log_Ref_IWORK.Count, "PI_ref")) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error writing header for PI_ref to MAT-file PI_ref.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_ref.mat");
        return;
      }

      model4_pi_controller_hw_DW.Log_Ref_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/Log_SpeedFilt' */
  {
    FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_SpeedFilt_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "PI_speed_filt.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_speed_filt.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error reopening MAT-file PI_speed_filt.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 1 + 1,
           model4_pi_controller_hw_DW.Log_SpeedFilt_IWORK.Count, "PI_speed_filt"))
      {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error writing header for PI_speed_filt to MAT-file PI_speed_filt.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_speed_filt.mat");
        return;
      }

      model4_pi_controller_hw_DW.Log_SpeedFilt_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/Log_SpeedRaw' */
  {
    FILE *fp = (FILE *) model4_pi_controller_hw_DW.Log_SpeedRaw_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "PI_speed_raw.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_speed_raw.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error reopening MAT-file PI_speed_raw.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 1 + 1,
           model4_pi_controller_hw_DW.Log_SpeedRaw_IWORK.Count, "PI_speed_raw"))
      {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error writing header for PI_speed_raw to MAT-file PI_speed_raw.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(model4_pi_controller_hw_M,
                          "Error closing MAT-file PI_speed_raw.mat");
        return;
      }

      model4_pi_controller_hw_DW.Log_SpeedRaw_PWORK.FilePtr = (NULL);
    }
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  model4_pi_controller_hw_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  model4_pi_controller_hw_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  model4_pi_controller_hw_initialize();
}

void MdlTerminate(void)
{
  model4_pi_controller_hw_terminate();
}

/* Registration function */
RT_MODEL_model4_pi_controller_T *model4_pi_controller_hw(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)model4_pi_controller_hw_M, 0,
                sizeof(RT_MODEL_model4_pi_controller_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&model4_pi_controller_hw_M->solverInfo,
                          &model4_pi_controller_hw_M->Timing.simTimeStep);
    rtsiSetTPtr(&model4_pi_controller_hw_M->solverInfo, &rtmGetTPtr
                (model4_pi_controller_hw_M));
    rtsiSetStepSizePtr(&model4_pi_controller_hw_M->solverInfo,
                       &model4_pi_controller_hw_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&model4_pi_controller_hw_M->solverInfo,
                          (&rtmGetErrorStatus(model4_pi_controller_hw_M)));
    rtsiSetRTModelPtr(&model4_pi_controller_hw_M->solverInfo,
                      model4_pi_controller_hw_M);
  }

  rtsiSetSimTimeStep(&model4_pi_controller_hw_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&model4_pi_controller_hw_M->solverInfo,
    false);
  rtsiSetIsContModeFrozen(&model4_pi_controller_hw_M->solverInfo, false);
  rtsiSetSolverName(&model4_pi_controller_hw_M->solverInfo,"FixedStepDiscrete");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = model4_pi_controller_hw_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    model4_pi_controller_hw_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    model4_pi_controller_hw_M->Timing.sampleTimes =
      (&model4_pi_controller_hw_M->Timing.sampleTimesArray[0]);
    model4_pi_controller_hw_M->Timing.offsetTimes =
      (&model4_pi_controller_hw_M->Timing.offsetTimesArray[0]);

    /* task periods */
    model4_pi_controller_hw_M->Timing.sampleTimes[0] = (0.0);
    model4_pi_controller_hw_M->Timing.sampleTimes[1] = (0.002);

    /* task offsets */
    model4_pi_controller_hw_M->Timing.offsetTimes[0] = (0.0);
    model4_pi_controller_hw_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(model4_pi_controller_hw_M,
             &model4_pi_controller_hw_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = model4_pi_controller_hw_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    model4_pi_controller_hw_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(model4_pi_controller_hw_M, 10.0);
  model4_pi_controller_hw_M->Timing.stepSize0 = 0.002;
  model4_pi_controller_hw_M->Timing.stepSize1 = 0.002;

  /* External mode info */
  model4_pi_controller_hw_M->Sizes.checksums[0] = (4191016644U);
  model4_pi_controller_hw_M->Sizes.checksums[1] = (1303787585U);
  model4_pi_controller_hw_M->Sizes.checksums[2] = (2877655019U);
  model4_pi_controller_hw_M->Sizes.checksums[3] = (3143279650U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[5];
    model4_pi_controller_hw_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(model4_pi_controller_hw_M->extModeInfo,
      &model4_pi_controller_hw_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(model4_pi_controller_hw_M->extModeInfo,
                        model4_pi_controller_hw_M->Sizes.checksums);
    rteiSetTPtr(model4_pi_controller_hw_M->extModeInfo, rtmGetTPtr
                (model4_pi_controller_hw_M));
  }

  model4_pi_controller_hw_M->solverInfoPtr =
    (&model4_pi_controller_hw_M->solverInfo);
  model4_pi_controller_hw_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&model4_pi_controller_hw_M->solverInfo, 0.002);
  rtsiSetSolverMode(&model4_pi_controller_hw_M->solverInfo,
                    SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  model4_pi_controller_hw_M->blockIO = ((void *) &model4_pi_controller_hw_B);
  (void) memset(((void *) &model4_pi_controller_hw_B), 0,
                sizeof(B_model4_pi_controller_hw_T));

  /* parameters */
  model4_pi_controller_hw_M->defaultParam = ((real_T *)
    &model4_pi_controller_hw_P);

  /* states (dwork) */
  model4_pi_controller_hw_M->dwork = ((void *) &model4_pi_controller_hw_DW);
  (void) memset((void *)&model4_pi_controller_hw_DW, 0,
                sizeof(DW_model4_pi_controller_hw_T));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    model4_pi_controller_hw_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 20;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  model4_pi_controller_hw_M->Sizes.numContStates = (0);/* Number of continuous states */
  model4_pi_controller_hw_M->Sizes.numY = (0);/* Number of model outputs */
  model4_pi_controller_hw_M->Sizes.numU = (0);/* Number of model inputs */
  model4_pi_controller_hw_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  model4_pi_controller_hw_M->Sizes.numSampTimes = (2);/* Number of sample times */
  model4_pi_controller_hw_M->Sizes.numBlocks = (44);/* Number of blocks */
  model4_pi_controller_hw_M->Sizes.numBlockIO = (9);/* Number of block outputs */
  model4_pi_controller_hw_M->Sizes.numBlockPrms = (106);/* Sum of parameter "widths" */
  return model4_pi_controller_hw_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
