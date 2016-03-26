/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SimulationModel_sfun.h"
#include "c11_SimulationModel.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SimulationModel_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c11_debug_family_names[4] = { "nargin", "nargout", "u", "y"
};

/* Function Declarations */
static void initialize_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance);
static void initialize_params_c11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance);
static void enable_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance);
static void disable_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance);
static void c11_update_debugger_state_c11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance);
static void set_sim_state_c11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance, const mxArray *c11_st);
static void finalize_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance);
static void sf_gateway_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance);
static void mdl_start_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance);
static void initSimStructsc11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c11_machineNumber, uint32_T
  c11_chartNumber, uint32_T c11_instanceNumber);
static const mxArray *c11_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static c11_bus_Robot c11_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_y, const char_T *c11_identifier);
static c11_bus_Robot c11_b_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId);
static void c11_c_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId,
  int32_T c11_b_y[2]);
static void c11_d_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId,
  int16_T c11_b_y[6]);
static void c11_e_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId,
  int8_T c11_b_y[6]);
static c11_EPosition c11_f_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId);
static void c11_g_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId,
  int8_T c11_b_y[3]);
static c11_bus_RemainingPosition c11_h_emlrt_marshallIn
  (SFc11_SimulationModelInstanceStruct *chartInstance, const mxArray *c11_b_u,
   const emlrtMsgIdentifier *c11_parentId);
static real32_T c11_i_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId);
static void c11_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_b_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static real_T c11_j_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId);
static void c11_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_c_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData);
static int32_T c11_k_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId);
static void c11_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData);
static const mxArray *c11_u_bus_io(void *chartInstanceVoid, void *c11_pData);
static uint8_T c11_l_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_is_active_c11_SimulationModel, const
  char_T *c11_identifier);
static uint8_T c11_m_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId);
static void init_dsm_address_info(SFc11_SimulationModelInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc11_SimulationModelInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc11_SimulationModel(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c11_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c11_is_active_c11_SimulationModel = 0U;
}

static void initialize_params_c11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c11_update_debugger_state_c11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance)
{
  const mxArray *c11_st;
  const mxArray *c11_b_y = NULL;
  const mxArray *c11_c_y = NULL;
  int32_T c11_i0;
  int32_T c11_b_u[2];
  const mxArray *c11_d_y = NULL;
  int32_T c11_i1;
  int16_T c11_c_u[6];
  const mxArray *c11_e_y = NULL;
  int32_T c11_i2;
  int8_T c11_d_u[6];
  const mxArray *c11_f_y = NULL;
  c11_EPosition c11_e_u;
  const mxArray *c11_g_y = NULL;
  static const int32_T c11_enumValues[4] = { 0, 1, 2, 3 };

  static const char * c11_enumNames[4] = { "NO", "LEFT", "MIDDLE", "RIGHT" };

  int32_T c11_f_u;
  const mxArray *c11_h_y = NULL;
  const mxArray *c11_m0 = NULL;
  int32_T c11_i3;
  int8_T c11_g_u[3];
  const mxArray *c11_i_y = NULL;
  c11_bus_RemainingPosition c11_h_u;
  const mxArray *c11_j_y = NULL;
  real32_T c11_i_u;
  const mxArray *c11_k_y = NULL;
  real32_T c11_j_u;
  const mxArray *c11_l_y = NULL;
  uint8_T c11_hoistedGlobal;
  uint8_T c11_k_u;
  const mxArray *c11_m_y = NULL;
  c11_st = NULL;
  c11_st = NULL;
  c11_b_y = NULL;
  sf_mex_assign(&c11_b_y, sf_mex_createcellmatrix(2, 1), false);
  c11_c_y = NULL;
  sf_mex_assign(&c11_c_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c11_i0 = 0; c11_i0 < 2; c11_i0++) {
    c11_b_u[c11_i0] = ((int32_T *)&((char_T *)chartInstance->c11_y)[0])[c11_i0];
  }

  c11_d_y = NULL;
  sf_mex_assign(&c11_d_y, sf_mex_create("y", c11_b_u, 6, 0U, 1U, 0U, 1, 2),
                false);
  sf_mex_addfield(c11_c_y, c11_d_y, "EncodersCount", "EncodersCount", 0);
  for (c11_i1 = 0; c11_i1 < 6; c11_i1++) {
    c11_c_u[c11_i1] = ((int16_T *)&((char_T *)chartInstance->c11_y)[8])[c11_i1];
  }

  c11_e_y = NULL;
  sf_mex_assign(&c11_e_y, sf_mex_create("y", c11_c_u, 4, 0U, 1U, 0U, 1, 6),
                false);
  sf_mex_addfield(c11_c_y, c11_e_y, "Distance", "Distance", 0);
  for (c11_i2 = 0; c11_i2 < 6; c11_i2++) {
    c11_d_u[c11_i2] = ((int8_T *)&((char_T *)chartInstance->c11_y)[20])[c11_i2];
  }

  c11_f_y = NULL;
  sf_mex_assign(&c11_f_y, sf_mex_create("y", c11_d_u, 2, 0U, 1U, 0U, 1, 6),
                false);
  sf_mex_addfield(c11_c_y, c11_f_y, "Bearing", "Bearing", 0);
  c11_e_u = *(c11_EPosition *)&((char_T *)chartInstance->c11_y)[28];
  c11_g_y = NULL;
  sf_mex_check_enum("EPosition", 4, c11_enumNames, c11_enumValues);
  c11_f_u = (int32_T)c11_e_u;
  c11_h_y = NULL;
  sf_mex_assign(&c11_h_y, sf_mex_create("y", &c11_f_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c11_m0, c11_h_y, false);
  sf_mex_assign(&c11_g_y, sf_mex_create_enum("EPosition", c11_m0), false);
  sf_mex_destroy(&c11_m0);
  sf_mex_addfield(c11_c_y, c11_g_y, "ScannerStatus", "ScannerStatus", 0);
  for (c11_i3 = 0; c11_i3 < 3; c11_i3++) {
    c11_g_u[c11_i3] = ((int8_T *)&((char_T *)chartInstance->c11_y)[32])[c11_i3];
  }

  c11_i_y = NULL;
  sf_mex_assign(&c11_i_y, sf_mex_create("y", c11_g_u, 2, 0U, 1U, 0U, 2, 1, 3),
                false);
  sf_mex_addfield(c11_c_y, c11_i_y, "ScannerValues", "ScannerValues", 0);
  c11_h_u.Angle = *(real32_T *)&((char_T *)(c11_bus_RemainingPosition *)
    &((char_T *)chartInstance->c11_y)[40])[0];
  c11_h_u.Distance = *(real32_T *)&((char_T *)(c11_bus_RemainingPosition *)
    &((char_T *)chartInstance->c11_y)[40])[4];
  c11_j_y = NULL;
  sf_mex_assign(&c11_j_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c11_i_u = c11_h_u.Angle;
  c11_k_y = NULL;
  sf_mex_assign(&c11_k_y, sf_mex_create("y", &c11_i_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c11_j_y, c11_k_y, "Angle", "Angle", 0);
  c11_j_u = c11_h_u.Distance;
  c11_l_y = NULL;
  sf_mex_assign(&c11_l_y, sf_mex_create("y", &c11_j_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c11_j_y, c11_l_y, "Distance", "Distance", 0);
  sf_mex_addfield(c11_c_y, c11_j_y, "Remaining", "Remaining", 0);
  sf_mex_setcell(c11_b_y, 0, c11_c_y);
  c11_hoistedGlobal = chartInstance->c11_is_active_c11_SimulationModel;
  c11_k_u = c11_hoistedGlobal;
  c11_m_y = NULL;
  sf_mex_assign(&c11_m_y, sf_mex_create("y", &c11_k_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c11_b_y, 1, c11_m_y);
  sf_mex_assign(&c11_st, c11_b_y, false);
  return c11_st;
}

static void set_sim_state_c11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance, const mxArray *c11_st)
{
  const mxArray *c11_b_u;
  c11_bus_Robot c11_r0;
  int32_T c11_i4;
  int32_T c11_i5;
  int32_T c11_i6;
  int32_T c11_i7;
  chartInstance->c11_doneDoubleBufferReInit = true;
  c11_b_u = sf_mex_dup(c11_st);
  c11_r0 = c11_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("y",
    c11_b_u, 0)), "y");
  for (c11_i4 = 0; c11_i4 < 2; c11_i4++) {
    ((int32_T *)&((char_T *)chartInstance->c11_y)[0])[c11_i4] =
      c11_r0.EncodersCount[c11_i4];
  }

  for (c11_i5 = 0; c11_i5 < 6; c11_i5++) {
    ((int16_T *)&((char_T *)chartInstance->c11_y)[8])[c11_i5] =
      c11_r0.Distance[c11_i5];
  }

  for (c11_i6 = 0; c11_i6 < 6; c11_i6++) {
    ((int8_T *)&((char_T *)chartInstance->c11_y)[20])[c11_i6] =
      c11_r0.Bearing[c11_i6];
  }

  *(c11_EPosition *)&((char_T *)chartInstance->c11_y)[28] = c11_r0.ScannerStatus;
  for (c11_i7 = 0; c11_i7 < 3; c11_i7++) {
    ((int8_T *)&((char_T *)chartInstance->c11_y)[32])[c11_i7] =
      c11_r0.ScannerValues[c11_i7];
  }

  *(real32_T *)&((char_T *)(c11_bus_RemainingPosition *)&((char_T *)
    chartInstance->c11_y)[40])[0] = c11_r0.Remaining.Angle;
  *(real32_T *)&((char_T *)(c11_bus_RemainingPosition *)&((char_T *)
    chartInstance->c11_y)[40])[4] = c11_r0.Remaining.Distance;
  chartInstance->c11_is_active_c11_SimulationModel = c11_l_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c11_SimulationModel",
       c11_b_u, 1)), "is_active_c11_SimulationModel");
  sf_mex_destroy(&c11_b_u);
  c11_update_debugger_state_c11_SimulationModel(chartInstance);
  sf_mex_destroy(&c11_st);
}

static void finalize_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance)
{
  int32_T c11_i8;
  c11_bus_Robot c11_b_u;
  int32_T c11_i9;
  int32_T c11_i10;
  int32_T c11_i11;
  uint32_T c11_debug_family_var_map[4];
  real_T c11_nargin = 1.0;
  real_T c11_nargout = 1.0;
  c11_bus_Robot c11_b_y;
  int32_T c11_i12;
  static char_T c11_cv0[13] = { 'E', 'n', 'c', 'o', 'd', 'e', 'r', 's', 'C', 'o',
    'u', 'n', 't' };

  c11_cell_0 c11_c_u;
  int32_T c11_i13;
  static char_T c11_cv1[8] = { 'D', 'i', 's', 't', 'a', 'n', 'c', 'e' };

  int32_T c11_i14;
  static char_T c11_cv2[7] = { 'B', 'e', 'a', 'r', 'i', 'n', 'g' };

  int32_T c11_i15;
  static char_T c11_cv3[13] = { 'S', 'c', 'a', 'n', 'n', 'e', 'r', 'S', 't', 'a',
    't', 'u', 's' };

  int32_T c11_i16;
  static char_T c11_cv4[13] = { 'S', 'c', 'a', 'n', 'n', 'e', 'r', 'V', 'a', 'l',
    'u', 'e', 's' };

  int32_T c11_i17;
  static char_T c11_cv5[9] = { 'R', 'e', 'm', 'a', 'i', 'n', 'i', 'n', 'g' };

  const mxArray *c11_c_y = NULL;
  int32_T c11_i18;
  int32_T c11_iv0[2];
  int32_T c11_i19;
  char_T c11_d_u[13];
  const mxArray *c11_d_y = NULL;
  int32_T c11_i20;
  char_T c11_e_u[8];
  const mxArray *c11_e_y = NULL;
  int32_T c11_i21;
  char_T c11_f_u[7];
  const mxArray *c11_f_y = NULL;
  int32_T c11_i22;
  const mxArray *c11_g_y = NULL;
  int32_T c11_i23;
  const mxArray *c11_h_y = NULL;
  int32_T c11_i24;
  char_T c11_g_u[9];
  const mxArray *c11_i_y = NULL;
  int32_T c11_i25;
  int32_T c11_i26;
  int32_T c11_i27;
  int32_T c11_i28;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 10U, chartInstance->c11_sfEvent);
  chartInstance->c11_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 10U, chartInstance->c11_sfEvent);
  for (c11_i8 = 0; c11_i8 < 2; c11_i8++) {
    c11_b_u.EncodersCount[c11_i8] = ((int32_T *)&((char_T *)chartInstance->c11_u)
      [0])[c11_i8];
  }

  for (c11_i9 = 0; c11_i9 < 6; c11_i9++) {
    c11_b_u.Distance[c11_i9] = ((int16_T *)&((char_T *)chartInstance->c11_u)[8])
      [c11_i9];
  }

  for (c11_i10 = 0; c11_i10 < 6; c11_i10++) {
    c11_b_u.Bearing[c11_i10] = ((int8_T *)&((char_T *)chartInstance->c11_u)[20])
      [c11_i10];
  }

  c11_b_u.ScannerStatus = *(c11_EPosition *)&((char_T *)chartInstance->c11_u)[28];
  for (c11_i11 = 0; c11_i11 < 3; c11_i11++) {
    c11_b_u.ScannerValues[c11_i11] = ((int8_T *)&((char_T *)chartInstance->c11_u)
      [32])[c11_i11];
  }

  c11_b_u.Remaining.Angle = *(real32_T *)&((char_T *)(c11_bus_RemainingPosition *)
    &((char_T *)chartInstance->c11_u)[40])[0];
  c11_b_u.Remaining.Distance = *(real32_T *)&((char_T *)
    (c11_bus_RemainingPosition *)&((char_T *)chartInstance->c11_u)[40])[4];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c11_debug_family_names,
    c11_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c11_nargin, 0U, c11_b_sf_marshallOut,
    c11_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c11_nargout, 1U, c11_b_sf_marshallOut,
    c11_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c11_b_u, 2U, c11_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c11_b_y, 3U, c11_sf_marshallOut,
    c11_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 3);
  for (c11_i12 = 0; c11_i12 < 13; c11_i12++) {
    c11_c_u.f1[c11_i12] = c11_cv0[c11_i12];
  }

  for (c11_i13 = 0; c11_i13 < 8; c11_i13++) {
    c11_c_u.f2[c11_i13] = c11_cv1[c11_i13];
  }

  for (c11_i14 = 0; c11_i14 < 7; c11_i14++) {
    c11_c_u.f3[c11_i14] = c11_cv2[c11_i14];
  }

  for (c11_i15 = 0; c11_i15 < 13; c11_i15++) {
    c11_c_u.f4[c11_i15] = c11_cv3[c11_i15];
  }

  for (c11_i16 = 0; c11_i16 < 13; c11_i16++) {
    c11_c_u.f5[c11_i16] = c11_cv4[c11_i16];
  }

  for (c11_i17 = 0; c11_i17 < 9; c11_i17++) {
    c11_c_u.f6[c11_i17] = c11_cv5[c11_i17];
  }

  c11_c_y = NULL;
  for (c11_i18 = 0; c11_i18 < 2; c11_i18++) {
    c11_iv0[c11_i18] = 6 + -5 * c11_i18;
  }

  sf_mex_assign(&c11_c_y, sf_mex_createcellarray(2, c11_iv0), false);
  for (c11_i19 = 0; c11_i19 < 13; c11_i19++) {
    c11_d_u[c11_i19] = c11_c_u.f1[c11_i19];
  }

  c11_d_y = NULL;
  sf_mex_assign(&c11_d_y, sf_mex_create("y", c11_d_u, 10, 0U, 1U, 0U, 2, 1, 13),
                false);
  sf_mex_setcell(c11_c_y, 0, c11_d_y);
  for (c11_i20 = 0; c11_i20 < 8; c11_i20++) {
    c11_e_u[c11_i20] = c11_c_u.f2[c11_i20];
  }

  c11_e_y = NULL;
  sf_mex_assign(&c11_e_y, sf_mex_create("y", c11_e_u, 10, 0U, 1U, 0U, 2, 1, 8),
                false);
  sf_mex_setcell(c11_c_y, 1, c11_e_y);
  for (c11_i21 = 0; c11_i21 < 7; c11_i21++) {
    c11_f_u[c11_i21] = c11_c_u.f3[c11_i21];
  }

  c11_f_y = NULL;
  sf_mex_assign(&c11_f_y, sf_mex_create("y", c11_f_u, 10, 0U, 1U, 0U, 2, 1, 7),
                false);
  sf_mex_setcell(c11_c_y, 2, c11_f_y);
  for (c11_i22 = 0; c11_i22 < 13; c11_i22++) {
    c11_d_u[c11_i22] = c11_c_u.f4[c11_i22];
  }

  c11_g_y = NULL;
  sf_mex_assign(&c11_g_y, sf_mex_create("y", c11_d_u, 10, 0U, 1U, 0U, 2, 1, 13),
                false);
  sf_mex_setcell(c11_c_y, 3, c11_g_y);
  for (c11_i23 = 0; c11_i23 < 13; c11_i23++) {
    c11_d_u[c11_i23] = c11_c_u.f5[c11_i23];
  }

  c11_h_y = NULL;
  sf_mex_assign(&c11_h_y, sf_mex_create("y", c11_d_u, 10, 0U, 1U, 0U, 2, 1, 13),
                false);
  sf_mex_setcell(c11_c_y, 4, c11_h_y);
  for (c11_i24 = 0; c11_i24 < 9; c11_i24++) {
    c11_g_u[c11_i24] = c11_c_u.f6[c11_i24];
  }

  c11_i_y = NULL;
  sf_mex_assign(&c11_i_y, sf_mex_create("y", c11_g_u, 10, 0U, 1U, 0U, 2, 1, 9),
                false);
  sf_mex_setcell(c11_c_y, 5, c11_i_y);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14, c11_c_y);
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, 5);
  c11_b_y = c11_b_u;
  _SFD_EML_CALL(0U, chartInstance->c11_sfEvent, -5);
  _SFD_SYMBOL_SCOPE_POP();
  for (c11_i25 = 0; c11_i25 < 2; c11_i25++) {
    ((int32_T *)&((char_T *)chartInstance->c11_y)[0])[c11_i25] =
      c11_b_y.EncodersCount[c11_i25];
  }

  for (c11_i26 = 0; c11_i26 < 6; c11_i26++) {
    ((int16_T *)&((char_T *)chartInstance->c11_y)[8])[c11_i26] =
      c11_b_y.Distance[c11_i26];
  }

  for (c11_i27 = 0; c11_i27 < 6; c11_i27++) {
    ((int8_T *)&((char_T *)chartInstance->c11_y)[20])[c11_i27] =
      c11_b_y.Bearing[c11_i27];
  }

  *(c11_EPosition *)&((char_T *)chartInstance->c11_y)[28] =
    c11_b_y.ScannerStatus;
  for (c11_i28 = 0; c11_i28 < 3; c11_i28++) {
    ((int8_T *)&((char_T *)chartInstance->c11_y)[32])[c11_i28] =
      c11_b_y.ScannerValues[c11_i28];
  }

  *(real32_T *)&((char_T *)(c11_bus_RemainingPosition *)&((char_T *)
    chartInstance->c11_y)[40])[0] = c11_b_y.Remaining.Angle;
  *(real32_T *)&((char_T *)(c11_bus_RemainingPosition *)&((char_T *)
    chartInstance->c11_y)[40])[4] = c11_b_y.Remaining.Distance;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 10U, chartInstance->c11_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SimulationModelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void mdl_start_c11_SimulationModel(SFc11_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc11_SimulationModel
  (SFc11_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c11_machineNumber, uint32_T
  c11_chartNumber, uint32_T c11_instanceNumber)
{
  (void)c11_machineNumber;
  (void)c11_chartNumber;
  (void)c11_instanceNumber;
}

static const mxArray *c11_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData;
  c11_bus_Robot c11_b_u;
  const mxArray *c11_b_y = NULL;
  int32_T c11_i29;
  int32_T c11_c_u[2];
  const mxArray *c11_c_y = NULL;
  int32_T c11_i30;
  int16_T c11_d_u[6];
  const mxArray *c11_d_y = NULL;
  int32_T c11_i31;
  int8_T c11_e_u[6];
  const mxArray *c11_e_y = NULL;
  c11_EPosition c11_f_u;
  const mxArray *c11_f_y = NULL;
  static const int32_T c11_enumValues[4] = { 0, 1, 2, 3 };

  static const char * c11_enumNames[4] = { "NO", "LEFT", "MIDDLE", "RIGHT" };

  int32_T c11_g_u;
  const mxArray *c11_g_y = NULL;
  const mxArray *c11_m1 = NULL;
  int32_T c11_i32;
  int8_T c11_h_u[3];
  const mxArray *c11_h_y = NULL;
  c11_bus_RemainingPosition c11_i_u;
  const mxArray *c11_i_y = NULL;
  real32_T c11_j_u;
  const mxArray *c11_j_y = NULL;
  real32_T c11_k_u;
  const mxArray *c11_k_y = NULL;
  SFc11_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc11_SimulationModelInstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_mxArrayOutData = NULL;
  c11_b_u = *(c11_bus_Robot *)c11_inData;
  c11_b_y = NULL;
  sf_mex_assign(&c11_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c11_i29 = 0; c11_i29 < 2; c11_i29++) {
    c11_c_u[c11_i29] = c11_b_u.EncodersCount[c11_i29];
  }

  c11_c_y = NULL;
  sf_mex_assign(&c11_c_y, sf_mex_create("y", c11_c_u, 6, 0U, 1U, 0U, 1, 2),
                false);
  sf_mex_addfield(c11_b_y, c11_c_y, "EncodersCount", "EncodersCount", 0);
  for (c11_i30 = 0; c11_i30 < 6; c11_i30++) {
    c11_d_u[c11_i30] = c11_b_u.Distance[c11_i30];
  }

  c11_d_y = NULL;
  sf_mex_assign(&c11_d_y, sf_mex_create("y", c11_d_u, 4, 0U, 1U, 0U, 1, 6),
                false);
  sf_mex_addfield(c11_b_y, c11_d_y, "Distance", "Distance", 0);
  for (c11_i31 = 0; c11_i31 < 6; c11_i31++) {
    c11_e_u[c11_i31] = c11_b_u.Bearing[c11_i31];
  }

  c11_e_y = NULL;
  sf_mex_assign(&c11_e_y, sf_mex_create("y", c11_e_u, 2, 0U, 1U, 0U, 1, 6),
                false);
  sf_mex_addfield(c11_b_y, c11_e_y, "Bearing", "Bearing", 0);
  c11_f_u = c11_b_u.ScannerStatus;
  c11_f_y = NULL;
  sf_mex_check_enum("EPosition", 4, c11_enumNames, c11_enumValues);
  c11_g_u = (int32_T)c11_f_u;
  c11_g_y = NULL;
  sf_mex_assign(&c11_g_y, sf_mex_create("y", &c11_g_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c11_m1, c11_g_y, false);
  sf_mex_assign(&c11_f_y, sf_mex_create_enum("EPosition", c11_m1), false);
  sf_mex_destroy(&c11_m1);
  sf_mex_addfield(c11_b_y, c11_f_y, "ScannerStatus", "ScannerStatus", 0);
  for (c11_i32 = 0; c11_i32 < 3; c11_i32++) {
    c11_h_u[c11_i32] = c11_b_u.ScannerValues[c11_i32];
  }

  c11_h_y = NULL;
  sf_mex_assign(&c11_h_y, sf_mex_create("y", c11_h_u, 2, 0U, 1U, 0U, 2, 1, 3),
                false);
  sf_mex_addfield(c11_b_y, c11_h_y, "ScannerValues", "ScannerValues", 0);
  c11_i_u = c11_b_u.Remaining;
  c11_i_y = NULL;
  sf_mex_assign(&c11_i_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c11_j_u = c11_i_u.Angle;
  c11_j_y = NULL;
  sf_mex_assign(&c11_j_y, sf_mex_create("y", &c11_j_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c11_i_y, c11_j_y, "Angle", "Angle", 0);
  c11_k_u = c11_i_u.Distance;
  c11_k_y = NULL;
  sf_mex_assign(&c11_k_y, sf_mex_create("y", &c11_k_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c11_i_y, c11_k_y, "Distance", "Distance", 0);
  sf_mex_addfield(c11_b_y, c11_i_y, "Remaining", "Remaining", 0);
  sf_mex_assign(&c11_mxArrayOutData, c11_b_y, false);
  return c11_mxArrayOutData;
}

static c11_bus_Robot c11_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_y, const char_T *c11_identifier)
{
  c11_bus_Robot c11_c_y;
  emlrtMsgIdentifier c11_thisId;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_thisId.bParentIsCell = false;
  c11_c_y = c11_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_y),
    &c11_thisId);
  sf_mex_destroy(&c11_b_y);
  return c11_c_y;
}

static c11_bus_Robot c11_b_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId)
{
  c11_bus_Robot c11_b_y;
  emlrtMsgIdentifier c11_thisId;
  static const char * c11_fieldNames[6] = { "EncodersCount", "Distance",
    "Bearing", "ScannerStatus", "ScannerValues", "Remaining" };

  c11_thisId.fParent = c11_parentId;
  c11_thisId.bParentIsCell = false;
  sf_mex_check_struct(c11_parentId, c11_b_u, 6, c11_fieldNames, 0U, NULL);
  c11_thisId.fIdentifier = "EncodersCount";
  c11_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c11_b_u,
    "EncodersCount", "EncodersCount", 0)), &c11_thisId, c11_b_y.EncodersCount);
  c11_thisId.fIdentifier = "Distance";
  c11_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c11_b_u,
    "Distance", "Distance", 0)), &c11_thisId, c11_b_y.Distance);
  c11_thisId.fIdentifier = "Bearing";
  c11_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c11_b_u,
    "Bearing", "Bearing", 0)), &c11_thisId, c11_b_y.Bearing);
  c11_thisId.fIdentifier = "ScannerStatus";
  c11_b_y.ScannerStatus = c11_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c11_b_u, "ScannerStatus", "ScannerStatus", 0)), &c11_thisId);
  c11_thisId.fIdentifier = "ScannerValues";
  c11_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c11_b_u,
    "ScannerValues", "ScannerValues", 0)), &c11_thisId, c11_b_y.ScannerValues);
  c11_thisId.fIdentifier = "Remaining";
  c11_b_y.Remaining = c11_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c11_b_u, "Remaining", "Remaining", 0)), &c11_thisId);
  sf_mex_destroy(&c11_b_u);
  return c11_b_y;
}

static void c11_c_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId,
  int32_T c11_b_y[2])
{
  int32_T c11_iv1[2];
  int32_T c11_i33;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_b_u), c11_iv1, 1, 6, 0U, 1, 0U, 1,
                2);
  for (c11_i33 = 0; c11_i33 < 2; c11_i33++) {
    c11_b_y[c11_i33] = c11_iv1[c11_i33];
  }

  sf_mex_destroy(&c11_b_u);
}

static void c11_d_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId,
  int16_T c11_b_y[6])
{
  int16_T c11_iv2[6];
  int32_T c11_i34;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_b_u), c11_iv2, 1, 4, 0U, 1, 0U, 1,
                6);
  for (c11_i34 = 0; c11_i34 < 6; c11_i34++) {
    c11_b_y[c11_i34] = c11_iv2[c11_i34];
  }

  sf_mex_destroy(&c11_b_u);
}

static void c11_e_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId,
  int8_T c11_b_y[6])
{
  int8_T c11_iv3[6];
  int32_T c11_i35;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_b_u), c11_iv3, 1, 2, 0U, 1, 0U, 1,
                6);
  for (c11_i35 = 0; c11_i35 < 6; c11_i35++) {
    c11_b_y[c11_i35] = c11_iv3[c11_i35];
  }

  sf_mex_destroy(&c11_b_u);
}

static c11_EPosition c11_f_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId)
{
  c11_EPosition c11_b_y;
  static const int32_T c11_enumValues[4] = { 0, 1, 2, 3 };

  static const char * c11_enumNames[4] = { "NO", "LEFT", "MIDDLE", "RIGHT" };

  (void)chartInstance;
  sf_mex_check_enum("EPosition", 4, c11_enumNames, c11_enumValues);
  sf_mex_check_builtin(c11_parentId, c11_b_u, "EPosition", 0, 0U, NULL);
  c11_b_y = (c11_EPosition)sf_mex_get_enum_element(c11_b_u, 0);
  sf_mex_destroy(&c11_b_u);
  return c11_b_y;
}

static void c11_g_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId,
  int8_T c11_b_y[3])
{
  int8_T c11_iv4[3];
  int32_T c11_i36;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_b_u), c11_iv4, 1, 2, 0U, 1, 0U, 2,
                1, 3);
  for (c11_i36 = 0; c11_i36 < 3; c11_i36++) {
    c11_b_y[c11_i36] = c11_iv4[c11_i36];
  }

  sf_mex_destroy(&c11_b_u);
}

static c11_bus_RemainingPosition c11_h_emlrt_marshallIn
  (SFc11_SimulationModelInstanceStruct *chartInstance, const mxArray *c11_b_u,
   const emlrtMsgIdentifier *c11_parentId)
{
  c11_bus_RemainingPosition c11_b_y;
  emlrtMsgIdentifier c11_thisId;
  static const char * c11_fieldNames[2] = { "Angle", "Distance" };

  c11_thisId.fParent = c11_parentId;
  c11_thisId.bParentIsCell = false;
  sf_mex_check_struct(c11_parentId, c11_b_u, 2, c11_fieldNames, 0U, NULL);
  c11_thisId.fIdentifier = "Angle";
  c11_b_y.Angle = c11_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c11_b_u, "Angle", "Angle", 0)), &c11_thisId);
  c11_thisId.fIdentifier = "Distance";
  c11_b_y.Distance = c11_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c11_b_u, "Distance", "Distance", 0)), &c11_thisId);
  sf_mex_destroy(&c11_b_u);
  return c11_b_y;
}

static real32_T c11_i_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId)
{
  real32_T c11_b_y;
  real32_T c11_f0;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_b_u), &c11_f0, 1, 1, 0U, 0, 0U, 0);
  c11_b_y = c11_f0;
  sf_mex_destroy(&c11_b_u);
  return c11_b_y;
}

static void c11_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_b_y;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  c11_bus_Robot c11_c_y;
  SFc11_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc11_SimulationModelInstanceStruct *)chartInstanceVoid;
  c11_b_y = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_thisId.bParentIsCell = false;
  c11_c_y = c11_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_y),
    &c11_thisId);
  sf_mex_destroy(&c11_b_y);
  *(c11_bus_Robot *)c11_outData = c11_c_y;
  sf_mex_destroy(&c11_mxArrayInData);
}

static const mxArray *c11_b_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  real_T c11_b_u;
  const mxArray *c11_b_y = NULL;
  SFc11_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc11_SimulationModelInstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_b_u = *(real_T *)c11_inData;
  c11_b_y = NULL;
  sf_mex_assign(&c11_b_y, sf_mex_create("y", &c11_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_b_y, false);
  return c11_mxArrayOutData;
}

static real_T c11_j_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId)
{
  real_T c11_b_y;
  real_T c11_d0;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_b_u), &c11_d0, 1, 0, 0U, 0, 0U, 0);
  c11_b_y = c11_d0;
  sf_mex_destroy(&c11_b_u);
  return c11_b_y;
}

static void c11_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_nargout;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  real_T c11_b_y;
  SFc11_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc11_SimulationModelInstanceStruct *)chartInstanceVoid;
  c11_nargout = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_thisId.bParentIsCell = false;
  c11_b_y = c11_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_nargout),
    &c11_thisId);
  sf_mex_destroy(&c11_nargout);
  *(real_T *)c11_outData = c11_b_y;
  sf_mex_destroy(&c11_mxArrayInData);
}

const mxArray *sf_c11_SimulationModel_get_eml_resolved_functions_info(void)
{
  const mxArray *c11_nameCaptureInfo = NULL;
  c11_nameCaptureInfo = NULL;
  sf_mex_assign(&c11_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c11_nameCaptureInfo;
}

static const mxArray *c11_c_sf_marshallOut(void *chartInstanceVoid, void
  *c11_inData)
{
  const mxArray *c11_mxArrayOutData = NULL;
  int32_T c11_b_u;
  const mxArray *c11_b_y = NULL;
  SFc11_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc11_SimulationModelInstanceStruct *)chartInstanceVoid;
  c11_mxArrayOutData = NULL;
  c11_b_u = *(int32_T *)c11_inData;
  c11_b_y = NULL;
  sf_mex_assign(&c11_b_y, sf_mex_create("y", &c11_b_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c11_mxArrayOutData, c11_b_y, false);
  return c11_mxArrayOutData;
}

static int32_T c11_k_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId)
{
  int32_T c11_b_y;
  int32_T c11_i37;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_b_u), &c11_i37, 1, 6, 0U, 0, 0U, 0);
  c11_b_y = c11_i37;
  sf_mex_destroy(&c11_b_u);
  return c11_b_y;
}

static void c11_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c11_mxArrayInData, const char_T *c11_varName, void *c11_outData)
{
  const mxArray *c11_b_sfEvent;
  const char_T *c11_identifier;
  emlrtMsgIdentifier c11_thisId;
  int32_T c11_b_y;
  SFc11_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc11_SimulationModelInstanceStruct *)chartInstanceVoid;
  c11_b_sfEvent = sf_mex_dup(c11_mxArrayInData);
  c11_identifier = c11_varName;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_thisId.bParentIsCell = false;
  c11_b_y = c11_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c11_b_sfEvent),
    &c11_thisId);
  sf_mex_destroy(&c11_b_sfEvent);
  *(int32_T *)c11_outData = c11_b_y;
  sf_mex_destroy(&c11_mxArrayInData);
}

static const mxArray *c11_u_bus_io(void *chartInstanceVoid, void *c11_pData)
{
  const mxArray *c11_mxVal = NULL;
  int32_T c11_i38;
  c11_bus_Robot c11_tmp;
  int32_T c11_i39;
  int32_T c11_i40;
  int32_T c11_i41;
  SFc11_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc11_SimulationModelInstanceStruct *)chartInstanceVoid;
  c11_mxVal = NULL;
  for (c11_i38 = 0; c11_i38 < 2; c11_i38++) {
    c11_tmp.EncodersCount[c11_i38] = ((int32_T *)&((char_T *)(c11_bus_Robot *)
      c11_pData)[0])[c11_i38];
  }

  for (c11_i39 = 0; c11_i39 < 6; c11_i39++) {
    c11_tmp.Distance[c11_i39] = ((int16_T *)&((char_T *)(c11_bus_Robot *)
      c11_pData)[8])[c11_i39];
  }

  for (c11_i40 = 0; c11_i40 < 6; c11_i40++) {
    c11_tmp.Bearing[c11_i40] = ((int8_T *)&((char_T *)(c11_bus_Robot *)c11_pData)
      [20])[c11_i40];
  }

  c11_tmp.ScannerStatus = *(c11_EPosition *)&((char_T *)(c11_bus_Robot *)
    c11_pData)[28];
  for (c11_i41 = 0; c11_i41 < 3; c11_i41++) {
    c11_tmp.ScannerValues[c11_i41] = ((int8_T *)&((char_T *)(c11_bus_Robot *)
      c11_pData)[32])[c11_i41];
  }

  c11_tmp.Remaining.Angle = *(real32_T *)&((char_T *)(c11_bus_RemainingPosition *)
    &((char_T *)(c11_bus_Robot *)c11_pData)[40])[0];
  c11_tmp.Remaining.Distance = *(real32_T *)&((char_T *)
    (c11_bus_RemainingPosition *)&((char_T *)(c11_bus_Robot *)c11_pData)[40])[4];
  sf_mex_assign(&c11_mxVal, c11_sf_marshallOut(chartInstance, &c11_tmp), false);
  return c11_mxVal;
}

static uint8_T c11_l_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_is_active_c11_SimulationModel, const
  char_T *c11_identifier)
{
  uint8_T c11_b_y;
  emlrtMsgIdentifier c11_thisId;
  c11_thisId.fIdentifier = c11_identifier;
  c11_thisId.fParent = NULL;
  c11_thisId.bParentIsCell = false;
  c11_b_y = c11_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c11_b_is_active_c11_SimulationModel), &c11_thisId);
  sf_mex_destroy(&c11_b_is_active_c11_SimulationModel);
  return c11_b_y;
}

static uint8_T c11_m_emlrt_marshallIn(SFc11_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c11_b_u, const emlrtMsgIdentifier *c11_parentId)
{
  uint8_T c11_b_y;
  uint8_T c11_u0;
  (void)chartInstance;
  sf_mex_import(c11_parentId, sf_mex_dup(c11_b_u), &c11_u0, 1, 3, 0U, 0, 0U, 0);
  c11_b_y = c11_u0;
  sf_mex_destroy(&c11_b_u);
  return c11_b_y;
}

static void init_dsm_address_info(SFc11_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc11_SimulationModelInstanceStruct
  *chartInstance)
{
  chartInstance->c11_u = (c11_bus_Robot *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c11_y = (c11_bus_Robot *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c11_SimulationModel_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1174934526U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1422371362U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1054707489U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(12195726U);
}

mxArray* sf_c11_SimulationModel_get_post_codegen_info(void);
mxArray *sf_c11_SimulationModel_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("tOhOLRdS2nHnZnJbNdbwhC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c11_SimulationModel_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c11_SimulationModel_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c11_SimulationModel_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c11_SimulationModel_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c11_SimulationModel_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c11_SimulationModel(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c11_SimulationModel\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c11_SimulationModel_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc11_SimulationModelInstanceStruct *chartInstance =
      (SFc11_SimulationModelInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SimulationModelMachineNumber_,
           11,
           1,
           1,
           0,
           2,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_SimulationModelMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_SimulationModelMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _SimulationModelMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"u");
          _SFD_SET_DATA_PROPS(1,2,0,1,"y");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,58);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_u_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c11_u_bus_io,(MexInFcnForType)NULL);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _SimulationModelMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc11_SimulationModelInstanceStruct *chartInstance =
      (SFc11_SimulationModelInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c11_u);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c11_y);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "se2FR1dFSqdTEZm2Y6VsvoB";
}

static void sf_opaque_initialize_c11_SimulationModel(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc11_SimulationModelInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c11_SimulationModel((SFc11_SimulationModelInstanceStruct*)
    chartInstanceVar);
  initialize_c11_SimulationModel((SFc11_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c11_SimulationModel(void *chartInstanceVar)
{
  enable_c11_SimulationModel((SFc11_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c11_SimulationModel(void *chartInstanceVar)
{
  disable_c11_SimulationModel((SFc11_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c11_SimulationModel(void *chartInstanceVar)
{
  sf_gateway_c11_SimulationModel((SFc11_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c11_SimulationModel(SimStruct* S)
{
  return get_sim_state_c11_SimulationModel((SFc11_SimulationModelInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c11_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  set_sim_state_c11_SimulationModel((SFc11_SimulationModelInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c11_SimulationModel(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc11_SimulationModelInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SimulationModel_optimization_info();
    }

    finalize_c11_SimulationModel((SFc11_SimulationModelInstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc11_SimulationModel((SFc11_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c11_SimulationModel(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c11_SimulationModel((SFc11_SimulationModelInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c11_SimulationModel(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SimulationModel_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      11);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,11,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 11);
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,11);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,11,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,11,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 1; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,11);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(439178042U));
  ssSetChecksum1(S,(3050699574U));
  ssSetChecksum2(S,(1027392532U));
  ssSetChecksum3(S,(3409554730U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c11_SimulationModel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c11_SimulationModel(SimStruct *S)
{
  SFc11_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc11_SimulationModelInstanceStruct *)utMalloc(sizeof
    (SFc11_SimulationModelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc11_SimulationModelInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c11_SimulationModel;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c11_SimulationModel;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c11_SimulationModel;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c11_SimulationModel;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c11_SimulationModel;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c11_SimulationModel;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c11_SimulationModel;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c11_SimulationModel;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c11_SimulationModel;
  chartInstance->chartInfo.mdlStart = mdlStart_c11_SimulationModel;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c11_SimulationModel;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c11_SimulationModel(chartInstance);
}

void c11_SimulationModel_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c11_SimulationModel(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c11_SimulationModel(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c11_SimulationModel(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c11_SimulationModel_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
