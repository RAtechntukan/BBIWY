/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SimulationModel_sfun.h"
#include "c12_SimulationModel.h"
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
static const char * c12_debug_family_names[4] = { "nargin", "nargout", "u", "y"
};

static const char * c12_b_debug_family_names[2] = { "nargin", "nargout" };

static const char * c12_c_debug_family_names[2] = { "nargin", "nargout" };

static const char * c12_d_debug_family_names[3] = { "nargin", "nargout",
  "o_busCommand" };

/* Function Declarations */
static void initialize_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance);
static void initialize_params_c12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance);
static void enable_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance);
static void disable_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance);
static void c12_update_debugger_state_c12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance);
static void set_sim_state_c12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance, const mxArray *c12_st);
static void finalize_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance);
static void sf_gateway_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance);
static void mdl_start_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance);
static void initSimStructsc12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c12_machineNumber, uint32_T
  c12_chartNumber, uint32_T c12_instanceNumber);
static void c12_RobotController_getInstance_free
  (SFc12_SimulationModelInstanceStruct *chartInstance);
static const mxArray *c12_emlrt_marshallOut(SFc12_SimulationModelInstanceStruct *
  chartInstance, const c12_bus_Command c12_b_u);
static const mxArray *c12_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static c12_bus_Command c12_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c12_b_y, const char_T *c12_identifier);
static c12_bus_Command c12_b_emlrt_marshallIn
  (SFc12_SimulationModelInstanceStruct *chartInstance, const mxArray *c12_b_u,
   const emlrtMsgIdentifier *c12_parentId);
static c12_EMode c12_c_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId);
static real32_T c12_d_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId);
static void c12_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static const mxArray *c12_b_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static const mxArray *c12_c_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static real_T c12_e_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId);
static void c12_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static void c12_info_helper(const mxArray **c12_info);
static const mxArray *c12_b_emlrt_marshallOut(const char * c12_b_u);
static const mxArray *c12_c_emlrt_marshallOut(const uint32_T c12_b_u);
static const mxArray *c12_d_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData);
static int32_T c12_f_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId);
static void c12_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData);
static const mxArray *c12_u_bus_io(void *chartInstanceVoid, void *c12_pData);
static const mxArray *c12_y_bus_io(void *chartInstanceVoid, void *c12_pData);
static uint8_T c12_g_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_is_active_c12_SimulationModel, const
  char_T *c12_identifier);
static uint8_T c12_h_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId);
static void init_dsm_address_info(SFc12_SimulationModelInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc12_SimulationModelInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc12_SimulationModel(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c12_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c12_RobotControllerSingleton_not_empty = false;
  chartInstance->c12_is_active_c12_SimulationModel = 0U;
}

static void initialize_params_c12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c12_update_debugger_state_c12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance)
{
  const mxArray *c12_st;
  const mxArray *c12_b_y = NULL;
  c12_bus_Command c12_r0;
  uint8_T c12_hoistedGlobal;
  uint8_T c12_b_u;
  const mxArray *c12_c_y = NULL;
  c12_st = NULL;
  c12_st = NULL;
  c12_b_y = NULL;
  sf_mex_assign(&c12_b_y, sf_mex_createcellmatrix(2, 1), false);
  c12_r0.Mode = *(c12_EMode *)&((char_T *)chartInstance->c12_y)[0];
  c12_r0.Angle = *(real32_T *)&((char_T *)chartInstance->c12_y)[4];
  c12_r0.Position = *(real32_T *)&((char_T *)chartInstance->c12_y)[8];
  sf_mex_setcell(c12_b_y, 0, c12_emlrt_marshallOut(chartInstance, c12_r0));
  c12_hoistedGlobal = chartInstance->c12_is_active_c12_SimulationModel;
  c12_b_u = c12_hoistedGlobal;
  c12_c_y = NULL;
  sf_mex_assign(&c12_c_y, sf_mex_create("y", &c12_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c12_b_y, 1, c12_c_y);
  sf_mex_assign(&c12_st, c12_b_y, false);
  return c12_st;
}

static void set_sim_state_c12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance, const mxArray *c12_st)
{
  const mxArray *c12_b_u;
  c12_bus_Command c12_r1;
  chartInstance->c12_doneDoubleBufferReInit = true;
  c12_b_u = sf_mex_dup(c12_st);
  c12_r1 = c12_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("y",
    c12_b_u, 0)), "y");
  *(c12_EMode *)&((char_T *)chartInstance->c12_y)[0] = c12_r1.Mode;
  *(real32_T *)&((char_T *)chartInstance->c12_y)[4] = c12_r1.Angle;
  *(real32_T *)&((char_T *)chartInstance->c12_y)[8] = c12_r1.Position;
  chartInstance->c12_is_active_c12_SimulationModel = c12_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c12_SimulationModel",
       c12_b_u, 1)), "is_active_c12_SimulationModel");
  sf_mex_destroy(&c12_b_u);
  c12_update_debugger_state_c12_SimulationModel(chartInstance);
  sf_mex_destroy(&c12_st);
}

static void finalize_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance)
{
  c12_RobotController_getInstance_free(chartInstance);
}

static void sf_gateway_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance)
{
  int32_T c12_i0;
  c12_bus_Robot c12_b_u;
  int32_T c12_i1;
  int32_T c12_i2;
  int32_T c12_i3;
  uint32_T c12_debug_family_var_map[4];
  real_T c12_nargin = 1.0;
  real_T c12_nargout = 1.0;
  c12_bus_Command c12_b_y;
  uint32_T c12_b_debug_family_var_map[2];
  real_T c12_b_nargin = 0.0;
  real_T c12_b_nargout = 1.0;
  real_T c12_c_nargin = 1.0;
  real_T c12_c_nargout = 1.0;
  uint32_T c12_c_debug_family_var_map[3];
  real_T c12_d_nargin = 0.0;
  real_T c12_d_nargout = 1.0;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 10U, chartInstance->c12_sfEvent);
  chartInstance->c12_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 10U, chartInstance->c12_sfEvent);
  for (c12_i0 = 0; c12_i0 < 2; c12_i0++) {
    c12_b_u.EncodersCount[c12_i0] = ((int32_T *)&((char_T *)chartInstance->c12_u)
      [0])[c12_i0];
  }

  for (c12_i1 = 0; c12_i1 < 6; c12_i1++) {
    c12_b_u.Distance[c12_i1] = ((int16_T *)&((char_T *)chartInstance->c12_u)[8])
      [c12_i1];
  }

  for (c12_i2 = 0; c12_i2 < 6; c12_i2++) {
    c12_b_u.Bearing[c12_i2] = ((int8_T *)&((char_T *)chartInstance->c12_u)[20])
      [c12_i2];
  }

  c12_b_u.ScannerStatus = *(c12_EPosition *)&((char_T *)chartInstance->c12_u)[28];
  for (c12_i3 = 0; c12_i3 < 3; c12_i3++) {
    c12_b_u.ScannerValues[c12_i3] = ((int8_T *)&((char_T *)chartInstance->c12_u)
      [32])[c12_i3];
  }

  c12_b_u.Remaining.Angle = *(real32_T *)&((char_T *)(c12_bus_RemainingPosition *)
    &((char_T *)chartInstance->c12_u)[40])[0];
  c12_b_u.Remaining.Distance = *(real32_T *)&((char_T *)
    (c12_bus_RemainingPosition *)&((char_T *)chartInstance->c12_u)[40])[4];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c12_debug_family_names,
    c12_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_nargin, 0U, c12_c_sf_marshallOut,
    c12_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_nargout, 1U, c12_c_sf_marshallOut,
    c12_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c12_b_u, 2U, c12_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_b_y, 3U, c12_sf_marshallOut,
    c12_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c12_sfEvent, 2);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c12_c_debug_family_names,
    c12_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_b_nargin, 0U, c12_c_sf_marshallOut,
    c12_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_b_nargout, 1U, c12_c_sf_marshallOut,
    c12_b_sf_marshallIn);
  CV_SCRIPT_FCN(0, 2);
  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, 25);
  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, 26);
  if (CV_SCRIPT_IF(0, 0, !chartInstance->c12_RobotControllerSingleton_not_empty))
  {
    _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, 27);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c12_b_debug_family_names,
      c12_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_c_nargin, 0U, c12_c_sf_marshallOut,
      c12_b_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_c_nargout, 1U,
      c12_c_sf_marshallOut, c12_b_sf_marshallIn);
    CV_SCRIPT_FCN(0, 0);
    _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, 1);
    _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, -1);
    _SFD_SYMBOL_SCOPE_POP();
    chartInstance->c12_RobotControllerSingleton_not_empty = true;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, 29);
  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, -29);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c12_sfEvent, 3);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c12_d_debug_family_names,
    c12_c_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_d_nargin, 0U, c12_c_sf_marshallOut,
    c12_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_d_nargout, 1U, c12_c_sf_marshallOut,
    c12_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c12_b_y, 2U, c12_sf_marshallOut,
    c12_sf_marshallIn);
  CV_SCRIPT_FCN(0, 1);
  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, 17);
  c12_b_y.Mode = EMode_SPEED;
  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, 18);
  c12_b_y.Angle = 0.1F;
  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, 19);
  c12_b_y.Position = 1.0F;
  _SFD_SCRIPT_CALL(0U, chartInstance->c12_sfEvent, -19);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c12_sfEvent, -3);
  _SFD_SYMBOL_SCOPE_POP();
  *(c12_EMode *)&((char_T *)chartInstance->c12_y)[0] = c12_b_y.Mode;
  *(real32_T *)&((char_T *)chartInstance->c12_y)[4] = c12_b_y.Angle;
  *(real32_T *)&((char_T *)chartInstance->c12_y)[8] = c12_b_y.Position;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 10U, chartInstance->c12_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SimulationModelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void mdl_start_c12_SimulationModel(SFc12_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc12_SimulationModel
  (SFc12_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c12_machineNumber, uint32_T
  c12_chartNumber, uint32_T c12_instanceNumber)
{
  (void)c12_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c12_chartNumber, c12_instanceNumber, 0U,
    sf_debug_get_script_id(
    "D:\\LOIC\\Documents\\MATLAB\\CompetRobot\\dev\\RobotController.m"));
}

static void c12_RobotController_getInstance_free
  (SFc12_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c12_emlrt_marshallOut(SFc12_SimulationModelInstanceStruct *
  chartInstance, const c12_bus_Command c12_b_u)
{
  const mxArray *c12_b_y;
  c12_EMode c12_c_u;
  const mxArray *c12_c_y = NULL;
  static const int32_T c12_enumValues[2] = { 0, 1 };

  static const char * c12_enumNames[2] = { "DISTANCE", "SPEED" };

  int32_T c12_d_u;
  const mxArray *c12_d_y = NULL;
  const mxArray *c12_m0 = NULL;
  real32_T c12_e_u;
  const mxArray *c12_e_y = NULL;
  real32_T c12_f_u;
  const mxArray *c12_f_y = NULL;
  (void)chartInstance;
  c12_b_y = NULL;
  c12_b_y = NULL;
  sf_mex_assign(&c12_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c12_c_u = c12_b_u.Mode;
  c12_c_y = NULL;
  sf_mex_check_enum("EMode", 2, c12_enumNames, c12_enumValues);
  c12_d_u = (int32_T)c12_c_u;
  c12_d_y = NULL;
  sf_mex_assign(&c12_d_y, sf_mex_create("y", &c12_d_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c12_m0, c12_d_y, false);
  sf_mex_assign(&c12_c_y, sf_mex_create_enum("EMode", c12_m0), false);
  sf_mex_destroy(&c12_m0);
  sf_mex_addfield(c12_b_y, c12_c_y, "Mode", "Mode", 0);
  c12_e_u = c12_b_u.Angle;
  c12_e_y = NULL;
  sf_mex_assign(&c12_e_y, sf_mex_create("y", &c12_e_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c12_b_y, c12_e_y, "Angle", "Angle", 0);
  c12_f_u = c12_b_u.Position;
  c12_f_y = NULL;
  sf_mex_assign(&c12_f_y, sf_mex_create("y", &c12_f_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c12_b_y, c12_f_y, "Position", "Position", 0);
  return c12_b_y;
}

static const mxArray *c12_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  sf_mex_assign(&c12_mxArrayOutData, c12_emlrt_marshallOut(chartInstance,
    *(c12_bus_Command *)c12_inData), false);
  return c12_mxArrayOutData;
}

static c12_bus_Command c12_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c12_b_y, const char_T *c12_identifier)
{
  c12_bus_Command c12_c_y;
  emlrtMsgIdentifier c12_thisId;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_thisId.bParentIsCell = false;
  c12_c_y = c12_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_b_y),
    &c12_thisId);
  sf_mex_destroy(&c12_b_y);
  return c12_c_y;
}

static c12_bus_Command c12_b_emlrt_marshallIn
  (SFc12_SimulationModelInstanceStruct *chartInstance, const mxArray *c12_b_u,
   const emlrtMsgIdentifier *c12_parentId)
{
  c12_bus_Command c12_b_y;
  emlrtMsgIdentifier c12_thisId;
  static const char * c12_fieldNames[3] = { "Mode", "Angle", "Position" };

  c12_thisId.fParent = c12_parentId;
  c12_thisId.bParentIsCell = false;
  sf_mex_check_struct(c12_parentId, c12_b_u, 3, c12_fieldNames, 0U, NULL);
  c12_thisId.fIdentifier = "Mode";
  c12_b_y.Mode = c12_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_b_u, "Mode", "Mode", 0)), &c12_thisId);
  c12_thisId.fIdentifier = "Angle";
  c12_b_y.Angle = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_b_u, "Angle", "Angle", 0)), &c12_thisId);
  c12_thisId.fIdentifier = "Position";
  c12_b_y.Position = c12_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c12_b_u, "Position", "Position", 0)), &c12_thisId);
  sf_mex_destroy(&c12_b_u);
  return c12_b_y;
}

static c12_EMode c12_c_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId)
{
  c12_EMode c12_b_y;
  static const int32_T c12_enumValues[2] = { 0, 1 };

  static const char * c12_enumNames[2] = { "DISTANCE", "SPEED" };

  (void)chartInstance;
  sf_mex_check_enum("EMode", 2, c12_enumNames, c12_enumValues);
  sf_mex_check_builtin(c12_parentId, c12_b_u, "EMode", 0, 0U, NULL);
  c12_b_y = (c12_EMode)sf_mex_get_enum_element(c12_b_u, 0);
  sf_mex_destroy(&c12_b_u);
  return c12_b_y;
}

static real32_T c12_d_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId)
{
  real32_T c12_b_y;
  real32_T c12_f0;
  (void)chartInstance;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_b_u), &c12_f0, 1, 1, 0U, 0, 0U, 0);
  c12_b_y = c12_f0;
  sf_mex_destroy(&c12_b_u);
  return c12_b_y;
}

static void c12_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_b_y;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  c12_bus_Command c12_c_y;
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)chartInstanceVoid;
  c12_b_y = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_thisId.bParentIsCell = false;
  c12_c_y = c12_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_b_y),
    &c12_thisId);
  sf_mex_destroy(&c12_b_y);
  *(c12_bus_Command *)c12_outData = c12_c_y;
  sf_mex_destroy(&c12_mxArrayInData);
}

static const mxArray *c12_b_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData;
  c12_bus_Robot c12_b_u;
  const mxArray *c12_b_y = NULL;
  int32_T c12_i4;
  int32_T c12_c_u[2];
  const mxArray *c12_c_y = NULL;
  int32_T c12_i5;
  int16_T c12_d_u[6];
  const mxArray *c12_d_y = NULL;
  int32_T c12_i6;
  int8_T c12_e_u[6];
  const mxArray *c12_e_y = NULL;
  c12_EPosition c12_f_u;
  const mxArray *c12_f_y = NULL;
  static const int32_T c12_enumValues[4] = { 0, 1, 2, 3 };

  static const char * c12_enumNames[4] = { "NO", "LEFT", "MIDDLE", "RIGHT" };

  int32_T c12_g_u;
  const mxArray *c12_g_y = NULL;
  const mxArray *c12_m1 = NULL;
  int32_T c12_i7;
  int8_T c12_h_u[3];
  const mxArray *c12_h_y = NULL;
  c12_bus_RemainingPosition c12_i_u;
  const mxArray *c12_i_y = NULL;
  real32_T c12_j_u;
  const mxArray *c12_j_y = NULL;
  real32_T c12_k_u;
  const mxArray *c12_k_y = NULL;
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_mxArrayOutData = NULL;
  c12_b_u = *(c12_bus_Robot *)c12_inData;
  c12_b_y = NULL;
  sf_mex_assign(&c12_b_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  for (c12_i4 = 0; c12_i4 < 2; c12_i4++) {
    c12_c_u[c12_i4] = c12_b_u.EncodersCount[c12_i4];
  }

  c12_c_y = NULL;
  sf_mex_assign(&c12_c_y, sf_mex_create("y", c12_c_u, 6, 0U, 1U, 0U, 1, 2),
                false);
  sf_mex_addfield(c12_b_y, c12_c_y, "EncodersCount", "EncodersCount", 0);
  for (c12_i5 = 0; c12_i5 < 6; c12_i5++) {
    c12_d_u[c12_i5] = c12_b_u.Distance[c12_i5];
  }

  c12_d_y = NULL;
  sf_mex_assign(&c12_d_y, sf_mex_create("y", c12_d_u, 4, 0U, 1U, 0U, 1, 6),
                false);
  sf_mex_addfield(c12_b_y, c12_d_y, "Distance", "Distance", 0);
  for (c12_i6 = 0; c12_i6 < 6; c12_i6++) {
    c12_e_u[c12_i6] = c12_b_u.Bearing[c12_i6];
  }

  c12_e_y = NULL;
  sf_mex_assign(&c12_e_y, sf_mex_create("y", c12_e_u, 2, 0U, 1U, 0U, 1, 6),
                false);
  sf_mex_addfield(c12_b_y, c12_e_y, "Bearing", "Bearing", 0);
  c12_f_u = c12_b_u.ScannerStatus;
  c12_f_y = NULL;
  sf_mex_check_enum("EPosition", 4, c12_enumNames, c12_enumValues);
  c12_g_u = (int32_T)c12_f_u;
  c12_g_y = NULL;
  sf_mex_assign(&c12_g_y, sf_mex_create("y", &c12_g_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c12_m1, c12_g_y, false);
  sf_mex_assign(&c12_f_y, sf_mex_create_enum("EPosition", c12_m1), false);
  sf_mex_destroy(&c12_m1);
  sf_mex_addfield(c12_b_y, c12_f_y, "ScannerStatus", "ScannerStatus", 0);
  for (c12_i7 = 0; c12_i7 < 3; c12_i7++) {
    c12_h_u[c12_i7] = c12_b_u.ScannerValues[c12_i7];
  }

  c12_h_y = NULL;
  sf_mex_assign(&c12_h_y, sf_mex_create("y", c12_h_u, 2, 0U, 1U, 0U, 2, 1, 3),
                false);
  sf_mex_addfield(c12_b_y, c12_h_y, "ScannerValues", "ScannerValues", 0);
  c12_i_u = c12_b_u.Remaining;
  c12_i_y = NULL;
  sf_mex_assign(&c12_i_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c12_j_u = c12_i_u.Angle;
  c12_j_y = NULL;
  sf_mex_assign(&c12_j_y, sf_mex_create("y", &c12_j_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c12_i_y, c12_j_y, "Angle", "Angle", 0);
  c12_k_u = c12_i_u.Distance;
  c12_k_y = NULL;
  sf_mex_assign(&c12_k_y, sf_mex_create("y", &c12_k_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c12_i_y, c12_k_y, "Distance", "Distance", 0);
  sf_mex_addfield(c12_b_y, c12_i_y, "Remaining", "Remaining", 0);
  sf_mex_assign(&c12_mxArrayOutData, c12_b_y, false);
  return c12_mxArrayOutData;
}

static const mxArray *c12_c_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  real_T c12_b_u;
  const mxArray *c12_b_y = NULL;
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_b_u = *(real_T *)c12_inData;
  c12_b_y = NULL;
  sf_mex_assign(&c12_b_y, sf_mex_create("y", &c12_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c12_mxArrayOutData, c12_b_y, false);
  return c12_mxArrayOutData;
}

static real_T c12_e_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId)
{
  real_T c12_b_y;
  real_T c12_d0;
  (void)chartInstance;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_b_u), &c12_d0, 1, 0, 0U, 0, 0U, 0);
  c12_b_y = c12_d0;
  sf_mex_destroy(&c12_b_u);
  return c12_b_y;
}

static void c12_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_nargout;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  real_T c12_b_y;
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)chartInstanceVoid;
  c12_nargout = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_thisId.bParentIsCell = false;
  c12_b_y = c12_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_nargout),
    &c12_thisId);
  sf_mex_destroy(&c12_nargout);
  *(real_T *)c12_outData = c12_b_y;
  sf_mex_destroy(&c12_mxArrayInData);
}

const mxArray *sf_c12_SimulationModel_get_eml_resolved_functions_info(void)
{
  const mxArray *c12_nameCaptureInfo = NULL;
  c12_nameCaptureInfo = NULL;
  sf_mex_assign(&c12_nameCaptureInfo, sf_mex_createstruct("structure", 2, 3, 1),
                false);
  c12_info_helper(&c12_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c12_nameCaptureInfo);
  return c12_nameCaptureInfo;
}

static void c12_info_helper(const mxArray **c12_info)
{
  const mxArray *c12_rhs0 = NULL;
  const mxArray *c12_lhs0 = NULL;
  const mxArray *c12_rhs1 = NULL;
  const mxArray *c12_lhs1 = NULL;
  const mxArray *c12_rhs2 = NULL;
  const mxArray *c12_lhs2 = NULL;
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut(""), "context", "context",
                  0);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut("RobotController"), "name",
                  "name", 0);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut(
    "[C]D:/LOIC/Documents/MATLAB/CompetRobot/dev/RobotController.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(1457382962U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c12_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c12_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c12_info, sf_mex_duplicatearraysafe(&c12_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c12_info, sf_mex_duplicatearraysafe(&c12_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut(
    "[C]D:/LOIC/Documents/MATLAB/CompetRobot/dev/RobotController.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut("RobotController"), "name",
                  "name", 1);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut(
    "[C]D:/LOIC/Documents/MATLAB/CompetRobot/dev/RobotController.m"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(1457382962U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c12_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c12_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c12_info, sf_mex_duplicatearraysafe(&c12_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c12_info, sf_mex_duplicatearraysafe(&c12_lhs1), "lhs", "lhs",
                  1);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut(
    "[C]D:/LOIC/Documents/MATLAB/CompetRobot/dev/RobotController.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut("EMode"), "name", "name", 2);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c12_info, c12_b_emlrt_marshallOut(
    "[N]D:/LOIC/Documents/MATLAB/CompetRobot/data/datatypes/EMode.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(1457001258U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c12_info, c12_c_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c12_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c12_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c12_info, sf_mex_duplicatearraysafe(&c12_rhs2), "rhs", "rhs",
                  2);
  sf_mex_addfield(*c12_info, sf_mex_duplicatearraysafe(&c12_lhs2), "lhs", "lhs",
                  2);
  sf_mex_destroy(&c12_rhs0);
  sf_mex_destroy(&c12_lhs0);
  sf_mex_destroy(&c12_rhs1);
  sf_mex_destroy(&c12_lhs1);
  sf_mex_destroy(&c12_rhs2);
  sf_mex_destroy(&c12_lhs2);
}

static const mxArray *c12_b_emlrt_marshallOut(const char * c12_b_u)
{
  const mxArray *c12_b_y = NULL;
  c12_b_y = NULL;
  sf_mex_assign(&c12_b_y, sf_mex_create("y", c12_b_u, 15, 0U, 0U, 0U, 2, 1,
    strlen(c12_b_u)), false);
  return c12_b_y;
}

static const mxArray *c12_c_emlrt_marshallOut(const uint32_T c12_b_u)
{
  const mxArray *c12_b_y = NULL;
  c12_b_y = NULL;
  sf_mex_assign(&c12_b_y, sf_mex_create("y", &c12_b_u, 7, 0U, 0U, 0U, 0), false);
  return c12_b_y;
}

static const mxArray *c12_d_sf_marshallOut(void *chartInstanceVoid, void
  *c12_inData)
{
  const mxArray *c12_mxArrayOutData = NULL;
  int32_T c12_b_u;
  const mxArray *c12_b_y = NULL;
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)chartInstanceVoid;
  c12_mxArrayOutData = NULL;
  c12_b_u = *(int32_T *)c12_inData;
  c12_b_y = NULL;
  sf_mex_assign(&c12_b_y, sf_mex_create("y", &c12_b_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c12_mxArrayOutData, c12_b_y, false);
  return c12_mxArrayOutData;
}

static int32_T c12_f_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId)
{
  int32_T c12_b_y;
  int32_T c12_i8;
  (void)chartInstance;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_b_u), &c12_i8, 1, 6, 0U, 0, 0U, 0);
  c12_b_y = c12_i8;
  sf_mex_destroy(&c12_b_u);
  return c12_b_y;
}

static void c12_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c12_mxArrayInData, const char_T *c12_varName, void *c12_outData)
{
  const mxArray *c12_b_sfEvent;
  const char_T *c12_identifier;
  emlrtMsgIdentifier c12_thisId;
  int32_T c12_b_y;
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)chartInstanceVoid;
  c12_b_sfEvent = sf_mex_dup(c12_mxArrayInData);
  c12_identifier = c12_varName;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_thisId.bParentIsCell = false;
  c12_b_y = c12_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c12_b_sfEvent),
    &c12_thisId);
  sf_mex_destroy(&c12_b_sfEvent);
  *(int32_T *)c12_outData = c12_b_y;
  sf_mex_destroy(&c12_mxArrayInData);
}

static const mxArray *c12_u_bus_io(void *chartInstanceVoid, void *c12_pData)
{
  const mxArray *c12_mxVal = NULL;
  int32_T c12_i9;
  c12_bus_Robot c12_tmp;
  int32_T c12_i10;
  int32_T c12_i11;
  int32_T c12_i12;
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)chartInstanceVoid;
  c12_mxVal = NULL;
  for (c12_i9 = 0; c12_i9 < 2; c12_i9++) {
    c12_tmp.EncodersCount[c12_i9] = ((int32_T *)&((char_T *)(c12_bus_Robot *)
      c12_pData)[0])[c12_i9];
  }

  for (c12_i10 = 0; c12_i10 < 6; c12_i10++) {
    c12_tmp.Distance[c12_i10] = ((int16_T *)&((char_T *)(c12_bus_Robot *)
      c12_pData)[8])[c12_i10];
  }

  for (c12_i11 = 0; c12_i11 < 6; c12_i11++) {
    c12_tmp.Bearing[c12_i11] = ((int8_T *)&((char_T *)(c12_bus_Robot *)c12_pData)
      [20])[c12_i11];
  }

  c12_tmp.ScannerStatus = *(c12_EPosition *)&((char_T *)(c12_bus_Robot *)
    c12_pData)[28];
  for (c12_i12 = 0; c12_i12 < 3; c12_i12++) {
    c12_tmp.ScannerValues[c12_i12] = ((int8_T *)&((char_T *)(c12_bus_Robot *)
      c12_pData)[32])[c12_i12];
  }

  c12_tmp.Remaining.Angle = *(real32_T *)&((char_T *)(c12_bus_RemainingPosition *)
    &((char_T *)(c12_bus_Robot *)c12_pData)[40])[0];
  c12_tmp.Remaining.Distance = *(real32_T *)&((char_T *)
    (c12_bus_RemainingPosition *)&((char_T *)(c12_bus_Robot *)c12_pData)[40])[4];
  sf_mex_assign(&c12_mxVal, c12_b_sf_marshallOut(chartInstance, &c12_tmp), false);
  return c12_mxVal;
}

static const mxArray *c12_y_bus_io(void *chartInstanceVoid, void *c12_pData)
{
  const mxArray *c12_mxVal = NULL;
  c12_bus_Command c12_tmp;
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)chartInstanceVoid;
  c12_mxVal = NULL;
  c12_tmp.Mode = *(c12_EMode *)&((char_T *)(c12_bus_Command *)c12_pData)[0];
  c12_tmp.Angle = *(real32_T *)&((char_T *)(c12_bus_Command *)c12_pData)[4];
  c12_tmp.Position = *(real32_T *)&((char_T *)(c12_bus_Command *)c12_pData)[8];
  sf_mex_assign(&c12_mxVal, c12_sf_marshallOut(chartInstance, &c12_tmp), false);
  return c12_mxVal;
}

static uint8_T c12_g_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_is_active_c12_SimulationModel, const
  char_T *c12_identifier)
{
  uint8_T c12_b_y;
  emlrtMsgIdentifier c12_thisId;
  c12_thisId.fIdentifier = c12_identifier;
  c12_thisId.fParent = NULL;
  c12_thisId.bParentIsCell = false;
  c12_b_y = c12_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c12_b_is_active_c12_SimulationModel), &c12_thisId);
  sf_mex_destroy(&c12_b_is_active_c12_SimulationModel);
  return c12_b_y;
}

static uint8_T c12_h_emlrt_marshallIn(SFc12_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c12_b_u, const emlrtMsgIdentifier *c12_parentId)
{
  uint8_T c12_b_y;
  uint8_T c12_u0;
  (void)chartInstance;
  sf_mex_import(c12_parentId, sf_mex_dup(c12_b_u), &c12_u0, 1, 3, 0U, 0, 0U, 0);
  c12_b_y = c12_u0;
  sf_mex_destroy(&c12_b_u);
  return c12_b_y;
}

static void init_dsm_address_info(SFc12_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc12_SimulationModelInstanceStruct
  *chartInstance)
{
  chartInstance->c12_u = (c12_bus_Robot *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c12_y = (c12_bus_Command *)ssGetOutputPortSignal_wrapper
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

void sf_c12_SimulationModel_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3782354541U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(815806530U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1329265270U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1984866522U);
}

mxArray* sf_c12_SimulationModel_get_post_codegen_info(void);
mxArray *sf_c12_SimulationModel_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("bJBOdNFpK3uNL9ue7wljMC");
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
    mxArray* mxPostCodegenInfo = sf_c12_SimulationModel_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c12_SimulationModel_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c12_SimulationModel_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("early");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c12_SimulationModel_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c12_SimulationModel_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c12_SimulationModel(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c12_SimulationModel\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c12_SimulationModel_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc12_SimulationModelInstanceStruct *chartInstance =
      (SFc12_SimulationModelInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SimulationModelMachineNumber_,
           12,
           1,
           1,
           0,
           2,
           0,
           0,
           0,
           0,
           1,
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,82);
        _SFD_CV_INIT_SCRIPT(0,3,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"RobotController_RobotController",148,-1,190);
        _SFD_CV_INIT_SCRIPT_FCN(0,2,"RobotController_getInstance",593,-1,848);
        _SFD_CV_INIT_SCRIPT_FCN(0,1,"RobotController_updateStep",255,-1,508);
        _SFD_CV_INIT_SCRIPT_IF(0,0,680,716,-1,792);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c12_u_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c12_y_bus_io,(MexInFcnForType)NULL);
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
    SFc12_SimulationModelInstanceStruct *chartInstance =
      (SFc12_SimulationModelInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c12_u);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c12_y);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sngghlV7TobrChNu3NYjelC";
}

static void sf_opaque_initialize_c12_SimulationModel(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc12_SimulationModelInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c12_SimulationModel((SFc12_SimulationModelInstanceStruct*)
    chartInstanceVar);
  initialize_c12_SimulationModel((SFc12_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c12_SimulationModel(void *chartInstanceVar)
{
  enable_c12_SimulationModel((SFc12_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c12_SimulationModel(void *chartInstanceVar)
{
  disable_c12_SimulationModel((SFc12_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c12_SimulationModel(void *chartInstanceVar)
{
  sf_gateway_c12_SimulationModel((SFc12_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c12_SimulationModel(SimStruct* S)
{
  return get_sim_state_c12_SimulationModel((SFc12_SimulationModelInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c12_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  set_sim_state_c12_SimulationModel((SFc12_SimulationModelInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c12_SimulationModel(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc12_SimulationModelInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SimulationModel_optimization_info();
    }

    finalize_c12_SimulationModel((SFc12_SimulationModelInstanceStruct*)
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
  initSimStructsc12_SimulationModel((SFc12_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c12_SimulationModel(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c12_SimulationModel((SFc12_SimulationModelInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c12_SimulationModel(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SimulationModel_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      12);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,12,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 12);
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,12);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,12,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,12,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,12);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4153474825U));
  ssSetChecksum1(S,(2563702199U));
  ssSetChecksum2(S,(3474528815U));
  ssSetChecksum3(S,(3915879158U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetSimStateCompliance(S, DISALLOW_SIM_STATE);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c12_SimulationModel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c12_SimulationModel(SimStruct *S)
{
  SFc12_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc12_SimulationModelInstanceStruct *)utMalloc(sizeof
    (SFc12_SimulationModelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc12_SimulationModelInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c12_SimulationModel;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c12_SimulationModel;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c12_SimulationModel;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c12_SimulationModel;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c12_SimulationModel;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c12_SimulationModel;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c12_SimulationModel;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c12_SimulationModel;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c12_SimulationModel;
  chartInstance->chartInfo.mdlStart = mdlStart_c12_SimulationModel;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c12_SimulationModel;
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
  mdl_start_c12_SimulationModel(chartInstance);
}

void c12_SimulationModel_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c12_SimulationModel(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c12_SimulationModel(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c12_SimulationModel(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c12_SimulationModel_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
