/* Include files */

#include "modelInterface.h"
#include "m_9c1cCXtKK36AOL9hRIyMrH.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static emlrtMCInfo emlrtMCI = { 1, 1, "SystemCore",
  "C:\\Program Files\\MATLAB\\R2015b\\toolbox\\shared\\system\\coder\\+matlab\\+system\\+coder\\SystemCore.p"
};

/* Function Declarations */
static void cgxe_mdl_start(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance);
static void cgxe_mdl_initialize(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance);
static void cgxe_mdl_outputs(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance);
static void cgxe_mdl_update(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance);
static void cgxe_mdl_terminate(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance);
static const mxArray *mw__internal__getSimState__fcn
  (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance);
static RobotBrain emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *b_sysobj, const char_T *identifier);
static RobotBrain b_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId);
static int32_T c_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId);
static boolean_T d_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId);
static void e_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId,
  uint32_T y[8]);
static void f_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId, real_T
  y[2]);
static boolean_T g_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *b_sysobj_not_empty, const char_T *identifier);
static void mw__internal__setSimState__fcn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *st);
static const mxArray *message(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *b, const mxArray *c, emlrtMCInfo *location);
static void error(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance, const
                  mxArray *b, emlrtMCInfo *location);
static const mxArray *b_message(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *b, emlrtMCInfo *location);
static int32_T h_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *src, const emlrtMsgIdentifier *msgId);
static boolean_T i_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *src, const emlrtMsgIdentifier *msgId);
static void j_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *src, const emlrtMsgIdentifier *msgId, uint32_T
  ret[8]);
static void k_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *src, const emlrtMsgIdentifier *msgId, real_T
  ret[2]);
static void init_simulink_io_address(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance);

/* Function Definitions */
static void cgxe_mdl_start(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance)
{
  boolean_T flag;
  int32_T i0;
  RobotBrain *obj;
  static char_T cv0[51] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'L', 'o', 'c', 'k', 'e', 'd', 'R', 'e', 'l', 'e',
    'a', 's', 'e', 'd', 'C', 'o', 'd', 'e', 'g', 'e', 'n' };

  char_T u[51];
  const mxArray *y;
  static const int32_T iv0[2] = { 1, 51 };

  const mxArray *m0;
  static char_T cv1[5] = { 's', 'e', 't', 'u', 'p' };

  char_T b_u[5];
  const mxArray *b_y;
  static const int32_T iv1[2] = { 1, 5 };

  static char_T cv2[44] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'i', 'n', 'v', 'a', 'l', 'i', 'd', 'T', 'u', 'n', 'a',
    'b', 'l', 'e', 'M', 'o', 'd', 'A', 'c', 'c', 'e', 's', 's', 'C', 'o', 'd',
    'e', 'g', 'e', 'n' };

  char_T c_u[44];
  const mxArray *c_y;
  static const int32_T iv2[2] = { 1, 44 };

  real_T (*m_position)[2];
  m_position = (real_T (*)[2])cgxertGetRunTimeParamInfoData(moduleInstance->S, 0);
  init_simulink_io_address(moduleInstance);

  /* Allocate instance data */
  covrtAllocateInstanceData(moduleInstance->covInst);

  /* Initialize Coverage Information */
  covrtScriptInit(moduleInstance->covInst,
                  "D:\\LOIC\\Documents\\MATLAB\\CompetRobot\\dev\\RobotBrain.m",
                  0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0);

  /* Initialize Function Information */
  covrtFcnInit(moduleInstance->covInst, 0, 0, "RobotBrain_RobotBrain", 329, -1,
               459);
  covrtFcnInit(moduleInstance->covInst, 0, 1, "RobotBrain_stepImpl", 873, -1,
               1043);

  /* Initialize Basic Block Information */
  covrtBasicBlockInit(moduleInstance->covInst, 0, 0, 413, -1, 450);
  covrtBasicBlockInit(moduleInstance->covInst, 0, 1, 954, -1, 1034);

  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(moduleInstance->covInst, 0U);
  if (!moduleInstance->sysobj_not_empty) {
    covrtLogFcn(moduleInstance->covInst, 0, 0);
    covrtLogBasicBlock(moduleInstance->covInst, 0, 0);

    /*  RobotBrain Matlab System */
    /*  Copyright 2015 The MathWorks, Inc. */
    /*  m_position */
    moduleInstance->sysobj.isInitialized = 0;
    moduleInstance->sysobj.TunablePropsChanged = false;

    /*  Support name-value pair arguments */
    moduleInstance->sysobj_not_empty = true;
    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    for (i0 = 0; i0 < 2; i0++) {
      moduleInstance->sysobj.m_position[i0] = (*m_position)[i0];
    }
  }

  obj = &moduleInstance->sysobj;
  if (moduleInstance->sysobj.isInitialized != 0) {
    for (i0 = 0; i0 < 51; i0++) {
      u[i0] = cv0[i0];
    }

    y = NULL;
    m0 = emlrtCreateCharArray(2, iv0);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 51, m0, &u[0]);
    emlrtAssign(&y, m0);
    for (i0 = 0; i0 < 5; i0++) {
      b_u[i0] = cv1[i0];
    }

    b_y = NULL;
    m0 = emlrtCreateCharArray(2, iv1);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 5, m0, &b_u[0]);
    emlrtAssign(&b_y, m0);
    error(moduleInstance, message(moduleInstance, y, b_y, &emlrtMCI), &emlrtMCI);
  }

  obj->isInitialized = 1;
  for (i0 = 0; i0 < 8; i0++) {
    obj->inputVarSize1[i0] = 1U;
  }

  if (obj->TunablePropsChanged) {
    for (i0 = 0; i0 < 44; i0++) {
      c_u[i0] = cv2[i0];
    }

    c_y = NULL;
    m0 = emlrtCreateCharArray(2, iv2);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 44, m0, &c_u[0]);
    emlrtAssign(&c_y, m0);
    error(moduleInstance, b_message(moduleInstance, c_y, &emlrtMCI), &emlrtMCI);
  }

  obj->TunablePropsChanged = false;
}

static void cgxe_mdl_initialize(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance)
{
  boolean_T tunablePropChangedBeforeResetImpl;
  int32_T i1;
  RobotBrain *obj;
  static char_T cv3[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o',
    'd', 'e', 'g', 'e', 'n' };

  char_T u[45];
  const mxArray *y;
  static const int32_T iv3[2] = { 1, 45 };

  const mxArray *m1;
  static char_T cv4[8] = { 'i', 's', 'L', 'o', 'c', 'k', 'e', 'd' };

  char_T b_u[8];
  const mxArray *b_y;
  static const int32_T iv4[2] = { 1, 8 };

  char_T c_u[45];
  const mxArray *c_y;
  static const int32_T iv5[2] = { 1, 45 };

  static char_T cv5[5] = { 'r', 'e', 's', 'e', 't' };

  char_T d_u[5];
  const mxArray *d_y;
  static const int32_T iv6[2] = { 1, 5 };

  static char_T cv6[44] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'i', 'n', 'v', 'a', 'l', 'i', 'd', 'T', 'u', 'n', 'a',
    'b', 'l', 'e', 'M', 'o', 'd', 'A', 'c', 'c', 'e', 's', 's', 'C', 'o', 'd',
    'e', 'g', 'e', 'n' };

  char_T e_u[44];
  const mxArray *e_y;
  static const int32_T iv7[2] = { 1, 44 };

  real_T (*m_position)[2];
  m_position = (real_T (*)[2])cgxertGetRunTimeParamInfoData(moduleInstance->S, 0);
  if (!moduleInstance->sysobj_not_empty) {
    covrtLogFcn(moduleInstance->covInst, 0, 0);
    covrtLogBasicBlock(moduleInstance->covInst, 0, 0);

    /*  RobotBrain Matlab System */
    /*  Copyright 2015 The MathWorks, Inc. */
    /*  m_position */
    moduleInstance->sysobj.isInitialized = 0;
    moduleInstance->sysobj.TunablePropsChanged = false;

    /*  Support name-value pair arguments */
    moduleInstance->sysobj_not_empty = true;
    tunablePropChangedBeforeResetImpl = (moduleInstance->sysobj.isInitialized ==
      1);
    if (tunablePropChangedBeforeResetImpl) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    tunablePropChangedBeforeResetImpl = (moduleInstance->sysobj.isInitialized ==
      1);
    if (tunablePropChangedBeforeResetImpl) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    tunablePropChangedBeforeResetImpl = (moduleInstance->sysobj.isInitialized ==
      1);
    if (tunablePropChangedBeforeResetImpl) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    for (i1 = 0; i1 < 2; i1++) {
      moduleInstance->sysobj.m_position[i1] = (*m_position)[i1];
    }
  }

  obj = &moduleInstance->sysobj;
  if (moduleInstance->sysobj.isInitialized == 2) {
    for (i1 = 0; i1 < 45; i1++) {
      u[i1] = cv3[i1];
    }

    y = NULL;
    m1 = emlrtCreateCharArray(2, iv3);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 45, m1, &u[0]);
    emlrtAssign(&y, m1);
    for (i1 = 0; i1 < 8; i1++) {
      b_u[i1] = cv4[i1];
    }

    b_y = NULL;
    m1 = emlrtCreateCharArray(2, iv4);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 8, m1, &b_u[0]);
    emlrtAssign(&b_y, m1);
    error(moduleInstance, message(moduleInstance, y, b_y, &emlrtMCI), &emlrtMCI);
  }

  tunablePropChangedBeforeResetImpl = (obj->isInitialized == 1);
  if (tunablePropChangedBeforeResetImpl) {
    obj = &moduleInstance->sysobj;
    if (moduleInstance->sysobj.isInitialized == 2) {
      for (i1 = 0; i1 < 45; i1++) {
        c_u[i1] = cv3[i1];
      }

      c_y = NULL;
      m1 = emlrtCreateCharArray(2, iv5);
      emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 45, m1, &c_u
        [0]);
      emlrtAssign(&c_y, m1);
      for (i1 = 0; i1 < 5; i1++) {
        d_u[i1] = cv5[i1];
      }

      d_y = NULL;
      m1 = emlrtCreateCharArray(2, iv6);
      emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 5, m1, &d_u[0]);
      emlrtAssign(&d_y, m1);
      error(moduleInstance, message(moduleInstance, c_y, d_y, &emlrtMCI),
            &emlrtMCI);
    }

    tunablePropChangedBeforeResetImpl = obj->TunablePropsChanged;
    if ((int32_T)tunablePropChangedBeforeResetImpl != (int32_T)
        obj->TunablePropsChanged) {
      for (i1 = 0; i1 < 44; i1++) {
        e_u[i1] = cv6[i1];
      }

      e_y = NULL;
      m1 = emlrtCreateCharArray(2, iv7);
      emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 44, m1, &e_u
        [0]);
      emlrtAssign(&e_y, m1);
      error(moduleInstance, b_message(moduleInstance, e_y, &emlrtMCI), &emlrtMCI);
    }
  }
}

static void cgxe_mdl_outputs(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance)
{
  boolean_T flag;
  int32_T k;
  real_T hoistedGlobal_m_position[2];
  boolean_T p;
  boolean_T exitg2;
  RobotBrain *obj;
  static char_T cv7[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o',
    'd', 'e', 'g', 'e', 'n' };

  char_T u[45];
  const mxArray *y;
  static const int32_T iv8[2] = { 1, 45 };

  const mxArray *m2;
  static char_T cv8[4] = { 's', 't', 'e', 'p' };

  char_T b_u[4];
  const mxArray *b_y;
  static const int32_T iv9[2] = { 1, 4 };

  static char_T cv9[51] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'L', 'o', 'c', 'k', 'e', 'd', 'R', 'e', 'l', 'e',
    'a', 's', 'e', 'd', 'C', 'o', 'd', 'e', 'g', 'e', 'n' };

  char_T c_u[51];
  const mxArray *c_y;
  static const int32_T iv10[2] = { 1, 51 };

  static char_T cv10[5] = { 's', 'e', 't', 'u', 'p' };

  char_T d_u[5];
  const mxArray *d_y;
  static const int32_T iv11[2] = { 1, 5 };

  static char_T cv11[44] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'i', 'n', 'v', 'a', 'l', 'i', 'd', 'T', 'u', 'n', 'a',
    'b', 'l', 'e', 'M', 'o', 'd', 'A', 'c', 'c', 'e', 's', 's', 'C', 'o', 'd',
    'e', 'g', 'e', 'n' };

  char_T e_u[44];
  const mxArray *e_y;
  static const int32_T iv12[2] = { 1, 44 };

  boolean_T exitg1;
  char_T f_u[44];
  const mxArray *f_y;
  static const int32_T iv13[2] = { 1, 44 };

  real_T (*m_position)[2];
  m_position = (real_T (*)[2])cgxertGetRunTimeParamInfoData(moduleInstance->S, 0);
  if (!moduleInstance->sysobj_not_empty) {
    covrtLogFcn(moduleInstance->covInst, 0, 0);
    covrtLogBasicBlock(moduleInstance->covInst, 0, 0);

    /*  RobotBrain Matlab System */
    /*  Copyright 2015 The MathWorks, Inc. */
    /*  m_position */
    moduleInstance->sysobj.isInitialized = 0;
    moduleInstance->sysobj.TunablePropsChanged = false;

    /*  Support name-value pair arguments */
    moduleInstance->sysobj_not_empty = true;
    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    for (k = 0; k < 2; k++) {
      moduleInstance->sysobj.m_position[k] = (*m_position)[k];
    }
  }

  for (k = 0; k < 2; k++) {
    hoistedGlobal_m_position[k] = moduleInstance->sysobj.m_position[k];
  }

  flag = false;
  p = true;
  k = 0;
  exitg2 = false;
  while ((exitg2 == false) && (k < 2)) {
    if (!(hoistedGlobal_m_position[k] == (*m_position)[k])) {
      p = false;
      exitg2 = true;
    } else {
      k++;
    }
  }

  if (p) {
    flag = true;
  }

  if (!flag) {
    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    for (k = 0; k < 2; k++) {
      moduleInstance->sysobj.m_position[k] = (*m_position)[k];
    }
  }

  obj = &moduleInstance->sysobj;
  if (moduleInstance->sysobj.isInitialized == 2) {
    for (k = 0; k < 45; k++) {
      u[k] = cv7[k];
    }

    y = NULL;
    m2 = emlrtCreateCharArray(2, iv8);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 45, m2, &u[0]);
    emlrtAssign(&y, m2);
    for (k = 0; k < 4; k++) {
      b_u[k] = cv8[k];
    }

    b_y = NULL;
    m2 = emlrtCreateCharArray(2, iv9);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 4, m2, &b_u[0]);
    emlrtAssign(&b_y, m2);
    error(moduleInstance, message(moduleInstance, y, b_y, &emlrtMCI), &emlrtMCI);
  }

  if (obj->isInitialized != 1) {
    if (obj->isInitialized != 0) {
      for (k = 0; k < 51; k++) {
        c_u[k] = cv9[k];
      }

      c_y = NULL;
      m2 = emlrtCreateCharArray(2, iv10);
      emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 51, m2, &c_u
        [0]);
      emlrtAssign(&c_y, m2);
      for (k = 0; k < 5; k++) {
        d_u[k] = cv10[k];
      }

      d_y = NULL;
      m2 = emlrtCreateCharArray(2, iv11);
      emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 5, m2, &d_u[0]);
      emlrtAssign(&d_y, m2);
      error(moduleInstance, message(moduleInstance, c_y, d_y, &emlrtMCI),
            &emlrtMCI);
    }

    obj->isInitialized = 1;
    for (k = 0; k < 8; k++) {
      obj->inputVarSize1[k] = 1U;
    }

    if (obj->TunablePropsChanged) {
      for (k = 0; k < 44; k++) {
        e_u[k] = cv11[k];
      }

      e_y = NULL;
      m2 = emlrtCreateCharArray(2, iv12);
      emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 44, m2, &e_u
        [0]);
      emlrtAssign(&e_y, m2);
      error(moduleInstance, b_message(moduleInstance, e_y, &emlrtMCI), &emlrtMCI);
    }

    obj->TunablePropsChanged = false;
  }

  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
  }

  k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (k < 8)) {
    if (obj->inputVarSize1[k] != 1U) {
      for (k = 0; k < 8; k++) {
        obj->inputVarSize1[k] = 1U;
      }

      exitg1 = true;
    } else {
      k++;
    }
  }

  covrtLogFcn(moduleInstance->covInst, 0, 1);
  covrtLogBasicBlock(moduleInstance->covInst, 0, 1);
  if (obj->TunablePropsChanged) {
    for (k = 0; k < 44; k++) {
      f_u[k] = cv11[k];
    }

    f_y = NULL;
    m2 = emlrtCreateCharArray(2, iv13);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 44, m2, &f_u[0]);
    emlrtAssign(&f_y, m2);
    error(moduleInstance, b_message(moduleInstance, f_y, &emlrtMCI), &emlrtMCI);
  }

  *(int8_T *)&((char_T *)moduleInstance->b_y0)[0] = 1;
  *(real32_T *)&((char_T *)moduleInstance->b_y0)[4] = 6.0F;
  *(real32_T *)&((char_T *)moduleInstance->b_y0)[8] = 42.0F;
}

static void cgxe_mdl_update(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance)
{
  (void)moduleInstance;
}

static void cgxe_mdl_terminate(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance)
{
  boolean_T flag;
  int32_T i2;
  RobotBrain *obj;
  static char_T cv12[45] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 's', 'y', 's',
    't', 'e', 'm', ':', 'm', 'e', 't', 'h', 'o', 'd', 'C', 'a', 'l', 'l', 'e',
    'd', 'W', 'h', 'e', 'n', 'R', 'e', 'l', 'e', 'a', 's', 'e', 'd', 'C', 'o',
    'd', 'e', 'g', 'e', 'n' };

  char_T u[45];
  const mxArray *y;
  static const int32_T iv14[2] = { 1, 45 };

  const mxArray *m3;
  static char_T cv13[8] = { 'i', 's', 'L', 'o', 'c', 'k', 'e', 'd' };

  char_T b_u[8];
  const mxArray *b_y;
  static const int32_T iv15[2] = { 1, 8 };

  char_T c_u[45];
  const mxArray *c_y;
  static const int32_T iv16[2] = { 1, 45 };

  static char_T cv14[7] = { 'r', 'e', 'l', 'e', 'a', 's', 'e' };

  char_T d_u[7];
  const mxArray *d_y;
  static const int32_T iv17[2] = { 1, 7 };

  real_T (*m_position)[2];
  m_position = (real_T (*)[2])cgxertGetRunTimeParamInfoData(moduleInstance->S, 0);
  if (!moduleInstance->sysobj_not_empty) {
    covrtLogFcn(moduleInstance->covInst, 0, 0);
    covrtLogBasicBlock(moduleInstance->covInst, 0, 0);

    /*  RobotBrain Matlab System */
    /*  Copyright 2015 The MathWorks, Inc. */
    /*  m_position */
    moduleInstance->sysobj.isInitialized = 0;
    moduleInstance->sysobj.TunablePropsChanged = false;

    /*  Support name-value pair arguments */
    moduleInstance->sysobj_not_empty = true;
    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    flag = (moduleInstance->sysobj.isInitialized == 1);
    if (flag) {
      moduleInstance->sysobj.TunablePropsChanged = true;
    }

    for (i2 = 0; i2 < 2; i2++) {
      moduleInstance->sysobj.m_position[i2] = (*m_position)[i2];
    }
  }

  obj = &moduleInstance->sysobj;
  if (moduleInstance->sysobj.isInitialized == 2) {
    for (i2 = 0; i2 < 45; i2++) {
      u[i2] = cv12[i2];
    }

    y = NULL;
    m3 = emlrtCreateCharArray(2, iv14);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 45, m3, &u[0]);
    emlrtAssign(&y, m3);
    for (i2 = 0; i2 < 8; i2++) {
      b_u[i2] = cv13[i2];
    }

    b_y = NULL;
    m3 = emlrtCreateCharArray(2, iv15);
    emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 8, m3, &b_u[0]);
    emlrtAssign(&b_y, m3);
    error(moduleInstance, message(moduleInstance, y, b_y, &emlrtMCI), &emlrtMCI);
  }

  flag = (obj->isInitialized == 1);
  if (flag) {
    obj = &moduleInstance->sysobj;
    if (moduleInstance->sysobj.isInitialized == 2) {
      for (i2 = 0; i2 < 45; i2++) {
        c_u[i2] = cv12[i2];
      }

      c_y = NULL;
      m3 = emlrtCreateCharArray(2, iv16);
      emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 45, m3, &c_u
        [0]);
      emlrtAssign(&c_y, m3);
      for (i2 = 0; i2 < 7; i2++) {
        d_u[i2] = cv14[i2];
      }

      d_y = NULL;
      m3 = emlrtCreateCharArray(2, iv17);
      emlrtInitCharArrayR2013a(moduleInstance->emlrtRootTLSGlobal, 7, m3, &d_u[0]);
      emlrtAssign(&d_y, m3);
      error(moduleInstance, message(moduleInstance, c_y, d_y, &emlrtMCI),
            &emlrtMCI);
    }

    if (obj->isInitialized == 1) {
      obj->isInitialized = 2;
    }
  }

  /* Free instance data */
  covrtFreeInstanceData(moduleInstance->covInst);
}

static const mxArray *mw__internal__getSimState__fcn
  (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance)
{
  const mxArray *st;
  const mxArray *y;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m4;
  const mxArray *d_y;
  const mxArray *e_y;
  static const int32_T iv18[2] = { 1, 8 };

  uint32_T *pData;
  int32_T i;
  const mxArray *f_y;
  static const int32_T iv19[2] = { 1, 2 };

  real_T *b_pData;
  const mxArray *g_y;
  st = NULL;
  y = NULL;
  emlrtAssign(&y, emlrtCreateCellMatrix(2, 1));
  b_y = NULL;
  emlrtAssign(&b_y, emlrtCreateStructMatrix(1, 1, 0, NULL));
  c_y = NULL;
  m4 = emlrtCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
  *(int32_T *)mxGetData(m4) = moduleInstance->sysobj.isInitialized;
  emlrtAssign(&c_y, m4);
  emlrtAddField(b_y, c_y, "isInitialized", 0);
  d_y = NULL;
  m4 = emlrtCreateLogicalScalar(moduleInstance->sysobj.TunablePropsChanged);
  emlrtAssign(&d_y, m4);
  emlrtAddField(b_y, d_y, "TunablePropsChanged", 0);
  e_y = NULL;
  m4 = emlrtCreateNumericArray(2, iv18, mxUINT32_CLASS, mxREAL);
  pData = (uint32_T *)mxGetData(m4);
  for (i = 0; i < 8; i++) {
    pData[i] = moduleInstance->sysobj.inputVarSize1[i];
  }

  emlrtAssign(&e_y, m4);
  emlrtAddField(b_y, e_y, "inputVarSize1", 0);
  f_y = NULL;
  m4 = emlrtCreateNumericArray(2, iv19, mxDOUBLE_CLASS, mxREAL);
  b_pData = (real_T *)mxGetPr(m4);
  for (i = 0; i < 2; i++) {
    b_pData[i] = moduleInstance->sysobj.m_position[i];
  }

  emlrtAssign(&f_y, m4);
  emlrtAddField(b_y, f_y, "m_position", 0);
  emlrtSetCell(y, 0, b_y);
  g_y = NULL;
  m4 = emlrtCreateLogicalScalar(moduleInstance->sysobj_not_empty);
  emlrtAssign(&g_y, m4);
  emlrtSetCell(y, 1, g_y);
  emlrtAssign(&st, y);
  return st;
}

static RobotBrain emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *b_sysobj, const char_T *identifier)
{
  RobotBrain y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(moduleInstance, emlrtAlias(b_sysobj), &thisId);
  emlrtDestroyArray(&b_sysobj);
  return y;
}

static RobotBrain b_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId)
{
  RobotBrain y;
  emlrtMsgIdentifier thisId;
  static const int32_T dims = 0;
  static const char * fieldNames[4] = { "isInitialized", "TunablePropsChanged",
    "inputVarSize1", "m_position" };

  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(moduleInstance->emlrtRootTLSGlobal, parentId, u, 4,
    fieldNames, 0U, &dims);
  thisId.fIdentifier = "isInitialized";
  y.isInitialized = c_emlrt_marshallIn(moduleInstance, emlrtAlias
    (emlrtGetFieldR2013a(moduleInstance->emlrtRootTLSGlobal, u, 0,
    "isInitialized")), &thisId);
  thisId.fIdentifier = "TunablePropsChanged";
  y.TunablePropsChanged = d_emlrt_marshallIn(moduleInstance, emlrtAlias
    (emlrtGetFieldR2013a(moduleInstance->emlrtRootTLSGlobal, u, 0,
    "TunablePropsChanged")), &thisId);
  thisId.fIdentifier = "inputVarSize1";
  e_emlrt_marshallIn(moduleInstance, emlrtAlias(emlrtGetFieldR2013a
    (moduleInstance->emlrtRootTLSGlobal, u, 0, "inputVarSize1")), &thisId,
                     y.inputVarSize1);
  thisId.fIdentifier = "m_position";
  f_emlrt_marshallIn(moduleInstance, emlrtAlias(emlrtGetFieldR2013a
    (moduleInstance->emlrtRootTLSGlobal, u, 0, "m_position")), &thisId,
                     y.m_position);
  emlrtDestroyArray(&u);
  return y;
}

static int32_T c_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId)
{
  int32_T y;
  y = h_emlrt_marshallIn(moduleInstance, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static boolean_T d_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = i_emlrt_marshallIn(moduleInstance, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void e_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId,
  uint32_T y[8])
{
  j_emlrt_marshallIn(moduleInstance, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void f_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *u, const emlrtMsgIdentifier *parentId, real_T
  y[2])
{
  k_emlrt_marshallIn(moduleInstance, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static boolean_T g_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *b_sysobj_not_empty, const char_T *identifier)
{
  boolean_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(moduleInstance, emlrtAlias(b_sysobj_not_empty), &thisId);
  emlrtDestroyArray(&b_sysobj_not_empty);
  return y;
}

static void mw__internal__setSimState__fcn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *st)
{
  const mxArray *u;
  u = emlrtAlias(st);
  moduleInstance->sysobj = emlrt_marshallIn(moduleInstance, emlrtAlias
    (emlrtGetCell(moduleInstance->emlrtRootTLSGlobal, "sysobj", u, 0)), "sysobj");
  moduleInstance->sysobj_not_empty = g_emlrt_marshallIn(moduleInstance,
    emlrtAlias(emlrtGetCell(moduleInstance->emlrtRootTLSGlobal,
    "sysobj_not_empty", u, 1)), "sysobj_not_empty");
  emlrtDestroyArray(&u);
  emlrtDestroyArray(&st);
}

static const mxArray *message(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *b, const mxArray *c, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m5;
  pArrays[0] = b;
  pArrays[1] = c;
  return emlrtCallMATLABR2012b(moduleInstance->emlrtRootTLSGlobal, 1, &m5, 2,
    pArrays, "message", true, location);
}

static void error(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance, const
                  mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(moduleInstance->emlrtRootTLSGlobal, 0, NULL, 1, &pArray,
                        "error", true, location);
}

static const mxArray *b_message(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  const mxArray *m6;
  pArray = b;
  return emlrtCallMATLABR2012b(moduleInstance->emlrtRootTLSGlobal, 1, &m6, 1,
    &pArray, "message", true, location);
}

static int32_T h_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *src, const emlrtMsgIdentifier *msgId)
{
  int32_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(moduleInstance->emlrtRootTLSGlobal, msgId, src,
    "int32", false, 0U, &dims);
  ret = *(int32_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static boolean_T i_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *src, const emlrtMsgIdentifier *msgId)
{
  boolean_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(moduleInstance->emlrtRootTLSGlobal, msgId, src,
    "logical", false, 0U, &dims);
  ret = *mxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void j_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *src, const emlrtMsgIdentifier *msgId, uint32_T
  ret[8])
{
  static const int32_T dims[2] = { 1, 8 };

  int32_T i3;
  emlrtCheckBuiltInR2012b(moduleInstance->emlrtRootTLSGlobal, msgId, src,
    "uint32", false, 2U, dims);
  for (i3 = 0; i3 < 8; i3++) {
    ret[i3] = (*(uint32_T (*)[8])mxGetData(src))[i3];
  }

  emlrtDestroyArray(&src);
}

static void k_emlrt_marshallIn(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance, const mxArray *src, const emlrtMsgIdentifier *msgId, real_T
  ret[2])
{
  static const int32_T dims[2] = { 1, 2 };

  int32_T i4;
  emlrtCheckBuiltInR2012b(moduleInstance->emlrtRootTLSGlobal, msgId, src,
    "double", false, 2U, dims);
  for (i4 = 0; i4 < 2; i4++) {
    ret[i4] = (*(real_T (*)[2])mxGetData(src))[i4];
  }

  emlrtDestroyArray(&src);
}

static void init_simulink_io_address(InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
  *moduleInstance)
{
  moduleInstance->emlrtRootTLSGlobal = cgxertGetEMLRTCtx(moduleInstance->S, 0);
  moduleInstance->covInst = (covrtInstance *)cgxertGetCovrtInstance
    (moduleInstance->S, 0);
  moduleInstance->u0 = (bus_RobotNoEnum *)cgxertGetInputPortSignal
    (moduleInstance->S, 0);
  moduleInstance->b_y0 = (bus_CommandNoEnum *)cgxertGetOutputPortSignal
    (moduleInstance->S, 0);
}

/* CGXE Glue Code */
static void mdlOutputs_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S, int_T tid)
{
  InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance =
    (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_outputs(moduleInstance);
}

static void mdlInitialize_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S)
{
  InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance =
    (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_initialize(moduleInstance);
}

static void mdlUpdate_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S, int_T tid)
{
  InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance =
    (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_update(moduleInstance);
}

static mxArray* getSimState_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S)
{
  mxArray* mxSS;
  InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance =
    (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *)cgxertGetRuntimeInstance(S);
  mxSS = (mxArray *) mw__internal__getSimState__fcn(moduleInstance);
  return mxSS;
}

static void setSimState_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S, const mxArray *ss)
{
  InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance =
    (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *)cgxertGetRuntimeInstance(S);
  mw__internal__setSimState__fcn(moduleInstance, emlrtAlias(ss));
}

static void mdlTerminate_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S)
{
  InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance =
    (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *)cgxertGetRuntimeInstance(S);
  cgxe_mdl_terminate(moduleInstance);
  free((void *)moduleInstance);
}

static void mdlStart_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S)
{
  InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *moduleInstance =
    (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH *)calloc(1, sizeof
    (InstanceStruct_9c1cCXtKK36AOL9hRIyMrH));
  moduleInstance->S = S;
  cgxertSetRuntimeInstance(S, (void *)moduleInstance);
  ssSetmdlOutputs(S, mdlOutputs_9c1cCXtKK36AOL9hRIyMrH);
  ssSetmdlInitializeConditions(S, mdlInitialize_9c1cCXtKK36AOL9hRIyMrH);
  ssSetmdlUpdate(S, mdlUpdate_9c1cCXtKK36AOL9hRIyMrH);
  ssSetmdlTerminate(S, mdlTerminate_9c1cCXtKK36AOL9hRIyMrH);
  cgxe_mdl_start(moduleInstance);

  {
    uint_T options = ssGetOptions(S);
    options |= SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE;
    ssSetOptions(S, options);
  }
}

static void mdlProcessParameters_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S)
{
}

void method_dispatcher_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_9c1cCXtKK36AOL9hRIyMrH(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_9c1cCXtKK36AOL9hRIyMrH(S);
    break;

   case SS_CALL_MDL_GET_SIM_STATE:
    *((mxArray**) data) = getSimState_9c1cCXtKK36AOL9hRIyMrH(S);
    break;

   case SS_CALL_MDL_SET_SIM_STATE:
    setSimState_9c1cCXtKK36AOL9hRIyMrH(S, (const mxArray *) data);
    break;

   default:
    /* Unhandled method */
    /*
       sf_mex_error_message("Stateflow Internal Error:\n"
       "Error calling method dispatcher for module: 9c1cCXtKK36AOL9hRIyMrH.\n"
       "Can't handle method %d.\n", method);
     */
    break;
  }
}

mxArray *cgxe_9c1cCXtKK36AOL9hRIyMrH_BuildInfoUpdate(void)
{
  mxArray * mxBIArgs;
  mxArray * elem_1;
  mxArray * elem_2;
  mxArray * elem_3;
  mxArray * elem_4;
  mxArray * elem_5;
  mxArray * elem_6;
  mxArray * elem_7;
  mxArray * elem_8;
  mxArray * elem_9;
  mxBIArgs = mxCreateCellMatrix(1,3);
  elem_1 = mxCreateCellMatrix(1,6);
  elem_2 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,0,elem_2);
  elem_3 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,1,elem_3);
  elem_4 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,2,elem_4);
  elem_5 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,3,elem_5);
  elem_6 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,4,elem_6);
  elem_7 = mxCreateCellMatrix(0,0);
  mxSetCell(elem_1,5,elem_7);
  mxSetCell(mxBIArgs,0,elem_1);
  elem_8 = mxCreateCellMatrix(1,0);
  mxSetCell(mxBIArgs,1,elem_8);
  elem_9 = mxCreateCellMatrix(1,0);
  mxSetCell(mxBIArgs,2,elem_9);
  return mxBIArgs;
}
