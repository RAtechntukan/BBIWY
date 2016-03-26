#ifndef __9c1cCXtKK36AOL9hRIyMrH_h__
#define __9c1cCXtKK36AOL9hRIyMrH_h__

/* Include files */
#include "simstruc.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_bus_CommandNoEnum_tag
#define struct_bus_CommandNoEnum_tag

struct bus_CommandNoEnum_tag
{
  int8_T Mode;
  real32_T Angle;
  real32_T Position;
};

#endif                                 /*struct_bus_CommandNoEnum_tag*/

#ifndef typedef_bus_CommandNoEnum
#define typedef_bus_CommandNoEnum

typedef struct bus_CommandNoEnum_tag bus_CommandNoEnum;

#endif                                 /*typedef_bus_CommandNoEnum*/

#ifndef struct_bus_RobotNoEnum_tag
#define struct_bus_RobotNoEnum_tag

struct bus_RobotNoEnum_tag
{
  int32_T EncodersCount[2];
  int16_T Distance[6];
  int8_T Bearing[6];
  int8_T ScannerStatus;
  int8_T ScannerValues[3];
  real32_T RemainingAngle;
  real32_T RemainingDistance;
};

#endif                                 /*struct_bus_RobotNoEnum_tag*/

#ifndef typedef_bus_RobotNoEnum
#define typedef_bus_RobotNoEnum

typedef struct bus_RobotNoEnum_tag bus_RobotNoEnum;

#endif                                 /*typedef_bus_RobotNoEnum*/

#ifndef struct_tag_sT2Q2Ly5E4XxaeVFaPG2OsF
#define struct_tag_sT2Q2Ly5E4XxaeVFaPG2OsF

struct tag_sT2Q2Ly5E4XxaeVFaPG2OsF
{
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  uint32_T inputVarSize1[8];
  real_T m_position[2];
};

#endif                                 /*struct_tag_sT2Q2Ly5E4XxaeVFaPG2OsF*/

#ifndef typedef_RobotBrain
#define typedef_RobotBrain

typedef struct tag_sT2Q2Ly5E4XxaeVFaPG2OsF RobotBrain;

#endif                                 /*typedef_RobotBrain*/

#ifndef typedef_InstanceStruct_9c1cCXtKK36AOL9hRIyMrH
#define typedef_InstanceStruct_9c1cCXtKK36AOL9hRIyMrH

typedef struct {
  SimStruct *S;
  RobotBrain sysobj;
  boolean_T sysobj_not_empty;
  void *emlrtRootTLSGlobal;
  covrtInstance *covInst;
  bus_RobotNoEnum *u0;
  bus_CommandNoEnum *b_y0;
} InstanceStruct_9c1cCXtKK36AOL9hRIyMrH;

#endif                                 /*typedef_InstanceStruct_9c1cCXtKK36AOL9hRIyMrH*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
extern void method_dispatcher_9c1cCXtKK36AOL9hRIyMrH(SimStruct *S, int_T method,
  void* data);

#endif
