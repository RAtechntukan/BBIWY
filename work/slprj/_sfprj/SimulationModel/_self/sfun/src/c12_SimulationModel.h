#ifndef __c12_SimulationModel_h__
#define __c12_SimulationModel_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef enum_EMode
#define enum_EMode

enum EMode
{
  EMode_DISTANCE,
  EMode_SPEED
};

#endif                                 /*enum_EMode*/

#ifndef typedef_c12_EMode
#define typedef_c12_EMode

typedef enum EMode c12_EMode;

#endif                                 /*typedef_c12_EMode*/

#ifndef enum_EPosition
#define enum_EPosition

enum EPosition
{
  EPosition_NO,
  EPosition_LEFT,
  EPosition_MIDDLE,
  EPosition_RIGHT
};

#endif                                 /*enum_EPosition*/

#ifndef typedef_c12_EPosition
#define typedef_c12_EPosition

typedef enum EPosition c12_EPosition;

#endif                                 /*typedef_c12_EPosition*/

#ifndef struct_bus_RemainingPosition_tag
#define struct_bus_RemainingPosition_tag

struct bus_RemainingPosition_tag
{
  real32_T Angle;
  real32_T Distance;
};

#endif                                 /*struct_bus_RemainingPosition_tag*/

#ifndef typedef_c12_bus_RemainingPosition
#define typedef_c12_bus_RemainingPosition

typedef struct bus_RemainingPosition_tag c12_bus_RemainingPosition;

#endif                                 /*typedef_c12_bus_RemainingPosition*/

#ifndef struct_tag_sMB5GaXc40ozYzvPFeCX3Q
#define struct_tag_sMB5GaXc40ozYzvPFeCX3Q

struct tag_sMB5GaXc40ozYzvPFeCX3Q
{
  int32_T dummy;
};

#endif                                 /*struct_tag_sMB5GaXc40ozYzvPFeCX3Q*/

#ifndef typedef_c12_RobotController
#define typedef_c12_RobotController

typedef struct tag_sMB5GaXc40ozYzvPFeCX3Q c12_RobotController;

#endif                                 /*typedef_c12_RobotController*/

#ifndef struct_bus_Robot_tag
#define struct_bus_Robot_tag

struct bus_Robot_tag
{
  int32_T EncodersCount[2];
  int16_T Distance[6];
  int8_T Bearing[6];
  c12_EPosition ScannerStatus;
  int8_T ScannerValues[3];
  c12_bus_RemainingPosition Remaining;
};

#endif                                 /*struct_bus_Robot_tag*/

#ifndef typedef_c12_bus_Robot
#define typedef_c12_bus_Robot

typedef struct bus_Robot_tag c12_bus_Robot;

#endif                                 /*typedef_c12_bus_Robot*/

#ifndef struct_bus_Command_tag
#define struct_bus_Command_tag

struct bus_Command_tag
{
  c12_EMode Mode;
  real32_T Angle;
  real32_T Position;
};

#endif                                 /*struct_bus_Command_tag*/

#ifndef typedef_c12_bus_Command
#define typedef_c12_bus_Command

typedef struct bus_Command_tag c12_bus_Command;

#endif                                 /*typedef_c12_bus_Command*/

#ifndef typedef_SFc12_SimulationModelInstanceStruct
#define typedef_SFc12_SimulationModelInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c12_sfEvent;
  boolean_T c12_isStable;
  boolean_T c12_doneDoubleBufferReInit;
  uint8_T c12_is_active_c12_SimulationModel;
  c12_RobotController c12_RobotControllerSingleton;
  boolean_T c12_RobotControllerSingleton_not_empty;
  c12_bus_Robot *c12_u;
  c12_bus_Command *c12_y;
} SFc12_SimulationModelInstanceStruct;

#endif                                 /*typedef_SFc12_SimulationModelInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c12_SimulationModel_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c12_SimulationModel_get_check_sum(mxArray *plhs[]);
extern void c12_SimulationModel_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
