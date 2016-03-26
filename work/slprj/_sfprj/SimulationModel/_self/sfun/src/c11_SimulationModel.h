#ifndef __c11_SimulationModel_h__
#define __c11_SimulationModel_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
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

#ifndef typedef_c11_EPosition
#define typedef_c11_EPosition

typedef enum EPosition c11_EPosition;

#endif                                 /*typedef_c11_EPosition*/

#ifndef struct_bus_RemainingPosition_tag
#define struct_bus_RemainingPosition_tag

struct bus_RemainingPosition_tag
{
  real32_T Angle;
  real32_T Distance;
};

#endif                                 /*struct_bus_RemainingPosition_tag*/

#ifndef typedef_c11_bus_RemainingPosition
#define typedef_c11_bus_RemainingPosition

typedef struct bus_RemainingPosition_tag c11_bus_RemainingPosition;

#endif                                 /*typedef_c11_bus_RemainingPosition*/

#ifndef typedef_c11_cell_0
#define typedef_c11_cell_0

typedef struct {
  char_T f1[13];
  char_T f2[8];
  char_T f3[7];
  char_T f4[13];
  char_T f5[13];
  char_T f6[9];
} c11_cell_0;

#endif                                 /*typedef_c11_cell_0*/

#ifndef struct_bus_Robot_tag
#define struct_bus_Robot_tag

struct bus_Robot_tag
{
  int32_T EncodersCount[2];
  int16_T Distance[6];
  int8_T Bearing[6];
  c11_EPosition ScannerStatus;
  int8_T ScannerValues[3];
  c11_bus_RemainingPosition Remaining;
};

#endif                                 /*struct_bus_Robot_tag*/

#ifndef typedef_c11_bus_Robot
#define typedef_c11_bus_Robot

typedef struct bus_Robot_tag c11_bus_Robot;

#endif                                 /*typedef_c11_bus_Robot*/

#ifndef typedef_SFc11_SimulationModelInstanceStruct
#define typedef_SFc11_SimulationModelInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c11_sfEvent;
  boolean_T c11_isStable;
  boolean_T c11_doneDoubleBufferReInit;
  uint8_T c11_is_active_c11_SimulationModel;
  c11_bus_Robot *c11_u;
  c11_bus_Robot *c11_y;
} SFc11_SimulationModelInstanceStruct;

#endif                                 /*typedef_SFc11_SimulationModelInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c11_SimulationModel_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c11_SimulationModel_get_check_sum(mxArray *plhs[]);
extern void c11_SimulationModel_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
