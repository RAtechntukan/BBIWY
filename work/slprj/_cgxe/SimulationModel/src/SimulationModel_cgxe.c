/* Include files */

#include "SimulationModel_cgxe.h"
#include "m_9c1cCXtKK36AOL9hRIyMrH.h"

unsigned int cgxe_SimulationModel_method_dispatcher(SimStruct* S, int_T method,
  void* data)
{
  if (ssGetChecksum0(S) == 2732742030 &&
      ssGetChecksum1(S) == 4007208234 &&
      ssGetChecksum2(S) == 2915213310 &&
      ssGetChecksum3(S) == 165730056) {
    method_dispatcher_9c1cCXtKK36AOL9hRIyMrH(S, method, data);
    return 1;
  }

  return 0;
}
