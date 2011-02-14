/*
 * testSHM_pt.c
 *
 * Real-Time Workshop code generation for Simulink model "testSHM.mdl".
 *
 * Model Version              : 1.5
 * Real-Time Workshop version : 7.2  (R2008b)  04-Aug-2008
 * C source code generated on : Fri Dec 10 10:34:20 2010
 */

#ifndef _PT_INFO_testSHM_
#define _PT_INFO_testSHM_
#include "pt_info.h"

/* Tunable block parameters */
static const BlockTuning rtBlockTuning[] = {
  /* blockName, parameterName,
   * class, nRows, nCols, nDims, dimsOffset, source, dataType, numInstances,
   * mapOffset
   */

  /* S-Function */
  { "testSHM/RTAI_SCOPE", "P1",
    { rt_SCALAR, 1, 1, 2, -1, rt_SL_PARAM, SS_DOUBLE, 1, 0 }
  },

  /* S-Function */
  { "testSHM/RTAI_SCOPE", "P2",
    { rt_SCALAR, 1, 1, 2, -1, rt_SL_PARAM, SS_DOUBLE, 1, 1 }
  },

  /* Constant */
  { "testSHM/Bias", "Value",
    { rt_SCALAR, 1, 1, 2, -1, rt_SL_PARAM, SS_DOUBLE, 1, 2 }
  },

  /* Gain */
  { "testSHM/Gain", "Gain",
    { rt_SCALAR, 1, 1, 2, -1, rt_SL_PARAM, SS_DOUBLE, 1, 3 }
  },

  { NULL, NULL,
    { (ParamClass)0, 0, 0, 0, 0, (ParamSource)0, 0, 0, 0 }
  }
};

/* Tunable variable parameters */
static const VariableTuning rtVariableTuning[] = {
  /* variableName,
   * class, nRows, nCols, nDims, dimsOffset, source, dataType, numInstances,
   * mapOffset
   */
  { NULL,
    { (ParamClass)0, 0, 0, 0, 0, (ParamSource)0, 0, 0, 0 }
  }
};

static void * rtParametersMap[4];
void testSHM_InitializeParametersMap(void)
{
  rtParametersMap[0] = &testSHM_P.RTAI_SCOPE_P1;/* 0 */
  rtParametersMap[1] = &testSHM_P.RTAI_SCOPE_P2;/* 1 */
  rtParametersMap[2] = &testSHM_P.Bias_Value;/* 2 */
  rtParametersMap[3] = &testSHM_P.Gain_Gain;/* 3 */
}

static uint_T const rtDimensionsMap[] = {
  0                                    /* Dummy */
};

#endif                                 /* _PT_INFO_testSHM_ */
