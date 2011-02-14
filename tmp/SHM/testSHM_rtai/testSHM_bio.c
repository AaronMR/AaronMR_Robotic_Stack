/*
 * testSHM_bio.c
 *
 * Real-Time Workshop code generation for Simulink model "testSHM.mdl".
 *
 * Model Version              : 1.5
 * Real-Time Workshop version : 7.2  (R2008b)  04-Aug-2008
 * C source code generated on : Fri Dec 10 10:34:20 2010
 */

#ifndef BLOCK_IO_SIGNALS
#define BLOCK_IO_SIGNALS
#include "bio_sig.h"

/* Block output signal information */
static BlockIOSignals rtBIOSignals[] = {
  /* blockName,
   * signalName, portNumber, signalWidth, signalAddr, dtName, dtSize
   */
  {
    "testSHM/S-Function",
    "(NULL)",
    0,
    3,
    (NULL),
    "double",
    sizeof(real_T)
  },

  {
    "testSHM/Opaque",
    "(NULL)",
    0,
    3,
    (NULL),
    "double",
    sizeof(real_T)
  },

  {
    (NULL),
    (NULL),
    0,
    0,
    0,
    (NULL),
    0
  }
};

#endif                                 /* BLOCK_IO_SIGNALS */

void testSHM_InitializeBlockIOMap(void)
{
  rtBIOSignals[0].signalAddr = &testSHM_B.SFunction[0];
  rtBIOSignals[1].signalAddr = &testSHM_B.TmpHiddenBufferAtSFunctionInpor[0];
}
