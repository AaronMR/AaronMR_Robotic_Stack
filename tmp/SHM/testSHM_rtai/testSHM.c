/*
 * testSHM.c
 *
 * Real-Time Workshop code generation for Simulink model "testSHM.mdl".
 *
 * Model Version              : 1.5
 * Real-Time Workshop version : 7.2  (R2008b)  04-Aug-2008
 * C source code generated on : Fri Dec 10 10:34:20 2010
 */

#include "testSHM.h"
#include "testSHM_private.h"
#include "mdl_info.h"
#include "testSHM_bio.c"
#include "testSHM_pt.c"

/* Block signals (auto storage) */
BlockIO_testSHM testSHM_B;

/* Block states (auto storage) */
D_Work_testSHM testSHM_DWork;

/* Real-time model */
RT_MODEL_testSHM testSHM_M_;
RT_MODEL_testSHM *testSHM_M = &testSHM_M_;

/* Model output function */
static void testSHM_output(int_T tid)
{
  /* local block i/o variables */
  real_T rtb_Gain;

  /* Level2 S-Function Block: '<Root>/S-Function' (sSHM) */
  {
    SimStruct *rts = testSHM_M->childSfunctions[0];
    sfcnOutputs(rts, 0);
  }

  /* Level2 S-Function Block: '<Root>/RTAI_SCOPE' (sfun_rtai_scope) */
  {
    SimStruct *rts = testSHM_M->childSfunctions[1];
    sfcnOutputs(rts, 0);
  }

  /* DigitalClock: '<Root>/Digital Clock' */
  rtb_Gain = testSHM_M->Timing.t[0];

  /* Gain: '<Root>/Gain' */
  rtb_Gain *= testSHM_P.Gain_Gain;

  /* SignalConversion: '<Root>/TmpHiddenBufferAtS-FunctionInport1' incorporates:
   *  Constant: '<Root>/Bias'
   *  Sum: '<Root>/Sum'
   *  Sum: '<Root>/Sum1'
   *  Trigonometry: '<Root>/Trigonometric Function'
   *  Trigonometry: '<Root>/Trigonometric Function1'
   */
  testSHM_B.TmpHiddenBufferAtSFunctionInpor[0] = rtb_Gain;
  testSHM_B.TmpHiddenBufferAtSFunctionInpor[1] = sin(rtb_Gain) +
    testSHM_P.Bias_Value;
  testSHM_B.TmpHiddenBufferAtSFunctionInpor[2] = cos(rtb_Gain) +
    testSHM_P.Bias_Value;

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model update function */
static void testSHM_update(int_T tid)
{
  /* Level2 S-Function Block: '<Root>/S-Function' (sSHM) */
  {
    SimStruct *rts = testSHM_M->childSfunctions[0];
    sfcnUpdate(rts, 0);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Update absolute time for base rate */
  if (!(++testSHM_M->Timing.clockTick0))
    ++testSHM_M->Timing.clockTickH0;
  testSHM_M->Timing.t[0] = testSHM_M->Timing.clockTick0 *
    testSHM_M->Timing.stepSize0 + testSHM_M->Timing.clockTickH0 *
    testSHM_M->Timing.stepSize0 * 4294967296.0;

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model initialize function */
void testSHM_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)testSHM_M,0,
                sizeof(RT_MODEL_testSHM));
  rtsiSetSolverName(&testSHM_M->solverInfo,"FixedStepDiscrete");
  testSHM_M->solverInfoPtr = (&testSHM_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = testSHM_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    testSHM_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    testSHM_M->Timing.sampleTimes = (&testSHM_M->Timing.sampleTimesArray[0]);
    testSHM_M->Timing.offsetTimes = (&testSHM_M->Timing.offsetTimesArray[0]);

    /* task periods */
    testSHM_M->Timing.sampleTimes[0] = (0.001);

    /* task offsets */
    testSHM_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(testSHM_M, &testSHM_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = testSHM_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    testSHM_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(testSHM_M, 10.0);
  testSHM_M->Timing.stepSize0 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    testSHM_M->rtwLogInfo = &rt_DataLoggingInfo;
    rtliSetLogXSignalInfo(testSHM_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(testSHM_M->rtwLogInfo, (NULL));
    rtliSetLogT(testSHM_M->rtwLogInfo, "tout");
    rtliSetLogX(testSHM_M->rtwLogInfo, "");
    rtliSetLogXFinal(testSHM_M->rtwLogInfo, "");
    rtliSetSigLog(testSHM_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(testSHM_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(testSHM_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(testSHM_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(testSHM_M->rtwLogInfo, 1);
    rtliSetLogY(testSHM_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(testSHM_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(testSHM_M->rtwLogInfo, (NULL));
  }

  testSHM_M->solverInfoPtr = (&testSHM_M->solverInfo);
  testSHM_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&testSHM_M->solverInfo, 0.001);
  rtsiSetSolverMode(&testSHM_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  testSHM_M->ModelData.blockIO = ((void *) &testSHM_B);
  (void) memset(((void *) &testSHM_B),0,
                sizeof(BlockIO_testSHM));

  /* parameters */
  testSHM_M->ModelData.defaultParam = ((real_T *) &testSHM_P);

  /* states (dwork) */
  testSHM_M->Work.dwork = ((void *) &testSHM_DWork);
  (void) memset((void *)&testSHM_DWork, 0,
                sizeof(D_Work_testSHM));

  /* C API for Parameter Tuning and/or Signal Monitoring */
  {
    static ModelMappingInfo mapInfo;
    (void) memset((char_T *) &mapInfo,0,
                  sizeof(mapInfo));

    /* block signal monitoring map */
    mapInfo.Signals.blockIOSignals = &rtBIOSignals[0];
    mapInfo.Signals.numBlockIOSignals = 2;

    /* parameter tuning maps */
    mapInfo.Parameters.blockTuning = &rtBlockTuning[0];
    mapInfo.Parameters.variableTuning = &rtVariableTuning[0];
    mapInfo.Parameters.parametersMap = rtParametersMap;
    mapInfo.Parameters.dimensionsMap = rtDimensionsMap;
    mapInfo.Parameters.numBlockTuning = 4;
    mapInfo.Parameters.numVariableTuning = 0;
    testSHM_M->SpecialInfo.mappingInfo = (&mapInfo);
  }

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &testSHM_M->NonInlinedSFcns.sfcnInfo;
    testSHM_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus(testSHM_M)));
    rtssSetNumRootSampTimesPtr(sfcnInfo, &testSHM_M->Sizes.numSampTimes);
    rtssSetTPtrPtr(sfcnInfo, &rtmGetTPtr(testSHM_M));
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(testSHM_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(testSHM_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput(testSHM_M));
    rtssSetStepSizePtr(sfcnInfo, &testSHM_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested(testSHM_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &testSHM_M->ModelData.derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo,
      &testSHM_M->ModelData.zCCacheNeedsReset);
    rtssSetBlkStateChangePtr(sfcnInfo, &testSHM_M->ModelData.blkStateChange);
    rtssSetSampleHitsPtr(sfcnInfo, &testSHM_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo, &testSHM_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &testSHM_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &testSHM_M->solverInfoPtr);
  }

  testSHM_M->Sizes.numSFcns = (2);

  /* register each child */
  {
    (void) memset((void *)&testSHM_M->NonInlinedSFcns.childSFunctions[0],0,
                  2*sizeof(SimStruct));
    testSHM_M->childSfunctions = (&testSHM_M->
      NonInlinedSFcns.childSFunctionPtrs[0]);
    testSHM_M->childSfunctions[0] = (&testSHM_M->
      NonInlinedSFcns.childSFunctions[0]);
    testSHM_M->childSfunctions[1] = (&testSHM_M->
      NonInlinedSFcns.childSFunctions[1]);

    /* Level2 S-Function Block: testSHM/<Root>/S-Function (sSHM) */
    {
      SimStruct *rts = testSHM_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod = testSHM_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset = testSHM_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap = testSHM_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset((void*)sfcnPeriod,0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset,0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &testSHM_M->NonInlinedSFcns.blkInfo2[0]);
        ssSetRTWSfcnInfo(rts, testSHM_M->sfcnInfo);
      }

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &testSHM_M->NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &testSHM_M->NonInlinedSFcns.methods3[0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &testSHM_M->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, testSHM_B.TmpHiddenBufferAtSFunctionInpor);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 3);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &testSHM_M->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 1);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 3);
          ssSetOutputPortSignal(rts, 0, ((real_T *) testSHM_B.SFunction));
        }
      }

      /* path info */
      ssSetModelName(rts, "S-Function");
      ssSetPath(rts, "testSHM/S-Function");
      ssSetRTModel(rts,testSHM_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* work vectors */
      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &testSHM_M->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &testSHM_M->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* DWORK1 */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &testSHM_DWork.SFunction_DWORK1);

        /* DWORK2 */
        ssSetDWorkWidth(rts, 1, 1);
        ssSetDWorkDataType(rts, 1,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &testSHM_DWork.SFunction_DWORK2);
      }

      /* registration */
      sSHM(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: testSHM/<Root>/RTAI_SCOPE (sfun_rtai_scope) */
    {
      SimStruct *rts = testSHM_M->childSfunctions[1];

      /* timing info */
      time_T *sfcnPeriod = testSHM_M->NonInlinedSFcns.Sfcn1.sfcnPeriod;
      time_T *sfcnOffset = testSHM_M->NonInlinedSFcns.Sfcn1.sfcnOffset;
      int_T *sfcnTsMap = testSHM_M->NonInlinedSFcns.Sfcn1.sfcnTsMap;
      (void) memset((void*)sfcnPeriod,0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset,0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &testSHM_M->NonInlinedSFcns.blkInfo2[1]);
        ssSetRTWSfcnInfo(rts, testSHM_M->sfcnInfo);
      }

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &testSHM_M->NonInlinedSFcns.methods2[1]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &testSHM_M->NonInlinedSFcns.methods3[1]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 3);
        ssSetPortInfoForInputs(rts,
          &testSHM_M->NonInlinedSFcns.Sfcn1.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &testSHM_M->NonInlinedSFcns.Sfcn1.UPtrs0;
          sfcnUPtrs[0] = &testSHM_B.SFunction[0];
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }

        /* port 1 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &testSHM_M->NonInlinedSFcns.Sfcn1.UPtrs1;
          sfcnUPtrs[0] = &testSHM_B.SFunction[1];
          ssSetInputPortSignalPtrs(rts, 1, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidth(rts, 1, 1);
        }

        /* port 2 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &testSHM_M->NonInlinedSFcns.Sfcn1.UPtrs2;
          sfcnUPtrs[0] = &testSHM_B.SFunction[2];
          ssSetInputPortSignalPtrs(rts, 2, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 2, 1);
          ssSetInputPortWidth(rts, 2, 1);
        }
      }

      /* path info */
      ssSetModelName(rts, "RTAI_SCOPE");
      ssSetPath(rts, "testSHM/RTAI_SCOPE");
      ssSetRTModel(rts,testSHM_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &testSHM_M->NonInlinedSFcns.Sfcn1.params;
        ssSetSFcnParamsCount(rts, 2);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)&testSHM_P.RTAI_SCOPE_P1_Size[0]);
        ssSetSFcnParam(rts, 1, (mxArray*)&testSHM_P.RTAI_SCOPE_P2_Size[0]);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &testSHM_DWork.RTAI_SCOPE_PWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &testSHM_M->NonInlinedSFcns.Sfcn1.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &testSHM_M->NonInlinedSFcns.Sfcn1.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* PWORK */
        ssSetDWorkWidth(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &testSHM_DWork.RTAI_SCOPE_PWORK);
      }

      /* registration */
      sfun_rtai_scope(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.001);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetInputPortConnected(rts, 2, 1);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
    }
  }
}

/* Model terminate function */
void testSHM_terminate(void)
{
  /* Level2 S-Function Block: '<Root>/S-Function' (sSHM) */
  {
    SimStruct *rts = testSHM_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Level2 S-Function Block: '<Root>/RTAI_SCOPE' (sfun_rtai_scope) */
  {
    SimStruct *rts = testSHM_M->childSfunctions[1];
    sfcnTerminate(rts);
  }
}

/*========================================================================*
 * Start of GRT compatible call interface                                 *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  testSHM_output(tid);
}

void MdlUpdate(int_T tid)
{
  testSHM_update(tid);
}

void MdlInitializeSizes(void)
{
  testSHM_M->Sizes.numContStates = (0);/* Number of continuous states */
  testSHM_M->Sizes.numY = (0);         /* Number of model outputs */
  testSHM_M->Sizes.numU = (0);         /* Number of model inputs */
  testSHM_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  testSHM_M->Sizes.numSampTimes = (1); /* Number of sample times */
  testSHM_M->Sizes.numBlocks = (11);   /* Number of blocks */
  testSHM_M->Sizes.numBlockIO = (2);   /* Number of block outputs */
  testSHM_M->Sizes.numBlockPrms = (8); /* Sum of parameter "widths" */
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  {
    /* user code (Start function Header) */
    testSHM_InitializeBlockIOMap();
    testSHM_InitializeParametersMap();

    /* Level2 S-Function Block: '<Root>/S-Function' (sSHM) */
    {
      SimStruct *rts = testSHM_M->childSfunctions[0];
      sfcnStart(rts);
      if (ssGetErrorStatus(rts) != (NULL))
        return;
    }

    /* Level2 S-Function Block: '<Root>/RTAI_SCOPE' (sfun_rtai_scope) */
    {
      SimStruct *rts = testSHM_M->childSfunctions[1];
      sfcnStart(rts);
      if (ssGetErrorStatus(rts) != (NULL))
        return;
    }
  }

  MdlInitialize();
}

RT_MODEL_testSHM *testSHM(void)
{
  testSHM_initialize(1);
  return testSHM_M;
}

void MdlTerminate(void)
{
  testSHM_terminate();
}

/*========================================================================*
 * End of GRT compatible call interface                                   *
 *========================================================================*/
