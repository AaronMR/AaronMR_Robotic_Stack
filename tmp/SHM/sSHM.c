#define S_FUNCTION_NAME  sSHM
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "comStruc.h"

#ifdef _WIN32
#else
    #include <rtai_shm.h>
    #include <rtai_nam2num.h>
    #include <sys/io.h>
#endif


/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;  
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 *     constants      *
 *====================*/
//CONSTANTS for DVECTOR
#define DVECSHMIN 0
#define DVECSHMOUT 1

/*====================*
 *  global variables  *
 *====================*/


/*====================*
 *    user methods    *
 *====================*/


/*====================*
 * S-function methods *
 *====================*/

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortRequiredContiguous(S, 0, true);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    //ssSetInputPortDataType(S,0,SS_DOUBLE);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 3);

    ssSetNumSampleTimes(S, 1);
    ssSetNumDWork(S, 2);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);

    //Pointer SHM_IN
    ssSetDWorkWidth(S, DVECSHMIN, 1);
    ssSetDWorkDataType(S, DVECSHMIN, SS_POINTER);

    //Pointer SHM_OUT
    ssSetDWorkWidth(S, DVECSHMOUT, 1);
    ssSetDWorkDataType(S, DVECSHMOUT, SS_POINTER);

    ssSetOptions(S, 0);
}




static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}



#undef MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
      struct comStruc_IN **dataIN = (struct comStruc_IN**) ssGetDWork(S,DVECSHMIN);
      struct comStruc_OUT **dataOUT = (struct comStruc_OUT**) ssGetDWork(S,DVECSHMOUT);
#ifdef _WIN32
#else
      dataIN[0] = rtai_malloc(nam2num(SHMNAM_IN), sizeof(struct comStruc_IN));
      dataOUT[0] = rtai_malloc(nam2num(SHMNAM_OUT), sizeof(struct comStruc_OUT));
      printf("OUT: %d, IN: %d\n", sizeof(struct comStruc_OUT), sizeof(struct comStruc_IN));
#endif
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T       *y = ssGetOutputPortRealSignal(S,0);
    struct comStruc_IN **dataIN = (struct comStruc_IN**) ssGetDWork(S,DVECSHMIN);

    y[0] = (float) dataIN[0]->index_counter;
    y[1] = (float) dataIN[0]->sin_value;
    y[2] = (float) dataIN[0]->cos_value;

     printf("value IN 1 %d\n", dataIN[0]->index_counter);
     printf("value IN 2 %f\n", dataIN[0]->sin_value);
     printf("value IN 3 %f\n\n", dataIN[0]->cos_value);
  

    //printf("%s\n", "outputOUT");
}


#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
     const real_T *u = (const real_T*) ssGetInputPortRealSignal(S,0);
     struct comStruc_OUT **dataOUT = (struct comStruc_OUT**) ssGetDWork(S,DVECSHMOUT);

     dataOUT[0]->index_counter = (int32_t) u[0];
     dataOUT[0]->sin_value = (float) u[1];
     dataOUT[0]->cos_value = (float) u[2];

     printf("value OUT 1 %d\n", dataOUT[0]->index_counter);
     printf("value OUT 2 %f\n", dataOUT[0]->sin_value);
     printf("value OUT 3 %f\n\n", dataOUT[0]->cos_value);
  }
#endif /* MDL_UPDATE */



#undef MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
      struct comStruc_IN **dataIN = (struct comStruc_IN**) ssGetDWork(S,DVECSHMIN);
      struct comStruc_OUT **dataOUT = (struct comStruc_IN**) ssGetDWork(S,DVECSHMOUT);
#ifdef _WIN32
#else
      rtai_free(nam2num(SHMNAM_IN), dataIN[0]);
      rtai_free(nam2num(SHMNAM_OUT), dataOUT[0]);
      printf("%s\n", "terminate");
#endif

}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
