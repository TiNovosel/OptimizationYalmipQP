/**********************************/
/*                                */
/* Matlab ver. 7.0 (R14)          */
/*                                */
/* (c) Branimir Skugor April 2012.*/
/*                                */
/**********************************/

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME torqueDemandstd

#include "simstruc.h"    /* Defines SimStruct and corresponding macros */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*************************** Transmission parameters ***************************/
#define	r	(double)(0.3170)	/* Wheel radius (RHEVE2011) */  
#define	h	(double)(2.24)      /* Planetary gear ratio (VOLTEC ELECTRIC DRIVE SYSTEM OVERVIEW)*/
#define	io	(double)(2.16)      /* Bevel gear and differential ratio (VOLTEC ELECTRIC DRIVE SYSTEM OVERVIEW)*/
/*******************************************************************************/
/*******************************************************************************/
#define	T	(double)(1.0)	/* Low_level_ctrl discretization time */
#define MAX (int_T)(500)

#define u(element) (*uPtrs[element])
#define NUM_OF_ARGS	0	/* Number of input arguments - data organized in ROW vectors */
#define NUM_OF_IN	8	/* Number of inputs:  1 - speed, 2 - torque ... */
#define NUM_OF_OUT	3	/* Number of outputs: 1 - speed, 2 - torque, 3 - trq. derivative ... */
#define NUM_OF_REAL	0	/* Number of real work vector elements - static vars. */
#define NUM_OF_PTR	0	/* Number of ptr. work vector elements - static vars. */
#define NUM_OF_INT	0	/* Number of ptr. work vector elements - static vars. */

static void mdlInitializeSizes(SimStruct *S)
{
    //real_T *id, *tq, *sc;
    ssSetNumSFcnParams(S, NUM_OF_ARGS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
    ssSetNumContStates(S, 0);   /* number of continuous states */
    ssSetNumDiscStates(S, 0);   /* number of discrete states */

    /* Configure the input ports. */
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, NUM_OF_IN); 
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, NUM_OF_OUT);
    ssSetNumSampleTimes(S, 1);   /* number of sample times */

    ssSetNumRWork(S,NUM_OF_REAL);   /* number of real work vector elements */
    ssSetNumIWork(S, NUM_OF_INT);   /* number of integer work vector elements*/
    ssSetNumPWork(S, NUM_OF_PTR);   /* number of pointer work vector elements*/
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
    
    /* DWork vectors initialization */
    ssSetNumDWork(S, 1);
    ssSetDWorkWidth(S, 0, MAX);
    ssSetDWorkDataType(S, 0, SS_DOUBLE);
    
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, T);
	ssSetOffsetTime(S, 0, 0);
}
/*************************************/
/* FUNCTION mdlInitializeConditions  */
/*************************************/
#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
    int i;
    real_T *buffer = (real_T*) ssGetDWork(S,0);
    
	if (ssGetSFcnParamsCount(S) != NUM_OF_ARGS)
	{
		#ifdef MATLAB_MEX_FILE
			ssSetErrorStatus(S,"Wrong number of input args!");
		#else
			printf("\nWrong number of input args!");
			exit(0);
		#endif    
    }
    printf("Inicijalizacija polja\n");
    /* Global array initialization */
    for(i = 0; i<MAX; i++){
        buffer[i] = 0;
    }
}
#endif

/**********************************************/
/*  FUNCTION mdlOutputs - Low-level control   */
/**********************************************/

static void mdlOutputs(SimStruct *S, int_T tid)
{
   InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
   real_T *buffer = (real_T*) ssGetDWork(S,0);
   real_T *y = ssGetOutputPortRealSignal(S,0);
   
   real_T vv, tcd, wcd, suma = 0, tcdmean, tcdstd, pom;
   real_T hyst, hystD, hystG, tcdstdD, tcdstdG;
   int_T i, brojac = 0, bufferSize;
   
   vv = u(0);           // vehicle velocity
   tcd = u(1);          // driver torque command
   bufferSize = u(2);   // buffer size
   hystD = u(3);        // lower limit of hysteresis width
   hystG = u(4);        // upper limit of hysteresis width
   tcdstdD = u(5);       // lower limit of power standard deviation 
   tcdstdG = u(6);       // upper limit of power standard deviation 
   brojac = u(7);       // variable that memorizes place in which  new Pdst.dev. will be stored
   
   /* Shaft velocity */
   wcd = vv/r;

   if(brojac >= bufferSize){
        brojac = 0; 
   }
   /* Pd input in buffer*/
   buffer[brojac] = tcd;
   
   /****** Pd mean value calculation ******/
   suma = 0;
   for(i = 0; i<bufferSize; i++){
        suma = suma + buffer[i];
        //printf("buffer[%d] = %f\n",i,buffer[i]);
   }
   tcdmean = suma/bufferSize;
   /****** Pd standard deviation calculation ******/
   suma = 0;
   for(i = 0; i<bufferSize; i++){
        suma = suma + (buffer[i] - tcdmean)*(buffer[i] - tcdmean);
   }
   tcdstd = sqrt(suma/(bufferSize - 1));

   if(tcdstd>tcdstdG){
        pom = tcdstdG;
   }else{
        pom = tcdstd;
   }
   /* Hysteresis width calculation */
   hyst = (hystG - hystD)*(pom - tcdstdD)/(tcdstdG - tcdstdD) + hystD;
   //printf("hyst = %f, brojac = %d, tcdstdG = %f\n", hyst, brojac, tcdstdG);
   brojac = brojac + 1;  
   
   y[0] = hyst;        // driver power demand standard deviation
   y[1] = tcdstd;
   y[2] = brojac;
   
}
/**********************/
/* FUNCTION mdlUpdate */
/**********************/
static void mdlUpdate(real_T *x, real_T *u, SimStruct *S, int_T tid)
{
}
/***************************/
/* FUNCTION mdlDerivatives */
/***************************/
static void mdlDerivatives(real_T *dx, real_T *x, real_T *u, SimStruct *S, int_T tid)
{
}
/*************************/
/* FUNCTION mdlTerminate */
/*************************/
static void mdlTerminate(SimStruct *S)
{
	int_T i;     
/* Oslobadja se memorija alocirana za radne vektore */      
	
	for(i=0;i<ssGetNumPWork(S);i++) {
		if (ssGetPWorkValue(S,i) != NULL) {
			free(ssGetPWorkValue(S,i));
		}
	}	
}

#ifdef    MATLAB_MEX_FILE  /* Is file being compiled into the MEX-file ? */
#include "simulink.c"      /* Add functions for linking with Matlab */
#else
#include "cg_sfun.h"
#endif
