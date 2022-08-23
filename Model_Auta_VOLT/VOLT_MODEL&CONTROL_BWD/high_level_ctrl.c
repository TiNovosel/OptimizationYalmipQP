/**********************************/
/*                                */
/* Matlab ver. 7.0 (R14)          */
/*                                */
/* (c) Branimir Skugor April 2012.*/
/*                                */
/**********************************/
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME high_level_ctrl

#include "simstruc.h"    /* Defines SimStruct and corresponding macros */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*************************** Transmission parameters ***************************/
#define	r	(double)(0.3170)	/* Wheel radius (RHEVE2011) */  
#define	h	(double)(2.24)      /* Planetary gear ratio (VOLTEC ELECTRIC DRIVE SYSTEM OVERVIEW)*/
#define	io	(double)(2.16)      /* Bevel gear and differential ratio (VOLTEC ELECTRIC DRIVE SYSTEM OVERVIEW)*/
/*******************************************************************************/
#define	T	(double)(1.0)	/* High_level_ctrl discretization time */
/*************************************************************************************************************/
/***************************** ICE max torque data - polynom approximation koefficients **********************/
/*************************************************************************************************************/
/* 1. segment */
#define ICEwe1   (double)(100.0)
/**************/
#define ICEk11   (double)(17.44680851064)
#define ICEk12   (double)(-1744.68085106382)

/* 2. segment */
#define ICEwe2   (double)(104.7)
/**************/
#define ICEk21   (double)(-0.00000541357060)
#define ICEk22   (double)(0.00256447952514)
#define ICEk23   (double)(-0.12668770514996)
#define ICEk24   (double)(73.39023225776117)

/* 3. segment */
#define ICEwe3   (double)(230.4)
/**************/
#define ICEk31   (double)(0.00000171644911)
#define ICEk32   (double)(-0.00188598498345)
#define ICEk33   (double)(0.74341649645164)
#define ICEk34   (double)(21.88823420619279)

/* 4. segment */
#define ICEwe4   (double)(450.3)
#define ICEwend   (double)(523.6)
/**************/
#define ICEk41  (double)(-0.00002307551)
#define ICEk42  (double)(0.03268552155)
#define ICEk43  (double)(-15.53740608275)
#define ICEk44  (double)(2606.93919276009)

/***********************************************************************************************************/
#define we10    (double)(100.0)     // Minimal ICE operating speed [rad/s]
/***********************************************************************************************************/
#define EPS     (double)(1.e-3)     /* Hooke-Jeeves parameter - Control variables accuracy demanded */
/***********************************************************************************************************/
/******************* MG1 machine max torque data - polynom approximation koefficients **********************/
/***********************************************************************************************************/
/* 1. segment */
#define MG1w1   (double)(26.2)
/**************/
#define MG1k11   (double)(0.114503816794)
#define MG1k12   (double)(178.0)

/* 2. segment */
#define MG1w2   (double)(314.2)
/**************/
#define MG1k21   (double)(-0.013888888889)
#define MG1k22   (double)(181.363888888889)

/* 3. segment */
#define MG1w3   (double)(392.7)
/**************/
#define MG1k31   (double)(-0.00000001801364)
#define MG1k32   (double)(-0.00216538134788)
#define MG1k33   (double)(1.34640418757991)
#define MG1k34   (double)(-31.71144469930563)

/* 4. segment */
#define MG1wend   (double)(629.0)
/**************/
#define MG1k41  (double)(-0.000000490840)
#define MG1k42  (double)(0.000911798621)
#define MG1k43  (double)(-0.845150037350)
#define MG1k44  (double)(383.004008311320)

/***********************************************************************************************************/
/******************* MG2 machine max torque data - polynom approximation koefficients **********************/
/***********************************************************************************************************/
/* 1. segment */
#define MG2w1   (double)(61.0)
/**************/
#define MG2k11   (double)(0.0)
#define MG2k12   (double)(350.0)

/* 2. segment */
#define MG2w2   (double)(153.0)
/**************/
#define MG2k21   (double)(0.108695652174)
#define MG2k22   (double)(343.369565217391)

/* 3. segment */
#define MG2w3   (double)(307.0)
/**************/
#define MG2k31   (double)(0.0)
#define MG2k32   (double)(360.0)

/* 4. segment */
#define MG2w4  (double)(429.0)
#define MG2wend  (double)(1100.0)
/**************/
#define MG2k41  (double)(-0.000011153733)
#define MG2k42  (double)(0.009895016081)
#define MG2k43  (double)(-3.267156617376)
#define MG2k44  (double)(753.148749468923)

/* 5. segment */
#define MG2wend   (double)(1100.0)
/**************/
#define MG2k51  (double)(-0.000000282722)
#define MG2k52  (double)( 0.001048892698)
#define MG2k53  (double)(-1.374206009406)
#define MG2k54  (double)(710.682258191761)

/***********************************************************************************************************/
/******************************************** SoC controller parameters *************************************************/
/***********************************************************************************************************/
#define Ksoc    (double)(1.5e5)      // SoC error proportional gain 1.5
#define SoC_min (double)(0.28)       // Minimal SoC demanded in CS (charge sustenance) mode
#define SoC_max (double)(0.32)       // Maximal SoC demanded in CS (charge sustenance) mode
#define P2      (double)(1.0e4)      // ICE turning threshold (upper)
#define P0      (double)(0.0e3)      // ICE turning threshold (lower)
#define P1      (double)(8.5e3)      // Minimum ICE power demanded by SoC controller
#define Pemax   (double)(62832.0)    // Maximum ICE power 

/***********************************************************************************************************/
/******************************************** Battery data *************************************************/
/***********************************************************************************************************/
#define Rch (double)(0.056128)        /* [ohm] */
#define Rdch (double)(0.115904)       /* [ohm] */
#define Qbat (double)(162000.0)       /* [As] (45 Ah)*/
#define Pb_max (double)(295290.0)     /* Battery theoretical max. power*/  
/* Open circuit voltage curve koefficients */
#define uock1   (double)(138.5933) 
#define uock2   (double)(-211.9425) 
#define uock3   (double)(134.4433) 
#define uock4   (double)(338.7939) 
/***********************************************************************************************************/
/******************************* Boundary curve between EV and TMEV polynom approximation koefficients *************************/
/***********************************************************************************************************/
#define a0      (double)(24.35349226329694)
#define a1      (double)(0.86752069775956)
#define a2      (double)(-0.00329661429852)
/***********************************************************************************************************/
/******************************* Boundary curve between SHEV and SPHEV polynom approximation koefficients *************************/
/***********************************************************************************************************/
#define HEVa5    (double)(0.000002512809)
#define HEVa4    (double)(-0.000622389026)
#define HEVa3    (double)(0.059221024242)
#define HEVa2    (double)(-2.786466772804)
#define HEVa1    (double)(74.354772316656)
#define HEVa0    (double)(-256.345831237298)
/***********************************************************************************************************/

#define u(element) (*uPtrs[element])
#define NUM_OF_ARGS	3	/* Number of input arguments - data organized in ROW vectors */
#define NUM_OF_IN	16	/* Number of inputs:  1 - speed, 2 - torque ... */
#define NUM_OF_OUT	20	/* Number of outputs: 1 - speed, 2 - torque, 3 - trq. derivative ... */
#define NUM_OF_REAL	0	/* Number of real work vector elements - static vars. */
#define NUM_OF_PTR	0	/* Number of ptr. work vector elements - static vars. */
#define NUM_OF_INT	0	/* Number of ptr. work vector elements - static vars. */

/* ICE engine fuel mass flow map*/
#define AE(S)           ssGetSFcnParam(S, 0)
/* Ae matrix dimensions */
#define Aex    (int_T)(141)
#define Aey    (int_T)(525)

/* M/G1 machine efficiency map */
#define AMG1(S)          ssGetSFcnParam(S, 1)
/* Amg1 matrix dimensions */
#define Amg1x       (int_T)(201)
#define Amg1y       (int_T)(630)

/* M/G2 machine efficiency map */
#define AMG2(S)          ssGetSFcnParam(S, 2)
/* Amg2 matrix dimensions */
#define Amg2x    (int_T)(366)
#define Amg2y    (int_T)(1044)

/******************************** Global control variables  **************************************************/
real_T weUpr = 0.0; real_T teUpr = 0.0;
/******************************************************************************************************/

static real_T etaICE(SimStruct *S, real_T, real_T);
static real_T etaMG1(SimStruct *S, real_T, real_T);
static real_T etaMG2(SimStruct *S, real_T, real_T);
static real_T interpolacija(SimStruct *S, real_T, real_T, int_T, int_T, real_T *Amg);
static real_T ICELimit(real_T);
static real_T MG1Limit(real_T);
static real_T MG2Limit(real_T);
static real_T boundary(real_T);
static real_T GMboundary(real_T);
static real_T HEVboundary(real_T);
static real_T electricMode(SimStruct *S, real_T, real_T);  // electricMode(S, wcd, tcd) - searching for optimal machines speed
static void optim2DECMS(SimStruct *S, real_T, real_T, real_T, int_T);  // optim2DECMS(S, wcd, tcd) - searching for and optimal machines speed
static int_T  modeDetermination(real_T, real_T, real_T, int_T, int_T, int_T, int_T, int_T, real_T, int_T);
static real_T costFunction(SimStruct *S, real_T, real_T, real_T, real_T, real_T, int_T); 
static real_T etaBat(real_T, real_T);
static real_T voltageOC(real_T);
static void Hooke_Jeeves(SimStruct *S, real_T, real_T, real_T, real_T, real_T, int_T);
static void Hooke_Jeeves1D(SimStruct *S, real_T, real_T, real_T, real_T, int_T, real_T);
static real_T powerEL(real_T, real_T, real_T, real_T, real_T);
static void optim1DECMS(SimStruct *S, real_T, real_T, real_T, real_T, int_T mode);
static void ICEarbitration(real_T, real_T, real_T, real_T, real_T, real_T, real_T, real_T, real_T);
static void RBSoCcontroller(real_T); 
static real_T pomCijena(SimStruct *S, real_T, real_T, real_T, real_T, real_T, int_T);
   
static void mdlInitializeSizes(SimStruct *S)
{
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
    ssSetDWorkWidth(S, 0, 1);
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
	if (ssGetSFcnParamsCount(S) != NUM_OF_ARGS)
	{
		#ifdef MATLAB_MEX_FILE
			ssSetErrorStatus(S,"Wrong number of input args!");
		#else
			printf("\nWrong number of input args!");
			exit(0);
		#endif    
    }
}
#endif

/***********************/
/* FUNCTION mdlOutputs */
/***********************/
static void mdlOutputs(SimStruct *S, int_T tid)
{
   InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
   real_T *x = (real_T*) ssGetDWork(S,0);
   real_T *y = ssGetOutputPortRealSignal(S,0);
   /************** Inputs ***********************/
   real_T  we = 0, wmg1 = 0, wmg2 = 0, tcd = 0, vv = 0; 
   /************* Outputs ***********************/
   real_T  wmg1R, teR, tmg2R = 0, soc = 0; 
   int_T   tuh1 = 0, tuh2 = 0, tuh3 = 0; 
   /*********************************************/
   real_T eta2 = 0, wcd = 0;
   
   real_T etaOpt = 0, wmg1Ropt = 0, wmg2Ropt = 0, eta = 0, wmg2R = 0, tmg1R = 0, gornjaGranica = 0, donjaGranica = 0, fix_hyst = 0, Pe = 0, granicaPar = 0;
   real_T socR = 0.3, dekrement_SoC = 0, b = 500000, wesoc = 0, tesoc = 0, weECMS = 0, teECMS = 0, hyst = 30, cijena = 0, cijenaSoC = 0, cijenaECMS = 0;
   int_T modOpt = 0, mod = 0, chooseMode = 0, chooseBound = 0, CSmode = 0, engine_start = 0, chooseControl = 0, RBextended = 1;
   real_T wesocPlot = 0, tesocPlot = 0, weECMSPlot = 0, teECMSPlot = 0, tePlot = 0, wePlot = 0, temax = 0; /* Za potrebe iscrtavanja definicija novih varijabli */
   int_T BLNDmode = 0;
   
   // Uèitavanja ulaznih varijabli
   soc = u(0);                      // Battery state_of_charge
   tcd = u(1);                      // Driver power demand
   vv = u(2);                       // Vehicle velocity [m/s]
   mod = (int_T) u(3);              // mode from previous step
   fix_hyst = u(4);                 // hysteresis width
   chooseMode = (int_T) u(5);       // Mode determination
   chooseBound = (int_T) u(6);      // Boundary curve determination: 1 - hyteresis, 2 - GM boundary curve
   CSmode = (int_T) u(7);           // Charge sustenance mode
   engine_start = (int_T) u(8);     // ICE engine start
   chooseControl = (int_T) u(9);    // If chooseControl == 1 --> classical Rule Based with SoC controller, chooseControl --> Rule Based + ECMS + modified SoC controller
   hyst = u(10);                    // Hysteresis width in dependence of driver power demand
   granicaPar = u(11);
   RBextended = u(12);              // if RBextended == 1 --> RB rules are extended, otherwise if RBextended == 0 --> RB rules are not extended (default is RBextended=1)
   dekrement_SoC = u(13);
   socR = u(14);
   BLNDmode = u(15);
  
   if(mod == 0){
        mod = 1; // EV
   }else if(mod == 1){
        mod = 2; // TMEV
   }else if(mod == 2){
        mod = 4; // SPHEV
   }else if(mod == 3){
        mod = 3; // SHEV
   }
  
   /* ICE Engine op. region lower bound setting */
   x[0] = granicaPar;
   
   socR = socR + dekrement_SoC;
   if (socR < 0.3){
        socR = 0.3;
   }
   
   /* Aktivacija CS naèina rada  */ 
   if(soc < 0.3){
        CSmode = 1;
        socR = 0.3;
   }
   wcd = vv/r; // demanded wheel speed 
   
   /* If BLNDmode (blended) or CSmode (charge sustaining) is enabled --> calculate demanded ICE power */ 
   if(BLNDmode == 1){
        Pe = powerEL(soc, wcd, tcd, socR, 0.02); // Demanded ICE power
        // RB ICE turning logic
        if(Pe > P2){
            engine_start = 1;
        }else if(Pe < P0){
            engine_start = 0;
        }  
        CSmode = 1;
   }else if(CSmode == 1){
        Pe = powerEL(soc, wcd, tcd, socR, 0.02); // Demanded ICE power
        // RB ICE turning logic
        if(Pe > P2){
            engine_start = 1;
        }else if(Pe < P0){
            engine_start = 0;
        }
   }
   /* Odredivanje nacina rada */
   modOpt = modeDetermination(vv, tcd, hyst, chooseMode, chooseBound, mod, CSmode, engine_start, soc, RBextended);
   
   // If engine is turned on calculate ICE engine operating point
   // Control variables calculations
   if(modOpt == 1){
        // EV mode1 - M/G2 machine only F1 = ON, F2 = OFF, F3 = OFF
        wmg1R = 0;
        teR = 0;
        tmg2R = tcd/io/(h+1);
        tuh1 = 1;
        tuh2 = 0;
        tuh3 = 0;
        /* Optimal ECMS cost function */
        cijena = 0;
        cijenaSoC = 0;
        cijenaECMS = 0;
        /* Plots values */
        wesocPlot = 0;
        tesocPlot = 0;
        weECMSPlot = 0;
        teECMSPlot = 0;
        wePlot = 0;
        tePlot = 0;
   }else if(modOpt == 2){
        // EV, TMEV mode - M/G1 and M/G2 machine enabled, F1 = OFF, F2 = ON, F3 = OFF
        wmg1R = electricMode(S, vv, tcd);
        teR = 0;
        tmg2R = tcd/io/(h+1);
        tuh1 = 0;
        tuh2 = 1;
        tuh3 = 0;
        /* Optimal ECMS cost function */
        cijena = 0;
        cijenaSoC = 0;
        cijenaECMS = 0;
        /* Plots values */
        wesocPlot = 0;
        tesocPlot = 0;
        weECMSPlot = 0;
        teECMSPlot = 0;
        wePlot = 0;
        tePlot = 0;
   }else if(modOpt == 3){
        // SHEV mode1 - series hybrid, F1 = ON, F2 = OFF, F3 = ON
        if(Pe<P1){
            Pe = P1;
        }else if(Pe > Pemax){
            Pe = Pemax;
        }
        if(chooseControl == 0){
            /* RB+ECMS */
            /* 1D-ECMS */
            optim1DECMS(S, wcd, tcd, soc, Pe, modOpt);
            wesoc = weUpr;
            tesoc = teUpr;
            /* 2D-ECMS */
            optim2DECMS(S, wcd, tcd, soc, modOpt); // linear search + Hooke_Jeeves 
            weECMS = weUpr;
            teECMS = teUpr;
            cijenaSoC = costFunction(S, wesoc, tesoc, wcd, tcd, soc, modOpt);
            cijenaECMS = costFunction(S, weECMS, teECMS, wcd, tcd, soc, modOpt);
            /* Arbitration between those two operating points obtained by SoC and ECMS controller */
            ICEarbitration(socR, soc, b, wesoc, tesoc, weECMS, teECMS, cijenaSoC, cijenaECMS);
        }else if(chooseControl == 1){
            /* Classical RB (rule-based) with SoC controller is used for ICE engine control */
            RBSoCcontroller(Pe);
            /* Variables setting for the plot */
            wesoc = weUpr;
            tesoc = teUpr;
            weECMS = weUpr;
            teECMS = teUpr;
        }else if(chooseControl == 2){
            /* RB + 1D-ECMS*/
            optim1DECMS(S, wcd, tcd, soc, Pe, modOpt);
            wesoc = weUpr;
            tesoc = teUpr; 
        }else{
            printf("Pogresno zadan parametar chooseControl\n");
        }
        
        temax = ICELimit(weUpr);
        if(teUpr > temax){
            teUpr = temax;
        }
        
        wmg1R = weUpr;
        teR = teUpr;
        tmg2R = tcd/io/(h+1);
        tuh1 = 1;
        tuh2 = 0;
        tuh3 = 1;
        /* Optimal ECMS cost function */
        cijena = costFunction(S, wmg1R, teR, wcd, tcd, soc, modOpt);
        
        /* Plots values */
        wesocPlot = wesoc;
        tesocPlot = tesoc;
        weECMSPlot = weECMS;
        teECMSPlot = teECMS;
        wePlot = weUpr;
        tePlot = teUpr;
   }else if(modOpt == 4){
        // SPHEV mode2 - series-parallel hybrid, F1 = OFF, F2 = ON, F3 = ON
        if(Pe<P1){
            Pe = P1;
        }else if(Pe > Pemax){
            Pe = Pemax;
        }
        if(chooseControl == 0){
            /* RB+1D-ECMS controller */
            optim1DECMS(S, wcd, tcd, soc, Pe, modOpt);
            wesoc = weUpr;
            tesoc = teUpr;
            /* 2D-ECMS controller*/
            optim2DECMS(S, wcd, tcd, soc, modOpt); // linear search + Hooke_Jeeves 
            weECMS = weUpr;
            teECMS = teUpr;
           
            cijenaSoC = costFunction(S, wesoc, tesoc, wcd, tcd, soc, modOpt);
            cijenaECMS = costFunction(S, weECMS, teECMS, wcd, tcd, soc, modOpt);
            /* Arbitration between those two operating points obtained by SoC and ECMS controller */
            ICEarbitration(socR, soc, b, wesoc, tesoc, weECMS, teECMS, cijenaSoC, cijenaECMS);
        }else if(chooseControl == 1){
            /* Classical RB (rule-based) with SoC controller is used for ICE engine control */
            RBSoCcontroller(Pe);
            /* Variables setting for the plot */
            wesoc = weUpr;
            tesoc = teUpr;
            weECMS = weUpr;
            teECMS = teUpr;
        }else if(chooseControl == 2){
            /* SoC controller + 1D-ECMS*/
            optim1DECMS(S, wcd, tcd, soc, Pe, modOpt);
            wesoc = weUpr;
            tesoc = teUpr; 
        }else{
            printf("Pogresno zadan parametar chooseControl\n");
        }
 
        temax = ICELimit(weUpr);
        if(teUpr > temax){
            teUpr = temax;
        }
        wmg1R = weUpr;
        teR = teUpr;
        tmg2R = tcd/io/(h+1);
        tuh1 = 0;
        tuh2 = 1;
        tuh3 = 1;
        /* Optimal ECMS cost function */
        cijena = costFunction(S, wmg1R, teR, wcd, tcd, soc, modOpt);
        
        /* Plots values */
        
        wesocPlot = wesoc;
        tesocPlot = tesoc;
        weECMSPlot = weECMS;
        teECMSPlot = teECMS;
        wePlot = weUpr;
        tePlot = teUpr;
   }else{
        // Default mode
        // EV mode1 - M/G2 machine only F1 = ON, F2 = OFF, F3 = OFF
        wmg1R = 0;
        teR = 0;
        tmg2R = tcd/io/(h+1);
        tuh1 = 1;
        tuh2 = 0;
        tuh3 = 0;
        /* Optimal ECMS cost function */
        cijena = 0;
        cijenaSoC = 0;
        cijenaECMS = 0;
        /* Plots values */
        wesocPlot = 0;
        tesocPlot = 0;
        weECMSPlot = 0;
        teECMSPlot = 0;
        wePlot = 0;
        tePlot = 0;
   }
   
   if(modOpt == 1){
        modOpt = 0; /* EV nacin rada*/
   }else if(modOpt == 2){
        modOpt = 1; /* TMEV nacin rada*/
   }else if(modOpt == 3){
        modOpt = 3; /* SHEV nacin rada*/
   }else if(modOpt == 4){
        modOpt = 2; /* SPHEV nacin rada*/
   }
  
   /* Postavljanje izlaznih varijabli */
   y[0] = wmg1R;    // M/G1 electric machine speed reference
   y[1] = teR;      // ICE engine torque reference
   y[2] = tmg2R;    // M/G2 electric machine speed reference
   y[3] = tuh1;     // F1 clutch state
   y[4] = tuh2;     // F2 clutch state
   y[5] = tuh3;     // F3 clutch state
   y[6] = modOpt;    
   y[7] = CSmode;
   y[8] = engine_start;
   y[9] = cijena;
   y[10] = tanh(b*((socR - soc)-tanh(socR - soc))); // Vrijednost funkcije razmazivanja koja se za potrebe iscrtavanja salje van funkcije
   y[11] = wesocPlot;
   y[12] = tesocPlot;
   y[13] = weECMSPlot;
   y[14] = teECMSPlot;
   y[15] = wePlot;
   y[16] = tePlot;
   y[17] = cijenaSoC;
   y[18] = cijenaECMS;
   y[19] = socR;
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
/****************************************************************************************/
/* FUNCTION ICEarbitration - function which calculates ICE operating point*/
/****************************************************************************************/
static void ICEarbitration(real_T socR, real_T soc, real_T b, real_T wesoc, real_T tesoc, real_T weECMS, real_T teECMS, real_T cijenaSoC, real_T cijenaECMS){
    real_T esoc, x;
    
    /* If SoC controller operating point is not attainable ECMS operating point is taken*/
    if(cijenaSoC > 20){
        weUpr = weECMS;
        teUpr = teECMS;
    }else{
        esoc = socR - soc;
        x = tanh(b*(esoc-tanh(esoc)));
        //printf("esoc = %f, x = %f\n", esoc, x);
        weUpr = (wesoc - weECMS)*fabs(x) + weECMS;
        teUpr = (tesoc - teECMS)*fabs(x) + teECMS;
    }
}
/*********************************************************************************/
/* FUNCTION pomCijena - pomocna funkcija koja sluzi za detekciju narusavanja ogranicenja*/
/*********************************************************************************/
static real_T pomCijena(SimStruct *S, real_T we, real_T te, real_T wcd, real_T tcd, real_T SoC, int_T mode){
    /* costFunction is calculated only for case when ICE engine is engaged (F3=ON), mode=3,4*/
    real_T tmg2 = 0, wmg2 = 0, wmg1 = 0, tmg1 = 0, f = 0, Pe = 0, Pmg1 = 0, Pmg2 = 0, Pbat = 0, eta1 = 0, eta2 = 0, etaB = 0, Aekwhp = 0, pom = 0, ICElowerbound = 0;
    real_T *granicaPar = (real_T*) ssGetDWork(S,0);
    
    if(mode == 3){
        /* SHEV mode */
        tmg2 = 1/io/(h+1)*tcd;
        wmg2 = io*(h+1)*wcd;
        wmg1 = we;
        tmg1 = -te;
    }else if(mode == 4){
        /* SPHEV mode */
        wmg1 = we;
        wmg2 = io*(h+1)*wcd-h*we;
        tmg1 = h/(io*(h+1))*tcd-te;
        tmg2 = 1/io/(h+1)*tcd;
        //printf("tmg1 = %f, tmg1MAX = %f\n", tmg1, MG1Limit(wmg1));
    }
    ICElowerbound = 0.0002*(we-105)*(we-105) + granicaPar[0];
    /* Constraints cost punishment*/
    if(te > ICELimit(we)){
        /* ICE engine torque constraint*/
        f = 25 + (fabs(te) - ICELimit(we));
        printf("te > ICEmax, ICEmax = %f, te = %f\n", ICELimit(we), te);
    }else if(fabs(wmg2) > 1044){
        /* M/G2 machine speed constraint*/
        f = 25 + (fabs(wmg2)-1044);
        printf("wmg2 > 1044\n");
    }else if(fabs(wmg1)>630){
        /* M/G1 machine speed constraint*/
        f = 25 + (fabs(wmg1)-630);
        printf("wmg1 > 630\n");
    }else if(fabs(we) > 523){
        /* ICE engine speed constraint*/
        f = 25 + (fabs(we)-523);
        printf("we > 523\n");
    }else if(tmg1 > MG1Limit(wmg1)){
        /* M/G1 machine torque constraint*/
        f = 25 + (fabs(tmg1) - MG1Limit(wmg1));
        printf("tmg1 > MG1max\n");
    }else if(tmg2 > MG2Limit(wmg2)){
        /* M/G2 machine torque constraint*/
        f = 25 + (fabs(tmg2) - MG1Limit(wmg2));
        printf("tmg2 > MG2max\n");
    }else if((te<0)||(we<0)){
        printf("ICE prekrsaj\n");
        f = 1e10;
    }else if(te < ICElowerbound){
        f = 25 + (fabs(te - ICElowerbound));
       printf("ICE lower bound prekrsaj\n");
    }
    return(f);
}
/****************************************************************************************/
/* FUNCTION optim1DECMS - function which calculates ICE operating point*/
/****************************************************************************************/
static void optim1DECMS(SimStruct *S, real_T wcd, real_T tcd, real_T soc, real_T Pe, int_T mode){
    real_T we, wepoc, cijenaMIN = 1000, cijena;
    // Linear search over constant power curve Pe
    for(we = 100; we < 523; we = we + 5){
        cijena = costFunction(S, we, Pe/we, wcd, tcd, soc, mode);
        if(cijena < cijenaMIN){
            wepoc = we;
            cijenaMIN = cijena;
        }
    }
    /* Hooke-Jeeves directional search which searches operating points over constant power curve (initial point obtained from linear search)*/
    Hooke_Jeeves1D(S, wepoc, wcd, tcd, soc, mode, Pe);  
}
/****************************************************************************************/
/* FUNCTION RBSoCcontroller - function which calculates ICE operating point*/
/****************************************************************************************/
static void RBSoCcontroller(real_T Pe){
    // This controller places ICE operating point on maximum output torque curve according to Pd (driver power demand), 
    // and SoC (battery state-of-charge)
    /* Intersection between max. output torque curve and constant ICE power curve*/
    /* Input Pe is constrained to Pe>P1*/
    double prva = 100.0, druga = 523.0, sredina;
    int i;
    
    if(Pe<P1){
        Pe = P1;
    }else if(Pe > Pemax){
        Pe = Pemax;
    }
    /* Binary search */
    for(i=1;i<=50;i++){
        sredina = (prva + druga)/2;
        if(Pe/sredina>ICELimit(sredina)){
            prva = sredina;
        }
        else{
            druga = sredina;
        }
    }
    weUpr = sredina;
    teUpr = ICELimit(sredina);
}
/****************************************************************************************/
/* FUNCTION powerEL - function which gives ICE power Pe demanded by SoC controller*/
/****************************************************************************************/
static real_T powerEL(real_T soc, real_T wcd, real_T tcd, real_T socR, real_T dead_zone){
    real_T Pe, Pel, Pd;
    
    Pd = wcd*tcd;
    // SoC proportional control
    if(soc > socR+dead_zone){
        /* In this case Pel is negative because soc > SoC_max, discharging is demanded */
        Pel = Ksoc*(socR+dead_zone - soc);
        if(fabs(Pel) > fabs(Pb_max)){
            Pel = -Pb_max;
        } 
    }else if(soc < socR-dead_zone){
        /* In this case Pel is positive because soc < SoC_min, charging is demanded */
        Pel = Ksoc*(socR-dead_zone - soc);
        if(fabs(Pel) > fabs(Pb_max)){
            Pel = Pb_max;
        } 
    }else{
        Pel = 0;
    }
    Pe = Pd + Pel;
    
    return(Pe);
}
/*********************************************************************************/
/* FUNCTION modeDetermination - function which determines optimal operating mode according
  to vehicle velocity vv and driver power demand tcd                                  */
/*********************************************************************************/
static int_T modeDetermination(real_T vv, real_T tcd, real_T hyster, int_T chooseMode, int_T chooseBound, int_T mod, int_T CSmode, int_T engine_start, real_T soc, int_T RBextended){
      
    real_T wcd, donjaGranica, gornjaGranica, tmg2R, HEVhyster = 20;
    int_T modOpt;

    wcd = vv/r;     // demanded wheel speed 
    vv = 3.6*vv;    // Speed conversion [m/s] --> [km/h]
    tmg2R = tcd/io/(h+1);
    //printf("CSmode = %d\n", CSmode);
    if(CSmode == 0){
        if(chooseMode == 1){
            /* Only EV mode is enabled*/
            modOpt = 1;
        }else if(chooseMode == 2){
            /* Both electric modes (EV, TMEV) are enabled*/
            /* Transition conditions (boundaries) determination */
            if(chooseBound == 1){
                gornjaGranica = boundary(vv);
                donjaGranica = gornjaGranica - hyster;     
            }else if(chooseBound == 2){
                gornjaGranica = GMboundary(vv);
                donjaGranica = gornjaGranica - hyster;
            }else if(chooseBound == 3){
                donjaGranica = GMboundary(vv);
                gornjaGranica = boundary(vv); 
            }

            if(donjaGranica<0){
                donjaGranica = 0.0;
            }
            // Boundaries in coordinate system (vv, tmg2R) 
            /* Switching between TMEV and EV or keeping mode*/
            //((fabs(tmg2R) < fabs(donjaGranica))||((fabs(tmg2R) < fabs(gornjaGranica)) && mod == 2))
            //((fabs(tmg2R)>=fabs(gornjaGranica))||((fabs(tmg2R) < fabs(gornjaGranica)) && mod == 1))
            if(fabs(tmg2R) < fabs(donjaGranica)){
                modOpt = 2;
            }else if(fabs(tmg2R)>=fabs(gornjaGranica)){
                modOpt = 1;
            }else{
                modOpt = mod;
            }
        }else{
            // Default mode EV in case that chooseMode is not defined
            modOpt = 1;
        }
    }else if(CSmode == 1){
        // CS (Charge sustenance) mode is enabled 
        if(engine_start == 1){
            gornjaGranica = HEVboundary(vv);
            donjaGranica = HEVboundary(vv) - HEVhyster;
            // Boundaries in coordinate system (vv, tcd) 
            if(fabs(tcd) >= fabs(gornjaGranica)){
                modOpt = 3;
            }else if(fabs(tcd) < fabs(donjaGranica)){
                modOpt = 4;
            }else if((mod==3)||(mod==1)){
                modOpt = 3;
            }else{
                modOpt = 4;
            } 
            
            if(RBextended == 1){
                /* RB extended rules */
                if((modOpt == 4)&&(soc > SoC_min) && (soc < SoC_max)){
                    if(fabs(tcd) < (1000*exp(-0.08*vv)+110)-50){
                        modOpt = 2;
                        //printf("granica = %f, soc = %f, SoC_min = %f, SoC_max = %f, tcd = %f\n",(1000*exp(-0.08*vv)+110), soc, SoC_min, SoC_max, tcd);
                    }else if((fabs(tcd) < (1000*exp(-0.08*vv)+110)) && (mod == 2)){
                        modOpt = 2;
                    }
                }

                if((modOpt == 3)&&(soc > SoC_min) && (soc < SoC_max)){
                    if(fabs(tcd) < (100*exp(-0.03*vv+3)+300)){
                        modOpt = 1;
                    }
                }
            }
        }else{
            /* ICE engine is turned off */
            /* Both electric modes (EV, TMEV) are enabled*/
            /* Transition conditions (boundaries) determination */
            if(chooseBound == 1){
                gornjaGranica = boundary(vv);
                donjaGranica = gornjaGranica - hyster;     
            }else if(chooseBound == 2){
                //hyster = 5;
                gornjaGranica = GMboundary(vv);
                donjaGranica = gornjaGranica - hyster;
                //printf("GM gor = %f, dol = %f\n", gornjaGranica, donjaGranica);
            }else if(chooseBound == 3){
                donjaGranica = GMboundary(vv);
                gornjaGranica = boundary(vv); 
            }

            if(donjaGranica<0){
                donjaGranica = 0.0;
            }
            // Boundaries in coordinate system (vv, tmg2R) 
            /* Switching between TMEV and EV or keeping mode*/
            if(fabs(tmg2R) < fabs(donjaGranica)){
                modOpt = 2;
            }else if(fabs(tmg2R)>=fabs(gornjaGranica)){
                modOpt = 1;
            }else{
                if((mod==3)||(mod==1)){
                    modOpt = 1;
                }else{
                    modOpt = 2;
                }
            }
        }
        //printf("tmg2R = %f, dG = %f, gG = %f, modOpt = %d\n", tmg2R, donjaGranica, gornjaGranica, modOpt);
    }else{
        // Default mode EV
        modOpt = 1;
       // printf("mod = %d\n", modOpt);
    }  
   return(modOpt);
}

/******************************************************************************************************/
/* FUNCTION electricMode - function returns optimal M/G1 electrical machine speed for EV or TMEV mode */
/******************************************************************************************************/
static real_T electricMode(SimStruct *S, real_T vv, real_T tcd){ 
    /* Both electrical modes are enabled */
    real_T wcd = 0, wmg1R = 0, wmg2R = 0, tmg2R = 0, etaOpt = 0, tmg1R = 0, wmg1Ropt = 0, wmg2Ropt = 0, eta = 0;

    /* EV mode with one electric machine is default mode*/ 
    wcd = vv/r; // demanded wheel speed 
    vv = 3.6*vv; // Speed conversion [m/s] --> [km/h]

    /*Efficiency calculation for one machine mode case*/
    wmg2R = wcd*io*(h+1);
    tmg2R = tcd/io/(h+1);
    etaOpt = etaMG2(S, wmg2R, tmg2R)/100;
    wmg1Ropt = 0;
    
    tmg1R = tcd*h/io/(h+1);
    for(wmg1R = 1;wmg1R<=io*(h+1)/h*wcd;wmg1R++){
        if(fabs(tcd)>fabs(io*(h+1)/h*MG1Limit(wmg1R))){
            // If tcd cannot be supported with M/G1 than leave loop
            break;
        }
        wmg2R = io*(h+1)*wcd - h*wmg1R;
        if(fabs(wmg1R*tmg1R) + fabs(wmg2R*tmg2R)<1e-5){
            eta = 0;
        }else{
            eta = (fabs(wmg2R*tmg2R)*etaMG2(S,wmg2R,tmg2R)/100 + fabs(wmg1R*tmg1R)*etaMG1(S,wmg1R,tmg1R)/100)/(fabs(wmg1R*tmg1R)+ fabs(wmg2R*tmg2R));
        }
        if(eta>etaOpt){
        /* If some speed combination of MG1 and MG2 machines is better reffered to overall efficiency then mode with two electric machines is optimal*/
            etaOpt = eta;
            wmg1Ropt = wmg1R;
        }
    }
    /*************** Case when wmg2R is 0 *********************/
    wmg1R = io*(h+1)/h*wcd;
    wmg2R = 0; 
    if(tmg1R<=MG1Limit(wmg1R)){
        wmg1Ropt = wmg1R;
    }
    //printf("unutra wmg1Ropt = %f\n", wmg1Ropt);
    /******************************************************************************/
    return(wmg1Ropt);
}
/*********************************************************************************/
/* FUNCTION optim2DECMS - function calculates ICE engine operating points and thus whole transmission 
 operating points which minimizes ECMS criterion function*/
/*********************************************************************************/
static void optim2DECMS(SimStruct *S, real_T wcd, real_T tcd, real_T soc, int_T modOpt){

    real_T wepoc, tepoc, fmin, pom, we, te;

    fmin = 1e5;
    //wmg2 = io*(h+1)*wcd-h*we;
    /* 2D Linear search*/
    for(we = 105; we < 520; we = we + 5){
        for(te = 0; te < ICELimit(we); te = te + 5){
            pom = costFunction(S, we, te, wcd, tcd, soc, modOpt); 
            //printf("we = %f, te = %f, pomCijena = %f\n", we, te, pomCijena(S, wepoc, tepoc, wcd, tcd, soc, modOpt));
            if(pom < fmin){
                fmin = pom;
                wepoc = we;
                tepoc = te;
            }
        }
    }
    //printf("fmin = %f, wepoc = %f, tepoc = %f\n", fmin, wepoc, tepoc);
    //printf("pomCijena = %f\n", pomCijena(S, wepoc, tepoc, wcd, tcd, soc, modOpt));
    /* For the case when tcd is very close to maximal torque which is possible to be supported by SPHEV mode
     in order to help Hooke_Jeeves function with providing attainable initial point */
    we = 324; /* speed point in which is combination of M/G1 torque and ICE torque maximal (obtained with offline analysis) */
    te = ICELimit(we);
    pom = costFunction(S, we, te, wcd, tcd, soc, modOpt); 
    //printf("pom = %f, wcd = %f\n", pom, wcd);
    
    if(pom < fmin){
        fmin = pom;
        wepoc = we;
        tepoc = te;
    }
    Hooke_Jeeves(S, wepoc, tepoc, wcd, tcd, soc, modOpt);
}

/*********************************************************************************/
/* FUNCTION costFunction - function returns cost according to equivalent fuel rate */
/*********************************************************************************/
static real_T costFunction(SimStruct *S, real_T we, real_T te, real_T wcd, real_T tcd, real_T SoC, int_T mode){
    /* costFunction is calculated only for case when ICE engine is engaged (F3=ON), mode=3,4*/
    real_T tmg2, wmg2, wmg1, tmg1, f, Pe, Pmg1, Pmg2, Pbat, eta1, eta2, etaB, Aekwhp, pom, ICElowerbound;
    real_T *granicaPar = (real_T*) ssGetDWork(S,0);
    
    if(mode == 3){
        /* SHEV mode */
        tmg2 = 1/io/(h+1)*tcd;
        wmg2 = io*(h+1)*wcd;
        wmg1 = we;
        tmg1 = -te;
    }else if(mode == 4){
        /* SPHEV mode*/
        wmg1 = we;
        wmg2 = io*(h+1)*wcd-h*we;
        tmg1 = h/(io*(h+1))*tcd-te;
        tmg2 = 1/io/(h+1)*tcd;
        //printf("tmg1 = %f, tmg1MAX = %f\n", tmg1, MG1Limit(wmg1));
    }
    ICElowerbound = 0.0002*(we-105)*(we-105) + granicaPar[0];
    //printf("lb = %f, grPar = %f\n", ICElowerbound, granicaPar[0]);
    // printf("wcd = %f, tcd = %f, we = %f, te = %f, wmg2 = %f, tmg2 = %f, wmg1 = %f, tmg1 = %f, mode = %d\n", wcd, tcd, we , te, wmg2, tmg2, wmg1, tmg2, mode);
    /* Constraints cost punishment*/
    if(te > ICELimit(we)){
        /* ICE engine torque constraint*/
        f = 25 + (fabs(te) - ICELimit(we));
       // printf("te > ICEmax, ICEmax = %f, te = %f\n", ICELimit(we), te);
    }else if(fabs(wmg2) > 1044){
        /* M/G2 machine speed constraint*/
        f = 25 + (fabs(wmg2)-1044);
        //printf("wmg2 > 1044\n");
    }else if(fabs(wmg1)>630){
        /* M/G1 machine speed constraint*/
        f = 25 + (fabs(wmg1)-630);
        //printf("wmg1 > 630\n");
    }else if(fabs(we) > 523){
        /* ICE engine speed constraint*/
        f = 25 + (fabs(we)-523);
        //printf("we > 523\n");
    }else if(tmg1 > MG1Limit(wmg1)){
        /* M/G1 machine torque constraint*/
        f = 25 + (fabs(tmg1) - MG1Limit(wmg1));
       // printf("tmg1 > MG1max\n");
    }else if(tmg2 > MG2Limit(wmg2)){
        /* M/G2 machine torque constraint*/
        f = 25 + (fabs(tmg2) - MG1Limit(wmg2));
       // printf("tmg2 > MG2max\n");
    }else if((te<0)||(we<0)){
       // printf("prekrsaj\n");
        f = 1e10;
    }else if(te < ICElowerbound){
        f = 25 + (fabs(te - ICElowerbound));
    }else{
        /* M/G1 - Adding losses through constraining machine speed to value +-1*/
        if(fabs(wmg1)<1){
            if(wmg1>0){
                wmg1 = 1;
            }else{
                wmg1 = -1;
            }
        } 
        /* M/G2 - Adding losses through constraining machine speed to value +-1*/
        if(fabs(wmg2)<1){
            if(wmg2>0){
                wmg2 = 1;
            }else{
                wmg2 = -1;
            }
        } 
        /* M/G1 - Adding losses through constraining machine torque to value +-1*/
        if(fabs(tmg1)<1){
            if(tmg1>0){ 
                tmg1 = 1;
            }else{
                tmg1 = -1;
            }
        }
        /* M/G2 - Adding losses through constraining machine torque to value +-1*/
        if(fabs(tmg2)<1){
            if(tmg2>0){ 
                tmg2 = 1;
            }else{
                tmg2 = -1;
            }
        }
        /* Powers calculation(ICE, M/G1, M/G2) */
        Pe = te*we;
        Pmg1 = wmg1*tmg1;
        Pmg2 = wmg2*tmg2;
        /* M/G1 machine efficiency calculation */
        eta1 = etaMG1(S, wmg1, tmg1)/100;
        if(Pmg1 > 0 ){
            eta1 = 1/eta1; 
        }
        /* M/G2 machine efficiency calculation */
        eta2 = etaMG2(S, wmg2, tmg2)/100;
        if(Pmg2 > 0 ){
            eta2 = 1/eta2; 
        }
        /* Battery power calculation */
        Pbat = eta1*Pmg1 + eta2*Pmg2;
        /* Battery efficiency calculation */
        
        etaB = etaBat(SoC, Pbat);
        pom = etaICE(S, we, te);
        //printf("wcd = %f, tcd = %f\n", wcd, tcd);
        if(Pbat > 0){
            /*For the case of positive battery power cost Aekwhp is fixed*/
            Aekwhp = 257.4251/3.6e6;
            f = (pom*Pe + Aekwhp*Pbat/etaB);
            //printf("Pbat > 0: f = %f, pom = %f, Aekwhp = %f, etaB = %f, Pe = %f,Pmg1 = %f, Pmg2 = %f, Pbat = %f, eta1 = %f, eta2 = %f\n", f, pom, Aekwhp, etaB, Pe, Pmg1, Pmg2, Pbat, eta1, eta2);
        }else{
            /* otherwise is instant engine specific fuel consumption */
            f = pom*(Pe + etaB*Pbat);
            //printf("Pbat > 0: f = %f, pom = %f, Aekwhp = %f, etaB = %f, Pe = %f,Pmg1 = %f, Pmg2 = %f, Pbat = %f, eta1 = %f, eta2 = %f\n", f, pom, Aekwhp, etaB, Pe, Pmg1, Pmg2, Pbat, eta1, eta2); 
        }
       // printf("wmg1 = %f, tmg1 = %f, tmg1MAX = %f, we = %f, te = %f, teMAX = %f, f = %f\n", wmg1, tmg1, MG1Limit(wmg1), we, te, ICELimit(we), f);
    }
    return(f);
}
/*********************************************************************************/
/* FUNCTION HEVboundary - function returns torque value which represents boundary beetwen MODE1
            (only one machine) and MODE2(two electric machines combined)*/
/*********************************************************************************/
static real_T HEVboundary(real_T vv){
    real_T tb;
    if(vv<=4){
        tb = 0;
    }else if((vv>4)&&(vv<=75)){
        tb = HEVa5*vv*vv*vv*vv*vv + HEVa4*vv*vv*vv*vv + HEVa3*vv*vv*vv + HEVa2*vv*vv + HEVa1*vv + HEVa0;
    }else{
        tb = 900;
    }
    return(tb);
}

/*********************************************************************************/
/* FUNCTION boundary - function returns torque value which represents boundary beetwen EV
            (only one machine) and TMEV(two electric machines combined)*/
/*********************************************************************************/
static real_T boundary(real_T vv){
    real_T tb;
    tb = a2*vv*vv + a1*vv + a0;
    return(tb);
}
/*********************************************************************************/
/* FUNCTION GMboundary - function returns torque value which represents boundary beetwen MODE1
            (only one machine) and MODE2(two electric machines combined) - boundary similar to 
            to boundary from GMVoltec paper*/
/*********************************************************************************/
static real_T GMboundary(real_T vv){
    real_T tb;
    if(fabs(vv)<80){
        tb = 0;
    }else if(fabs(vv)<125){
        tb = 21*log(vv-79);
    }else{
        tb = MG1Limit(26);
    }
    return(tb);
}
/*********************************************************************************/
/* FUNCTION etaICE - ICE engine efficiency [g/Ws]*/
/*********************************************************************************/
static real_T etaICE(SimStruct *S, real_T we, real_T te){
    real_T eta, etaInt;
    const real_T *Ae = mxGetPr(AE(S));
    
    /* Increasing engine variables for +1 they are shifted in map for +1*/
    we = fabs(we + 1);
    te = fabs(te + 1);
   
    // function interpolacija gives Ae in g/s which is then converted in g/kWh
    etaInt = interpolacija(S, te, we, Aex, Aey, Ae);
    eta = etaInt/(we*te);
    //printf("we = %f, te = %f, etaInt = %f, eta = %f, etagkwh= %f\n", we, te, etaInt, eta, eta*3.6e6);
    return(eta);
}
/*********************************************************************************/
/* FUNCTION etaMG1 - MG1 machine efficiency [%]*/
/*********************************************************************************/
static real_T etaMG1(SimStruct *S, real_T wmg1, real_T tmg1){
    real_T eta;
    const real_T *Amg1 = mxGetPr(AMG1(S));
    
    /* Increasing machine variables for +1 they are shifted in map for +1*/
    wmg1 = fabs(wmg1 + 1);
    tmg1 = fabs(tmg1 + 1);
    eta = interpolacija(S, tmg1, wmg1, Amg1x, Amg1y, Amg1);
    return(eta);
}
/*********************************************************************************/
/* FUNCTION etaMG2 - MG2 machine efficiency [%]*/
/*********************************************************************************/
static real_T etaMG2(SimStruct *S, real_T wmg2, real_T tmg2){
    real_T eta;
    const real_T *Amg2 = mxGetPr(AMG2(S));
    
    /* Increasing machine variables for +1 they are shifted in map for +1*/
    wmg2 = fabs(wmg2 + 1);
    tmg2 = fabs(tmg2 + 1);
    eta = interpolacija(S, tmg2, wmg2, Amg2x, Amg2y, Amg2);
    return(eta);
}
/*********************************************************************************/
/* FUNCTION etaBat - function returns battery efficience in dependence of battery state-of-charge(SoC)
  and battery power(Pbat)*/
/*********************************************************************************/
static real_T etaBat(real_T SoC, real_T Pbatt){
    real_T etaB, Uoc, I1, I2;
    
    /*Open circuit voltage*/
    Uoc = voltageOC(SoC);

    if(Pbatt >= 0){
        /*Battery discharging*/
        if(fabs(Pbatt)>Pb_max){
            /* Battery power saturation */
            Pbatt = Pb_max;
        }
        /* Battery current calculation */
        I1 = (Uoc - sqrt(Uoc*Uoc-4*Rdch*Pbatt))/(2*Rdch);
        I2 = (-Uoc - sqrt(Uoc*Uoc-4*Rdch*Pbatt))/(2*Rdch);
        
        if(Pbatt<1e-5){
            Pbatt = 1e-5;
        }
        /* Current solution choosing and efficiency calculation */
        if(fabs(Uoc*I1 - Pbatt) < fabs(Uoc*I2 - Pbatt)){
            //etaB = 1/(1+I1*I1*Rdch/Pbatt);
            etaB = (Uoc-fabs(I1)*Rdch)/fabs(Uoc+Rch*fabs(I1));
        }else{
            //etaB = 1/(1+I2*I2*Rdch/Pbatt);
            etaB = (Uoc-fabs(I2)*Rdch)/fabs(Uoc+Rch*fabs(I2));
        }
        //printf("Batt. discharg. = %f\n", etaB);
    }else{
        /*Battery charging*/
        if(fabs(Pbatt)>Pb_max){
            /* Battery power saturation */
            Pbatt = -Pb_max;
        }
        /* Battery current calculation */        
        I1 = (Uoc - sqrt(Uoc*Uoc-4*Rch*Pbatt))/(2*Rch);
        I2 = (-Uoc - sqrt(Uoc*Uoc-4*Rch*Pbatt))/(2*Rch);
        
        if(fabs(Pbatt)<1e-5){
            Pbatt = -1e-5;
        }
         /* Current solution choosing and efficiency calculation */
        if(fabs(Uoc*I1 - Pbatt) < fabs(Uoc*I2 - Pbatt)){
            //etaB = 1+I1*I1*Rch/Pbatt;
            etaB = (Uoc-fabs(I1)*Rdch)/fabs(Uoc+Rch*fabs(I1));
        }else{
            //etaB = 1+I2*I2*Rch/Pbatt;
            etaB = (Uoc-fabs(I2)*Rdch)/fabs(Uoc+Rch*fabs(I2));
        } 
        //printf("Batt. charg. = %f\n", etaB);
    }
    return(etaB);
}
/*********************************************************************************/
/* FUNCTION ICELimit - maximum output torque te on particular speed we*/
/*********************************************************************************/
static real_T ICELimit(real_T we){
    real_T Mlim;
    /*********************************/
    /*  ICEwe1 = 100 [rad/s]        */
    /*  ICEwe2 = 104.7 [rad/s]      */
    /*  ICEwe3 = 230.4 [rad/s]      */
    /*  ICEwe4 = 450.3 [rad/s]      */
    /*  ICEwend = 523.6 [rad/s]     */
   /*********************************/
    
    if(fabs(we)<=ICEwe1){
        Mlim = 0;
        //printf("0. segment we = %f, Mlim = %f\n", we, Mlim);
    }
    else if((fabs(we)>ICEwe1)&&(fabs(we)<=ICEwe2)){
        /* 1. segment */
        Mlim = ICEk11*fabs(we) + ICEk12;
        //printf("1. segment we = %f, Mlim = %f\n", we, Mlim);
    }
    else if((fabs(we)>ICEwe2)&&(fabs(we)<=ICEwe3)){
        /* 2. segment */
        Mlim = ICEk21*fabs(we*we*we) + ICEk22*fabs(we*we) + ICEk23*fabs(we) + ICEk24;
        //printf("2. segment we = %f, Mlim = %f\n", we, Mlim);
    }
    else if((fabs(we)>ICEwe3)&&(fabs(we)<=ICEwe4)){
        /* 3. segment */
        Mlim = ICEk31*fabs(we*we*we) + ICEk32*fabs(we*we) + ICEk33*fabs(we) + ICEk34;
        //printf("3. segment we = %f, Mlim = %f\n", we, Mlim);
    }
    else if((fabs(we)>ICEwe4)&&(fabs(we)<=ICEwend)){
        /* 4. segment */
        Mlim = ICEk41*fabs(we*we*we) + ICEk42*fabs(we*we) + ICEk43*fabs(we) + ICEk44;
        //printf("4. segment we = %f, Mlim = %f\n", we, Mlim);
    }else{
        // keep the torque defined in the biggest defined speed for bigger speeds
        Mlim = 120;
    }
    
    if(Mlim<0){
        Mlim = 0;
    } 
    return(Mlim);
}
/*********************************************************************************/
/* FUNCTION MG1Limit - maximum output torque tmg1 on particular speed wmg1*/
/*********************************************************************************/
static real_T MG1Limit(real_T w){
    real_T Mlim;
    /*********************************/
    /*  MG2w1 = 26.2   [rad/s]      */
    /*  MG2w2 = 314.2  [rad/s]      */
    /*  MG2w3 =  392.7 [rad/s]      */
    /*  MG2wend = 629.0 [rad/s]      */
   /**********************************/
    
    if(fabs(w)<=MG1w1){
        /* 1. segment */
        Mlim = MG1k11*fabs(w) + MG1k12;
        //printf("1. segment w = %f, Mlim = %f\n", w, Mlim);
    }
    else if((fabs(w)>MG1w1)&&(fabs(w)<=MG1w2)){
        /* 1. segment */
        Mlim = MG1k21*fabs(w) + MG1k22;
        //printf("2. segment w = %f, Mlim = %f\n", w, Mlim);
    }
    else if((fabs(w)>MG1w2)&&(fabs(w)<=MG1w3)){
        /* 2. segment */
        Mlim = MG1k31*fabs(w*w*w) + MG1k32*fabs(w*w) + MG1k33*fabs(w) + MG1k34;
        //printf("3. segment w = %f, Mlim = %f\n", w, Mlim);
    }
    else if((fabs(w)>MG1w3)&&(fabs(w)<=MG1wend)){
        /* 3. segment */
        Mlim = MG1k41*fabs(w*w*w) + MG1k42*fabs(w*w) + MG1k43*fabs(w) + MG1k44;
        //printf("4. segment w = %f, Mlim = %f\n", w, Mlim);
    }else{
        // keep the torque defined in the biggest defined speed for bigger speeds
        Mlim = 90;
    }
    
    if(Mlim<0){
        Mlim = 0;
    } 
    return(Mlim);
}

/*********************************************************************************/
/* FUNCTION MG2Limit - maximum output torque tmg1 on particular speed wmg1*/
/*********************************************************************************/
static real_T MG2Limit(real_T w){
    real_T Mlim;
    /*********************************/
    /*  MG2w1 = 61   [rad/s]      */
    /*  MG2w2 = 153  [rad/s]      */
    /*  MG2w3 = 307  [rad/s]      */
    /*  MG2w4 = 429   [rad/s]     */
    /*  MG2wend = 1100 [rad/s]    */
   /**********************************/
    
    if(fabs(w)<=MG2w1){
        /* 1. segment */
        Mlim = MG2k11*fabs(w) + MG2k12;
        //printf("1. segment w = %f, Mlim = %f\n", w, Mlim);
    }
    else if((fabs(w)>MG2w1)&&(fabs(w)<=MG2w2)){
        /* 2. segment */
        Mlim = MG2k21*fabs(w) + MG2k22;
        //printf("2. segment w = %f, Mlim = %f\n", w, Mlim);
    }
    else if((fabs(w)>MG2w2)&&(fabs(w)<=MG2w3)){
        /* 3. segment */
        Mlim = MG2k31*fabs(w) + MG2k32;
        //printf("3. segment w = %f, Mlim = %f\n", w, Mlim);
    }
    else if((fabs(w)>MG2w3)&&(fabs(w)<=MG2w4)){
        /* 4. segment */
        Mlim = MG2k41*fabs(w*w*w) + MG2k42*fabs(w*w) + MG2k43*fabs(w) + MG2k44;
        //printf("4. segment w = %f, Mlim = %f\n", w, Mlim);
    }
    else if((fabs(w)>MG2w4)&&(fabs(w)<=MG2wend)){
        /* 5. segment */
        Mlim = MG2k51*fabs(w*w*w) + MG2k52*fabs(w*w) + MG2k53*fabs(w) + MG2k54;
        //printf("5. segment w = %f, Mlim = %f\n", w, Mlim);
    }else{
        // keep the torque defined in the biggest defined speed for bigger speeds
        Mlim = 92;
    }
    
    if(Mlim<0){
        Mlim = 0;
    } 
    return(Mlim);
}

/*******************************************************************************************/
/* FUNCTION interpolacija - function which interpolates efficiency and fuel mass flow maps*/
/*******************************************************************************************/
static real_T interpolacija(SimStruct *S, real_T x0, real_T y0, int_T a, int_T b, real_T *Amg)
{
    // x0 - row element
    // y0 - column element
    // *Amg - array pointer
    
    /* Amg matrix dimension */
    int_T xIndexLen = a; // number of rows
    int_T yIndexLen = b; // number of columns   
    
    real_T xLo, xHi, yLo, yHi;
    real_T zxLoyLo, zxLoyHi, zxHiyLo, zxHiyHi;
    real_T xRatio, zx0yLo, zx0yHi;
    int_T  i, xPtrLo, xPtrHi, yPtrLo, yPtrHi;
    
    x0 = x0 - 1;
    y0 = y0 - 1;  
    for (i = 0; (i < xIndexLen) && (i < x0); i++) {
        ;
    }
    /*
     * if i is equal to the length of the xIndex array, then x0 is less
     * than all values in xIndex, use the last two points in xIndex
     * else if i is zero, x0 is greater than all values in xIndex,
     * use the first two points in xIndex. If the index is a scalar
     * handle that as a special case of a 1-d look-up table
     */ 
    if ((i == xIndexLen) && (xIndexLen != 1)) {
        xPtrLo = xIndexLen - 2;
        xPtrHi = xIndexLen - 1;
    } else if ((i == 0) && (xIndexLen != 1)) {
        xPtrLo = 0;
        xPtrHi = 1;
    } else if (xIndexLen == 1) {
        xPtrLo = 0;
        xPtrHi = 0;
    }  else {
        xPtrLo = i - 1;
        xPtrHi = i;
    }
 
    xLo = xPtrLo;
    xHi = xPtrHi;

    for (i = 0; (i < yIndexLen) && (i < y0); i++) {
        ;
    }
    /*
     * if i is equal to the length of the yIndex array, then y0 is less
     * than all values in yIndex, use the last two points in yIndex
     * else if i is zero, y0 is greater than all values in yIndex,
     * use the first two points in yIndex.
     */
    if ((i == yIndexLen) && (yIndexLen != 1)) {
        yPtrLo = yIndexLen - 2;
        yPtrHi = yIndexLen - 1;
    } else if ((i == 0) && (yIndexLen != 1)) {
        yPtrLo = 0;
        yPtrHi = 1;
    } else if (yIndexLen == 1) {
        yPtrLo = 0;
        yPtrHi = 0;
    } else {
        yPtrLo = i - 1;
        yPtrHi = i;
    }
 
    yLo = yPtrLo;
    yHi = yPtrHi;
 
    /* Find the four z values on the table that bracket (x0,y0) */
    zxLoyLo = Amg[xPtrLo + yPtrLo*xIndexLen];
    zxLoyHi = Amg[xPtrLo + yPtrHi*xIndexLen];
    zxHiyLo = Amg[xPtrHi + yPtrLo*xIndexLen];
    zxHiyHi = Amg[xPtrHi + yPtrHi*xIndexLen];
    /*
     * Now interpolate to find the desired answer
     * First the intermediate step at "yLo"
     * Calculate xRatio only once, as it is used twice below.
     * Avoid divide by zero caused by scalar case of xIndex
     */
    
    if (xHi-xLo != 0) {
        xRatio = (x0-xLo)/(xHi-xLo);
    } else {
        xRatio = (x0-xLo);
    }
    zx0yLo = xRatio*(zxHiyLo-zxLoyLo) + zxLoyLo;
 
    /* then the intermediate value at "yHi" */
    zx0yHi = xRatio*(zxHiyHi-zxLoyHi) + zxLoyHi;
    /*
     * Finally, interpolate to find the value at (x0,y0)
     * Where y[0] = zx0y0, the desired interpolated value .
     * Avoid divide by zero caused by scalar indeces
     */
    if (yHi-yLo != 0) {
        return((y0-yLo)/(yHi-yLo)*(zx0yHi-zx0yLo) + zx0yLo);
    } else {
        return((y0-yLo) * (zx0yHi-zx0yLo) + zx0yLo);
    }
}
/*********************************************************************************/
/* FUNCTION voltageOC - function returns open circuit voltage in dependence of SoC*/
/*********************************************************************************/
static real_T voltageOC(real_T soc){
    real_T Uoc;
     
    Uoc = uock1*soc*soc*soc + uock2*soc*soc + uock3*soc + uock4;
    return(Uoc);
}
/*********************************************************************************/
/* FUNCTION HookeJeeves - function returns "optimal" ICE engine operating point  */
/*********************************************************************************/
static void Hooke_Jeeves(SimStruct *S, real_T wepoc, real_T tepoc, real_T wcd, real_T tcd, real_T soc, int_T mode){
    /* 2D optimization Hooke_Jeeves */
    real_T F, Fest, Fold;
    real_T wold, told, w, t, west, test;
    real_T ht = 10, hw = 10;
    real_T vt, vw; // control variables direction 
    int_T k = 1, i, broj_tocaka = 0;
    
    for(i = 1;i<=broj_tocaka+1;i++){
        /*********************************************************************************/
        /* Control variables initialization weUpr, teUpr ICE motora */
        wold = w = west = wepoc;
        if(wold < we10){
            wold = w = west = we10;
        }
        told = t = test = tepoc;
        Fold = costFunction(S, west, test, wcd, tcd, soc, mode); // cost function
        Fest = Fold;
        // printf("F = %f, west = %f, test = %f\n", Fest, west, test);
        /*********************************************************************************/
        do{
            do{
                // searching in w speed axes
                w = west + hw;
                if(w < we10){
                    w = we10;
                }
                F = costFunction(S, w, test, wcd, tcd, soc, mode); // cost function
                // printf("F = %f, west = %f, test = %f\n", F, w, test);
                if(Fest>F){
                    west = w;
                    Fest = F;
                }else{
                    w = west - hw;
                    if(w < we10){
                        w = we10;
                    }
                    F = costFunction(S, w, test, wcd, tcd, soc, mode); // cost function
                    // printf("F = %f, west = %f, test = %f\n", F, w, test);
                    if(Fest>F){
                        west = w;
                        Fest = F;
                    }
                }
                // searching in t torque axes
                t = test + ht;
                F = costFunction(S, west, t, wcd, tcd, soc, mode); // cost function
                // printf("F = %f, we = %f, te = %f\n", F, west, t);

                if(Fest>F){
                    test = t;
                    Fest = F;
                }else{
                    t = test - ht;
                    F = costFunction(S, west, t, wcd, tcd, soc, mode); // cost function
                    // printf("F = %f, west = %f, t = %f\n", F, w, test);
                    if(Fest>F){
                        test = t;
                        Fest = F;
                    }
                }
                if(Fest>=Fold){
                    // neuspjeh u trazenju tocaka
                    break;
                }            
                // uspjeh
                vt = test - told;
                vw = west - wold;
                Fold = Fest;
                wold = west;
                told = test;
                w = west = wold + vw; // new test variables
                t = test = told + vt; // new test variables
                k = 0;
            }while(1);

            if(k < 1){
                w = west = wold;
                t = test = told;
                k = 1;
            }
            if((ht>EPS)||(hw>EPS)){
                hw = hw/2;
                ht = ht/2;
                //printf("hw = %f, ht = %f, wold = %f, told = %f\n", hw, ht, wold, told);
            }else{
                break;
            }
        }while(1);
        
        weUpr = wold;
        teUpr = told;
    }
}

/**********************************************************************************************************/
/* FUNCTION HookeJeeves1D - function returns "optimal" ICE engine operating point on constant power curve */
/**********************************************************************************************************/
static void Hooke_Jeeves1D(SimStruct *S, real_T wepoc, real_T wcd, real_T tcd, real_T soc, int_T mode, real_T Pe){
    /* 2D optimization Hooke_Jeeves */
    real_T F, Fest, Fold;
    real_T wold, w, west;
    real_T hw = 5;
    real_T vw; // control variables direction 
    int_T k = 1, i, broj_tocaka = 0;
    
    for(i = 1;i<=broj_tocaka+1;i++){
        /*********************************************************************************/
        /* Control variables initialization weUpr, teUpr ICE motora */
        wold = w = west = wepoc;
        if(wold < we10){
            wold = w = west = we10;
        }
        
        Fold = costFunction(S, west, Pe/west, wcd, tcd, soc, mode); // cost function
        Fest = Fold;
        // printf("F = %f, west = %f, test = %f\n", Fest, west, test);
        /*********************************************************************************/
        do{
            do{
                // searching in w speed axes
                w = west + hw;
                if(w < we10){
                    w = we10;
                }
                F = costFunction(S, w, Pe/w, wcd, tcd, soc, mode); // cost function
                // printf("F = %f, west = %f, test = %f\n", F, w, test);
                if(Fest>F){
                    west = w;
                    Fest = F;
                }else{
                    w = west - hw;
                    if(w < we10){
                        w = we10;
                    }
                    F = costFunction(S, w, Pe/w, wcd, tcd, soc, mode); // cost function
                    // printf("F = %f, west = %f, test = %f\n", F, w, test);
                    if(Fest>F){
                        west = w;
                        Fest = F;
                    }
                }
               
                if(Fest>=Fold){
                    // neuspjeh u trazenju tocaka
                    break;
                }            
                // uspjeh
                vw = west - wold;
                Fold = Fest;
                wold = west;
                w = west = wold + vw; // new test variables
                k = 0;
            }while(1);

            if(k < 1){
                w = west = wold;
                k = 1;
            }
            if(hw > EPS){
                hw = hw/2;
                //printf("hw = %f, ht = %f, wold = %f, told = %f\n", hw, ht, wold, told);
            }else{
                break;
            }
        }while(1);
        
        weUpr = wold;
        teUpr = Pe/wold;
    }
}
#ifdef    MATLAB_MEX_FILE  /* Is file being compiled into the MEX-file ? */
#include "simulink.c"      /* Add functions for linking with Matlab */
#else
#include "cg_sfun.h"
#endif




