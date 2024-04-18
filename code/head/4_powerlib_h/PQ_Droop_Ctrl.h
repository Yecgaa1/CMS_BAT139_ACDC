/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  £º
*****************************************************************************/


#ifndef __PQ_DROOP_CTRL_H
#define __PQ_DROOP_CTRL_H


typedef struct{   
                int  P_In;
                int  Q_In;
                int  P_Back;
                int  Q_Back;
                int  omigaCoeff_P;
                int  omigaCoeff_Q;
                int  omigaCoeff_Back_P;
                int  omigaCoeff_Back_Q;
                int  omigaCoeff_Diffe_Q;
                int  AMPCoeff_P;
                int  AMPCoeff_Q;
                int  AMPCoeff_Diffe_P;
                int  omiga_OutMax;
                int  omiga_OutMin;
                int  AMP_OutMax;
                int  AMP_OutMin;
                int  omiga_Out;
                int  AMP_Out;
               void  (*Calc)();	 // Pointer to calculation function    
}INV_PQ_Droop_Ctrl_Var_t;

#define INV_PQ_DROOP_CTRL_DEFAULTS { 0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
                                    0,\
	                         (void*)0 }

extern void INV_PQ_Droop_Ctrl(INV_PQ_Droop_Ctrl_Var_t *PQ_Droop_Info);                                   
extern INV_PQ_Droop_Ctrl_Var_t INV_PQ_Droop_Info;  

#endif /* PQ_DROOP_CTRL_H_ */
/*----------------------------------------------------------------------------*/
/*  No more.                                                                  */
/*----------------------------------------------------------------------------*/
