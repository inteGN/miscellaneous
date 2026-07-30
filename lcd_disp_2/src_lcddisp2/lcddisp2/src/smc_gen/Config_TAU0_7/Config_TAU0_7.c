/*
* Copyright (c) 2021 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
* File Name        : Config_TAU0_7.c
* Component Version: 1.9.0
* Device(s)        : R5F1214CxSP
* Description      : This file implements device driver for Config_TAU0_7.
***********************************************************************************************************************/
/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "Config_TAU0_7.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_TAU0_7_Create
* Description  : This function initializes the TAU0 channel7 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_TAU0_7_Create(void)
{
    uint16_t temp = TPS0;

    temp &= _FFF0_TAU_CKM0_CLEAR;
    temp |= _0003_TAU_CKM0_FCLK_3;
    TPS0 = temp;
    /* Stop channel 7 */
    TT0L_bit.no7 = 1U;    /* the count operation is stopped */
    /* TAU07 used as interval timer */
    TMR07 = _0000_TAU_CLOCK_SELECT_CKM0 | _0000_TAU_CLOCK_MODE_CKS | _0000_TAU_TRIGGER_SOFTWARE | 
            _0000_TAU_MODE_INTERVAL_TIMER | _0000_TAU_START_INT_UNUSED;
    TDR07 = _FFFF_TAU_TDR07_VALUE;
    TOM0L &= (uint8_t)~_80_TAU_CH7_SLAVE_OUTPUT;
    TOL0L &= (uint8_t)~_80_TAU_CH7_OUTPUT_LEVEL_L;
    TO0L &= (uint8_t)~_80_TAU_CH7_OUTPUT_VALUE_1;
    TOE0L_bit.no7 = 0U;    /* disables timer output */

    R_Config_TAU0_7_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_TAU0_7_Start
* Description  : This function starts the TAU0 channel7 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_TAU0_7_Start(void)
{
    TS0L_bit.no7 = 1U;    /* the count operation becomes enabled */
}

/***********************************************************************************************************************
* Function Name: R_Config_TAU0_7_Stop
* Description  : This function stops the TAU0 channel7 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_TAU0_7_Stop(void)
{
    TT0L_bit.no7 = 1U;    /* the count operation is stopped */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

