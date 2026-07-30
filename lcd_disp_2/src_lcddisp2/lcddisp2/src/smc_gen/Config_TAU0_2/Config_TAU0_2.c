/*
* Copyright (c) 2021 - 2026 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/***********************************************************************************************************************
* File Name        : Config_TAU0_2.c
* Component Version: 1.12.0
* Device(s)        : R5F1214CxSP
* Description      : This file implements device driver for Config_TAU0_2.
***********************************************************************************************************************/
/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "Config_TAU0_2.h"
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
* Function Name: R_Config_TAU0_2_Create
* Description  : This function initializes the TAU0 channel2 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_TAU0_2_Create(void)
{
    uint16_t temp = TPS0;

    temp &= _FFF0_TAU_CKM0_CLEAR;
    temp |= _0003_TAU_CKM0_FCLK_3;
    TPS0 = temp;
    TT0L |= (_08_TAU_CH3_STOP_TRG_ON | _04_TAU_CH2_STOP_TRG_ON);
    TMMK02 = 1U;    /* disable INTTM02 interrupt */
    TMIF02 = 0U;    /* clear INTTM02 interrupt flag */
    TMMK03 = 1U;    /* disable INTTM03 interrupt */
    TMIF03 = 0U;    /* clear INTTM03 interrupt flag */
    /* Channel 2 is used as master channel for PWM output function */
    TMR02 = _0000_TAU_CLOCK_SELECT_CKM0 | _0000_TAU_CLOCK_MODE_CKS | _0800_TAU_COMBINATION_MASTER | 
            _0000_TAU_TRIGGER_SOFTWARE | _0001_TAU_MODE_PWM_MASTER;
    TDR02 = _0063_TAU_TDR02_VALUE;
    TOM0L &= (uint8_t)~_04_TAU_CH2_SLAVE_OUTPUT;
    TOL0L &= (uint8_t)~_04_TAU_CH2_OUTPUT_LEVEL_L;
    TO0L &= (uint8_t)~_04_TAU_CH2_OUTPUT_VALUE_1;
    TOE0L_bit.no2 = 0U;    /* disables timer output */
    /* Channel 3 is used as slave channel for PWM output function */
    TMR03 = _0000_TAU_CLOCK_SELECT_CKM0 | _0000_TAU_CLOCK_MODE_CKS | _0000_TAU_COMBINATION_SLAVE | 
            _0400_TAU_TRIGGER_MASTER_INT | _0009_TAU_MODE_PWM_SLAVE;
    TDR03 = _0064_TAU_TDR03_VALUE;
    TOM0L |= _08_TAU_CH3_SLAVE_OUTPUT;
    TOL0L &= (uint8_t)~_08_TAU_CH3_OUTPUT_LEVEL_L;
    TO0L &= (uint8_t)~_08_TAU_CH3_OUTPUT_VALUE_1;
    TOE0L_bit.no3 = 1U;    /* enables timer output */
    /* Set TO03 pin */
    POM4 &= 0xFDU;
    P4 &= 0xFDU;
    PM4 &= 0xFDU;

    R_Config_TAU0_2_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_TAU0_2_Start
* Description  : This function starts the TAU0 channel2 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_TAU0_2_Start(void)
{
    TOE0L |= _08_TAU_CH3_OUTPUT_ENABLE;
    TS0L |= (_08_TAU_CH3_START_TRG_ON | _04_TAU_CH2_START_TRG_ON);
}

/***********************************************************************************************************************
* Function Name: R_Config_TAU0_2_Stop
* Description  : This function stops the TAU0 channel2 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_TAU0_2_Stop(void)
{
    TT0L |= (_08_TAU_CH3_STOP_TRG_ON | _04_TAU_CH2_STOP_TRG_ON);
    TOE0L &= (uint8_t)~_08_TAU_CH3_OUTPUT_ENABLE;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

