/*********************************************************************************************************//**
 * @file    EXTI/GPIO_Interrupt/ht32f5xxxx_01_it.c
 * @version $Rev:: 2445         $
 * @date    $Date:: 2017-12-25 #$
 * @brief   This file provides all interrupt service routine.
 *************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/

/* Includes ------------------------------------------------------------------------------------------------*/
#include "ht32.h"
#include "ht32_board.h"

/** @addtogroup HT32_Series_Peripheral_Examples HT32 Peripheral Examples
  * @{
  */

/** @addtogroup EXTI_Examples EXTI
  * @{
  */

/** @addtogroup GPIO_Interrupt
  * @{
  */

//---------------------------------------------ring-----------------------------------------------------//
#include "ht32_board_config.h"

void TM_IRQHandler(void)
{
  extern __IO uint32_t wMctmUpdateDownCounter;
  TM_ClearFlag(BUZZER_TM, TM_INT_UEV);
  if (wMctmUpdateDownCounter)
    wMctmUpdateDownCounter--;
}

//---------------------------------------------ring-----------------------------------------------------//

void EXTI4_15_IRQHandler(void)
{
  int i;	
  //讀取在main.c中的變數guKeyState
  extern u32 ring;
	extern u32 initial;
	extern u32 up;
	extern u32 down;
  if(EXTI_GetEdgeStatus(EXTI_CHANNEL_12,EXTI_EDGE_POSITIVE))
  {
		ring = 0;
		printf("%d",ring);
		initial=1;
		//up=0;//
		//down=0;//
		for(i=0;i<100000;i++);//10000000
		EXTI_ClearEdgeFlag(EXTI_CHANNEL_12);

  }
}


extern vu32 BFTM_OverCnt;
void BFTM0_IRQHandler(void)
{
  BFTM_OverCnt++;

  BFTM_ClearFlag(HT_BFTM0);
}




/* Global functions ----------------------------------------------------------------------------------------*/
/*********************************************************************************************************//**
 * @brief   This function handles NMI exception.
 * @retval  None
 ************************************************************************************************************/
void NMI_Handler(void)
{
}

/*********************************************************************************************************//**
 * @brief   This function handles Hard Fault exception.
 * @retval  None
 ************************************************************************************************************/
void HardFault_Handler(void)
{
  while (1);
}

/*********************************************************************************************************//**
 * @brief   This function handles SVCall exception.
 * @retval  None
 ************************************************************************************************************/
void SVC_Handler(void)
{
}

/*********************************************************************************************************//**
 * @brief   This function handles PendSVC exception.
 * @retval  None
 ************************************************************************************************************/
void PendSV_Handler(void)
{
}

/*********************************************************************************************************//**
 * @brief   This function handles SysTick Handler.
 * @retval  None
 ************************************************************************************************************/
void SysTick_Handler(void)
{
}
