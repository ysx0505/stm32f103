/**
  ******************************************************************************
  * @file    system_stm32f10x.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
							CMSIS Cortex-M3�豸������ʲ�ϵͳԴ�ļ���
  * 					
  * 1.  This file provides two functions and one global variable to be called from 
  *     user application:
				����ļ��ṩ������������һ��ȫ�ֱ������û���Ӧ�ó����е���:
  *      - SystemInit(): Setups the system clock (System clock source, PLL Multiplier
  *                      factors, AHB/APBx prescalers and Flash settings). 
  *                      This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f10x_xx.s" file.
				����ϵͳʱ��(ϵͳʱ��Դ�����໷�˷�����AHB/APBxԤ������Flash����)��
				��������ڸ�λ֮��main����������֮ǰ�����á�
				��������ڡ�startup_stm32f10x_xx.s���ļ��н��С�
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
				��������ʱ��(HCLK)���û�Ӧ�ó������ʹ����������SysTick��ʱ������������������
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
				���±���SystemCoreClock�����ұ����ڸú�������ִ���ڼ���ĺ���ʱ�ӡ�
	*
  * 2. After each device reset the HSI (8 MHz) is used as system clock source.
  *    Then SystemInit() function is called, in "startup_stm32f10x_xx.s" file, to
  *    configure the system clock before to branch to main program.
			ÿ���豸��λ��ʹ��HSI (8mhz)��Ϊϵͳʱ��Դ��
			Ȼ���ڡ�startup_stm32f10x_xx���ļ��е���SystemInit()���������ú�ϵͳʱ�ӣ���main����������֮ǰ����
  *
  * 3. If the system clock source selected by user fails to startup, the SystemInit()
  *    function will do nothing and HSI still used as system clock source. User can 
  *    add some code to deal with this issue inside the SetSysClock() function.
			����û�ѡ���ϵͳʱ��Դ�޷�������SystemInit()��������ִ���κβ�����HSI��Ȼ����ϵͳʱ��Դ��
			�û�������SetSysClock()���������һЩ����������������⡣
  *
  * 4. The default value of HSE crystal is set to 8 MHz (or 25 MHz, depedning on
  *    the product used), refer to "HSE_VALUE" define in "stm32f10x.h" file. 
  *    When HSE is used as system clock source, directly or through PLL, and you
  *    are using different crystal you have to adapt the HSE value to your own
  *    configuration.
			HSE �����Ĭ��ֵ����Ϊ8 MHz(��25 MHz����ʹ�õĲ�Ʒ���н���)��
			�ο���stm32f10x.h���ļ��ж���ġ�HSE_VALUE��������
			��HSE������ϵͳʱ��Դʱ��������ֱ��ʹ�û���ͨ�����໷��Ƶ��
			�㶼Ҫ���ݾ���Ĳ�������������õ�HSEֵ��
  *        
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f10x_system
  * @{
  */  
  
/** @addtogroup STM32F10x_System_Private_Includes
  * @{
  */

#include "stm32f10x.h"

/**
  * @}
  */

/** @addtogroup STM32F10x_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F10x_System_Private_Defines
  * @{
  */

/*!< Uncomment the line corresponding to the desired System clock (SYSCLK)
   frequency (after reset the HSI is used as SYSCLK source)
   ȡ��ע��������������ϵͳʱ��(SYSCLK)��Ӧ��Ƶ��(��λ����HSI��ΪSYSCLKԴ)
	 
   IMPORTANT NOTE:
	 ��Ҫ��ʾ��
   ============== 
   1. After each device reset the HSI is used as System clock source.
		ÿ���豸��λ��HSI��Ϊϵͳʱ��Դ��
   2. Please make sure that the selected System clock doesn't exceed your device's
      maximum frequency.
    ��ȷ��ѡ���ϵͳʱ�Ӳ����������豸��ʱ�����Ƶ�ʡ�
   3. If none of the define below is enabled, the HSI is used as System clock
    source.
		���û����������Ķ��壬��HSI����ϵͳʱ��Դ��
   4. The System clock configuration functions provided within this file assume that:
		���ļ��ṩ��ϵͳʱ�����ù�����Ҫ������������:
        - For Low, Medium and High density Value line devices an external 8MHz 
          crystal is used to drive the System clock.
					����С���С�������(flash��С)�ĳ�ֵ���豸���ⲿʹ��8MHz��������ϵͳʱ�ӡ�
					
        - For Low, Medium and High density devices an external 8MHz crystal is
          used to drive the System clock.
					����С���С�������(flash��С)�������ⲿʹ��8MHz��������ϵͳʱ�ӡ�
					
        - For Connectivity line devices an external 25MHz crystal is used to drive
          the System clock.
					�����������豸���ⲿʹ��25MHz��������ϵͳʱ�ӡ�
					
     If you are using different crystal you have to adapt those functions accordingly.
		 �����ʹ�ò�ͬ�ľ�������������Щ������
    */
   
#if defined (STM32F10X_LD_VL) || (defined STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
/* #define SYSCLK_FREQ_HSE    HSE_VALUE */
 #define SYSCLK_FREQ_24MHz  24000000
#else
/* #define SYSCLK_FREQ_HSE    HSE_VALUE */
/* #define SYSCLK_FREQ_24MHz  24000000 */ 
/* #define SYSCLK_FREQ_36MHz  36000000 */
/* #define SYSCLK_FREQ_48MHz  48000000 */
/* #define SYSCLK_FREQ_56MHz  56000000 */
#define SYSCLK_FREQ_72MHz  72000000
#endif

/*!< Uncomment the following line if you need to use external SRAM mounted
     on STM3210E-EVAL board (STM32 High density and XL-density devices) or on 
     STM32100E-EVAL board (STM32 High-density value line devices) as data memory
		�����Ҫʹ���ⲿSRAM���أ���ΪSTM3210E-EVAL����(STM32��������С��������)
		����ΪSTM32100E-EVAL��(STM32��������ֵ�豸)�����ݴ洢������ȡ��ע��������С�
*/ 
#if defined (STM32F10X_HD) || (defined STM32F10X_XL) || (defined STM32F10X_HD_VL)
/* #define DATA_IN_ExtSRAM */
#endif

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. 
		�����Ҫ���ڲ�SRAM�����¶�λ��������ȡ��ע��������С�
		 */ 
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field. 
                                  This value must be a multiple of 0x200. 
																	�������ƫ���ֶΡ�
																	���ֵ������0x200�ı�����
																	*/


/**
  * @}
  */

/** @addtogroup STM32F10x_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F10x_System_Private_Variables
  * @{
  */

/*******************************************************************************
*  Clock Definitions
*******************************************************************************/
#ifdef SYSCLK_FREQ_HSE
  uint32_t SystemCoreClock         = SYSCLK_FREQ_HSE;        /*!< System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_24MHz
  uint32_t SystemCoreClock         = SYSCLK_FREQ_24MHz;        /*!< System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_36MHz
  uint32_t SystemCoreClock         = SYSCLK_FREQ_36MHz;        /*!< System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_48MHz
  uint32_t SystemCoreClock         = SYSCLK_FREQ_48MHz;        /*!< System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_56MHz
  uint32_t SystemCoreClock         = SYSCLK_FREQ_56MHz;        /*!< System Clock Frequency (Core Clock) */
#elif defined SYSCLK_FREQ_72MHz
  uint32_t SystemCoreClock         = SYSCLK_FREQ_72MHz;        /*!< System Clock Frequency (Core Clock) */
#else /*!< HSI Selected as System Clock source */
  uint32_t SystemCoreClock         = HSI_VALUE;        /*!< System Clock Frequency (Core Clock) */
#endif

__I uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
/**
  * @}
  */

/** @addtogroup STM32F10x_System_Private_FunctionPrototypes
  * @{
  */

static void SetSysClock(void);

#ifdef SYSCLK_FREQ_HSE
  static void SetSysClockToHSE(void);
#elif defined SYSCLK_FREQ_24MHz
  static void SetSysClockTo24(void);
#elif defined SYSCLK_FREQ_36MHz
  static void SetSysClockTo36(void);
#elif defined SYSCLK_FREQ_48MHz
  static void SetSysClockTo48(void);
#elif defined SYSCLK_FREQ_56MHz
  static void SetSysClockTo56(void);  
#elif defined SYSCLK_FREQ_72MHz
  static void SetSysClockTo72(void);
#endif

#ifdef DATA_IN_ExtSRAM
  static void SystemInit_ExtMemCtl(void); 
#endif /* DATA_IN_ExtSRAM */

/**
  * @}
  */

/** @addtogroup STM32F10x_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the 
  *         SystemCoreClock variable.
						����΢������ϵͳ
						��ʼ��Ƕ��ʽFlash�ӿڡ�PLL������SystemCoreClock������
  * @note   This function should be used only after reset.
						�˺���Ӧ��ֻ�ڸ�λ��ʹ��.
  * @param  None
  * @retval None
  */
void SystemInit (void)
{
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
	/*��RCCʱ����������ΪĬ������״̬(���ڵ���)*/
	/*����HSIONλ(RCC_CR�Ĵ�����0λ) 0���ڲ�8MHz�����رգ�
																	 1���ڲ�8MHz����������*/
  RCC->CR |= (uint32_t)0x00000001;	//�����ڲ�����

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
	/*����SW[1:0], HPRE[7:4], PPRE1[10:8], PPRE2[13:11], ADCPRE[15:14]��MCO[26:24]λ*/
	/*��λ��RCC_CFGR�Ĵ�����*/
	/*SW[1:0] 00��HSI��Ϊϵͳʱ�ӣ�
						01��HSE��Ϊϵͳʱ�ӣ�
						10��PLL�����Ϊϵͳʱ�ӣ�
						11��������*/
	/*HPRE[7:4],  0xxx��SYSCLK����Ƶ��ΪAHBʱ��
								1000��SYSCLK 2��Ƶ��ΪAHBʱ��  1100��SYSCLK 64��Ƶ��ΪAHBʱ��
								1001��SYSCLK 4��Ƶ��ΪAHBʱ��  1101��SYSCLK 128��Ƶ��ΪAHBʱ��
								1010��SYSCLK 8��Ƶ��ΪAHBʱ��  1110��SYSCLK 256��Ƶ��ΪAHBʱ��
								1011��SYSCLK 16��Ƶ  1111��SYSCLK 512��Ƶ*/
	/*PPRE1[10:8] 0xx��HCLK����Ƶ��Ϊ����APB(APB1)ʱ��
								100��HCLK 2��Ƶ��Ϊ����APB(APB1)ʱ��
								101��HCLK 4��Ƶ��Ϊ����APB(APB1)ʱ��
								110��HCLK 8��Ƶ��Ϊ����APB(APB1)ʱ��
								111��HCLK 16��Ƶ��Ϊ����APB(APB1)ʱ��*/
	/*PPRE2[13:11] 	0xx��HCLK����Ƶ����Ϊ����APB(APB2)ʱ��
									100��HCLK 2��Ƶ����Ϊ����APB(APB2)ʱ��
									101��HCLK 4��Ƶ����Ϊ����APB(APB2)ʱ��
									110��HCLK 8��Ƶ����Ϊ����APB(APB2)ʱ��
									111��HCLK 16��Ƶ����Ϊ����APB(APB2)ʱ��*/
	/*ADCPRE[15:14] 	00��PCLK2 2��Ƶ����ΪADCʱ��
										01��PCLK2 4��Ƶ����ΪADCʱ��
										10��PCLK2 6��Ƶ����ΪADCʱ��
										11��PCLK2 8��Ƶ����ΪADCʱ��*/
	/*MCO[26:24]	0xx��û��ʱ�������
								100��ϵͳʱ��(SYSCLK)�����
								101���ڲ�RC����ʱ��(HSI)�����
								110���ⲿ����ʱ��(HSE)�����
								111��PLLʱ��2��Ƶ�����*/
#ifndef STM32F10X_CL
  RCC->CFGR &= (uint32_t)0xF8FF0000; //����SW, HPRE, PPRE1, PPRE2, ADCPRE��MCOλ 
#else
  RCC->CFGR &= (uint32_t)0xF0FF0000;
#endif /* STM32F10X_CL */   
  
  /* Reset HSEON, CSSON and PLLON bits */
	/*����HSEON[16], CSSON[19]��PLLON[24]λ*/
	/*��λ��RCC_CR�Ĵ�����*/
	/*HSEON[16] 0��HSE�����رգ�
							1��HSE����������*/
	/*CSSON[19] 0��ʱ�Ӽ�����رգ�
							1������ⲿ4-16MHz����������ʱ�Ӽ����������*/
	/*PLLON[24] 0��PLL�رգ�
							1��PLLʹ�ܡ�*/
  RCC->CR &= (uint32_t)0xFEF6FFFF; //����HSEON, CSSON �� PLLONλ

  /* Reset HSEBYP bit */
	/*����HSEBYPλ*/
	/*λ��RCC_CR�Ĵ�����18λ��0���ⲿ4-16MHz����û����·��
														1���ⲿ4-16MHz�ⲿ������������·��*/
  RCC->CR &= (uint32_t)0xFFFBFFFF;	//����HSEBYPλ

	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
	/*����PLLSRC[16]��PLLXTPRE[17]��PLLMUL[21:18]��USBPRE[22]/OTGFSPREλ*/
	/*��λ��RCC_CFGR�Ĵ�����*/
	/*PLLSRC[16]  0��HSI����ʱ�Ӿ�2��Ƶ����ΪPLL����ʱ��
								1��HSEʱ����ΪPLL����ʱ�ӡ�*/
	/*PLLXTPRE[17]  0��HSE����Ƶ��ΪPLL����ʱ�� 
									1��HSE 2��Ƶ��ΪPLL����ʱ�� */
	/*PLLMUL[21:18] 0000��PLL 2��Ƶ���  1000��PLL 10��Ƶ���
									0001��PLL 3��Ƶ���  1001��PLL 11��Ƶ���
									0010��PLL 4��Ƶ���  1010��PLL 12��Ƶ���
									0011��PLL 5��Ƶ���  1011��PLL 13��Ƶ���
									0100��PLL 6��Ƶ���  1100��PLL 14��Ƶ���
									0101��PLL 7��Ƶ���  1101��PLL 15��Ƶ���
									0110��PLL 8��Ƶ���  1110��PLL 16��Ƶ���
									0111��PLL 9��Ƶ���  1111��PLL 16��Ƶ���*/
	/*USBPRE[22]  0��PLLʱ��1.5����Ƶ��ΪUSBʱ��
								1��PLLʱ��ֱ����ΪUSBʱ��*/
  RCC->CFGR &= (uint32_t)0xFF80FFFF;	//����PLLSRC��PLLXTPRE��PLLMUL��USBPRE/OTGFSPREλ
#ifdef STM32F10X_CL
  /* Reset PLL2ON and PLL3ON bits */
  RCC->CR &= (uint32_t)0xEBFFFFFF;

  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x00FF0000;

  /* Reset CFGR2 register */
  RCC->CFGR2 = 0x00000000;
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;

  /* Reset CFGR2 register */
  RCC->CFGR2 = 0x00000000;      
#else
  /* Disable all interrupts and clear pending bits  */
	/*���������жϲ��������λ*/
  RCC->CIR = 0x009F0000;
#endif /* STM32F10X_CL */
    
#if defined (STM32F10X_HD) || (defined STM32F10X_XL) || (defined STM32F10X_HD_VL)
  #ifdef DATA_IN_ExtSRAM
    SystemInit_ExtMemCtl(); 
  #endif /* DATA_IN_ExtSRAM */
#endif 

  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
  /* Configure the Flash Latency cycles and enable prefetch buffer */
	/*����ϵͳʱ��Ƶ�ʡ�HCLK��PCLK2��PCLK1Ԥ��Ƶ��*/
	/*����Flash�ӳ����ڲ�����Ԥȡ������*/
  SetSysClock();

#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH. 
																							���ڲ�FLASH�е��������ض�λ��*/
#endif 
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *           
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.         
  *     
  * @note   - The system frequency computed by this function is not the real 
  *           frequency in the chip. It is calculated based on the predefined 
  *           constant and the selected clock source:
  *             
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f1xx.h file (default value
  *             8 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f1xx.h file (default value
  *              8 MHz or 25 MHz, depedning on the product used), user has to ensure
  *              that HSE_VALUE is same as the real frequency of the crystal used.
  *              Otherwise, this function may have wrong result.
  *                
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0;

#ifdef  STM32F10X_CL
  uint32_t prediv1source = 0, prediv1factor = 0, prediv2factor = 0, pll2mull = 0;
#endif /* STM32F10X_CL */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
  uint32_t prediv1factor = 0;
#endif /* STM32F10X_LD_VL or STM32F10X_MD_VL or STM32F10X_HD_VL */
    
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;
  
  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock */

      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
      
#ifndef STM32F10X_CL      
      pllmull = ( pllmull >> 18) + 2;
      
      if (pllsource == 0x00)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
      }
      else
      {
 #if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
       prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
       /* HSE oscillator clock selected as PREDIV1 clock entry */
       SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull; 
 #else
        /* HSE selected as PLL clock entry */
        if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t)RESET)
        {/* HSE oscillator clock divided by 2 */
          SystemCoreClock = (HSE_VALUE >> 1) * pllmull;
        }
        else
        {
          SystemCoreClock = HSE_VALUE * pllmull;
        }
 #endif
      }
#else
      pllmull = pllmull >> 18;
      
      if (pllmull != 0x0D)
      {
         pllmull += 2;
      }
      else
      { /* PLL multiplication factor = PLL input clock * 6.5 */
        pllmull = 13 / 2; 
      }
            
      if (pllsource == 0x00)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
      }
      else
      {/* PREDIV1 selected as PLL clock entry */
        
        /* Get PREDIV1 clock source and division factor */
        prediv1source = RCC->CFGR2 & RCC_CFGR2_PREDIV1SRC;
        prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
        
        if (prediv1source == 0)
        { 
          /* HSE oscillator clock selected as PREDIV1 clock entry */
          SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;          
        }
        else
        {/* PLL2 clock selected as PREDIV1 clock entry */
          
          /* Get PREDIV2 division factor and PLL2 multiplication factor */
          prediv2factor = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> 4) + 1;
          pll2mull = ((RCC->CFGR2 & RCC_CFGR2_PLL2MUL) >> 8 ) + 2; 
          SystemCoreClock = (((HSE_VALUE / prediv2factor) * pll2mull) / prediv1factor) * pllmull;                         
        }
      }
#endif /* STM32F10X_CL */ 
      break;

    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  
  /* Compute HCLK clock frequency ----------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;  
}

/**
  * @brief  Configures the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers.
						����ϵͳʱ��Ƶ�ʡ�HCLK��PCLK2��PCLK1Ԥ��Ƶ��
  * @param  None
  * @retval None
  */
static void SetSysClock(void)
{
#ifdef SYSCLK_FREQ_HSE
  SetSysClockToHSE();
#elif defined SYSCLK_FREQ_24MHz
  SetSysClockTo24();
#elif defined SYSCLK_FREQ_36MHz
  SetSysClockTo36();
#elif defined SYSCLK_FREQ_48MHz
  SetSysClockTo48();
#elif defined SYSCLK_FREQ_56MHz
  SetSysClockTo56();  
#elif defined SYSCLK_FREQ_72MHz
  SetSysClockTo72();
#endif
 
 /* If none of the define above is enabled, the HSI is used as System clock
    source (default after reset) 
		������϶��嶼û�����ã���HSI����ϵͳʱ��Դ(���ú��Ĭ��ֵ)*/ 
}

/**
  * @brief  Setup the external memory controller. Called in startup_stm32f10x.s 
  *          before jump to __main
  * @param  None
  * @retval None
  */ 
#ifdef DATA_IN_ExtSRAM
/**
  * @brief  Setup the external memory controller. 
  *         Called in startup_stm32f10x_xx.s/.c before jump to main.
  * 	      This function configures the external SRAM mounted on STM3210E-EVAL
  *         board (STM32 High density devices). This SRAM will be used as program
  *         data memory (including heap and stack).
  * @param  None
  * @retval None
  */ 
void SystemInit_ExtMemCtl(void) 
{
/*!< FSMC Bank1 NOR/SRAM3 is used for the STM3210E-EVAL, if another Bank is 
  required, then adjust the Register Addresses */

  /* Enable FSMC clock */
  RCC->AHBENR = 0x00000114;
  
  /* Enable GPIOD, GPIOE, GPIOF and GPIOG clocks */  
  RCC->APB2ENR = 0x000001E0;
  
/* ---------------  SRAM Data lines, NOE and NWE configuration ---------------*/
/*----------------  SRAM Address lines configuration -------------------------*/
/*----------------  NOE and NWE configuration --------------------------------*/  
/*----------------  NE3 configuration ----------------------------------------*/
/*----------------  NBL0, NBL1 configuration ---------------------------------*/
  
  GPIOD->CRL = 0x44BB44BB;  
  GPIOD->CRH = 0xBBBBBBBB;

  GPIOE->CRL = 0xB44444BB;  
  GPIOE->CRH = 0xBBBBBBBB;

  GPIOF->CRL = 0x44BBBBBB;  
  GPIOF->CRH = 0xBBBB4444;

  GPIOG->CRL = 0x44BBBBBB;  
  GPIOG->CRH = 0x44444B44;
   
/*----------------  FSMC Configuration ---------------------------------------*/  
/*----------------  Enable FSMC Bank1_SRAM Bank ------------------------------*/
  
  FSMC_Bank1->BTCR[4] = 0x00001011;
  FSMC_Bank1->BTCR[5] = 0x00000200;
}
#endif /* DATA_IN_ExtSRAM */

#ifdef SYSCLK_FREQ_HSE
/**
  * @brief  Selects HSE as System clock source and configure HCLK, PCLK2
  *         and PCLK1 prescalers.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockToHSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {

#if !defined STM32F10X_LD_VL && !defined STM32F10X_MD_VL && !defined STM32F10X_HD_VL
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 0 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);

#ifndef STM32F10X_CL
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;
#else
    if (HSE_VALUE <= 24000000)
	{
      FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;
	}
	else
	{
      FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_1;
	}
#endif /* STM32F10X_CL */
#endif
 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;
    
    /* Select HSE as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSE;    

    /* Wait till HSE is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x04)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  }  
}
#elif defined SYSCLK_FREQ_24MHz
/**
  * @brief  Sets System clock frequency to 24MHz and configure HCLK, PCLK2 
  *         and PCLK1 prescalers.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockTo24(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
#if !defined STM32F10X_LD_VL && !defined STM32F10X_MD_VL && !defined STM32F10X_HD_VL 
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 0 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;    
#endif
 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;
    
#ifdef STM32F10X_CL
    /* Configure PLLs ------------------------------------------------------*/
    /* PLL configuration: PLLCLK = PREDIV1 * 6 = 24 MHz */ 
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | 
                            RCC_CFGR_PLLMULL6); 

    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 10 = 4 MHz */       
    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                              RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                             RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV10);
  
    /* Enable PLL2 */
    RCC->CR |= RCC_CR_PLL2ON;
    /* Wait till PLL2 is ready */
    while((RCC->CR & RCC_CR_PLL2RDY) == 0)
    {
    }   
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
    /*  PLL configuration:  = (HSE / 2) * 6 = 24 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLXTPRE_PREDIV1_Div2 | RCC_CFGR_PLLMULL6);
#else    
    /*  PLL configuration:  = (HSE / 2) * 6 = 24 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLXTPRE_HSE_Div2 | RCC_CFGR_PLLMULL6);
#endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  } 
}
#elif defined SYSCLK_FREQ_36MHz
/**
  * @brief  Sets System clock frequency to 36MHz and configure HCLK, PCLK2 
  *         and PCLK1 prescalers. 
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockTo36(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 1 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_1;    
 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;
    
#ifdef STM32F10X_CL
    /* Configure PLLs ------------------------------------------------------*/
    
    /* PLL configuration: PLLCLK = PREDIV1 * 9 = 36 MHz */ 
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | 
                            RCC_CFGR_PLLMULL9); 

	/*!< PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 10 = 4 MHz */
        
    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                              RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                             RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV10);
  
    /* Enable PLL2 */
    RCC->CR |= RCC_CR_PLL2ON;
    /* Wait till PLL2 is ready */
    while((RCC->CR & RCC_CR_PLL2RDY) == 0)
    {
    }
    
#else    
    /*  PLL configuration: PLLCLK = (HSE / 2) * 9 = 36 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLXTPRE_HSE_Div2 | RCC_CFGR_PLLMULL9);
#endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  } 
}
#elif defined SYSCLK_FREQ_48MHz
/**
  * @brief  Sets System clock frequency to 48MHz and configure HCLK, PCLK2 
  *         and PCLK1 prescalers. 
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockTo48(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 1 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_1;    
 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
    
#ifdef STM32F10X_CL
    /* Configure PLLs ------------------------------------------------------*/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */
        
    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                              RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                             RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);
  
    /* Enable PLL2 */
    RCC->CR |= RCC_CR_PLL2ON;
    /* Wait till PLL2 is ready */
    while((RCC->CR & RCC_CR_PLL2RDY) == 0)
    {
    }
    
   
    /* PLL configuration: PLLCLK = PREDIV1 * 6 = 48 MHz */ 
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | 
                            RCC_CFGR_PLLMULL6); 
#else    
    /*  PLL configuration: PLLCLK = HSE * 6 = 48 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL6);
#endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  } 
}

#elif defined SYSCLK_FREQ_56MHz
/**
  * @brief  Sets System clock frequency to 56MHz and configure HCLK, PCLK2 
  *         and PCLK1 prescalers. 
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockTo56(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/   
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    
 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

#ifdef STM32F10X_CL
    /* Configure PLLs ------------------------------------------------------*/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */
        
    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                              RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                             RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);
  
    /* Enable PLL2 */
    RCC->CR |= RCC_CR_PLL2ON;
    /* Wait till PLL2 is ready */
    while((RCC->CR & RCC_CR_PLL2RDY) == 0)
    {
    }
    
   
    /* PLL configuration: PLLCLK = PREDIV1 * 7 = 56 MHz */ 
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | 
                            RCC_CFGR_PLLMULL7); 
#else     
    /* PLL configuration: PLLCLK = HSE * 7 = 56 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL7);

#endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  } 
}

#elif defined SYSCLK_FREQ_72MHz
/**
  * @brief  Sets System clock frequency to 72MHz and configure HCLK, PCLK2 
  *         and PCLK1 prescalers.
						����ϵͳʱ��Ƶ��Ϊ72MHz��������HCLK, PCLK2��PCLK1��Ƶ����
  * @note   This function should be used only after reset.
						�˺���Ӧ��ֻ�ڸ�λ��ʹ�á�
  * @param  None
  * @retval None
  */
static void SetSysClockTo72(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */
	/*SYSCLK, HCLK, PCLK2��PCLK1������*/
	/*ʹ��HSE*/
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);	//����HSE����
 
  /* Wait till HSE is ready and if Time out is reached exit */
	/*��HSE׼��������׼����ʱʱ������ѭ��*/
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;	//HSEStatus=1ʱ���ⲿ��������
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));	

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Enable Prefetch Buffer */
		/*ʹ��Ԥȡ������*/
    FLASH->ACR |= FLASH_ACR_PRFTBE;	//����Ԥȡ������

    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);	
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;	//���������ȴ�״̬
 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

#ifdef STM32F10X_CL
    /* Configure PLLs ------------------------------------------------------*/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */
        
    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                              RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                             RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);
  
    /* Enable PLL2 */
    RCC->CR |= RCC_CR_PLL2ON;
    /* Wait till PLL2 is ready */
    while((RCC->CR & RCC_CR_PLL2RDY) == 0)
    {
    }
    
   
    /* PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz */ 
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 | 
                            RCC_CFGR_PLLMULL9); 
#else    
    /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
		/*PLL����:PLLCLK = HSE * 9 = 72 MHz*/
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                                        RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
#endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
		/*�ȴ�PLL׼����*/
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }
    
    /* Select PLL as system clock source */
		/*ѡ�����໷��Ϊϵͳʱ��Դ*/
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
		/*�ȴ�PLL������ϵͳʱ��Դ*/
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
		/*���HSEû��������Ӧ�ó��򽫻��д����ʱ�����á��û��������������һЩ�����������������*/
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */
  
/**
  * @}
  */    
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
