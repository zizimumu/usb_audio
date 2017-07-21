/**
  ******************************************************************************
  * @file    app.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides all the Application firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/ 

#include "usbd_audio_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include <stdio.h>

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup APP_AUDIO 
  * @brief Mass storage application module
  * @{
  */ 

/** @defgroup APP_AUDIO_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup APP_AUDIO_Private_Defines
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup APP_AUDIO_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup APP_AUDIO_Private_Variables
  * @{
  */ 
  
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
 #if defined   (__CC_ARM) /*!< ARM Compiler */
  __align(4) 
 #elif defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
 #elif defined (__GNUC__) /*!< GNU Compiler */
 #pragma pack(4) 
 #elif defined  (__TASKING__) /*!< TASKING Compiler */                           
  __align(4) 
 #endif /* __CC_ARM */
#endif

USB_OTG_CORE_HANDLE           USB_OTG_dev;




#define UART_DEFINED USART2

void COM_Init(uint32_t BaudRate)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
  
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	
  	USART_InitStructure.USART_BaudRate = BaudRate;//²¨ÌØÂÊÉèÖÃ
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;
  	USART_InitStructure.USART_Parity = USART_Parity_No;
  	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
  	USART_Init(UART_DEFINED, &USART_InitStructure);
	
  	USART_Cmd(UART_DEFINED, ENABLE);

  	USART_ClearFlag(UART_DEFINED, USART_FLAG_TC);
}

void USART_SendByte(USART_TypeDef* USARTx, unsigned char Data)
{
 // 	USART_SendData(USART1, (u8) ch);
 // 	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	

	USARTx->DR = (Data & (uint16_t)0x01FF);
	while((USARTx->SR & USART_FLAG_TC) == (uint16_t)RESET);
}
void USART_SendStr(unsigned char *pBuf)
{
 // 	USART_SendData(USART1, (u8) ch);
 // 	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	
 	uint16_t i=0;
 	USART_TypeDef* USARTx = UART_DEFINED;
 	
	if( 0 == pBuf)
		return ;
	while( 0 != pBuf[i])
	{
		USART_SendByte(USARTx,pBuf[i]);
		i++;
	}
}

#if 1
#pragma import(__use_no_semihosting)                
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
_sys_exit(int x) 
{ 
	x = x; 
} 
int fputc(int ch, FILE *f)
{      
	USART_SendByte(UART_DEFINED,(unsigned char) ch);
	return ch;
}
#endif 




void key_press_Init(void )
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void delay_mm(u32 delay)
{
	while(delay--);
}


extern u16 test_buff[256];
extern u32 uart_wr;
extern u32 uart_rd;



int main(void)
{
  __IO uint32_t i = 0;

  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32fxxx_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32fxxx.c file
  */  
 audio_dev.work_freq = 48000;
  
  USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
#else            
            USB_OTG_FS_CORE_ID,
#endif 
            &USR_desc, 
            &AUDIO_cb, 
            &USR_cb);

  COM_Init(921600);
  key_press_Init();
  USART_SendStr("system init done \r\n");

  printf("printf OK\r\n");
  /* Main loop */
  while (1)
  {    
  	if(audio_dev.PlayFlag)
		if (i++ == 0x200000){
			  STM_EVAL_LEDToggle(LED1);
			  i = 0;
		}

#ifdef TEST_MODE
	if(uart_wr  != uart_rd){
		printf("%d\r\n",test_buff[uart_rd]);
		uart_rd++;
		if(uart_rd >= 256)
			uart_rd = 0 ;
	}
#endif
  }
} 

#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
