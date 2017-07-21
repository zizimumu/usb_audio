/**
  ******************************************************************************
  * @file    stm324xg_usb_audio_codec.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file includes the low layer driver for CS43L22 Audio Codec.
  * 
  *     *******************************************************************  
  *                           IMPORTANT NOTES
  *     *******************************************************************  
  *     This file is extracted and modified from the audio codec driver
  *     "stm324xg_eval_audio_codec.c" provided by STMicroelectronics for
  *     the STM32F4xx family (in folder Utilities\STM32_EVAL\STM3240_41_G_EVAL).
  *     This modified driver is intended for use ONLY in the scope of the
  *     USB Device Audio Example.
  *        
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
#include "stm324xg_usb_audio_codec.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#if 0
/* Mask for the bit EN of the I2S CFGR register */
#define I2S_ENABLE_MASK                 0x0400

/* Delay for the Codec to be correctly reset */
#define CODEC_RESET_DELAY               0x4FFF

/* Codec audio Standards */
#ifdef I2S_STANDARD_PHILLIPS
 #define  CODEC_STANDARD                0x04
 #define I2S_STANDARD                   I2S_Standard_Phillips         
#elif defined(I2S_STANDARD_MSB)
 #define  CODEC_STANDARD                0x00
 #define I2S_STANDARD                   I2S_Standard_MSB    
#elif defined(I2S_STANDARD_LSB)
 #define  CODEC_STANDARD                0x08
 #define I2S_STANDARD                   I2S_Standard_LSB    
#else 
 #error "Error: No audio communication standard selected !"
#endif /* I2S_STANDARD */

/* The 7 bits Codec adress (sent through I2C interface) */
#define CODEC_ADDRESS                   0x94  /* b00100111 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* This structure is declared glabal because it is handled by two different functions */
static DMA_InitTypeDef DMA_InitStructure; 
static I2S_InitTypeDef I2S_InitStructure;
static uint8_t OutputDev = 0;

uint32_t AudioTotalSize = 0xFFFF; /* This variable holds the total size of the audio file */
uint32_t AudioRemSize   = 0xFFFF; /* This variable holds the remaining data in audio file */
uint16_t *CurrentPos;             /* This variable holds the current poisition of audio pointer */

__IO uint32_t  CODECTimeout = CODEC_LONG_TIMEOUT;   

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*-----------------------------------
                           Audio Codec functions 
                                    ------------------------------------------*/
/* Low layer codec functions */
static void     Codec_CtrlInterface_Init(void);
static void     Codec_CtrlInterface_DeInit(void);
static void     Codec_AudioInterface_Init(uint32_t AudioFreq);
static void     Codec_AudioInterface_DeInit(void);
static void     Codec_Reset(void);
static uint32_t Codec_WriteRegister(uint32_t RegisterAddr, uint32_t RegisterValue);
static void     Codec_GPIO_Init(void);
static void     Codec_GPIO_DeInit(void);
static void     Delay(__IO uint32_t nCount);
#ifdef VERIFY_WRITTENDATA
static uint32_t Codec_ReadRegister(uint32_t RegisterAddr);
#endif /* VERIFY_WRITTENDATA */
/*----------------------------------------------------------------------------*/

/**
  * @brief  Configure the audio peripherals.
  * @param  OutputDevice: OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *                       OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO .
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  AudioFreq: Audio frequency used to paly the audio stream.
  * @retval o if correct communication, else wrong communication
  */
uint32_t EVAL_AUDIO_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)
{    


  return 0;
}

/**
  * @brief Dinitializes all the resources used by the codec (those initialized 
  *        by EVAL_AUDIO_Init() function) EXCEPT the I2C resources since they are 
  *        used by the IOExpander as well (and eventually other modules). 
  * @param None.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t EVAL_AUDIO_DeInit(void)
{ 

  return 0;
}

/**
  * @brief Starts playing audio stream from a data buffer for a determined size. 
  * @param pBuffer: Pointer to the buffer 
  * @param Size: Number of audio data BYTES.
  * @retval o if correct communication, else wrong communication
  */
uint32_t EVAL_AUDIO_Play(uint16_t* pBuffer, uint32_t Size)
{

  return 0;
}

/**
  * @brief This function Pauses or Resumes the audio file stream. In case
  *        of using DMA, the DMA Pause feature is used. In all cases the I2S 
  *        peripheral is disabled. 
  * 
  * @WARNING When calling EVAL_AUDIO_PauseResume() function for pause, only
  *         this function should be called for resume (use of EVAL_AUDIO_Play() 
  *         function for resume could lead to unexpected behaviour).
  * 
  * @param Cmd: AUDIO_PAUSE (or 0) to pause, AUDIO_RESUME (or any value different
  *        from 0) to resume. 
  * @param Addr: Address from/at which the audio stream should resume/pause.
  * @param Size: Number of data to be configured for next resume.
  * @retval o if correct communication, else wrong communication
  */
uint32_t EVAL_AUDIO_PauseResume(uint32_t Cmd, uint32_t Addr, uint32_t Size)
{    

      return 0;

}

/**
  * @brief Stops audio playing and Power down the Audio Codec. 
  * @param Option: could be one of the following parameters 
  * @arg   CODEC_PDWN_SW for software power off (by writing registers) Then no 
  *        need to reconfigure the Codec after power on.
  * @arg   ODEC_PDWN_HW completely shut down the codec (physically). Then need 
  *        to reconfigure the Codec after power on.  
  * @retval o if correct communication, else wrong communication
  */
uint32_t EVAL_AUDIO_Stop(uint32_t Option)
{

    return 0;

}

/**
  * @brief Controls the current audio volume level. 
  * @param Volume: Volume level to be set in percentage from 0% to 100% (0 for 
  *        Mute and 100 for Max volume level).
  * @retval o if correct communication, else wrong communication
  */
uint32_t EVAL_AUDIO_VolumeCtl(uint8_t Volume)
{
  /* Call the codec volume control function with converted volume value */
  return 0
}

/**
  * @brief Enable or disable the MUTE mode by software 
  * @param Command: could be AUDIO_MUTE_ON to mute sound or AUDIO_MUTE_OFF to 
  *        unmute the codec and restore previous volume level.
  * @retval o if correct communication, else wrong communication
  */
uint32_t EVAL_AUDIO_Mute(uint32_t Cmd)
{ 
  /* Call the Codec Mute function */
  return0
}

/**
  * @brief This function handles main Media layer interrupt. 
  * @param None.
  * @retval o if correct communication, else wrong communication
  */
void Audio_MAL_IRQHandler(void)
{    

}

/*========================

                CS43L22 Audio Codec Control Functions
                                                ==============================*/
/**
* @brief Initializes the audio codec and all related interfaces (control 
  *      interface: I2C and audio interface: I2S)
  * @param OutputDevice: can be OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
  *                       OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO .
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  AudioFreq: Audio frequency used to paly the audio stream.
  * @retval o if correct communication, else wrong communication
  */
uint32_t Codec_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)
{

  
  /* Return communication control value */
  return 0;  
}

/**
  * @brief Restore the audio codec state to default state and free all used 
  *        resources.
  * @param None.
  * @retval o if correct communication, else wrong communication
  */
uint32_t Codec_DeInit(void)
{
 
  
  /* Return communication control value */
  return 0;  
}

/**
  * @brief Start the audio Codec play feature.
  *        For this codec no Play options are required.
  * @param None.
  * @retval o if correct communication, else wrong communication
  */
uint32_t Codec_Play(void)
{
  /* 
     No actions required on Codec level for play command
     */  

  /* Return communication control value */
  return 0;  
}

/**
  * @brief Pauses and resumes playing on the audio codec.
  * @param Cmd: AUDIO_PAUSE (or 0) to pause, AUDIO_RESUME (or any value different
  *        from 0) to resume. 
  * @retval o if correct communication, else wrong communication
  */
uint32_t Codec_PauseResume(uint32_t Cmd)
{

  return 0;
}

/**
  * @brief Stops audio Codec playing. It powers down the codec.
  * @param CodecPdwnMode: selects the  power down mode.
  * @arg   CODEC_PDWN_SW: only mutes the audio codec. When resuming from this 
  *        mode the codec keeps the prvious initialization (no need to re-Initialize
  *        the codec registers).
  * @arg   CODEC_PDWN_HW: Physically power down the codec. When resuming from this
  *        mode, the codec is set to default configuration (user should re-Initialize
  *        the codec in order to play again the audio stream).
  * @retval o if correct communication, else wrong communication
  */
uint32_t Codec_Stop(uint32_t CodecPdwnMode)
{
  
  return 0;    
}

/**
  * @brief Highers or Lowers the codec volume level.
  * @param Volume: a byte value from 0 to 255 (refer to codec registers 
  *        description for more details).
  * @retval o if correct communication, else wrong communication
  */
uint32_t Codec_VolumeCtrl(uint8_t Volume)
{


  return 0;  
}

/**
  * @brief Enables or disables the mute feature on the audio codec.
  * @param Cmd: AUDIO_MUTE_ON to enable the mute or AUDIO_MUTE_OFF to disable the
  *             mute mode.
  * @retval o if correct communication, else wrong communication
  */
uint32_t Codec_Mute(uint32_t Cmd)
{

  return 0;  
}

/**
  * @brief Resets the audio codec. It restores the default configuration of the 
  *        codec (this function shall be called before initializing the codec).
  *
  * @note  This function calls an external driver function: The IO Expander driver.
  *
  * @param None.
  * @retval 0 if correct communication, else wrong communication
  */
static void Codec_Reset(void)
{

}

/**
  * @brief Switch dynamically (while audio file is played) the output target (speaker or headphone).
  *
  * @note  This function modifies a global variable of the audio codec driver: OutputDev.
  *
  * @param None.
  * @retval 0 if correct communication, else wrong communication
  */
uint32_t Codec_SwitchOutput(uint8_t Output)
{

  return 0;
}

/**
  * @brief Writes a Byte to a given register into the audio codec through the 
           control interface (I2C)
  * @param RegisterAddr: The address (location) of the register to be written.
  * @param RegisterValue: the Byte value to be written into destination register.
  * @retval o if correct communication, else wrong communication
  */
static uint32_t Codec_WriteRegister(uint32_t RegisterAddr, uint32_t RegisterValue)
{

  
  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return 0;  
}

#ifdef VERIFY_WRITTENDATA
/**
  * @brief Reads and returns te value of an audio codec register through the 
  *        control interface (I2C).
  * @param RegisterAddr: Address of the register to be read.
  * @retval Value of the register to be read or dummy value if the communication
  *         fails.
  */
static uint32_t Codec_ReadRegister(uint32_t RegisterAddr)
{

  /* Return the byte read from Codec */
  return 0;
}
#endif /* VERIFY_WRITTENDATA */

/**
  * @brief Initializes the Audio Codec control interface (I2C).
  * @param  None.
  * @retval None.
  */
static void Codec_CtrlInterface_Init(void)
{

  }
}

/**
  * @brief Restore the Audio Codec control interface to its default state.
  *        This function doesn't de-initialize the I2C because the I2C peripheral
  *        may be used by other modules.
  * @param  None.
  * @retval None.
  */
static void Codec_CtrlInterface_DeInit(void)
{
  /* Disable the I2C peripheral */ /* This step is not done here because 
     the I2C interface can be used by other modules */
  /* I2C_DeInit(CODEC_I2C); */
}

/**
  * @brief  Initializes the Audio Codec audio interface (I2S)
  * @note   This function assumes that the I2S input clock (through PLL_R in 
  *         Devices RevA/Z and through dedicated PLLI2S_R in Devices RevB/Y)
  *         is already configured and ready to be used.    
  * @param  AudioFreq: Audio frequency to be configured for the I2S peripheral. 
  * @retval None.
  */
static void Codec_AudioInterface_Init(uint32_t AudioFreq)
{
 
}

/**
  * @brief Restores the Audio Codec audio interface to its default state.
  * @param  None.
  * @retval None.
  */
static void Codec_AudioInterface_DeInit(void)
{

}

/**
  * @brief Initializes IOs used by the Audio Codec (on the control and audio 
  *        interfaces).
  * @param  None.
  * @retval None.
  */
static void Codec_GPIO_Init(void)
{

}

/**
  * @brief Restores the IOs used by the Audio Codec interface to their default 
  *        state
  * @param  None.
  * @retval None.
  */
static void Codec_GPIO_DeInit(void)
{

}

/**
  * @brief  Inserts a delay time (not accurate timing).
  * @param  nCount: specifies the delay time length.
  * @retval None.
  */
static void Delay( __IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}

#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t Codec_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {   
  }
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */
/*========================

                Audio MAL Interface Control Functions

                                                ==============================*/

/**
  * @brief  Initializes and prepares the Media to perform audio data transfer 
  *         from Media to the I2S peripheral.
  * @param  None.
  * @retval None.
  */
void Audio_MAL_Init(void)  
{   

}

/**
  * @brief  Restore default state of the used Media.
  * @param  None.
  * @retval None.
  */
void Audio_MAL_DeInit(void)  
{   

}

/**
  * @brief  Starts playing audio stream from the audio Media.
  * @param  Addr: Pointer to the audio stream buffer
  * @param  Size: Number of data in the audio stream buffer
  * @retval None.
  */
void Audio_MAL_Play(uint32_t Addr, uint32_t Size)
{   

}

/**
  * @brief  Pauses or Resumes the audio stream playing from the Media.
  * @param Cmd: AUDIO_PAUSE (or 0) to pause, AUDIO_RESUME (or any value different
  *        from 0) to resume. 
  * @param Addr: Address from/at which the audio stream should resume/pause.
  * @param Size: Number of data to be configured for next resume.
  * @retval None.
  */
void Audio_MAL_PauseResume(uint32_t Cmd, uint32_t Addr, uint32_t Size)
{
 
}

/**
  * @brief  Stops audio stream playing on the used Media.
  * @param  None.
  * @retval None.
  */
void Audio_MAL_Stop(void)
{   

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
