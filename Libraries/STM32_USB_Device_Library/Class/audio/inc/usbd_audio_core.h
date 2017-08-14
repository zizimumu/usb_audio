/**
  ******************************************************************************
  * @file    usbd_audio_core.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   header file for the usbd_audio_core.c file.
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

#ifndef __USB_AUDIO_CORE_H_
#define __USB_AUDIO_CORE_H_

#include "usbd_ioreq.h"
#include "usbd_req.h"
#include "usbd_desc.h"



#define AUDIO_IN_ENABLED 1

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup usbd_audio
  * @brief This file is the Header file for USBD_audio.c
  * @{
  */ 


/** @defgroup usbd_audio_Exported_Defines
  * @{
  */ 

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define AUDIO_OUT_PACKET                              (uint32_t)(((USBD_AUDIO_FREQ * 2 * 2) /1000)) 
#define MAX_AUDIO_OUT_PACKET                              (uint32_t)(((USBD_AUDIO_MAX_FREQ * 2 * 2) /1000)) 


#define AUDIO_FEED_UP_PACKET 3
#define MAX_RX_SIZE  0xffff// (sizeof(IsocOutBuff))


#define FEED_RATE 3
#define MAX_PACKET_NUM ((1<<FEED_RATE) * 60)

#ifdef FEED_UP_ENABLE
#define FEED_UP_EP_DESC_SIZE 9
#else
#define FEED_UP_EP_DESC_SIZE 0
#endif

#define AUDIO_CONFIG_DESC_SIZE                        (109 + 3 + FEED_UP_EP_DESC_SIZE+83 * AUDIO_IN_ENABLED)
#define AUDIO_INTERFACE_DESC_SIZE                     9
#define USB_AUDIO_DESC_SIZ                            0x09
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07

#define AUDIO_DESCRIPTOR_TYPE                         0x21
#define USB_DEVICE_CLASS_AUDIO                        0x01
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02
#define AUDIO_PROTOCOL_UNDEFINED                      0x00
#define AUDIO_STREAMING_GENERAL                       0x01
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                          0x01
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06

#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0C
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07

#define AUDIO_CONTROL_MUTE                            0x0001
#define AUDIO_CONTROL_VOL                            0x0002

#define AUDIO_FORMAT_TYPE_I                           0x01
#define AUDIO_FORMAT_TYPE_III                         0x03

#define USB_ENDPOINT_TYPE_ISOCHRONOUS                 0x01
#define AUDIO_ENDPOINT_GENERAL                        0x01

#define AUDIO_REQ_GET_CUR                             0x81
#define AUDIO_REQ_SET_CUR                             0x01

#define AUDIO_OUT_STREAMING_CTRL                      0x02


#define USB_REQ_DEST_DEVICE 0
#define USB_REQ_DEST_INTERFACE 1
#define USB_REQ_DEST_ENDPOINT 2




/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef struct _Audio_Fops
{
    uint8_t  (*Init)         (uint32_t  AudioFreq, uint32_t Volume, uint32_t options);
    uint8_t  (*DeInit)       (uint32_t options);
    uint8_t  (*AudioCmd)     (uint8_t* pbuf, uint32_t size, uint8_t cmd);
    uint8_t  (*VolumeCtl)    (uint8_t vol);
    uint8_t  (*MuteCtl)      (uint8_t cmd);
    uint8_t  (*PeriodicTC)   (uint8_t cmd);
    uint8_t  (*GetState)     (void);
}AUDIO_FOPS_TypeDef;
/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 
#define AUDIO_PACKET_SZE(frq)          (uint8_t)(((frq * 2 * 2)/1000) & 0xFF), \
                                       (uint8_t)((((frq * 2 * 2)/1000) >> 8) & 0xFF)
#define SAMPLE_FREQ(frq)               (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))
/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_Class_cb_TypeDef  AUDIO_cb;

/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
/**
  * @}
  */ 
#define FEED_FREQ_2_BUFF(buf,freq) ( *(u32 *)buf = ( ( ((u32) freq /1000) << 14) | ((freq %1000) << 4) )  )
#define AUDIO_FRAME_BITS 16

#define LOW_THRD_SIZE (sizeof(IsocOutBuff) /2 - AUDIO_OUT_PACKET*10)
#define HIGHT_THRD_SIZE (sizeof(IsocOutBuff) /2 + AUDIO_OUT_PACKET*10)


#define FEED_MIN_VALUE (USBD_AUDIO_FREQ - USBD_AUDIO_FREQ/8)
#define FEED_MAX_VALUE (USBD_AUDIO_FREQ + USBD_AUDIO_FREQ/4)

#define FAST_FEED_STEP    60 //20000  // (sizeof(IsocOutBuff)/2 /(USBD_AUDIO_FREQ/4) ) 
#define SLOW_FEED_STEP  60 //20000 // (sizeof(IsocOutBuff)/2 /(USBD_AUDIO_FREQ/8) ) 

struct AUDIO_DEV_S{
	u32 wr_buf_pt;
	u32 rd_buf_pt;
	u32 feed_freq;
	u32 feed_state  ;
	u32 work_freq  ;
	u32 total_size ;
	u32 last_size ;
	u32 PlayFlag ;
	u32 recordFlag;
	u32 host_cmd;
	u32 volume;
	u32 open;
	u32 recd_pt;
	char feed[16]; 
};


extern struct AUDIO_DEV_S audio_dev;

#define FABS(a,b) ((a)>(b)?((a)-(b)):((b)-(a)))
#define RX_BUFF_SIZE (100*192)










#endif  // __USB_AUDIO_CORE_H_
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
