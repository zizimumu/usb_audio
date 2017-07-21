/**
  ******************************************************************************
  * @file    usbd_audio_core.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB Audio Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as Audio Streaming Device
  *           - Audio Streaming data transfer
  *           - AudioControl requests management
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                Audio Class Driver Description
  *          =================================================================== 
  *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          
  *           @note
  *            The Audio Class 1.0 is based on USB Specification 1.0 and thus supports only
  *            Low and Full speed modes and does not allow High Speed transfers.
  *            Please refer to "USB Device Class Definition for Audio Devices V1.0 Mar 18, 98"
  *            for more details.
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - AudioControl Endpoint management
  *             - AudioControl requsests other than SET_CUR and GET_CUR
  *             - Abstraction layer for AudioControl requests (only Mute functionality is managed)
  *             - Audio Synchronization type: Adaptive
  *             - Audio Compression modules and interfaces
  *             - MIDI interfaces and modules
  *             - Mixer/Selector/Processing/Extension Units (Feature unit is limited to Mute control)
  *             - Any other application-specific modules
  *             - Multiple and Variable audio sampling rates
  *             - Out Streaming Endpoint/Interface (microphone)
  *      
  *  @endverbatim
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

#include "usbd_audio_core.h"
#include "usbd_audio_out_if.h"
#include "delay.h"
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup usbd_audio 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup usbd_audio_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_audio_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_audio_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 
extern void USART_SendStr(unsigned char *pBuf);

/** @defgroup usbd_audio_Private_FunctionPrototypes
  * @{
  */

/*********************************************
   AUDIO Device library callbacks
 *********************************************/
static uint8_t  usbd_audio_Init       (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_audio_DeInit     (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_audio_Setup      (void  *pdev, USB_SETUP_REQ *req);
static uint8_t  usbd_audio_EP0_RxReady(void *pdev);
static uint8_t  usbd_audio_DataIn     (void *pdev, uint8_t epnum);
static uint8_t  usbd_audio_DataOut    (void *pdev, uint8_t epnum);
static uint8_t  usbd_audio_SOF        (void *pdev);
static uint8_t  usbd_audio_OUT_Incplt (void  *pdev);

/*********************************************
   AUDIO Requests management functions
 *********************************************/
static void AUDIO_Req_GetCurrent(void *pdev, USB_SETUP_REQ *req);
static void AUDIO_Req_SetCurrent(void *pdev, USB_SETUP_REQ *req);
static uint8_t  *USBD_audio_GetCfgDesc (uint8_t speed, uint16_t *length);

void cacu_feed_up(void );
void wait_dma_done(void);

/**
  * @}
  */ 

/** @defgroup usbd_audio_Private_Variables
  * @{
  */ 
/* Main Buffer for Audio Data Out transfers and its relative pointers */


uint8_t  IsocOutBuff [MAX_AUDIO_OUT_PACKET*MAX_PACKET_NUM]; // 170 frames,32K


/* Main Buffer for Audio Control Rrequests transfers and its relative variables */
uint8_t  AudioCtl[64];
uint8_t  AudioCtlCmd = 0;
uint32_t AudioCtlLen = 0;
uint8_t  AudioCtlUnit = 0;



static __IO uint32_t  usbd_audio_AltSet = 0;
static uint8_t usbd_audio_CfgDesc[AUDIO_CONFIG_DESC_SIZE];

#ifdef TEST_MODE

u16 test_buff[256];
u32 uart_wr = 0;
u32 uart_rd = 0;
#endif


static DMA_InitTypeDef DMA_InitStructure; 

#if 0
char feed[16];
volatile u32 wr_buf_pt = 0;
u32 feed_freq;
u32 feed_state = 0;
u32 work_freq = USBD_AUDIO_MAX_FREQ;
volatile u32 total_size = 0;
volatile u32 last_size = 0;
u32 PlayFlag = 0;
#endif


struct AUDIO_DEV_S audio_dev;
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;

#define FABS(a,b) ((a)>(b)?((a)-(b)):(b)-(a))




uint8_t  usbd_audio_IN_Incplt(void *pdev)
{
	
	  return USBD_OK;

}


/* AUDIO interface class callbacks structure */
USBD_Class_cb_TypeDef  AUDIO_cb = 
{
  usbd_audio_Init,
  usbd_audio_DeInit,
  usbd_audio_Setup,
  NULL, /* EP0_TxSent */
  usbd_audio_EP0_RxReady,
  usbd_audio_DataIn,
  usbd_audio_DataOut,
  usbd_audio_SOF,
  usbd_audio_IN_Incplt,
  usbd_audio_OUT_Incplt,   
  USBD_audio_GetCfgDesc,
#ifdef USB_OTG_HS_CORE  
  USBD_audio_GetCfgDesc, /* use same config as per FS */
#endif    
};

/* USB AUDIO device Configuration Descriptor */
static uint8_t usbd_audio_CfgDesc[AUDIO_CONFIG_DESC_SIZE] =
{
  /* Configuration 1 */
  0x09,                                 /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE,    /* bDescriptorType */
  LOBYTE(AUDIO_CONFIG_DESC_SIZE),       /* wTotalLength  109 bytes*/
  HIBYTE(AUDIO_CONFIG_DESC_SIZE),      
  0x02,                                 /* bNumInterfaces */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
  0xC0,                                 /* bmAttributes  BUS Powred*/
  0x32*5,                                 /* bMaxPower = 100 mA*/
  /* 09 byte*/
  
  /* USB Speaker Standard interface descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x00,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/
  
  /* USB Speaker Class-specific AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
  0x00,          /* 1.00 */             /* bcdADC */
  0x01,
  0x27,                                 /* wTotalLength = 39*/
  0x00,
  0x01,                                 /* bInCollection */
  0x01,                                 /* baInterfaceNr */
  /* 09 byte*/
  
  /* USB Speaker Input Terminal Descriptor */
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
  0x01,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
  0x01,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bNrChannels */
  0x03,                                 /* wChannelConfig 0x0000  Mono */
  0x00,
  0x00,                                 /* iChannelNames */
  0x00,                                 /* iTerminal */
  /* 12 byte*/
  
  /* USB Speaker Audio Feature Unit Descriptor */
  0x09,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
  AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
  0x01,                                 /* bSourceID */
  0x01,                                 /* bControlSize */
  AUDIO_CONTROL_MUTE,                   /* bmaControls(0) */
  0x00,                                 /* bmaControls(1) */
  0x00,                                 /* iTerminal */
  /* 09 byte*/
  
  /*USB Speaker Output Terminal Descriptor */
  0x09,      /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
  0x03,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType  0x0301*/
  0x03,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bSourceID */
  0x00,                                 /* iTerminal */
  /* 09 byte*/
  
  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
  /* Interface 1, Alternate Setting 0                                             */
  AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/
  
  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
  AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x01,                                 /* bAlternateSetting */

#ifdef FEED_UP_ENABLE
  0x02,                                 /* bNumEndpoints */
#else 
  0x01,                                 /* bNumEndpoints */

#endif
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/
  
  /* USB Speaker Audio Streaming Interface Descriptor */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
  0x01,                                 /* bTerminalLink */
  0x01,                                 /* bDelay */
  0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
  0x00,
  /* 07 byte*/
  
  /* USB Speaker Audio Type I Format Interface Descriptor */
  0x0B+3,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
  AUDIO_FORMAT_TYPE_I,                /* bFormatType */ 
  0x02,                                 /* bNrChannels */
  0x02,                                 /* bSubFrameSize :  2 Bytes per frame (16bits) */
  16,                                   /* bBitResolution (16-bits per sample) */ 
  0x02,                                 /* bSamFreqType only one frequency supported */ 
  SAMPLE_FREQ(USBD_AUDIO_MAX_FREQ),         /* Audio sampling frequency coded on 3 bytes */
  SAMPLE_FREQ(44100),         /* Audio sampling frequency coded on 3 bytes */
  /* 11 byte*/
  
  /* Endpoint 1 - Standard Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
  AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/

#ifdef FEED_UP_ENABLE
  USB_ENDPOINT_TYPE_ISOCHRONOUS  | 0x04,        /* bmAttributes */
  (u8)(USB_OTG_MAX_RX_SIZE & 0xff),(u8)((USB_OTG_MAX_RX_SIZE>>8)&0xff),
#else
  USB_ENDPOINT_TYPE_ISOCHRONOUS,
   AUDIO_PACKET_SZE(USBD_AUDIO_MAX_FREQ),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
#endif
  //
 
  
  0x01,                                 /* bInterval */
  0x0,                                 /* bRefresh */
#ifdef FEED_UP_ENABLE
  AUDIO_FEED_UP_EP,                                 /* bSynchAddress */
#else 
  0,
#endif
  /* 09 byte*/

 
  /* Endpoint - Audio Streaming Descriptor*/
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
  AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
  0x01,                                 /* bmAttributes */
  0x00,                                 /* bLockDelayUnits */
  0x00,                                 /* wLockDelay */
  0x00,
  /* 07 byte*/

#ifdef FEED_UP_ENABLE

	  /* ##Endpoint 2 for feedback - Standard Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,  /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,         	  /* bDescriptorType */
  AUDIO_FEED_UP_EP,                        /* bEndpointAddress 2 in endpoint*/
  0x11,                               /* bmAttributes */
  3,0,                                /* wMaxPacketSize in Bytes 3 */
  1,                                  /* bInterval 1ms*/
  FEED_RATE,            /* bRefresh 1 ~ 9,power of 2*/
  0x00,                               /* bSynchAddress */
  /* 09 byte*/
#endif
} ;

/**
  * @}
  */ 

/** @defgroup usbd_audio_Private_Functions
  * @{
  */ 

#if 1
void I2S_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//ws clk data
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

// mclk config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_SPI2); 

}


void I2S_user_Init(u32 sample ,u32 frame_bits)
{
  	I2S_InitTypeDef I2S_InitStructure;
	u16 data_fmt= 0;

	I2S_GPIO_Init();



	switch(frame_bits){
	    case 32 :
	        data_fmt = I2S_DataFormat_32b;
	        break;
	    case 16 :
	        data_fmt = I2S_DataFormat_16b;
	        break;

	     default :
	        printf("I2S_user_Init: err frame bit frame_bits %d\r\n",frame_bits);
	        break;
	};

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  	SPI_I2S_DeInit(SPI2);
  	I2S_InitStructure.I2S_Standard = I2S_Standard_MSB;
  	I2S_InitStructure.I2S_DataFormat = data_fmt;
  	I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;
  	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;
  	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable; 
	I2S_InitStructure.I2S_AudioFreq = sample; 
  	I2S_Init(SPI2, &I2S_InitStructure);

	//I2S_Cmd(SPI2,ENABLE);
}




#define CODEC_DATA_FORTMAT_DSP 3
#define CODEC_DATA_FORTMAT_I2S 2
#define CODEC_DATA_FORTMAT_MSB_L 1
#define CODEC_DATA_FORTMAT_MSB_R 0

#define CODEC_DATA_LENGTH_32 3
#define CODEC_DATA_LENGTH_24 2
#define CODEC_DATA_LENGTH_20 1
#define CODEC_DATA_LENGTH_16 0

#define CODEC_MODEL_SLAVE  0
#define CODEC_MODEL_MAST 1


#define USE_256_FS_SAMPLE
#ifdef USE_256_FS_SAMPLE 
#define CODEC_SMAPLE_FMT_48K  0
#define CODEC_SMAPLE_FMT_96K 0x0f
#define CODEC_SMAPLE_FMT_44_1K 0x10
#define CODEC_SMAPLE_FMT_32K 0x0c
#else // 384 fs
#define CODEC_SMAPLE_FMT_48K  0x01
#define CODEC_SMAPLE_FMT_96K 0x0f
#define CODEC_SMAPLE_FMT_44_1K 0x11
#define CODEC_SMAPLE_FMT_32K 0x0d
#endif

void wm_spi_init(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);

	// GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);	// for nss
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  	SPI_I2S_DeInit(SPI1);
  	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;//8位数据模式
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//空闲模式下SCK为1
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//数据采样从第2个时间边沿开始
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS软件管理
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;//波特率
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//大端模式
  	SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC多项式
  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//主机模式

	
  	SPI_Init(SPI1, &SPI_InitStructure);
	//SPI_SSOutputCmd(SPI1,ENABLE);
  	SPI_Cmd(SPI1, ENABLE);
}


void wm8731_gpio_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Configure the GPIO_LED pin */
  //CSB sclk  SDIN 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;// | GPIO_Pin_5  | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

#define CSB_HIGH()		GPIO_SetBits(GPIOA,GPIO_Pin_4)
#define CSB_LOW()		GPIO_ResetBits(GPIOA,GPIO_Pin_4)


u16 SPI1_RW_16bit(u16 byte)
{
 	while((SPI1->SR&SPI_I2S_FLAG_TXE)==RESET);
 	SPI1->DR = byte;
 	while((SPI1->SR&SPI_I2S_FLAG_RXNE)==RESET);
 	return(SPI1->DR);
}


void write_register(u16 addr,u16 value)
{
	CSB_LOW();
	delay_us(1);
	SPI1_RW_16bit( ((addr<< 9) | (value&0x01ff)) );
	delay_us(1);
	CSB_HIGH();
}


void wm_8731_init(u32 sample,u32 frame_bits )
{
    u16 date_len = 0;
    u16 samp_fmt= 0;

    switch(frame_bits){
        case 32 :
            date_len = CODEC_DATA_LENGTH_32;
            break;
        case 24 :
            date_len = CODEC_DATA_LENGTH_24;
            break;
        case 16 :
            date_len = CODEC_DATA_LENGTH_16;
            break;

         default :
            printf("err frame bit frame_bits\r\n",frame_bits);
            break;
    };
    switch(sample){
        case 96000 :
            samp_fmt = CODEC_SMAPLE_FMT_96K;
            break;
        case 48000 :
            samp_fmt = CODEC_SMAPLE_FMT_48K;
            break;
        case 32000 :
            samp_fmt = CODEC_SMAPLE_FMT_32K;
            break;
	case 44100 :
		samp_fmt = CODEC_SMAPLE_FMT_44_1K;
		break;

         default :
	    samp_fmt = CODEC_SMAPLE_FMT_48K;
            printf("sample err %d,set to 48K\r\n",sample);
            break;
    };    
	wm_spi_init();
	wm8731_gpio_init();
	
	write_register(0x0f,0x00);		//reset all

	delay_ms(1);
	write_register(0x09,0x00);		//inactive
	delay_ms(1);
	
	write_register(0x02,0x6a);		//Left Headphone Out: set left line out volume,the max is 0x7f
	write_register(3, 0x6a);  	// Right Headphone Out: set right line out volume,,the max is 0x7f
	write_register(4, 0x15); 		 // Analogue Audio Path Control: set mic as input and boost it, and enable dac 
	write_register(5, 0x00);  	// ADC ,DAC Digital Audio Path Control: disable soft mute   
	write_register(6, 0);  			// power down control: power on all 
	write_register(7, CODEC_DATA_FORTMAT_MSB_L | (date_len <<2) );  	// 0x01:MSB,left,iwl=16-bits, Enable slave Mode;0x09 : MSB,left,24bit
	
	write_register(8, samp_fmt << 1);  	// Normal, Base OVer-Sampleing Rate 384 fs (BOSR=1) 
	
	write_register(9, 0x01);  	// active interface
}


void Audio_DMA_Init(u32 frame_bit)  
{ 
  	NVIC_InitTypeDef NVIC_InitStructure;
	u32 m_size,p_size;

	if(frame_bit == 16){
		m_size = DMA_MemoryDataSize_HalfWord;
		p_size = DMA_PeripheralDataSize_HalfWord;
	}
	else if(frame_bit == 32){
		m_size = DMA_MemoryDataSize_Word;
		p_size = DMA_PeripheralDataSize_Word;
		}
	else{
		printf("dma init err\r\n");
		return ;
	}
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); 
  	DMA_Cmd(DMA1_Stream4, DISABLE);
  	DMA_DeInit(DMA1_Stream4);
  	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x4000380C;
  	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;
  	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  	DMA_InitStructure.DMA_BufferSize = (uint32_t)0xFFFE;
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	DMA_InitStructure.DMA_PeripheralDataSize = p_size;
  	DMA_InitStructure.DMA_MemoryDataSize = m_size; 
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
  	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
  	DMA_Init(DMA1_Stream4, &DMA_InitStructure);  
	
  	//DMA_ITConfig(DMA1_Stream4, DMA_IT_TC | DMA_IT_HT, ENABLE);

  	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);   
  	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
}

//note : the i2s control need more times to be inited
static void AUDIO_Init(u32 audio_sample,u32 frame_bit)
{
	audio_sample &= 0x00ffffff;
	wm_8731_init(audio_sample,frame_bit);
	I2S_user_Init(audio_sample,frame_bit);
	Audio_DMA_Init(frame_bit);	
	audio_dev.wr_buf_pt = 0;
	audio_dev.feed_state = 0;

}

static void AUDIO_Disable(USB_OTG_CORE_HANDLE *pdev)
{
	DMA_Cmd(DMA1_Stream4,DISABLE); 
	I2S_Cmd(SPI2,DISABLE);
	audio_dev.wr_buf_pt = 0;
	audio_dev.feed_state = 0;
	audio_dev.PlayFlag = 0;

#ifdef TEST_MODE

	uart_rd = uart_wr = 0;
#endif
	  DCD_EP_Flush(pdev, AUDIO_OUT_EP);
	  DCD_EP_Flush(pdev, AUDIO_FEED_UP_EP);
	  DCD_EP_PrepareRx(pdev,
	                   AUDIO_OUT_EP,
	                   (uint8_t*)IsocOutBuff,                        
	                   MAX_RX_SIZE);  

	
}



void Audio_Play(u32 Addr, u32 Size)
{   

#define I2S_ENABLE_MASK                 0x0400



	audio_dev.PlayFlag = 1;
  	DMA_InitStructure.DMA_Memory0BaseAddr=(uint32_t)Addr;
  	DMA_InitStructure.DMA_BufferSize=(uint32_t)Size/(AUDIO_FRAME_BITS/8);
  	DMA_Init(DMA1_Stream4,&DMA_InitStructure);
  	DMA_Cmd(DMA1_Stream4,ENABLE); 

  	if ((SPI2->I2SCFGR & I2S_ENABLE_MASK)==0)I2S_Cmd(SPI2,ENABLE);

	
}

#endif


/**
* @brief  usbd_audio_Init
*         Initilaizes the AUDIO interface.
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
//u8 feed[16];
static uint8_t  usbd_audio_Init (void  *pdev, 
                                 uint8_t cfgidx)
{  
//u32 data = 0;
  /* Open EP OUT */
  DCD_EP_Open(pdev,
              AUDIO_OUT_EP,
             USB_OTG_MAX_RX_SIZE, 
              USB_OTG_EP_ISOC);

  DCD_EP_Flush(pdev, AUDIO_OUT_EP);


  /* Open EP OUT */
  DCD_EP_Open(pdev,
              AUDIO_FEED_UP_EP,
              AUDIO_FEED_UP_PACKET,
              USB_OTG_EP_ISOC);

DCD_EP_Flush(pdev, AUDIO_FEED_UP_EP);

    
  /* Prepare Out endpoint to receive audio data */
  DCD_EP_PrepareRx(pdev,
                   AUDIO_OUT_EP,
                   (uint8_t*)IsocOutBuff,                        
                   MAX_RX_SIZE);  

  //work_freq = work_freq & 0x00ffffff;
  //AUDIO_Init(work_freq,AUDIO_FRAME_BITS);

  

  
  return USBD_OK;
}

/**
* @brief  usbd_audio_Init
*         DeInitializes the AUDIO layer.
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  usbd_audio_DeInit (void  *pdev, 
                                   uint8_t cfgidx)
{ 
  DCD_EP_Close (pdev , AUDIO_OUT_EP);

  DCD_EP_Close(pdev,AUDIO_FEED_UP_EP);

  /* DeInitialize the Audio output Hardware layer */
  //if (AUDIO_OUT_fops.DeInit(0) != USBD_OK)
  {
   // return USBD_FAIL;
  }
  
  return USBD_OK;
}

/**
  * @brief  usbd_audio_Setup
  *         Handles the Audio control request parsing.
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */

static uint8_t  usbd_audio_Setup (void  *pdev, 
                                  USB_SETUP_REQ *req)
{
  uint16_t len=USB_AUDIO_DESC_SIZ;
  uint8_t  *pbuf=usbd_audio_CfgDesc + 18;
  uint8_t dest;
  // USB_OTG_DRXSTS_TypeDef   status;
//   u32 i,max;
//   USB_OTG_CORE_HANDLE *dev = pdev;
   
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* AUDIO Class Requests -------------------------------*/
  case USB_REQ_TYPE_CLASS :    
	debug_log("audio class request type %x : %x ,Value %d\r\n",req->bmRequest,req->bRequest,req->wValue);
	
    switch (req->bRequest)
    {
    case AUDIO_REQ_GET_CUR:
      AUDIO_Req_GetCurrent(pdev, req);
      break;
      
    case AUDIO_REQ_SET_CUR:
      AUDIO_Req_SetCurrent(pdev, req);   
      break;

    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
    }


    dest = req->bmRequest & USB_REQ_RECIPIENT_MASK;
    switch (dest)
    {
    case USB_REQ_RECIPIENT_ENDPOINT:
		if( (req->wValue >> 8) == SAMPLING_FREQ_CONTROL){
			debug_log("%s sample freq\r\n",(req->bRequest==AUDIO_REQ_GET_CUR?"get":"set"));
			if(req->bRequest==AUDIO_REQ_GET_CUR){
				audio_dev.work_freq = audio_dev.work_freq & 0x00ffffff;
				USBD_CtlSendData(pdev,(u8 *)&audio_dev.work_freq,3);

				printf("freq set to %d\r\n",audio_dev.work_freq);
			}
			else if(req->bRequest==AUDIO_REQ_SET_CUR){
				//AudioCtlCmd = SAMPLING_FREQ_CONTROL;
				USBD_CtlPrepareRx(pdev,(u8 *)&audio_dev.work_freq,3);
			}
		}
		else{
			USBD_CtlError (pdev, req);
		}

      break;
      	
    case USB_REQ_RECIPIENT_INTERFACE:

      break;
    case USB_REQ_RECIPIENT_DEVICE:

      break;

    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
    }
	

    break;
    
    /* Standard Requests -------------------------------*/
  case USB_REQ_TYPE_STANDARD:

	debug_log("audio Standard request type %x : %x ,Value %d\r\n",req->bmRequest,req->bRequest,req->wValue);
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
      {
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
        pbuf = usbd_audio_Desc;   
#else
        pbuf = usbd_audio_CfgDesc + 18;
#endif 
        len = MIN(USB_AUDIO_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, 
                        pbuf,
                        len);
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)&usbd_audio_AltSet,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < AUDIO_TOTAL_IF_NUM)
      {
        usbd_audio_AltSet = (uint8_t)(req->wValue);
	if(req->wValue == 0){

		if(audio_dev.PlayFlag){
			//audio_dev.PlayFlag = 0;
			wait_dma_done();
			AUDIO_Disable(pdev);
			printf("audio aplay done,total data %d\r\n",audio_dev.total_size);
		}
		audio_dev.total_size = 0;
	}
	else{
		audio_dev.work_freq = audio_dev.work_freq & 0x00ffffff;
		AUDIO_Init(audio_dev.work_freq,AUDIO_FRAME_BITS);
	}
      }
      else
      {
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req);
      }
      break;
    }
  }
  return USBD_OK;
}

/**
  * @brief  usbd_audio_EP0_RxReady
  *         Handles audio control requests data.
  * @param  pdev: device device instance
  * @retval status
  */
static uint8_t  usbd_audio_EP0_RxReady (void  *pdev)
{ 
  /* Check if an AudioControl request has been issued */
  if (AudioCtlCmd == AUDIO_REQ_SET_CUR)
  {/* In this driver, to simplify code, only SET_CUR request is managed */
    /* Check for which addressed unit the AudioControl request has been issued */
    if (AudioCtlUnit == AUDIO_OUT_STREAMING_CTRL)
    {/* In this driver, to simplify code, only one unit is manage */
      /* Call the audio interface mute function */
      //AUDIO_OUT_fops.MuteCtl(AudioCtl[0]);
      
      /* Reset the AudioCtlCmd variable to prevent re-entering this function */
      AudioCtlCmd = 0;
      AudioCtlLen = 0;
    }
  } 
  
  return USBD_OK;
}

/**
  * @brief  usbd_audio_DataIn
  *         Handles the audio IN data stage.
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_audio_DataIn (void *pdev, uint8_t epnum)
{

 #ifdef FEED_UP_ENABLE
  if (epnum == (AUDIO_FEED_UP_EP & 0x7F))
  {

   if(audio_dev.PlayFlag)
    {

      /* Flush endpoint */
      DCD_EP_Flush(pdev, AUDIO_FEED_UP_EP);
      
      /* Prepare next data to be sent */
      DCD_EP_Tx (pdev,
                 AUDIO_FEED_UP_EP,
                 (u8 *)audio_dev.feed,
                 3);    

	//  FEED_FREQ_2_BUFF(feed,47000);
	cacu_feed_up();
    }

  }

#endif
  return USBD_OK;
}

/**
  * @brief  usbd_audio_DataOut
  *         Handles the Audio Out data stage.
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */


static uint8_t  usbd_audio_DataOut (void *pdev, uint8_t epnum)
{     

	
  if (epnum == AUDIO_OUT_EP)
  {    
    /* Increment the Buffer pointer or roll it back when all buffers are full */

    /* Toggle the frame index */  
    ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame = 
      (((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame)? 0:1;
      
    /* Prepare Out endpoint to receive next audio packet */
    DCD_EP_PrepareRx(pdev,
                     AUDIO_OUT_EP,
                     NULL,
                     MAX_RX_SIZE);

  }
  
  return USBD_OK;
}

/**
  * @brief  usbd_audio_SOF
  *         Handles the SOF event (data buffer update and synchronization).
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */


// the feed value :   freq - freq/8 <= value <= freq + freq/4
//note,the endpoint MaxPakcet size is related to max feed freq

#define FEED_SLOW_FRE(stat,fre) 	\
								{\
									if(stat != SLOW_STATE){\
										fre = USBD_AUDIO_FREQ;\
										stat = SLOW_STATE;\
									}\
									fre -= SLOW_FEED_STEP; \
									if(fre <= FEED_MIN_VALUE) \
										fre = FEED_MIN_VALUE;}
		

#define  FEED_FAST_FRE(stat,fre) \
								{\
									if(stat != FAST_STATE){\
										fre = USBD_AUDIO_FREQ;\
										stat = FAST_STATE;\
									}\
									fre += FAST_FEED_STEP;\
									 if(fre >= FEED_MAX_VALUE)\
										 fre = FEED_MAX_VALUE;}
								
#define SLOW_STATE 1
#define FAST_STATE 2
#define NORMAL_STATE 0

#define DAM_MIN_GEP 3
void wait_dma_done(void)
{
	u32 dma,remaind_sz;
	//u8 *r_packet;
	int pre,next;
	
	dma = DMA_GetCurrDataCounter(DMA1_Stream4)*(AUDIO_FRAME_BITS/8);
	remaind_sz = (sizeof(IsocOutBuff)  - audio_dev.wr_buf_pt)/(AUDIO_FRAME_BITS/8);

	pre = remaind_sz - DAM_MIN_GEP;
	next = remaind_sz + DAM_MIN_GEP;

	if(pre < 0)
		pre = 0;
	if(next > sizeof(IsocOutBuff)/(AUDIO_FRAME_BITS/8) )
		next = sizeof(IsocOutBuff)/(AUDIO_FRAME_BITS/8);

	while(1){
		dma = DMA_GetCurrDataCounter(DMA1_Stream4) ;
		if(dma >= pre && dma <= next)
			break;
	}
}
 void cacu_feed_up(void )
{
#if 1
	u32 free_packet;
	u32 r_packet;
	static u32 state = NORMAL_STATE;
	u32 dma;
	//u32 tmp1,tmp2;
	

	dma = DMA_GetCurrDataCounter(DMA1_Stream4)*(AUDIO_FRAME_BITS/8);
	r_packet = (sizeof(IsocOutBuff) - dma ) ;


	if(r_packet > audio_dev.wr_buf_pt){
		free_packet = sizeof(IsocOutBuff) - (r_packet - audio_dev.wr_buf_pt);
		if(free_packet <= AUDIO_OUT_PACKET){
			printf("data over!!! %d--%d\r\n",dma,audio_dev.wr_buf_pt);

			AUDIO_Disable(&USB_OTG_dev);
		}
	}
	else{
		free_packet = audio_dev.wr_buf_pt - r_packet;
		if(free_packet <= AUDIO_OUT_PACKET){
			printf("data under!!! %d--%d\r\n",dma,audio_dev.wr_buf_pt);

			AUDIO_Disable(&USB_OTG_dev);
		}
	}
#ifdef TEST_MODE

	test_buff[uart_wr] = (free_packet/(AUDIO_FRAME_BITS/8));
	uart_wr++;
	if(uart_wr >= 256)
		uart_wr = 0;
#endif

	if(free_packet >= HIGHT_THRD_SIZE){
		//FEED_FAST_FRE(state,feed_freq);

		FEED_SLOW_FRE(state,audio_dev.feed_freq);
	}
	else if(free_packet <= LOW_THRD_SIZE){
		//FEED_SLOW_FRE(state,feed_freq);
		FEED_FAST_FRE(state,audio_dev.feed_freq);
	}
	else if(free_packet > LOW_THRD_SIZE && free_packet <= sizeof(IsocOutBuff) /(AUDIO_FRAME_BITS/8)){

		if(state ==SLOW_STATE ){
			audio_dev.feed_freq = USBD_AUDIO_FREQ;
			state = NORMAL_STATE;
		}

	}
	
	else if(free_packet < HIGHT_THRD_SIZE && free_packet >= sizeof(IsocOutBuff) /(AUDIO_FRAME_BITS/8)){
		if(state ==FAST_STATE ){
			audio_dev.feed_freq = USBD_AUDIO_FREQ;
			state = NORMAL_STATE;
		}
	}
		
	FEED_FREQ_2_BUFF(audio_dev.feed,audio_dev.feed_freq);
	
#endif
}
static uint8_t  usbd_audio_SOF (void *pdev)
{     
	//u32 data;
	
  /* Check if there are available data in stream buffer.
    In this function, a single variable (PlayFlag) is used to avoid software delays.
    The play operation must be executed as soon as possible after the SOF detection. */
  if (audio_dev.PlayFlag)
  {      

	if(audio_dev.feed_state == 0){


		DCD_EP_Flush(pdev, AUDIO_FEED_UP_EP);
		
		/* Prepare data to be sent for the feedback endpoint */
		DCD_EP_Tx (pdev,
				   AUDIO_FEED_UP_EP,
				   (u8 *)audio_dev.feed,
				   3);

		audio_dev.feed_state = 1;
	}
  }
  
  return USBD_OK;
}

/**
  * @brief  usbd_audio_OUT_Incplt
  *         Handles the iso out incomplete event.
  * @param  pdev: instance
  * @retval status
  */
static uint8_t  usbd_audio_OUT_Incplt (void  *pdev)
{
  return USBD_OK;
}

/******************************************************************************
     AUDIO Class requests management
******************************************************************************/
/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_GetCurrent(void *pdev, USB_SETUP_REQ *req)
{  
  /* Send the current mute state */
  USBD_CtlSendData (pdev, 
                    AudioCtl,
                    req->wLength);
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_SetCurrent(void *pdev, USB_SETUP_REQ *req)
{ 
  if (req->wLength)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev, 
                       AudioCtl,
                       req->wLength);
    
    /* Set the global variables indicating current request and its length 
    to the function usbd_audio_EP0_RxReady() which will process the request */
    AudioCtlCmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    AudioCtlLen = req->wLength;          /* Set the request data length */
    AudioCtlUnit = HIBYTE(req->wIndex);  /* Set the request target unit */
  }
}

/**
  * @brief  USBD_audio_GetCfgDesc 
  *         Returns configuration descriptor.
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_audio_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_audio_CfgDesc);
  return usbd_audio_CfgDesc;
}
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
