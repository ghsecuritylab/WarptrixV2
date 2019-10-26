
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "tcp.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi5_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

DMA_HandleTypeDef hdma_memtomem_dma2_stream4;
SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define PORT_TCP_RECEIVE 1236
#define TCP_CONNECTION_TIMEOUT 10

#define USB_MAP_DEBUGG 0
#define USB_SPI_DEBUGG 0
#define USB_TCP_DEBUGG 0
#define USB_TIM_DEBUGG 0
#define USB_NET_STATUS 0
#define USB_PBUF_DEBUGG 0
#define USB_WELCOME_MSG 0

#define VERSION_STRING "V0.1"
#define WELCOME_MSG "\r\n\r\nWarptrix LED-Panel on duty! \r\nWarptrix COMPORT debugg interface. \r\nVersion: '%s' \r\n\r\n"

#define SHIFTREGS_PER_PANEL 6 // Number of 8-bit shift registers per panel. Each register controlls 8 colums of the matrix
#define ROWS 8 // Number of row drivers. 1:8 row plexed matrix
#define X_PANEL_NR 10 // Nubmer of panels in X 
#define Y_PANEL_NR 4 // Number of panels in Y.
#define SHIFTREGS (X_PANEL_NR*Y_PANEL_NR*SHIFTREGS_PER_PANEL) // Total number of registers (240 with 4 panels in Y, 120 with 2 in Y)
#define LATCH_DELAY 40 // number of NOPs to wait after the data is latched. Prevents ghosting between frames.
#define X_SIZE (X_PANEL_NR * SHIFTREGS_PER_PANEL * 4 / 3) //Number of registers (bytes) in X-direction 80.
#define Y_SIZE (Y_PANEL_NR * ROWS * 3)                //Number of LED rows, NOT bytes! 96
#define COLUMS (SHIFTREGS * 4) // 960 
#define COLUMS_PER_PANEL (SHIFTREGS_PER_PANEL * 4)  // 24
#define SUBFRAMES 16// Max Number of Subframes. 4-bit SoftPWM for each LED

// State-machiene defines
#define STATE_NONE 0
// start command
#define STATE_S 1
#define STATE_ST 2
#define STATE_STA 3 
#define STATE_STAR 4
#define STATE_START 5
#define STATE_HIGHBYTE 6
#define STATE_LOWBYTE 7
// mode command
#define STATE_M 10
#define STATE_MO 11
#define STATE_MOD 12
#define STATE_MODE 13
#define STATE_MODE_1 14
#define STATE_MODE_2 15
#define STATE_MODE_3 16
#define STATE_MODE_4 17
#define STATE_MODE_5 18
#define STATE_MODE_6 19

// dma commands
#define STATE_D 20
#define STATE_DM 21
#define STATE_DMA 22

#define NEXT 255
#define GET_FRAME 254
#define GET_RAW_FRAME 253 // deprecated


// One frame consists of up to 16 subframes to imitate a simple software PWM. 
// Each LED has 4-bit brightness. One byte in an incoming frames consists of two half-bytes, describing to 2 LEDs. 
// The high half (high nibble) is for the left, low half (low nibble) for the right LED.

// one panel consists of 6 shift registers (6 bytes), format: 16*24 leds
/* 
*        1       2
*   1 oooooooo oooooooo 
*   2 oooooooo oooooooo
*   3 oooooooo oooooooo
*   4 oooooooo oooooooo
*   5 oooooooo oooooooo
*   6 oooooooo oooooooo
*   7 oooooooo oooooooo
*   8 oooooooo oooooooo
*        3       4
*   1 oooooooo oooooooo
*   2 oooooooo oooooooo
*   3 oooooooo oooooooo
*   4 oooooooo oooooooo
*   5 oooooooo oooooooo
*   6 oooooooo oooooooo
*   7 oooooooo oooooooo
*   8 oooooooo oooooooo
*        5       6
*   1 oooooooo oooooooo
*   2 oooooooo oooooooo
*   3 oooooooo oooooooo
*   4 oooooooo oooooooo
*   5 oooooooo oooooooo
*   6 oooooooo oooooooo
*   7 oooooooo oooooooo
*   8 oooooooo oooooooo
*/

extern struct netif gnetif;
struct tcp_pcb *pcb_tcp;

struct ip_struct {
    uint32_t ip_hex;
    uint8_t a;
    uint8_t b;
    uint8_t c;
    uint8_t d;
};

struct ip_struct my_ip;

char usb_data_out[2000];
//char usb_data_in[100];
char tcp1_status[100];
char tcp2_status[100];
char tcp3_status[100];
//char tcp4_status[400];
//char tcp5_status[400];
//char tcp6_status[400];
uint32_t status_counter = 0;
volatile uint32_t tcp_connection_timeout_counter = 0;

uint32_t current_position = 0; // deprecated?
uint32_t receiving_frame = 0;
uint32_t new_input_frame = 1;
volatile uint32_t new_output_frame = 1;
uint32_t processing_frame = 0;
volatile uint32_t next_frame = 1;
volatile uint32_t dma_swiched = 0;
uint32_t t_fps = 0;
uint32_t t_rps = 0;
uint32_t fps = 0;
uint32_t rps = 0;

uint32_t len = 0;
uint8_t *data;
uint32_t pos = 0;
uint8_t tcp_state = STATE_NONE;
uint8_t tcp_recv_state = NEXT;
uint32_t frame_len = 0;
uint32_t bytes_to_copy = 0;
uint32_t bytes_copied = 0;

uint8_t currentRow = 0;
uint8_t currentPanelRow = 0;
uint8_t currentSubframe = 0;
uint8_t inputBuffer[Y_SIZE*X_SIZE] = {0};
uint8_t frameBuffer[ROWS][COLUMS] = {0};
uint8_t outputBuffer_0[SUBFRAMES*ROWS*SHIFTREGS] = {0};
uint8_t outputBuffer_1[SUBFRAMES*ROWS*SHIFTREGS] = {0};
uint8_t current_buffer = 0; // buffer currently read from
uint8_t use_both_panels = 1; // use one or two panels. using one panel is twice as fast
uint8_t stop_dmas = 1;
uint8_t dma_spi_done = 1;
uint32_t dma_timeout = 0;
volatile uint32_t tim9_counter = 0;
HAL_StatusTypeDef spi2_err = HAL_OK;
HAL_StatusTypeDef spi5_err = HAL_OK;
HAL_StatusTypeDef spi2_error = HAL_OK; // used as flagg for debugging
HAL_StatusTypeDef spi5_error = HAL_OK;

uint8_t startup_pattern = 0;
uint8_t use_subframe_nr = SUBFRAMES;
uint8_t max_fps = 30;
volatile uint32_t milli_counter = 0;
volatile uint32_t milli_max = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI5_Init(void);
static void MX_FMC_Init(void);
static void MX_TIM9_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void tcp_setup(void);
static err_t tcp_connection_accepted(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_receive_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void close_connection(struct tcp_pcb *pcb, char *reason);
static err_t tcp_poll_timeout(void *args,struct tcp_pcb *tpcb);
static uint8_t tcp_StateMachiene(char d);

static void map_inputBuffer_to_frameBuffer();
static void map_frameBuffer_to_outputBuffer(void);
static void test_Pattern_inputBuffer(void);
static void test_Pattern_frameBuffer(void);
static void test_Pattern_outputBuffer(void);
static void reset(void);
static void send_status(void);
static void update_fps(void);
static void update_rps(void);
static void stop_display(void);
static void start_display(void);

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * spi);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI5_Init();
  MX_LWIP_Init();
  MX_USB_DEVICE_Init();
  MX_FMC_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Base_Start_IT(&htim9);
  stop_display();
  HAL_Delay(2000);  
  my_ip.ip_hex = netif_ip4_addr(&gnetif)->addr;
  my_ip.d = (my_ip.ip_hex & 0xff000000) >> 24;
  my_ip.c = (my_ip.ip_hex & 0x00ff0000) >> 16;
  my_ip.b = (my_ip.ip_hex & 0x0000ff00) >> 8;
  my_ip.a = (my_ip.ip_hex & 0x000000ff) >> 0; 
  if(USB_WELCOME_MSG) memset(usb_data_out, 0, sizeof(usb_data_out));
  if(USB_WELCOME_MSG) sprintf(usb_data_out,WELCOME_MSG,my_ip.a,my_ip.b,my_ip.c,my_ip.d,VERSION_STRING); 
  if(USB_WELCOME_MSG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000); 
  milli_max = (uint32_t) 1000.0/max_fps;
  MX_LWIP_Process();
  tcp_setup();
  reset();
  start_display();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    MX_LWIP_Process();
    status_counter++;
    dma_timeout++;
    if(status_counter == 5000000){ send_status(); status_counter = 0;}
    if(spi2_error != HAL_OK || spi5_error != HAL_OK){
        if(USB_SPI_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
        if(USB_SPI_DEBUGG) sprintf(usb_data_out,"ERROR IN SPI_DMA ROUTINE: \r\nspi2: %d\r\nspi5: %d\r\n",spi2_error,spi5_error); 
        if(USB_SPI_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
        spi2_error = HAL_OK;
        spi5_error = HAL_OK;
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART6
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 2;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 5000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream4
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream4 on DMA2_Stream4 */
  hdma_memtomem_dma2_stream4.Instance = DMA2_Stream4;
  hdma_memtomem_dma2_stream4.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream4.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream4.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream4.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream4.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream4.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream4.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream4.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream4.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream4.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream4.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream4.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins
     PE4   ------> LTDC_B0
     PE2   ------> QUADSPI_BK1_IO2
     PB8   ------> I2C1_SCL
     PB5   ------> USB_OTG_HS_ULPI_D7
     PD7   ------> SPDIFRX_IN0
     PC12   ------> SDMMC1_CK
     PE5   ------> DCMI_D6
     PE6   ------> DCMI_D7
     PB9   ------> I2C1_SDA
     PB6   ------> QUADSPI_BK1_NCS
     PJ13   ------> LTDC_B1
     PC11   ------> SDMMC1_D3
     PC10   ------> SDMMC1_D2
     PI4   ------> SAI2_MCLK_A
     PK7   ------> LTDC_DE
     PK6   ------> LTDC_B7
     PK5   ------> LTDC_B6
     PG12   ------> LTDC_B4
     PG10   ------> SAI2_SD_B
     PJ14   ------> LTDC_B2
     PD3   ------> DCMI_D5
     PI5   ------> SAI2_SCK_A
     PI7   ------> SAI2_FS_A
     PI10   ------> LTDC_HSYNC
     PI6   ------> SAI2_SD_A
     PK4   ------> LTDC_B5
     PG9   ------> DCMI_VSYNC
     PJ15   ------> LTDC_B3
     PD2   ------> SDMMC1_CMD
     PI9   ------> LTDC_VSYNC
     PH14   ------> DCMI_D4
     PK1   ------> LTDC_G6
     PK2   ------> LTDC_G7
     PC9   ------> SDMMC1_D1
     PI15   ------> LTDC_R0
     PJ11   ------> LTDC_G4
     PK0   ------> LTDC_G5
     PC8   ------> SDMMC1_D0
     PI14   ------> LTDC_CLK
     PH4   ------> USB_OTG_HS_ULPI_NXT
     PJ8   ------> LTDC_G1
     PJ10   ------> LTDC_G3
     PJ7   ------> LTDC_G0
     PJ9   ------> LTDC_G2
     PF6   ------> ADC3_IN4
     PJ6   ------> LTDC_R7
     PB13   ------> USB_OTG_HS_ULPI_D6
     PB12   ------> USB_OTG_HS_ULPI_D5
     PC0   ------> USB_OTG_HS_ULPI_STP
     PC2   ------> USB_OTG_HS_ULPI_DIR
     PB2   ------> QUADSPI_CLK
     PJ4   ------> LTDC_R5
     PD12   ------> QUADSPI_BK1_IO1
     PD13   ------> QUADSPI_BK1_IO3
     PJ5   ------> LTDC_R6
     PH12   ------> DCMI_D3
     PA0/WKUP   ------> ADCx_IN0
     PA4   ------> DCMI_HSYNC
     PJ3   ------> LTDC_R4
     PD11   ------> QUADSPI_BK1_IO0
     PH7   ------> I2C3_SCL
     PH9   ------> DCMI_D0
     PH11   ------> DCMI_D2
     PA6   ------> DCMI_PIXCLK
     PA5   ------> USB_OTG_HS_ULPI_CK
     PJ2   ------> LTDC_R3
     PB10   ------> USB_OTG_HS_ULPI_D3
     PH8   ------> I2C3_SDA
     PH10   ------> DCMI_D1
     PA3   ------> USB_OTG_HS_ULPI_D0
     PB1   ------> USB_OTG_HS_ULPI_D2
     PB0   ------> USB_OTG_HS_ULPI_D1
     PJ0   ------> LTDC_R1
     PJ1   ------> LTDC_R2
     PB11   ------> USB_OTG_HS_ULPI_D4
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, row1_Pin|latch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(row7_GPIO_Port, row7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, row5_Pin|row6_Pin|row3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, DCMI_PWR_EN_Pin|row4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(bank_mode_GPIO_Port, bank_mode_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, row2_Pin|row0_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CS2_Pin|bank_sel_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_B0_Pin */
  GPIO_InitStruct.Pin = LCD_B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(LCD_B0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin|ARDUINO_SDA_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin 
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin 
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : row1_Pin latch_Pin */
  GPIO_InitStruct.Pin = row1_Pin|latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC_CK_Pin SDMMC_D3_Pin SDMMC_D2_Pin PC9 
                           PC8 */
  GPIO_InitStruct.Pin = SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : row7_Pin bank_mode_Pin */
  GPIO_InitStruct.Pin = row7_Pin|bank_mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D6_Pin DCMI_D7_Pin */
  GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_B1_Pin LCD_B2_Pin LCD_B3_Pin LCD_G4_Pin 
                           LCD_G1_Pin LCD_G3_Pin LCD_G0_Pin LCD_G2_Pin 
                           LCD_R7_Pin LCD_R5_Pin LCD_R6_Pin LCD_R4_Pin 
                           LCD_R3_Pin LCD_R1_Pin LCD_R2_Pin */
  GPIO_InitStruct.Pin = LCD_B1_Pin|LCD_B2_Pin|LCD_B3_Pin|LCD_G4_Pin 
                          |LCD_G1_Pin|LCD_G3_Pin|LCD_G0_Pin|LCD_G2_Pin 
                          |LCD_R7_Pin|LCD_R5_Pin|LCD_R6_Pin|LCD_R4_Pin 
                          |LCD_R3_Pin|LCD_R1_Pin|LCD_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DE_Pin LCD_B7_Pin LCD_B6_Pin LCD_B5_Pin 
                           LCD_G6_Pin LCD_G7_Pin LCD_G5_Pin */
  GPIO_InitStruct.Pin = LCD_DE_Pin|LCD_B7_Pin|LCD_B6_Pin|LCD_B5_Pin 
                          |LCD_G6_Pin|LCD_G7_Pin|LCD_G5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_B4_Pin */
  GPIO_InitStruct.Pin = LCD_B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(LCD_B4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SDB_Pin */
  GPIO_InitStruct.Pin = SAI2_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_D5_Pin */
  GPIO_InitStruct.Pin = DCMI_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : row5_Pin row6_Pin LCD_DISP_Pin row3_Pin */
  GPIO_InitStruct.Pin = row5_Pin|row6_Pin|LCD_DISP_Pin|row3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_HSYNC_Pin LCD_VSYNC_Pin LCD_R0_Pin LCD_CLK_Pin */
  GPIO_InitStruct.Pin = LCD_HSYNC_Pin|LCD_VSYNC_Pin|LCD_R0_Pin|LCD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_VSYNC_Pin */
  GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC_CMD_Pin */
  GPIO_InitStruct.Pin = SDMMC_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(SDMMC_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_PWR_EN_Pin row4_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin|row4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D4_Pin DCMI_D3_Pin DCMI_D0_Pin DCMI_D2_Pin 
                           DCMI_D1_Pin */
  GPIO_InitStruct.Pin = DCMI_D4_Pin|DCMI_D3_Pin|DCMI_D0_Pin|DCMI_D2_Pin 
                          |DCMI_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : row2_Pin row0_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = row2_Pin|row0_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_A5_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARDUINO_A5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS2_Pin bank_sel_Pin */
  GPIO_InitStruct.Pin = CS2_Pin|bank_sel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_A0_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARDUINO_A0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_HSYNC_Pin PA6 */
  GPIO_InitStruct.Pin = DCMI_HSYNC_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_SCL_Pin LCD_SDA_Pin */
  GPIO_InitStruct.Pin = LCD_SCL_Pin|LCD_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*
###############################################
                    MAPPING 
###############################################
*/

static void map_inputBuffer_to_frameBuffer(void){
    int row = 0; // from 0 to 7 (8)
    int part = 2; // from 0 to 2 (3)
    int counter = 0; // from 0 to 7 (8)
    int panel_row = 0; //from 0 to 3 (4)   4 10 24 
    for(int j = 0; j<Y_SIZE;j++){
        row = j % ROWS;
        for(int i = 0; i<X_SIZE;i+=4){
            frameBuffer[row][panel_row*X_PANEL_NR*COLUMS_PER_PANEL+i*3+4*part+0]=inputBuffer[j*X_SIZE+i+0];
            frameBuffer[row][panel_row*X_PANEL_NR*COLUMS_PER_PANEL+i*3+4*part+1]=inputBuffer[j*X_SIZE+i+1];
            frameBuffer[row][panel_row*X_PANEL_NR*COLUMS_PER_PANEL+i*3+4*part+2]=inputBuffer[j*X_SIZE+i+2];
            frameBuffer[row][panel_row*X_PANEL_NR*COLUMS_PER_PANEL+i*3+4*part+3]=inputBuffer[j*X_SIZE+i+3];
        }
        counter++;
        if(counter % ROWS == 0){
            if(part != 0) part--;
            else part = 2;
        }
    if(counter % (ROWS*3) == 0) panel_row++;
    }
}

static void map_frameBuffer_to_outputBuffer(void){
    uint8_t reg = 0;
    for(int csf = 0; csf!=use_subframe_nr;csf++){
        for(int j = 0; j!=ROWS;j++){
            for(int i = 0; i!=SHIFTREGS;i+=1){
                reg = 0;
                if((frameBuffer[j][i*4+0] >> 4)   > csf) reg |=  1;
                if((frameBuffer[j][i*4+0] & 0x0f) > csf) reg |= (1 << 1);
                if((frameBuffer[j][i*4+1] >> 4)   > csf) reg |= (1 << 2);
                if((frameBuffer[j][i*4+1] & 0x0f) > csf) reg |= (1 << 3);
                if((frameBuffer[j][i*4+2] >> 4)   > csf) reg |= (1 << 4);
                if((frameBuffer[j][i*4+2] & 0x0f) > csf) reg |= (1 << 5);
                if((frameBuffer[j][i*4+3] >> 4)   > csf) reg |= (1 << 6);
                if((frameBuffer[j][i*4+3] & 0x0f) > csf) reg |= (1 << 7);
                if(current_buffer == 0) outputBuffer_0[csf*ROWS*SHIFTREGS + j*SHIFTREGS + SHIFTREGS-i-1] = reg; 
                else if(current_buffer == 1) outputBuffer_1[csf*ROWS*SHIFTREGS + j*SHIFTREGS + SHIFTREGS-i-1] = reg; 
            }
        }
    }
}   

/*
###############################################
                  TEST PATTERN 
###############################################
*/

static void test_Pattern_outputBuffer(void){
    for(int csf = 0; csf<SUBFRAMES;csf++){
        for(int j = 0; j<ROWS;j++){
            for(int i = 0; i<SHIFTREGS;i+=6){
                outputBuffer_0[csf*ROWS*SHIFTREGS + j*SHIFTREGS + i]   = 2;
                outputBuffer_0[csf*ROWS*SHIFTREGS + j*SHIFTREGS + i+1] = 2;
                outputBuffer_0[csf*ROWS*SHIFTREGS + j*SHIFTREGS + i+2] = 2;
                outputBuffer_0[csf*ROWS*SHIFTREGS + j*SHIFTREGS + i+3] = 2;
                outputBuffer_0[csf*ROWS*SHIFTREGS + j*SHIFTREGS + i+4] = 2;
                outputBuffer_0[csf*ROWS*SHIFTREGS + j*SHIFTREGS + i+5] = 2;
            }
        }
    }
}

static void test_Pattern_frameBuffer(void){
    for(int j = 0; j<ROWS;j++){
        for(int i = 0; i<COLUMS;i+=8){
            frameBuffer[j][i]   = 0x77;
            frameBuffer[j][i+1] = 0x66;
            frameBuffer[j][i+2] = 0x55;
            frameBuffer[j][i+3] = 0x44;
            frameBuffer[j][i+4] = 0x33;
            frameBuffer[j][i+5] = 0x22;
            frameBuffer[j][i+6] = 0x11;
            frameBuffer[j][i+7] = 0x00;
        }
    }
}

static void test_Pattern_inputBuffer(void){
  for(int j = 0; j<Y_SIZE;j++){ 
    for(int i = 0; i<X_SIZE;i+=8){
        inputBuffer[j*X_SIZE+i]   = 0x50;
        inputBuffer[j*X_SIZE+i+1] = 0x00;
        inputBuffer[j*X_SIZE+i+2] = 0x00;
        inputBuffer[j*X_SIZE+i+3] = 0x00;
        inputBuffer[j*X_SIZE+i+4] = 0x00;
        inputBuffer[j*X_SIZE+i+5] = 0x00;
        inputBuffer[j*X_SIZE+i+6] = 0x00;
        inputBuffer[j*X_SIZE+i+7] = 0x00;
    }
  }
}

/*
###############################################
                      SPI 
###############################################
*/

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * spi){
  //Transfer complete routien, called after DMA finnished sending data to spi  

}

static void stop_display(void){
    stop_dmas = 1;
    new_input_frame = 0;
    new_output_frame = 0;
    reset();
}

static void start_display(void){
    stop_dmas = 0;
}

static void reset(void){
  currentRow = 0;
  currentPanelRow = 0;
  currentSubframe = 0;
  current_buffer = 0;
  HAL_GPIO_WritePin(latch_GPIO_Port,latch_Pin,0);
  HAL_GPIO_WritePin(row0_GPIO_Port,row0_Pin,0);
  HAL_GPIO_WritePin(row1_GPIO_Port,row1_Pin,0); 
  HAL_GPIO_WritePin(row2_GPIO_Port,row2_Pin,0);
  HAL_GPIO_WritePin(row3_GPIO_Port,row3_Pin,0);
  HAL_GPIO_WritePin(row4_GPIO_Port,row4_Pin,0);
  HAL_GPIO_WritePin(row5_GPIO_Port,row5_Pin,0); 
  HAL_GPIO_WritePin(row6_GPIO_Port,row6_Pin,0);
  HAL_GPIO_WritePin(row7_GPIO_Port,row7_Pin,0);
  memset(inputBuffer, 0, sizeof(inputBuffer));
  memset(frameBuffer, 0, sizeof(frameBuffer));
  memset(outputBuffer_0, 0, sizeof(outputBuffer_0));
  memset(outputBuffer_1, 0, sizeof(outputBuffer_1));
}

/*
###############################################
                       TCP  
###############################################
*/

static uint8_t tcp_StateMachiene(char d){
   switch(tcp_state){

   case STATE_NONE:
      if(d=='s') tcp_state = STATE_S;
      else if(d=='d') tcp_state = STATE_D;
      else if(d=='m') tcp_state = STATE_M;
      else tcp_state = STATE_NONE;
      return NEXT;
	
   case STATE_S:
      if(d=='t') tcp_state = STATE_ST;
      else tcp_state = STATE_NONE;
      return NEXT;
      
   case STATE_ST:
      if(d=='a') tcp_state = STATE_STA;
      else tcp_state = STATE_NONE;
      return NEXT;
   
   case STATE_STA:
      if(d=='r') tcp_state = STATE_STAR;
      else tcp_state = STATE_NONE;
      return NEXT;
      
   case STATE_STAR:
      if(d=='t') tcp_state = STATE_START;
      else tcp_state = STATE_NONE;
      return NEXT;
    
   case STATE_START:
      tcp_state = STATE_HIGHBYTE;
      frame_len = d << 8;
      return NEXT;
      
   case STATE_HIGHBYTE:
      tcp_state = STATE_NONE;
      frame_len |=  d;
      if(frame_len <= sizeof(inputBuffer)) return GET_FRAME;
      
   case STATE_M:
      if(d=='o') tcp_state = STATE_MO;
      else tcp_state = STATE_NONE;
      return NEXT;
      
   case STATE_MO:
      if(d=='d') tcp_state = STATE_MOD;
      else tcp_state = STATE_NONE;
      return NEXT; 
      
   case STATE_MOD:
      if(d=='e') tcp_state = STATE_MODE;
      else tcp_state = STATE_NONE;
      return NEXT; 
      
   case STATE_MODE:
      if(d > 0 && d <= 255){ 
        max_fps = d;
        milli_max = 1000.0/(float)max_fps;
        if(USB_TCP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
        if(USB_TCP_DEBUGG) sprintf(usb_data_out, "Set framerate: %ld \r\n",max_fps);
        if(USB_TCP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
        tcp_state = STATE_MODE_1;
        return NEXT;
      }else{
        tcp_state = STATE_NONE;
        return NEXT;
      }
      
   case STATE_MODE_1:
      if(d > 0 && d <= 16){ 
        use_subframe_nr = d;
        if(USB_TCP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
        if(USB_TCP_DEBUGG) sprintf(usb_data_out, "Using %ld subframes.\r\n",use_subframe_nr);
        if(USB_TCP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
        tcp_state = STATE_MODE_2;
        return NEXT;  
      }else{
        tcp_state = STATE_NONE;
        return NEXT; 
      }   
      
    case STATE_MODE_2:
      if(d == 0 || d==1){ 
        use_both_panels = d;
        if(USB_TCP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
        if(USB_TCP_DEBUGG) sprintf(usb_data_out, "Using both panels: %ld \r\n",use_both_panels);
        if(USB_TCP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
        tcp_state = STATE_NONE;
        return NEXT;  
      }else{
        tcp_state = STATE_NONE;
        return NEXT; 
      }
       
   case STATE_D:
      if(d=='m') tcp_state = STATE_DM;
      else tcp_state = STATE_NONE;
      return NEXT; 
    
   case STATE_DM:
      if(d=='a') tcp_state = STATE_DMA;
      else tcp_state = STATE_NONE;
      return NEXT;
      
   case STATE_DMA:
      if(d=='0') stop_display();
      if(d=='1') start_display();
      if(USB_TCP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
      if(USB_TCP_DEBUGG && !stop_dmas) sprintf(usb_data_out, "Starting Display\r\n");
      if(USB_TCP_DEBUGG && stop_dmas) sprintf(usb_data_out, "Stopping Display\r\n");
      if(USB_TCP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
      tcp_state = STATE_NONE;
      return NEXT;
  
   default : 
      if(USB_TCP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
      if(USB_TCP_DEBUGG) sprintf(usb_data_out, "Undefiend state. Current char: '%c'\r\n",d);
      if(USB_TCP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
      tcp_state = STATE_NONE;
      return NEXT;
  }
}

static void tcp_setup(void){
  err_t error;
  pcb_tcp = tcp_new(); 
  if(pcb_tcp == NULL){
    sprintf(tcp1_status,"no pcb_tcp");
    return;
  } else {
    sprintf(tcp1_status,"created pcb_tcp");
  }
  
  error = tcp_bind(pcb_tcp,IP4_ADDR_ANY,PORT_TCP_RECEIVE); 
  if(error == ERR_USE){
    sprintf(tcp2_status, "Port %d already in use.", PORT_TCP_RECEIVE);
    return;
  } else if(error == ERR_VAL){
    sprintf(tcp2_status, "pcb_tcp not ready. Invalid Value.");
    return;
  } else if(error == ERR_OK){
    sprintf(tcp2_status, "pcb_tcp succesfully bound to port %d.",PORT_TCP_RECEIVE);
  }
  
  pcb_tcp = tcp_listen(pcb_tcp); 
  if(pcb_tcp == NULL){
    sprintf(tcp3_status,"no pcb_tcp_listen");
    return;
  } else {
    sprintf(tcp3_status,"created pcb_tcp_listen");
  }
  
  tcp_accept(pcb_tcp,tcp_connection_accepted); 	// sets callback function for incoming connections
  
}

static err_t tcp_connection_accepted(void *arg, struct tcp_pcb *newpcb, err_t err){
    if(USB_TCP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
    if(USB_TCP_DEBUGG) sprintf(usb_data_out, "New connection.\r\n");    
    if(USB_TCP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
    
    tcp_setprio(newpcb, TCP_PRIO_MAX);  // does this even do something?
    tcp_recv(newpcb, tcp_receive_callback); // sets receive callback function
    tcp_err(newpcb, NULL); //Don't care about error here
    tcp_poll(newpcb, tcp_poll_timeout, 1); // sets poll callback function and intervall (half-second wise)
    
    return ERR_OK;
}


static err_t tcp_receive_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err){ 
   if(p == NULL){ //check if remote host closed the connection.
        close_connection(tpcb, "Connection closed by remote host.");
        pbuf_free(p);
        current_position = 0;
        
   } else if (err == ERR_OK ){ // Process data
        uint32_t total_length = p->tot_len; // gets total length of pbuf
        while(1){
            uint32_t len = p->len;
            uint8_t *data = p->payload;
            uint32_t pos = 0; // next byte position to read
            tcp_connection_timeout_counter = 0; 
            while(pos<len){ // feeds the statemachiene until tcp_recv_state == GET_FRAME
                if(tcp_recv_state == NEXT){
                    for(;pos < len;){ 
                        tcp_recv_state = tcp_StateMachiene((char)data[pos]);
                        pos++;
                        if(tcp_recv_state == GET_FRAME){
                            bytes_copied = 0;
                            if(USB_TCP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
                            if(USB_TCP_DEBUGG) sprintf(usb_data_out,\
                            "Receiving frame of length: %ld \r\nFirst 16 bytes: %x:%x:%x:%x:%x:%x:%x:%x \r\n",frame_len
                                ,data[pos],data[pos+1],data[pos+2],data[pos+3],data[pos+4],data[pos+5],data[pos+6],data[pos+7]); 
                            if(USB_TCP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
                            break;
                        }
                    }
                }                         
                if(tcp_recv_state == GET_FRAME){ // copies bytes into input_buffer until frame_len bytes are copied
                    bytes_to_copy = len-pos;
                    if(bytes_to_copy+bytes_copied > frame_len) bytes_to_copy = frame_len-bytes_copied;         
                    memcpy(&inputBuffer[bytes_copied],&data[pos],bytes_to_copy);
                    bytes_copied += bytes_to_copy;
                    pos += bytes_to_copy;
                    if(bytes_copied >= frame_len){ // frame fully recieved, start display, set new_input_frame flag and move frame to output_buffer
                        if(USB_TCP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
                        if(USB_TCP_DEBUGG) sprintf(usb_data_out, "New frame complete! Received %ld\r\n",bytes_copied); 
                        if(USB_TCP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
                        tcp_recv_state = NEXT; // reset state
                        bytes_copied = 0; 
                        bytes_to_copy = 0;
                        start_display();            
                        while(!next_frame){} // next_frame gets set by a timer to keep the framerate
                        new_output_frame = 1;
                        dma_swiched = 0; 
                        next_frame = 0; // reset 
                        map_inputBuffer_to_frameBuffer(); // to frame buffer
                        map_frameBuffer_to_outputBuffer(); // to output buffer
                        if(USB_MAP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
                        if(USB_MAP_DEBUGG) sprintf(usb_data_out, 
                            "Processed new frame. fps: %ld rps: %ld buffer: %ld \r\n",fps,rps,current_buffer); 
                        if(USB_MAP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
                    }
                }  
            }
            if (p->tot_len > p->len){
                if(USB_PBUF_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
                if(USB_PBUF_DEBUGG) sprintf(usb_data_out, "Next Pbuf. tot_len: %ld, len: %ld \r\n",p->tot_len,p->len); 
                if(USB_PBUF_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
                p = p->next;
            }
            else{
                if(USB_PBUF_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
                if(USB_PBUF_DEBUGG) sprintf(usb_data_out, "Pbuf empty.\r\n"); 
                if(USB_PBUF_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
                break;
            }
        }
        tcp_recved(tpcb, total_length);
        pbuf_free(p);     
        
              
    } else { // if a different error then ERR_OK appears, the connection will be closed.
        close_connection(tpcb, "Transfer aborted or oder unknown error.");
        pbuf_free(p);
        current_position = 0;
        new_input_frame = 0;
    }
    return ERR_OK;
}

static err_t tcp_poll_timeout(void *args,struct tcp_pcb *tpcb){ // poll for timeout
    tcp_connection_timeout_counter++;
    if(tcp_connection_timeout_counter < TCP_CONNECTION_TIMEOUT){
        
    } else {
        close_connection(tpcb,"Connection timeout.");
        tcp_connection_timeout_counter = 0;
    }
    return ERR_OK;
}

static void close_connection(struct tcp_pcb *pcb, char *reason){
      if(USB_TCP_DEBUGG) memset(usb_data_out, 0, sizeof(usb_data_out));
      if(USB_TCP_DEBUGG) sprintf(usb_data_out, "Connection was closed.\r\nReason: '%s' \r\n",reason); 
      if(USB_TCP_DEBUGG) usb_send_data(usb_data_out,sizeof(usb_data_out),10000);  
      
      
      
      //stop_display();
      tcp_recv_state = NEXT;

      // close the tcp connection
      tcp_connection_timeout_counter = 0;
      tcp_arg(pcb, NULL);
      tcp_sent(pcb, NULL);
      tcp_recv(pcb, NULL);
      tcp_close(pcb);
}

/*
###############################################
                     MISC 
###############################################
*/

static void update_fps(void){
        fps = 1000.0/(HAL_GetTick()-t_fps);
        t_fps = HAL_GetTick();
}

static void update_rps(void){
        rps = 1000.0/(HAL_GetTick()-t_rps);
        t_rps = HAL_GetTick();
}

static void send_status(void){
    if(USB_NET_STATUS){
        my_ip.ip_hex = netif_ip4_addr(&gnetif)->addr;
        my_ip.d = (my_ip.ip_hex & 0xff000000) >> 24;
        my_ip.c = (my_ip.ip_hex & 0x00ff0000) >> 16;
        my_ip.b = (my_ip.ip_hex & 0x0000ff00) >> 8;
        my_ip.a = (my_ip.ip_hex & 0x000000ff) >> 0;   
        
        memset(usb_data_out, 0, sizeof(usb_data_out));        
        sprintf(usb_data_out,\
"##### Status Message #####\r\n \
    IP Addr: %d.%d.%d.%d\r\n \
    netif_is_up: %d\r\n \
    netif_is_link_up: %d\r\n \
    %s\r\n \
    %s\r\n \
    %s\r\n \
    fps: %ld\r\n \
    rps: %ld\r\n \
#######################\r\n"
        ,my_ip.a,my_ip.b,my_ip.c,my_ip.d,netif_is_up(&gnetif),netif_is_link_up(&gnetif),tcp1_status,tcp2_status,tcp3_status,fps,rps);   
        usb_send_data(usb_data_out,sizeof(usb_data_out),50000);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM6) {
    milli_counter++;
    if(milli_counter >= milli_max){
        milli_counter = 0;
        next_frame = 1;
        
    }
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim->Instance == TIM9){
  //tim9_counter++;
  dma_spi_done = 1;          
  if(!stop_dmas){
        dma_timeout = 0;
        HAL_SPI_DMAStop(&hspi2);
        HAL_SPI_DMAStop(&hspi5);
  
        if(currentPanelRow == 0){       
                
            HAL_GPIO_WritePin(row0_GPIO_Port,row0_Pin,0); // turn all rows off 
            HAL_GPIO_WritePin(row1_GPIO_Port,row1_Pin,0);
            HAL_GPIO_WritePin(row2_GPIO_Port,row2_Pin,0); 
            HAL_GPIO_WritePin(row3_GPIO_Port,row3_Pin,0);
            HAL_GPIO_WritePin(row4_GPIO_Port,row4_Pin,0);
            HAL_GPIO_WritePin(row5_GPIO_Port,row5_Pin,0); 
            HAL_GPIO_WritePin(row6_GPIO_Port,row6_Pin,0);
            HAL_GPIO_WritePin(row7_GPIO_Port,row7_Pin,0);
            
            //for(volatile int i=0;i<LATCH_DELAY;i++){} // NOPs to wait after rows are turned off. Prevents ghosting between frames.
            
            // Latch the previosly sent data to shift-register output  
            HAL_GPIO_WritePin(bank_sel_GPIO_Port,bank_sel_Pin,!currentPanelRow); // Latch for both bank_sel states
            HAL_GPIO_WritePin(latch_GPIO_Port,latch_Pin,1); 
            HAL_GPIO_WritePin(bank_sel_GPIO_Port,bank_sel_Pin,currentPanelRow);   
            HAL_GPIO_WritePin(latch_GPIO_Port,latch_Pin,0);   
           
            HAL_GPIO_WritePin(row0_GPIO_Port,row0_Pin,currentRow==0);
            HAL_GPIO_WritePin(row1_GPIO_Port,row1_Pin,currentRow==1); // turn on respective row
            HAL_GPIO_WritePin(row2_GPIO_Port,row2_Pin,currentRow==2); 
            HAL_GPIO_WritePin(row3_GPIO_Port,row3_Pin,currentRow==3);
            HAL_GPIO_WritePin(row4_GPIO_Port,row4_Pin,currentRow==4);
            HAL_GPIO_WritePin(row5_GPIO_Port,row5_Pin,currentRow==5); 
            HAL_GPIO_WritePin(row6_GPIO_Port,row6_Pin,currentRow==6);
            HAL_GPIO_WritePin(row7_GPIO_Port,row7_Pin,currentRow==7);          
            currentRow++;
            if(currentRow >= ROWS){
                currentRow = 0;
                currentSubframe++;
                if(currentSubframe >= use_subframe_nr){                                    
                    currentSubframe = 0;
                    update_rps();
                    // if the current subframe is 0, the last frame has been completly displayed. Check if there is a new frame.
                    // otherwise reuse the old one.
                    if(new_output_frame){
                        current_buffer = (current_buffer==0?1:0);
                        update_fps();
                        new_output_frame = 0;
                        dma_swiched = 1;
                    }
                }      
            } 
        }
              
        dma_spi_done = 0;
        HAL_GPIO_WritePin(bank_sel_GPIO_Port,bank_sel_Pin,currentPanelRow);
        if(current_buffer==1){
           spi2_err = HAL_SPI_Transmit_DMA(&hspi2,&outputBuffer_0[currentSubframe*ROWS*SHIFTREGS+currentRow*SHIFTREGS+currentPanelRow*120],60);            
           spi5_err = HAL_SPI_Transmit_DMA(&hspi5,&outputBuffer_0[currentSubframe*ROWS*SHIFTREGS+currentRow*SHIFTREGS+currentPanelRow*120+60],60);    
        }else if(current_buffer==0){
           spi2_err = HAL_SPI_Transmit_DMA(&hspi2,&outputBuffer_1[currentSubframe*ROWS*SHIFTREGS+currentRow*SHIFTREGS+currentPanelRow*120],60);            
           spi5_err = HAL_SPI_Transmit_DMA(&hspi5,&outputBuffer_1[currentSubframe*ROWS*SHIFTREGS+currentRow*SHIFTREGS+currentPanelRow*120+60],60);         
        }    
        if(use_both_panels){
            currentPanelRow++;
            if(currentPanelRow >= 2) currentPanelRow = 0;
        }
        else currentPanelRow = 0;
              
        if(spi2_err != HAL_OK || spi5_err != HAL_OK){
            spi2_error = spi2_err;
            spi5_error = spi5_err;
        }
    }
        
        
        
   }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
    HAL_GPIO_WritePin(row0_GPIO_Port,row0_Pin,0); // turn all rows off 
    HAL_GPIO_WritePin(row1_GPIO_Port,row1_Pin,0);
    HAL_GPIO_WritePin(row2_GPIO_Port,row2_Pin,0); 
    HAL_GPIO_WritePin(row3_GPIO_Port,row3_Pin,0);
    HAL_GPIO_WritePin(row4_GPIO_Port,row4_Pin,0);
    HAL_GPIO_WritePin(row5_GPIO_Port,row5_Pin,0); 
    HAL_GPIO_WritePin(row6_GPIO_Port,row6_Pin,0);
    HAL_GPIO_WritePin(row7_GPIO_Port,row7_Pin,0);
    memset(usb_data_out, 0, sizeof(usb_data_out));
    sprintf(usb_data_out, "#### CRITICAL ERROR!! ####");
    usb_send_data(usb_data_out,sizeof(usb_data_out),10000);
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
