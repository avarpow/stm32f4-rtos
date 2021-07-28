/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "cJSON.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNTOF(a) (sizeof(a) / sizeof(*(a)))
#define MSG_LEN 10
#define USART_REC_LEN 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
cJSON *_flag;
cJSON *_t_x;
cJSON *_t_y;
cJSON *_p_x;
cJSON *_p_y;
cJSON *_v_x;
cJSON *_v_y;
int flag;
double t_x;
double t_y;
double p_x;
double p_y;
double v_x;
double v_y;
motor m1, m2;

uint16_t Mode;

uint8_t m_Uart1_RcvByte;
uint16_t USART1_RX_STA;
uint8_t USART1_RX_BUF[USART_REC_LEN];

uint8_t m_Uart2_RcvByte;
uint16_t USART2_RX_STA;
uint8_t USART2_RX_BUF[USART_REC_LEN];

int Start;               //�???????键启�???????
int point_choose[4];     //模式六�?�择�???????
double target_pos[9][2]; //储存的九个目标点的位�???????
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdio.h"
uint8_t myBuffer[] = "I have gotten your message: "; //User prompt information
uint8_t Enter[] = "\r\n";                            //Enter carriage return
uint8_t getBuffer[100] = "ttttttttttttt";            //user-defined buffer
uint8_t UART1_rxBuffer[1000];                        //user-defined buffer
uint8_t UART1_msg_ok = 0;
// Redirect print start

// The _write function is defined in syscalls.c using __weak, so you can define the _write function directly in other files
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i = 0;
  for (i = 0; i < len; i++)
    ITM_SendChar((*ptr++));
  return len;
}
// redirect print end
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  if (huart->ErrorCode & HAL_UART_ERROR_ORE)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(huart, &m_Uart1_RcvByte, 1);
  }
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_ErrorCallback can be implemented in the user file.
   */
}

void UART1_ReceiveByte(uint8_t Res)
{

  if ((USART1_RX_STA & 0x8000) == 0) //????δ???
  {
    if (USART1_RX_STA & 0x4000) //???????0x0d
    {
      if (Res != 0x0a)
        USART1_RX_STA = 0; //???????,??????
      else
        USART1_RX_STA |= 0x8000; //?????????
    }
    else //??????0X0D
    {
      if (Res == 0x0d)
        USART1_RX_STA |= 0x4000;
      else
      {
        USART1_RX_BUF[USART1_RX_STA & 0X3FFF] = Res;
        USART1_RX_STA++;
        if (USART1_RX_STA > (USART_REC_LEN - 1))
          USART1_RX_STA = 0; //???????????,??????????
      }
    }
  }
}
void UART2_ReceiveByte(uint8_t Res)
{

  if ((USART2_RX_STA & 0x8000) == 0)
  {
    if (USART2_RX_STA & 0x4000)
    {
      if (Res != 0x0a)
        USART2_RX_STA = 0;
      else
        USART2_RX_STA |= 0x8000; //?????????
    }
    else //??????0X0D
    {
      if (Res == 0x0d)
        USART2_RX_STA |= 0x4000;
      else
      {
        USART2_RX_BUF[USART2_RX_STA & 0X3FFF] = Res;
        USART2_RX_STA++;
        if (USART2_RX_STA > (USART_REC_LEN - 1))
          USART2_RX_STA = 0; //???????????,??????????
      }
    }
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  HAL_StatusTypeDef status;

  if (UartHandle->Instance == USART1)
  {
    UART1_ReceiveByte(m_Uart1_RcvByte);
    status = HAL_UART_Receive_IT(&huart1, &m_Uart1_RcvByte, 1);
    // if (HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, MSG_LEN) != HAL_OK)
    // {
    //   Error_Handler();
    // }
  }
  if (UartHandle->Instance == USART2)
  {
    UART2_ReceiveByte(m_Uart2_RcvByte);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    status = HAL_UART_Receive_IT(&huart2, &m_Uart2_RcvByte, 1);
    // if (HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, MSG_LEN) != HAL_OK)
    // {
    //   Error_Handler();
    // }
  }
}
void uart_print(UART_HandleTypeDef *uart, uint8_t *s)
{
  while (*s != '\0')
  {
    if (HAL_UART_Transmit(uart, s, 1, 100) != HAL_OK)
    {
      Error_Handler();
    }
    s++;
  }
}
void My_Jsonparse(const char *USART_RX_BUF)
{
  cJSON *recv_json = cJSON_Parse(USART_RX_BUF);
  //if (!root) printf("wwwwwwwwwwwwwww\r\n");
  //else printf("yyyyyyyyyyyyyyyyy!!!!\r\n");
  cJSON *_flag = cJSON_GetObjectItem(recv_json, "flag");
  cJSON *_p_x = cJSON_GetObjectItem(recv_json, "pos_x");
  cJSON *_p_y = cJSON_GetObjectItem(recv_json, "pos_y");
  cJSON *_v_x = cJSON_GetObjectItem(recv_json, "v_x");
  cJSON *_v_y = cJSON_GetObjectItem(recv_json, "v_y");
  cJSON *_t_x = cJSON_GetObjectItem(recv_json, "x");
  cJSON *_t_y = cJSON_GetObjectItem(recv_json, "y");
  double t_x = _t_x->valuedouble;
  double t_y = _t_y->valuedouble;
  double p_x = _p_x->valuedouble;
  double p_y = _p_y->valuedouble;
  double v_x = _v_x->valuedouble;
  double v_y = _v_y->valuedouble;
  ;
  //printf("qaq%s\r\n\r\n", USART_RX_BUF);
  // printf("----%d\r\n", _flag->valueint);
  // if (!recv_json) {
  // 	printf("abaaagbabababababababaab\r\n");
  // 	return;
  // }
  // else
  // 	printf("yeeeeeeeeeeeeeeeeah!!!!\r\n");
  // printf("p_x: %.2lf\r\n", p_x);
  flag = _flag->valueint;
  m1.pos.cur = 50;
  if (flag == 1)
  {
    // _t_x = cJSON_GetObjectItem(recv_json, "x");
    //printf("+++++%s\n",_t_x->valuestring);
    // t_x = _t_x->valuedouble;
    //???    // target_pos[cnt_tar][0] = t_x;
    // // _t_y = cJSON_GetObjectItem(recv_json, "y");
    // // t_y = _t_y->valuedouble;
    // target_pos[cnt_tar][1] = t_y;
    //???    cnt_tar++;
  }
  else
  {
    // printf("success point 1\r\n");
    // _p_x = cJSON_GetObjectItem(recv_json, "pos_x");
    // printf("+++++%.2lf\r\n", _p_x->valuedouble);
    // p_x = _p_x->valuedouble;
    // printf("p_x=%.2lf\xff\xff\xff\r\n", p_x);
    m1.speed.cur = p_x;
    // printf("p_x: %.2lf\r\n", p_x);
    // _p_y = cJSON_GetObjectItem(recv_json, "pos_y");
    // p_y = _p_y->valuedouble;
    m2.speed.cur = p_y;
    // _v_x = cJSON_GetObjectItem(recv_json, "v_x");
    // v_x = _v_x->valuedouble;
    m1.pos.cur = v_x;
    // _v_y = cJSON_GetObjectItem(recv_json, "v_y");
    // v_y = _v_y->valuedouble;
    m2.pos.cur = v_y;
  }
  cJSON_Delete(recv_json);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_UART_Receive_IT(&huart1, &m_Uart1_RcvByte, 1);
  HAL_UART_Receive_IT(&huart2, &m_Uart2_RcvByte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //pid init
  motor_init(&m1,
             0, 0, 0, 0,
             0, 0, 0, 0,
             50, 250, 0, 180, htim3, TIM_CHANNEL_3);
  motor_init(&m2,
             0, 0, 0, 0,
             0, 0, 0, 0,
             50, 250, 0, 180, htim3, TIM_CHANNEL_4);
  // int32_t Mode = 0;
  // printf("START!"); // print
  // int32_t CH1_DC = 0, CH2_DC = 0;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1550);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1450);
  while (1)
  {
    // for (int i = 0; i < 180; i++)
    // {
    //   motor_move_angle(&m1, 1.0 * i);
    //   HAL_Delay(30);
    // }
    // for (int i = 180; i >= 0; i--)
    // {
    //   motor_move_angle(&m1, 1.0 * i);
    //   HAL_Delay(30);
    // }
    // while (CH1_DC < 300)
    // {
    //   CH1_DC += 10;
    //   CH2_DC += 10;
    //   // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, CH1_DC + 1450);
    //   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, CH2_DC + 1450);
    //   HAL_Delay(10);
    // }
    // while (CH1_DC > 0)
    // {
    //   // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, CH1_DC + 1450);
    //   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, CH2_DC + 1450);
    //   CH1_DC -= 10;
    //   CH2_DC -= 10;
    //   HAL_Delay(10);
    // }
    // }
    // while (CH1_DC > 0)
    // {
    //   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, CH1_DC);
    //   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, CH2_DC);
    //   CH1_DC -= 10;
    //   CH2_DC -= 10;
    //   HAL_Delay(100);
    // }
    // HAL_UART_Transmit(&huart2, "LIVE\n", 2, 1000);
    int count = 0;
    printf("Hello SWV debugging prints...count = %d\n", count++);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(1000);
    if (USART1_RX_STA & 0x8000)
    {
      UART1_rxBuffer[MSG_LEN] = '\n';
      My_Jsonparse(UART1_rxBuffer);

      UART1_msg_ok = 0;
      // printf("px.val=%.2lf\xff\xff\xff\r\n", m1.speed.cur);
      // printf("py.val=%.2lf\xff\xff\xff\r\n", m2.speed.cur);
      // printf("vx.val=%.2lf\xff\xff\xff\r\n", m1.pos.cur);
      // printf("vy.val=%.2lf\xff\xff\xff\r\n", m2.pos.cur);
      // printf("tx.val=%.2lf\xff\xff\xff\r\n", m1.pos.input);
      // printf("ty.val=%.2lf\xff\xff\xff\r\n", m2.pos.input);
      // printf("testing2!"); // print
      // printf("\r\n");      // manual new line
      switch (Mode) //������Ŀѡ����
      {
      case 1:
        Mode_1();
        break;
      case 2:
        Mode_2();
        break;
      case 3:
        Mode_3();
        break;
      case 4:
        Mode_4();
        break;
      case 5:
        Mode_5();
        break;
      case 6:
        Mode_6();
        break;
      case 7:
        Mode_7();
        break;
      case 8:
        Mode_8();
        break;
      default:
        break;
      }
    }
    char end[] = {0xff, 0xff, 0xff}; //������ͨ��Э��
    uint16_t len;
    uint16_t t;
    uint16_t sel;
    uint16_t num;
    uint16_t i;
    uint32_t times;

    uint16_t len_uart;
    if (USART2_RX_STA & 0x8000)
    {
      len = USART2_RX_STA & 0x3fff; //�õ��˴ν��յ������ݳ���
      sel = USART2_RX_BUF[0];       //��ʶ��
      num = USART2_RX_BUF[2] + USART2_RX_BUF[3] * 256;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      if (sel == 0)
      {
        printf("n3.val=%d\xff\xff\xff", num);
        fflush(stdout);
      }
      else if (sel == 1)
      {
        printf("n4.val=%d\xff\xff\xff", num);
        fflush(stdout);
      }
      else if (sel == 2)
      {
        printf("n5.val=%d\xff\xff\xff", num);
        fflush(stdout);
      }

      else if (sel == 3)
      {
        // LED0 = !LED0;
        Mode = 1;
      }
      else if (sel == 4)
      {
        Mode = 2;
      }
      else if (sel == 5)
      {
        Mode = 3;
      }
      else if (sel == 6)
      {
        Mode = 4;
      }
      else if (sel == 7)
      {
        Mode = 5;
      }
      else if (sel == 8)
      {
        Mode = 6;
      }
      else if (sel == 9)
      {
        Mode = 7;
      }
      else if (sel == 'A')
      {
        Mode = 8;
      }
      //ģʽ��ѡ���ĸ�Ŀ���???????0~3=A~D
      else if (sel == 'B')
      {
        point_choose[3] = num % 10;
        num = num / 10;
        point_choose[2] = num % 10;
        num = num / 10;
        point_choose[1] = num % 10;
        num = num / 10;
        point_choose[0] = num % 10;
      }
      //һ������
      else if (sel == 'C')
      {
        Start = 1;
      }
      //һ��ֹͣ
      else if (sel == 'D')
      {
        Start = 0;
      }

      USART2_RX_STA = 0;
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 101;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("ERROR\n");

  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
