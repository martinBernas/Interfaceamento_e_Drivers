/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

osThreadId tsk_main_gameHandle;
osThreadId tsk_p1_buttonsHandle;
osThreadId tsk_p2_buttonsHandle;
osThreadId tsk_p1_jogadorHandle;
osThreadId tsk_p2_jogadorHandle;
osThreadId tsk_displayHandle;
osMutexId mtxBotoesP1Handle;
osMutexId mtxBotoesP2Handle;
osMutexId mtxStatusHandle;
osMutexId mtxPalpiteP1Handle;
osMutexId mtxPalpiteP2Handle;
osMutexId mtxSenhaP1Handle;
osMutexId mtxSenhaP2Handle;
osMutexId mtxApostaP1Handle;
osMutexId mtxApostaP2Handle;
osMutexId mtxRespostaP1Handle;
osMutexId mtxRespostaP2Handle;
/* USER CODE BEGIN PV */
int flag_p1_up = 0;
int flag_p1_select = 0;
int flag_p1_accept = 0;

int flag_p2_up = 0;
int flag_p2_select = 0;
int flag_p2_accept = 0;

int status_jogo = 0;

char senha_jogador1[4] = "0000";
char senha_jogador2[4] = "0000";
char aposta_jogador1[4] = "0000";
char aposta_jogador2[4] = "0000";
char palpite_jogador1[4] = "0000";
char palpite_jogador2[4] = "0000";

char resposta_jogador1[4] = "0000";
char resposta_jogador2[4] = "0000";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void main_game(void const * argument);
void buttons_p1(void const * argument);
void p2_buttons(void const * argument);
void p1_jogador(void const * argument);
void p2_jogador(void const * argument);
void display(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
LCD_Init();



  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of mtxBotoesP1 */
  osMutexDef(mtxBotoesP1);
  mtxBotoesP1Handle = osMutexCreate(osMutex(mtxBotoesP1));

  /* definition and creation of mtxBotoesP2 */
  osMutexDef(mtxBotoesP2);
  mtxBotoesP2Handle = osMutexCreate(osMutex(mtxBotoesP2));

  /* definition and creation of mtxStatus */
  osMutexDef(mtxStatus);
  mtxStatusHandle = osMutexCreate(osMutex(mtxStatus));

  /* definition and creation of mtxPalpiteP1 */
  osMutexDef(mtxPalpiteP1);
  mtxPalpiteP1Handle = osMutexCreate(osMutex(mtxPalpiteP1));

  /* definition and creation of mtxPalpiteP2 */
  osMutexDef(mtxPalpiteP2);
  mtxPalpiteP2Handle = osMutexCreate(osMutex(mtxPalpiteP2));

  /* definition and creation of mtxSenhaP1 */
  osMutexDef(mtxSenhaP1);
  mtxSenhaP1Handle = osMutexCreate(osMutex(mtxSenhaP1));

  /* definition and creation of mtxSenhaP2 */
  osMutexDef(mtxSenhaP2);
  mtxSenhaP2Handle = osMutexCreate(osMutex(mtxSenhaP2));

  /* definition and creation of mtxApostaP1 */
  osMutexDef(mtxApostaP1);
  mtxApostaP1Handle = osMutexCreate(osMutex(mtxApostaP1));

  /* definition and creation of mtxApostaP2 */
  osMutexDef(mtxApostaP2);
  mtxApostaP2Handle = osMutexCreate(osMutex(mtxApostaP2));

  /* definition and creation of mtxRespostaP1 */
  osMutexDef(mtxRespostaP1);
  mtxRespostaP1Handle = osMutexCreate(osMutex(mtxRespostaP1));

  /* definition and creation of mtxRespostaP2 */
  osMutexDef(mtxRespostaP2);
  mtxRespostaP2Handle = osMutexCreate(osMutex(mtxRespostaP2));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of tsk_main_game */
  osThreadDef(tsk_main_game, main_game, osPriorityAboveNormal, 0, 128);
  tsk_main_gameHandle = osThreadCreate(osThread(tsk_main_game), NULL);

  /* definition and creation of tsk_p1_buttons */
  osThreadDef(tsk_p1_buttons, buttons_p1, osPriorityLow, 0, 128);
  tsk_p1_buttonsHandle = osThreadCreate(osThread(tsk_p1_buttons), NULL);

  /* definition and creation of tsk_p2_buttons */
  osThreadDef(tsk_p2_buttons, p2_buttons, osPriorityLow, 0, 128);
  tsk_p2_buttonsHandle = osThreadCreate(osThread(tsk_p2_buttons), NULL);

  /* definition and creation of tsk_p1_jogador */
  osThreadDef(tsk_p1_jogador, p1_jogador, osPriorityNormal, 0, 128);
  tsk_p1_jogadorHandle = osThreadCreate(osThread(tsk_p1_jogador), NULL);

  /* definition and creation of tsk_p2_jogador */
  osThreadDef(tsk_p2_jogador, p2_jogador, osPriorityNormal, 0, 128);
  tsk_p2_jogadorHandle = osThreadCreate(osThread(tsk_p2_jogador), NULL);

  /* definition and creation of tsk_display */
  osThreadDef(tsk_display, display, osPriorityBelowNormal, 0, 128);
  tsk_displayHandle = osThreadCreate(osThread(tsk_display), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LCD_2_GPIO_Port, LCD_2_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  LCD_Write_String(0,0,"Jogador 1");


	  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LCD_2_GPIO_Port, LCD_2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  LCD_Write_String(0,0,"Jogador 2");
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin|LCD_2_Pin|LCD_CE_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RST_Pin LCD_2_Pin LCD_CE_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_2_Pin|LCD_CE_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : btn_p1_up_Pin btn_p1_select_Pin btn_p1_accept_Pin btn_p2_up_Pin
                           btn_p2_select_Pin btn_p2_accept_Pin */
  GPIO_InitStruct.Pin = btn_p1_up_Pin|btn_p1_select_Pin|btn_p1_accept_Pin|btn_p2_up_Pin
                          |btn_p2_select_Pin|btn_p2_accept_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_main_game */
/**
  * @brief  Function implementing the tsk_main_game thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_main_game */
void main_game(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_buttons_p1 */
/**
* @brief Function implementing the tsk_p1_buttons thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buttons_p1 */
void buttons_p1(void const * argument)
{
  /* USER CODE BEGIN buttons_p1 */
  /* Infinite loop */

	int estado_atual_up 		 = HAL_GPIO_ReadPin(btn_p1_up_GPIO_Port, btn_p1_up_Pin);
	int estado_atual_select 	 = HAL_GPIO_ReadPin(btn_p1_select_GPIO_Port, btn_p1_select_Pin);
	int estado_atual_accept 	 = HAL_GPIO_ReadPin(btn_p1_accept_GPIO_Port, btn_p1_accept_Pin);
	int estado_anterior_up     = estado_atual_up;
	int estado_anterior_select = estado_atual_select;
	int estado_anterior_accept  = estado_atual_accept;
	for(;;)
	{
	  int acionou_up = 0;
	  int acionou_select = 0;
	  int acionou_accept = 0;

	  estado_atual_up 		 = HAL_GPIO_ReadPin(btn_p1_up_GPIO_Port, btn_p1_up_Pin);
	  estado_atual_select 	 = HAL_GPIO_ReadPin(btn_p1_select_GPIO_Port, btn_p1_select_Pin);
	  estado_atual_accept 	 = HAL_GPIO_ReadPin(btn_p1_accept_GPIO_Port, btn_p1_accept_Pin);

	  if(estado_atual_up != estado_anterior_up && estado_atual_up == 0){

		  acionou_up = 1;

	  }
	  if(estado_atual_select != estado_anterior_select && estado_atual_select == 0){

	  	  acionou_select = 1;

	  }
	  if(estado_atual_accept != estado_anterior_accept && estado_atual_accept == 0){

	  	  acionou_accept = 1;

	  }
	  estado_anterior_up     = estado_atual_up;
	  estado_anterior_select = estado_atual_select;
	  estado_anterior_accept  = estado_atual_accept;

	  osMutexWait(mtxBotoesP1Handle, 1000);
	  if(acionou_up > 0){
		  flag_p1_up = acionou_up;
	  }
	  if(acionou_select > 0){
		  flag_p1_select = acionou_up;
	  }
	  if(acionou_accept > 0){
		  flag_p1_accept = acionou_accept;
	  }
	  osMutexRelease(mtxBotoesP1Handle);
	  osDelay(100);
	}
  /* USER CODE END buttons_p1 */
}

/* USER CODE BEGIN Header_p2_buttons */
/**
* @brief Function implementing the tsk_p2_buttons thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_p2_buttons */
void p2_buttons(void const * argument)
{
  /* USER CODE BEGIN p2_buttons */
  /* Infinite loop */
	int estado_atual_up 		 = HAL_GPIO_ReadPin(btn_p2_up_GPIO_Port, btn_p2_up_Pin);
	int estado_atual_select 	 = HAL_GPIO_ReadPin(btn_p2_select_GPIO_Port, btn_p2_select_Pin);
	int estado_atual_accept 	 = HAL_GPIO_ReadPin(btn_p2_accept_GPIO_Port, btn_p2_accept_Pin);
	int estado_anterior_up     = estado_atual_up;
	int estado_anterior_select = estado_atual_select;
	int estado_anterior_accept  = estado_atual_accept;

	for(;;)
	{
	  int acionou_up = 0;
	  int acionou_select = 0;
	  int acionou_accept = 0;

	  estado_atual_up 		 = HAL_GPIO_ReadPin(btn_p2_up_GPIO_Port, btn_p2_up_Pin);
	  estado_atual_select 	 = HAL_GPIO_ReadPin(btn_p2_select_GPIO_Port, btn_p2_select_Pin);
	  estado_atual_accept 	 = HAL_GPIO_ReadPin(btn_p2_accept_GPIO_Port, btn_p2_accept_Pin);

	  if(estado_atual_up != estado_anterior_up && estado_atual_up == 0){
		  acionou_up = 1;
	  }
	  if(estado_atual_select != estado_anterior_select && estado_atual_select == 0){
	  	  acionou_select = 1;
	  }
	  if(estado_atual_accept != estado_anterior_accept && estado_atual_accept == 0){
	  	  acionou_accept = 1;
	  }
	  estado_anterior_up     = estado_atual_up;
	  estado_anterior_select = estado_atual_select;
	  estado_anterior_accept  = estado_atual_accept;

	  osMutexWait(mtxBotoesP2Handle, 1000);
	  if(acionou_up > 0){
		  flag_p2_up = acionou_up;
	  }
	  if(acionou_select > 0){
		flag_p2_select = acionou_up;
	  }
	  if(acionou_accept > 0){
		  flag_p2_accept = acionou_accept;
	  }
	  osMutexRelease(mtxBotoesP2Handle);

	  osDelay(100);
	}
  /* USER CODE END p2_buttons */
}

/* USER CODE BEGIN Header_p1_jogador */
/**
* @brief Function implementing the tsk_p1_jogador thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_p1_jogador */
void p1_jogador(void const * argument)
{
  /* USER CODE BEGIN p1_jogador */
  /* Infinite loop */
	int acionou_up = 0;
	int acionou_select = 0;
	int acionou_accept = 0;
	int local_status_jogo = 0;
	int posicao_tela = 0;
	char local_senha[4] ;
	char local_palpite[4];
  for(;;)
  {
	osMutexWait(mtxBotoesP1Handle, 1000);
	acionou_up = flag_p1_up;
	acionou_select = flag_p1_select;
	acionou_accept = flag_p2_accept;
	osMutexRelease(mtxBotoesP1Handle);

	osMutexWait(mtxStatusHandle, 1000);
	local_status_jogo = status_jogo;
	osMutexRelease(mtxStatusHandle);

	osMutexWait(mtxPalpiteP1Handle,1000);
	palpite_jogador1[0] = local_palpite[0];
	palpite_jogador1[1] = local_palpite[1];
	palpite_jogador1[2] = local_palpite[2];
	palpite_jogador1[3] = local_palpite[3];
	osMutexRelease(mtxPalpiteP1Handle);

	if(local_status_jogo == 0){
		if(acionou_select > 0){
			if(posicao_tela >= 3){
				posicao_tela = 0;
			}else{
				posicao_tela ++;
			}
			osMutexWait(mtxBotoesP1Handle, 1000);
			flag_p1_select = 0;
			osMutexRelease(mtxBotoesP1Handle);
			acionou_select = 0;
		}
		if(acionou_up > 0 ){

			if( local_palpite[posicao_tela] >= 9){
				local_palpite[posicao_tela] = 0;
			}else{
				local_palpite[posicao_tela] ++;
			}
			osMutexWait(mtxBotoesP1Handle, 1000);
			flag_p1_up = 0;
			osMutexRelease(mtxBotoesP1Handle);
			acionou_up = 0;
		}
		if(acionou_accept > 0 ){
			osMutexWait(mtxSenhaP1Handle, 1000);
			senha_jogador1[0] = local_palpite[0];
			senha_jogador1[1] = local_palpite[1];
			senha_jogador1[2] = local_palpite[2];
			senha_jogador1[3] = local_palpite[3];
			osMutexRelease(mtxSenhaP1Handle);
			osMutexWait(mtxBotoesP1Handle, 1000);
			flag_p1_accept = 0;
			osMutexRelease(mtxBotoesP1Handle);
			acionou_accept = 0;
		}
	}else if(local_status_jogo == 1 || local_status_jogo == 2){
		if(acionou_select > 0){
					if(posicao_tela >= 3){
						posicao_tela = 0;
					}else{
						posicao_tela ++;
					}
					osMutexWait(mtxBotoesP1Handle, 1000);
					flag_p1_select = 0;
					osMutexRelease(mtxBotoesP1Handle);
					acionou_select = 0;
				}
				if(acionou_up > 0 ){

					if( local_palpite[posicao_tela] >= 9){
						local_palpite[posicao_tela] = 0;
					}else{
						local_palpite[posicao_tela] ++;
					}
					osMutexWait(mtxBotoesP1Handle, 1000);
					flag_p1_up = 0;
					osMutexRelease(mtxBotoesP1Handle);
					acionou_up = 0;
				}
				if(acionou_accept > 0 ){
					if(local_status_jogo == 1){
						osMutexWait(mtxApostaP1Handle, 1000);
						aposta_jogador1[0] = local_palpite[0];
						aposta_jogador1[1] = local_palpite[1];
						aposta_jogador1[2] = local_palpite[2];
						aposta_jogador1[3] = local_palpite[3];
						osMutexRelease(mtxApostaP1Handle);
					}
					osMutexWait(mtxBotoesP1Handle, 1000);
					flag_p1_accept = 0;
					osMutexRelease(mtxBotoesP1Handle);
					acionou_accept = 0;
				}

	}

    osDelay(1);
  }
  /* USER CODE END p1_jogador */
}

/* USER CODE BEGIN Header_p2_jogador */
/**
* @brief Function implementing the tsk_p2_jogador thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_p2_jogador */
void p2_jogador(void const * argument)
{
  /* USER CODE BEGIN p2_jogador */
  /* Infinite loop */
	int acionou_up = 0;
	int acionou_select = 0;
	int acionou_accept = 0;
	int local_status_jogo = 0;
	int posicao_tela = 0;
	char local_palpite[4];
  for(;;)
  {
	osMutexWait(mtxBotoesP2Handle, 1000);
	acionou_up = flag_p2_up;
	acionou_select = flag_p2_select;
	acionou_accept = flag_p2_accept;
	osMutexRelease(mtxBotoesP2Handle);

	osMutexWait(mtxStatusHandle, 1000);
	local_status_jogo = status_jogo;
	osMutexRelease(mtxStatusHandle);

	osMutexWait(mtxPalpiteP2Handle,1000);
	palpite_jogador2[0] = local_palpite[0];
	palpite_jogador2[1] = local_palpite[1];
	palpite_jogador2[2] = local_palpite[2];
	palpite_jogador2[3] = local_palpite[3];
	osMutexRelease(mtxPalpiteP2Handle);

	if(local_status_jogo == 0){
		if(acionou_select > 0){
			if(posicao_tela >= 3){
				posicao_tela = 0;
			}else{
				posicao_tela ++;
			}
			osMutexWait(mtxBotoesP2Handle, 1000);
			flag_p1_select = 0;
			osMutexRelease(mtxBotoesP2Handle);
			acionou_select = 0;
		}
		if(acionou_up > 0 ){

			if( local_palpite[posicao_tela] >= 9){
				local_palpite[posicao_tela] = 0;
			}else{
				local_palpite[posicao_tela] ++;
			}
			osMutexWait(mtxBotoesP2Handle, 1000);
			flag_p2_up = 0;
			osMutexRelease(mtxBotoesP2Handle);
			acionou_up = 0;
		}
		if(acionou_accept > 0 ){
			osMutexWait(mtxSenhaP2Handle, 1000);
			senha_jogador2[0] = local_palpite[0];
			senha_jogador2[1] = local_palpite[1];
			senha_jogador2[2] = local_palpite[2];
			senha_jogador2[3] = local_palpite[3];
			osMutexRelease(mtxSenhaP2Handle);
			osMutexWait(mtxBotoesP2Handle, 1000);
			flag_p2_accept = 0;
			osMutexRelease(mtxBotoesP2Handle);
			acionou_accept = 0;
		}
	}else if(local_status_jogo == 1 || local_status_jogo == 2){
		if(acionou_select > 0){
					if(posicao_tela >= 3){
						posicao_tela = 0;
					}else{
						posicao_tela ++;
					}
					osMutexWait(mtxBotoesP2Handle, 1000);
					flag_p1_select = 0;
					osMutexRelease(mtxBotoesP1Handle);
					acionou_select = 0;
				}
				if(acionou_up > 0 ){

					if( local_palpite[posicao_tela] >= 9){
						local_palpite[posicao_tela] = 0;
					}else{
						local_palpite[posicao_tela] ++;
					}
					osMutexWait(mtxBotoesP2Handle, 1000);
					flag_p2_up = 0;
					osMutexRelease(mtxBotoesP2Handle);
					acionou_up = 0;
				}
				if(acionou_accept > 0 ){
					if(local_status_jogo == 2){
						osMutexWait(mtxApostaP2Handle, 1000);
						aposta_jogador2[0] = local_palpite[0];
						aposta_jogador2[1] = local_palpite[1];
						aposta_jogador2[2] = local_palpite[2];
						aposta_jogador2[3] = local_palpite[3];
						osMutexRelease(mtxApostaP2Handle);
					}
					osMutexWait(mtxBotoesP2Handle, 1000);
					flag_p1_accept = 0;
					osMutexRelease(mtxBotoesP2Handle);
					acionou_accept = 0;
				}

	}

    osDelay(1);
  }
  /* USER CODE END p2_jogador */
}

/* USER CODE BEGIN Header_display */
/**
* @brief Function implementing the tsk_display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_display */
void display(void const * argument)
{
  /* USER CODE BEGIN display */
  /* Infinite loop */
	char local_palpiteP1[4];
	char local_palpiteP2[4];
	char local_SenhaP1[4];
	char local_SenhaP2[4];
	char local_ApostaP1[4];
	char local_ApostaP2[4];
	char local_RespostaP1[4];
	char local_RespostaP2[4];
  for(;;)
  {
	  osMutexWait(mtxPalpiteP1Handle,1000);
	  local_palpiteP1[0] = palpite_jogador1[0];
	  local_palpiteP1[1] = palpite_jogador1[1];
	  local_palpiteP1[2] = palpite_jogador1[2];
	  local_palpiteP1[3] = palpite_jogador1[3];
	  osMutexRelease(mtxPalpiteP1Handle);
	  osMutexWait(mtxPalpiteP2Handle,1000);
	  local_palpiteP2[0] = palpite_jogador2[0];
	  local_palpiteP2[1] = palpite_jogador2[1];
	  local_palpiteP2[2] = palpite_jogador2[2];
	  local_palpiteP2[3] = palpite_jogador2[3];
	  osMutexRelease(mtxPalpiteP2Handle);

	  osMutexWait(mtxSenhaP1Handle,1000);
	  local_SenhaP1[0] = senha_jogador1[0];
	  local_SenhaP1[1] = senha_jogador1[1];
	  local_SenhaP1[2] = senha_jogador1[2];
	  local_SenhaP1[3] = senha_jogador1[3];
	  osMutexRelease(mtxSenhaP1Handle);

	  osMutexWait(mtxSenhaP2Handle,1000);
	  local_SenhaP2[0] = senha_jogador2[0];
	  local_SenhaP2[1] = senha_jogador2[1];
	  local_SenhaP2[2] = senha_jogador2[2];
	  local_SenhaP2[3] = senha_jogador2[3];
	  osMutexRelease(mtxSenhaP2Handle);

	  osMutexWait(mtxApostaP1Handle,1000);
	  local_ApostaP1[0] = aposta_jogador1[0];
	  local_ApostaP1[1] = aposta_jogador1[1];
	  local_ApostaP1[2] = aposta_jogador1[2];
	  local_ApostaP1[3] = aposta_jogador1[3];
	  osMutexRelease(mtxApostaP1Handle);

	  osMutexWait(mtxApostaP2Handle,1000);
	  local_ApostaP2[0] = aposta_jogador2[0];
	  local_ApostaP2[1] = aposta_jogador2[1];
	  local_ApostaP2[2] = aposta_jogador2[2];
	  local_ApostaP2[3] = aposta_jogador2[3];
	  osMutexRelease(mtxApostaP2Handle);

	  osMutexWait(mtxRespostaP1Handle,1000);
	  local_RespostaP1[0] = resposta_jogador1[0];
	  local_RespostaP1[1] = resposta_jogador1[1];
	  local_RespostaP1[2] = resposta_jogador1[2];
	  local_RespostaP1[3] = resposta_jogador1[3];
	  osMutexRelease(mtxRespostaP1Handle);

	  osMutexWait(mtxRespostaP2Handle,1000);
	  local_RespostaP2[0] = resposta_jogador2[0];
	  local_RespostaP2[1] = resposta_jogador2[1];
	  local_RespostaP2[2] = resposta_jogador2[2];
	  local_RespostaP2[3] = resposta_jogador2[3];
	  osMutexRelease(mtxRespostaP2Handle);



	  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LCD_2_GPIO_Port, LCD_2_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  LCD_Write_String(0,0,"Jogador 1");


	 	  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_SET);
	 	  HAL_GPIO_WritePin(LCD_2_GPIO_Port, LCD_2_Pin, GPIO_PIN_RESET);
	 	  HAL_Delay(100);
	 	  LCD_Write_String(0,0,"Jogador 2");
  }
  /* USER CODE END display */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
