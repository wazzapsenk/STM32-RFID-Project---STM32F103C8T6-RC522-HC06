/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stdio.h"
#include "stm32f1_rc522.h"
#include "stdio.h"
#include "string.h"
#include "lcd.h"
#include "delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t sayi=0;
char data[32]=" ";

#define	Precise_Delay(x) {\
		uint32_t x1 = x * 72;\
		DWT->CYCCNT = 0;\
		while (DWT->CYCCNT < x1);\


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//buton
uint8_t Key1;
uint8_t Key2;
uint8_t i;
uint8_t status;
uint8_t str[MAX_LEN]; // Max_LEN = 16


uint8_t serNum[5];
/*parolali islem icin kullanilacak*/
//char password[16]="123456"; //Max lenght of password is 16 charaters
//char keypass[16];
//int cnt=0;
/*parolali islem icin kullanilacak*/
uint8_t value = 0;
char str1[17]={'\0'};
char str2[17]={'\0'};
char str3[17]={'\0'};
char str4[17]={'\0'};
char tmp_str[65]={'\0'};     
//uint8_t  BLOCK_ADDRS[] =  {1}; //{1, 2, 3};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
	
	while(DWT_Delay_Init());
	lcd_init();
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	lcd_clear();
	//
	for (int i = 0; i < 16; i++){
		tmp_str[i] = 'A';
	}
	for (int i = 16; i < 32; i++){
		tmp_str[i] = 'B';
	}
	for (int i = 32; i < 48; i++){
		tmp_str[i] = 'C';
	}
	for (int i = 48; i < 64; i++){
		tmp_str[i] = 'D';
	}
	lcd_puts_long(tmp_str);
	DWT_Delay_ms(2000);
	lcd_clear();
	DWT_Delay_ms(1000);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(100);
	MFRC522_Init();
	HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t status, cardstr[MAX_LEN+1];
	uint8_t card_data[17];
	uint32_t delay_val = 1000; //ms
	uint16_t result = 0;
	uint8_t UID[5];
	
	// a private key to scramble data writing/reading to/from RFID card:
	uint8_t Mx1[7][5]={{0x12,0x45,0xF2,0xA8},{0xB2,0x6C,0x39,0x83},{0x55,0xE5,0xDA,0x18},{0x1F,0x09,0xCA,0x75},{0x99,0xA2,0x50,0xEC},{0x2C,0x88,0x7F,0x3D}};
	uint8_t SectorKey[7];
	//
		
		
	//status = Read_MFRC522(VersionReg);
		HAL_Delay(1000);
	sprintf(str1," RC522 CALISIYOR \n \r");
	HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),1000);
	HAL_Delay(1000);
	sprintf(str2,"BURAK KAAN SAHIN \n \r");
	HAL_UART_Transmit(&huart2,(uint8_t*)str2,strlen(str2),1000);
	HAL_Delay(1000);
	sprintf(str3,"201513152099 \n \r");
	HAL_UART_Transmit(&huart2,(uint8_t*)str3,strlen(str3),1000);
	HAL_Delay(1000);
	sprintf(str4,"ver: Bitirme Projesi \n \r");
	HAL_UART_Transmit(&huart2,(uint8_t*)str4,strlen(str4),1000);
	HAL_Delay(1000);
	
  while (1)
  {
		status = MFRC522_Request(PICC_REQIDL, str);	//MFRC522_Request(0x26, str)
		status = MFRC522_Anticoll(str);//Take a collision, look up 5 bytes
		memcpy(serNum, str, 5);//c dilinde karsiligi:(veri1:verilerin kaydedildigi konum ,veri2:verilerin kaynagi ,veri3:boyut)

				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET);
				HAL_Delay(4);
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET);
			  //LED1_GPIO_Port -> BSRR = LED1_Pin;
			  //LED1_GPIO_Port -> BRR = LED1_Pin;
			  for (int i = 0; i < 16; i++) {
				  cardstr[i] = 0;
			  }
				 status = 0;
		
			  status = MFRC522_Request(PICC_REQIDL, cardstr);
			  if(status == MI_OK) {
				  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_RESET);
				  result = 0;
				  result++;
				  sprintf(str1,"Card:%x,%x,%x \n \r", cardstr[0], cardstr[1], cardstr[2]);
					HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),1000);
					HAL_Delay(100);
				  //
				  // Anti-collision, return card serial number == 4 bytes
				  HAL_Delay(1);
				  status = MFRC522_Anticoll(cardstr);
				  if(status == MI_OK) {
					  result++;
					  sprintf(str2,"UID:%x %x %x %x \n \r", cardstr[0], cardstr[1], cardstr[2], cardstr[3]);
						HAL_UART_Transmit(&huart2,(uint8_t*)str2,strlen(str2),1000);
					HAL_Delay(100);
					  UID[0] = cardstr[0];
					  UID[1] = cardstr[1];
					  UID[2] = cardstr[2];
					  UID[3] = cardstr[3];
					  UID[4] = cardstr[4];
					  //
					  HAL_Delay(1);
					  status = MFRC522_SelectTag(cardstr);
					  if (status > 0){
						  result++;
						  //
						  SectorKey[0] = ((Mx1[0][0])^(UID[0])) + ((Mx1[0][1])^(UID[1])) + ((Mx1[0][2])^(UID[2])) + ((Mx1[0][3])^(UID[3]));// 0x11; //KeyA[0]
						  SectorKey[1] = ((Mx1[1][0])^(UID[0])) + ((Mx1[1][1])^(UID[1])) + ((Mx1[1][2])^(UID[2])) + ((Mx1[1][3])^(UID[3]));// 0x11; //KeyA[0]
						  SectorKey[2] = ((Mx1[2][0])^(UID[0])) + ((Mx1[2][1])^(UID[1])) + ((Mx1[2][2])^(UID[2])) + ((Mx1[2][3])^(UID[3]));// 0x11; //KeyA[0]
						  SectorKey[3] = ((Mx1[3][0])^(UID[0])) + ((Mx1[3][1])^(UID[1])) + ((Mx1[3][2])^(UID[2])) + ((Mx1[3][3])^(UID[3]));// 0x11; //KeyA[0]
						  SectorKey[4] = ((Mx1[4][0])^(UID[0])) + ((Mx1[4][1])^(UID[1])) + ((Mx1[4][2])^(UID[2])) + ((Mx1[4][3])^(UID[3]));// 0x11; //KeyA[0]
						  SectorKey[5] = ((Mx1[5][0])^(UID[0])) + ((Mx1[5][1])^(UID[1])) + ((Mx1[5][2])^(UID[2])) + ((Mx1[5][3])^(UID[3]));// 0x11; //KeyA[0]
						  HAL_Delay(1);
						  status = MFRC522_Auth(0x60, 3, SectorKey, cardstr);
						  if (status == MI_OK){
							  result++;
							  sprintf(str3, "Auth. OK \n \r");
								HAL_UART_Transmit(&huart2,(uint8_t*)str3,strlen(str3),1000);
								HAL_Delay(100);
							  if (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == 0){
								  // Clean-Up the Card:
									
								  card_data[0] = 0xFF;
								  card_data[1] = 0xFF;
								  card_data[2] = 0xFF;
								  card_data[3] = 0xFF;
								  card_data[4] = 0xFF;
								  card_data[5] = 0xFF;
								  card_data[6] = 0xFF; //Access_bits[6]
								  card_data[7] = 0x07; //Access_bits[7]
								  card_data[8] = 0x80; //Access_bits[8]
								  card_data[9] = 0x88; //user_byte[9]
								  card_data[10] = 0x88; //user_byte[10]
								  card_data[11] = 0x88; //user_byte[11]
								  card_data[12] = 0x88; //user_byte[12]
								  card_data[13] = 0x88; //user_byte[13]
								  card_data[14] = 0x88; //user_byte[14]
								  card_data[15] = 0x88; //user_byte[15]
								  HAL_Delay(1);
								  status = MFRC522_Write(3, card_data);
								  if(status == MI_OK) {
									  result++;
									  sprintf(str1, "UID:%x %x %x %x \n \r", cardstr[0], cardstr[1], cardstr[2], cardstr[3]);
										HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),1000);
										HAL_Delay(100);
									  sprintf(str4, "KART SILINDI! \n \r");
										HAL_UART_Transmit(&huart2,(uint8_t*)str4,strlen(str4),1000);
								    HAL_Delay(1);
									  delay_val = 2000;
								  }

							  }
						  }
						  else{
							  for (int i = 0; i < 16; i++) {cardstr[i] = 0;}
							  status = 0;
							  // Find cards
							  HAL_Delay(1);
							  status = MFRC522_Request(PICC_REQIDL, cardstr);
							  HAL_Delay(1);
							  status = MFRC522_Anticoll(cardstr);
							  HAL_Delay(1);
							  status = MFRC522_SelectTag(cardstr);
							  SectorKey[0] = 0xFF;
							  SectorKey[1] = 0xFF;
							  SectorKey[2] = 0xFF;
							  SectorKey[3] = 0xFF;
							  SectorKey[4] = 0xFF;
							  SectorKey[5] = 0xFF;
							  HAL_Delay(1);
							  status = MFRC522_Auth(0x60, 3, SectorKey, cardstr);
							  if (status == MI_OK){
								  if (HAL_GPIO_ReadPin(Key1_GPIO_Port, Key1_Pin) == 0){
									  card_data[0] = ((Mx1[0][0])^(UID[0])) + ((Mx1[0][1])^(UID[1])) + ((Mx1[0][2])^(UID[2])) + ((Mx1[0][3])^(UID[3]));// 0x11; //KeyA[0]
									  card_data[1] = ((Mx1[1][0])^(UID[0])) + ((Mx1[1][1])^(UID[1])) + ((Mx1[1][2])^(UID[2])) + ((Mx1[1][3])^(UID[3]));// 0x11; //KeyA[0]
									  card_data[2] = ((Mx1[2][0])^(UID[0])) + ((Mx1[2][1])^(UID[1])) + ((Mx1[2][2])^(UID[2])) + ((Mx1[2][3])^(UID[3]));// 0x11; //KeyA[0]
									  card_data[3] = ((Mx1[3][0])^(UID[0])) + ((Mx1[3][1])^(UID[1])) + ((Mx1[3][2])^(UID[2])) + ((Mx1[3][3])^(UID[3]));// 0x11; //KeyA[0]
									  card_data[4] = ((Mx1[4][0])^(UID[0])) + ((Mx1[4][1])^(UID[1])) + ((Mx1[4][2])^(UID[2])) + ((Mx1[4][3])^(UID[3]));// 0x11; //KeyA[0]
									  card_data[5] = ((Mx1[5][0])^(UID[0])) + ((Mx1[5][1])^(UID[1])) + ((Mx1[5][2])^(UID[2])) + ((Mx1[5][3])^(UID[3]));// 0x11; //KeyA[0]
									  card_data[6] = 0xFF; //Access_bits[6]
									  card_data[7] = 0x07; //Access_bits[7]
									  card_data[8] = 0x80; //Access_bits[8]
									  card_data[9] = 0x88; //user_byte[9]
									  card_data[10] = 0x88; //user_byte[10]
									  card_data[11] = 0x88; //user_byte[11]
									  card_data[12] = 0x88; //user_byte[12]
									  card_data[13] = 0x88; //user_byte[13]
									  card_data[14] = 0x88; //user_byte[14]
									  card_data[15] = 0x88; //user_byte[15]
									  HAL_Delay(1);
									  status = MFRC522_Write(3, card_data);
									  if(status == MI_OK) {
										  result++;
										  sprintf(str3, "KART TANIMLANDI! \n \r");
											HAL_UART_Transmit(&huart2,(uint8_t*)str3,strlen(str3),1000);
											HAL_Delay(1);
										  delay_val = 2000;
									  }
								  }
								  else{
										
									  sprintf(str4, "YENI KART! \n \r");
										HAL_UART_Transmit(&huart2,(uint8_t*)str4,strlen(str4),1000);
								    HAL_Delay(1);
								  }
							  }
							  else if (status != MI_OK){
								  sprintf(str1, "Auth. Error \n \r");
									HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),1000);
									HAL_Delay(1);

							  }
						  }
						  HAL_Delay(1);
						  MFRC522_StopCrypto1();
					  }
				  }
					HAL_Delay(1);
				  MFRC522_Halt();
				  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET);
				  

				  DWT_Delay_ms(delay_val);
				  delay_val = 1000;
				  sprintf(str1, "BEKLEYIN \n \r           ");
					HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),1000);
					HAL_Delay(1);
				  sprintf(str2, "  BEKLEYIN \n \r         ");
					HAL_UART_Transmit(&huart2,(uint8_t*)str2,strlen(str2),1000);
					HAL_Delay(1);
					sprintf(str3, "    BEKLEYIN \n \r       ");
					HAL_UART_Transmit(&huart2,(uint8_t*)str3,strlen(str3),1000);
					HAL_Delay(1);
				  sprintf(str4, "      TAMAMLANDI \n \r     ");
					HAL_UART_Transmit(&huart2,(uint8_t*)str4,strlen(str4),1000);
					HAL_Delay(1);

			  }
			  else{
				  sprintf(str1, "    waiting for Card \n \r     ");
					HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),1000);
					HAL_Delay(1000);
					sprintf(str3, "    KARTI OKUTUN \n \r     ");
					HAL_UART_Transmit(&huart2,(uint8_t*)str3,strlen(str3),1000);
					HAL_Delay(10);


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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Key2_Pin Key1_Pin */
  GPIO_InitStruct.Pin = Key2_Pin|Key1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

