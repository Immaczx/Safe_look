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
#include <string.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t start=1;			//Bandera que permite indicar el estado en el que comienzan los controladores
uint32_t p;					//Almacena el estado del botón
uint32_t marca=0;
uint32_t event=1;
int my_password[4]={2,0,2,0};
int check_password[4]={0,0,0,0};
enum states2 {RELEASED,WAIT,DETECT} boton_state;	//Estados controlador boton
enum states {STATE0, STATE1, STATE2, STATE3} current_state;
enum events{PREST,PRESTto,NC} new_event;
char input;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void contr_password(int);
char controlador_keypad(void);
char read_keypad(void);
void lock_safe(); //lock the safe
void unlock_safe(); //unlock the safe
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PB4 --> Dig 1
// PA10 --> Dig 2
// PC4 --> Dig 3
// Define GPIO Pins 7-Segment Display
// PB5 --> A
// PB13 --> B
// PC2 --> C
// PH1 --> D
// PH0 --> E
// PB3 --> F
// PC3 --> G
// BusOut display7Seg(PC3, PA10, PH0, PH1, PC2, PB4, PA2);
GPIO_TypeDef * segs[7] =
{
		PB5_GPIO_Port, //A
		PB13_GPIO_Port, //B
		PC2_GPIO_Port, //C
		PH1_GPIO_Port, //D
		PH0_GPIO_Port, //E
		PB3_GPIO_Port, //F
		PC3_GPIO_Port, //G
};
uint16_t Pin[7] ={
		PB5_Pin, // PA2 --> A
		PB13_Pin, // PB4 --> B
		PC2_Pin, // PC2 --> C
		PH1_Pin, // PH1 --> D
		PH0_Pin, // PH0 --> E
		PB3_Pin, // PA10 --> F
		PC3_Pin // PC3 --> G
};
void segmentos(int n[]){
	for(int i = 0; i < 7; i++)
	{
			HAL_GPIO_WritePin(segs[i], Pin[i], n[i]);
}
};
// Define Logic Anode Common Display
//                        ABCDEFG
int anodeComun[17][7] = {
{0,0,0,0,0,0,1},      // 0
{1,0,0,1,1,1,1},      // 1
{0,0,1,0,0,1,0},      // 2
{0,0,0,1,0,1,0},      // 3
{1,0,0,1,1,0,0},      // 4
{0,1,0,1,0,0,0},      // 5
{0,1,0,0,0,0,0},      // 6
{0,0,0,1,1,1,1},      // 7
{0,0,0,0,0,0,0},      // 8
{0,0,0,1,1,0,0},      // 9
{0,0,0,0,1,0,0},      // A
{1,1,0,0,0,0,0},      // B
{1,1,1,0,0,1,0},      // C
{1,0,0,0,0,1,0},      // D
{0,1,1,0,0,0,0},      // E
{0,1,1,0,1,0,0},      // F
{0,0,1,1,0,0,0} };    // P
// Define Logic Catode Comun Display
// ABCDEFG
int catodeComun[17][7] = {
{1,1,1,1,1,1,0}, // 0
{0,1,1,0,0,0,0}, // 1
{1,1,0,1,1,0,1}, // 2
{1,1,1,1,0,0,1}, // 3
{0,1,1,0,0,1,1}, // 4
{1,0,1,1,0,1,1}, // 5
{1,0,1,1,1,1,1}, // 6
{1,1,1,0,0,0,0}, // 7
{1,1,1,1,1,1,1}, // 8
{1,1,1,0,0,1,1}, // 9
{1,1,1,0,1,1,1}, // A
{0,0,1,1,1,1,1}, // B
{0,0,0,1,1,0,1}, // C
{0,1,1,1,1,0,1}, // D
{1,0,0,1,1,1,1}, // E
{1,0,0,0,1,1,1}, // F
{1,1,0,0,1,1,1}}; // P


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  lock_safe(); //initialize the safe
  current_state = STATE0; //set the initial state
  my_password[0]=2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(PB4_GPIO_Port, PB4_Pin, 0);
	  HAL_GPIO_WritePin(PA10_GPIO_Port, PA10_Pin, 0);
	  HAL_GPIO_WritePin(PC4_GPIO_Port, PC4_Pin, 1);
	  // Wait 500 millisecond
	  if (HAL_GetTick()>marca){//Si ha pasado un milisegundo desde la ultima ejecucion
		  input = controlador_keypad();
		  if(input!='F')
		  {
		  contr_password(input);
		  }
		  marca=HAL_GetTick();  //Almacene el tiempo actual
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, PH0_Pin|PH1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC2_Pin|PC3_Pin|PC4_Pin|R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|R1_Pin|R2_Pin|PA10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PB13_Pin|PB3_Pin|PB4_Pin|PB5_Pin
                          |R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0_Pin PH1_Pin */
  GPIO_InitStruct.Pin = PH0_Pin|PH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2_Pin PC3_Pin PC4_Pin R4_Pin */
  GPIO_InitStruct.Pin = PC2_Pin|PC3_Pin|PC4_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin R1_Pin R2_Pin PA10_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|R1_Pin|R2_Pin|PA10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : C3_Pin */
  GPIO_InitStruct.Pin = C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(C3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13_Pin PB3_Pin PB4_Pin PB5_Pin
                           R3_Pin */
  GPIO_InitStruct.Pin = PB13_Pin|PB3_Pin|PB4_Pin|PB5_Pin
                          |R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C2_Pin C1_Pin */
  GPIO_InitStruct.Pin = C2_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

char read_keypad(void)
{
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);

	if(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))) // if the col 1 is low
	{
		//while(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)));  // wait till the button pressed
		return 1;
	}

	if(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))) // if the col 2 is low
	{
		//while(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)));  // wait till the button pressed
		return 2;
	}

	if(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))) // if the col 3 is low
	{
		//while(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)));  // wait till the button pressed
		return 3;
	}


	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);

	if(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))) // if the col 1 is low
	{
		//while(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)));  // wait till the button pressed
		return 4;
	}

	if(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))) // if the col 2 is low
	{
		//while(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)));  // wait till the button pressed
		return 5;
	}

	if(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))) // if the col 3 is low
	{
		//while(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)));  // wait till the button pressed
		return 6;
	}

	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);

	if(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))) // if the col 1 is low
	{
		//while(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)));  // wait till the button pressed
		return 7;
	}

	if(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))) // if the col 2 is low
	{
		//while(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)));  // wait till the button pressed
		return 8;
	}

	if(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))) // if the col 3 is low
	{
		//while(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)));  // wait till the button pressed
		return 9;
	}

	/* Make ROW 4 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);

	if(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))) // if the col 1 is low
	{
		//while(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)));  // wait till the button pressed
		return '#';
	}

	if(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))) // if the col 2 is low
	{
		//while(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)));  // wait till the button pressed
		return 0;
	}

	if(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))) // if the col 3 is low
	{
		//while(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))){};  // wait till the button pressed
		return '*';
	}

	return 'F';
}
uint32_t volatile antire=0;			//Contador que indica cuando se cumplio el antirrebote
int dato;
char controlador_keypad(void){
	if (start==1){			//Cuando la funcion se ejecuta por primera vez:
		boton_state=WAIT;	//Se pone el estado en OFF
		start=0;
	}

	input=read_keypad();	//Se lee el estado del boton
	if(input == 'F'){
		p=1;
	}
	else
	{
		p=0;
	}
		switch(boton_state){	//Cambio de estados boton
			case DETECT:		//Se detecto un touch
				switch (p){		//Si el boton esta presionado
				case(0):
					input='F';
					antire+=1;	//Aumente el contador en 1
					break;
				case (1):		//Si el boton no esta presionado
					if (antire>50){//Si el boton paso mas de 50 ms presionado (no es un rebote)
						boton_state=RELEASED;	//pase al estado liberado
					}else{boton_state=WAIT;}	//Si no pase al estado de espera
					break;
				}
				break;
			case RELEASED://El boton fue presionado y liberado
				event=0;	//Se envia el evento al controlador del led
				antire=0;	//Se reinicia la variable del antirrebote
				boton_state=WAIT;//Se pasa al estado de espera
				input=dato;
				break;
			case WAIT:		//Espera a detectar un touch
				switch (p){
				case(0)://Si el boton es presionado pase al estado detect
					dato=input;
					input='F';
					boton_state=DETECT;
					break;
				case (1)://Si no esta presionado no haga nada
					break;
				}
				break;
			default:

				break;

		}

		return input;//se entrega valor
}
static uint32_t con=0;
uint32_t t_step=0;
uint32_t to=5000;
void contr_password(int input){

		if (start==1){			//Cuando la funcion se ejecuta por primera vez:
			current_state = STATE0;	//Se pone el estado en OFF
			start=0;			//Y se cambia la bandera para no volver a ejecutar esto
		}

		#define MAX_STATES 4
		#define MAX_EVENTS 3
		typedef void (*transition)();
		void step0(){	//Accion de apagar el led
			current_state = STATE1;
			check_password[0]=input;
			segmentos(anodeComun[input]);
			t_step=HAL_GetTick();	//Guarda el tiempo actual
		}
		void step1(){	//Accion de apagar el led
			current_state = STATE2;
			check_password[1]=input;
			segmentos(anodeComun[input]);
			t_step=HAL_GetTick();	//Guarda el tiempo actual
		}
		void step2(){	//Accion de apagar el led
			current_state = STATE3;
			check_password[2]=input;
			segmentos(anodeComun[input]);
			t_step=HAL_GetTick();	//Guarda el tiempo actual
		}
		void step3(){//Accion de encender el led
			  check_password[3]=input;
			  segmentos(anodeComun[input]);
			  HAL_Delay(500);	        //Delay
			  if(!memcmp(check_password,my_password,sizeof(int)*4)){
				  unlock_safe();
				  segmentos(anodeComun[10]);
				  current_state= STATE0;
			  }
			  else{
				    lock_safe();
					segmentos(anodeComun[15]);
					current_state= STATE0;
			  }
		}
		void error(){
			segmentos(anodeComun[15]);
			current_state= STATE0;
		}

		transition state_table[MAX_STATES][MAX_EVENTS] = {
				{step0,step0,error},//almacenar
				{step1,step0,error},//almacenar
				{step2,step0,error},//almacenar
				{step3,step0,error}//verificar
		};
		if (event==0){	//Si se presiono el boton
			event=1;	//Limpie la bandera del boton
			con=HAL_GetTick();	//Lea el tiempo actual
			if(con<(t_step+to)){	//Si ha transcurrido menos de un tiempo limite entre
				  new_event=PREST;	//cada cambio de estado, presente el evento
			  }
			else{
				 new_event=PRESTto;
			 }
		}
		else{new_event=NC;}		//Nada cambio
		if ((new_event >= 0) && (new_event < MAX_EVENTS)	//El evento actual esta entre los eventos que se crearon
				&& (current_state >= 0) && (current_state < MAX_STATES)) {//El estado actual esta entre los eventos que se crearon
		/* call the transition function */
			state_table[current_state][new_event]();//Se ejecuta la tabla de estados usando el evento y estado actual
		}
		else {/* invalid event/state - handle appropriately */
	}
}

void lock_safe(void)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void unlock_safe(void)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
