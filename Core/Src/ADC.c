/*
 *      Conversor analógico digital - ADC.c
 *
 *      Data: 13 de junho, 2023
 *      Autor: Gabriel Luiz
 *      Contato: (31) 97136-4334 || gabrielluiz.eletro@gmail.com
 */
/*
 * 		- Links Úteis -
 *
 *      DATASHEET: https://www.st.com/resource/en/product_training/STM32H7-Analog-ADC_ADC.pdf
 *      Sobre o ADC: https://www.st.com/resource/en/application_note/an5354-getting-started-with-the-stm32h7-series-mcu-16bit-adc-stmicroelectronics.pdf
 *      ADC single channel: https://www.youtube.com/watch?v=TXRdQ1UvFZc
 *      ADC multi channel: https://www.youtube.com/watch?v=5l-b6lsubBE&t=123s
 */

#include "ADC.h"

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_VALUE_CHANNEL_1 0 //	PRIMEIRO CANAL DE CONVERSÃO ADC

#define ADC_VALUE_CHANNEL_2 1 //	SEGUNDO CANAL DE CONVERSÃO ADC

#define ADC_VALUE_CHANNEL_3 2 //	TERCEIRO CANAL DE CONVERSÃO ADC

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

extern ADC_HandleTypeDef hadc1; /* Variável externa de configuração do ADC */

extern TIM_HandleTypeDef htim1; /* Variável externa de configuração do TIMER */

/* USER CODE END EV */

/* External functions ------------------------------------------------------------*/
/* USER CODE BEGIN EF */

extern void Error_Handler(); /* Função utilizada para tratamento de erros */

/* USER CODE END EF */

/* Private variables --------------------------------------------------------*/
/* USER CODE BEGIN PV */

uint32_t ADC_value[3]; /*Vetor para armazenamento de
 todos os dados do ADC*/

/* USER CODE END PV */

/* Private functions ------------------------------------------------------------*/
/* USER CODE BEGIN PF */

/*
 * @brief  Função chamada periodicamente para medir a(s) entrada(s) analogica(s)
 * @param  htim: Handle do TIMER || normalmente "htim1"
 * @retval ***NONE***
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* Inicia a converção, e armazena na memória logo após terminar a conversão*/
	HAL_ADC_Start_DMA(&hadc1, ADC_value, 3);
}

/**
 * @brief  Inicialização do timer necessário para leitura do ADC
 * @param  ***NONE***
 * @retval ***NONE***
 */
void TIM_Configure_Init(void) {
	/* Variável para configuração do tipo de clock do timer */
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	/* Variável para configuração do modo de trigger e dominância */
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* Configura os parametros do TIMER - LEITURA DO DATASHEET */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 10000;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	/* Inicializa o TIMER com os parâmetros definidos */
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
	/* Define o tipo de clock */
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	/* Inicializa o clock do TIMER */
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}

	/* Configura o modo de trigger e dominâcia */
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE; //CONFIGURAÇÃO DO MODO DO 1°TRIGGER
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET; //CONFIGURAÇÃO DO MODO DO 2°TRIGGER
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; //CONFIGURAÇÃO DE DOMINÂNCIA

	/* Inicializa o modo de operação do TIMER */
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}

}

/**
 * @brief  Inicialização da(s) entrada(s) ADC
 * @param  ***NONE***
 * @retval ***NONE***
 */
void ADC_Init() {
	/* Chama a função de configuração do TIMER */
	TIM_Configure_Init();

	/* Calibra as entradas ADC */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)
			!= HAL_OK) {
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}

	/* Inicia o TIMER */
	if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
		/* Caso de errado, chama a função de erro */
		Error_Handler();
	}
}

/* USER CODE END PF */
