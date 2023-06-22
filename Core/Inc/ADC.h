/*
 *      Conversor analógico digital - ADC.h
 *
 *      Data: 13 de junho, 2023
 *      Autor: Gabriel Luiz
 *      Contato: (31) 97136-4334 || gabrielluiz.eletro@gmail.com
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32h7xx.h"

/*
 * @brief  Função chamada periodicamente para medir a(s) entrada(s) analogica(s)
 * @param  htim: Handle do ADC || normalmente "hadc1"
 * @retval ***NONE***
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/**
 * @brief  Inicialização do timer necessário para leitura do ADC
 * @param  ***NONE***
 * @retval ***NONE***
 */
void TIM_Configure_Init(void);

/**
 * @brief  Inicialização da(s) entrada(s) ADC
 * @param  ***NONE***
 * @retval ***NONE***
 */
void ADC_Init(void);

#endif /* INC_ADC_H_ */
