/*
 *      Comunicação via CAN - CAN.h
 *
 *      Data: 13 de junho, 2023
 *      Autor: Gabriel Luiz
 *      Contato: (31) 97136-4334 || gabrielluiz.eletro@gmail.com
 */

#ifndef CAN_LOG_H_
#define CAN_LOG_H_

#include "stm32h7xx.h"

#define CAN_IDS_NUMBER 400

/* USADO PARA REGISTRAR UM BUFFER DA CAN */
typedef struct {
	uint16_t word_0; // PRIMEIRA WORD DO BUFFER
	uint16_t word_1; // SEGUNDA WORD DO BUFFER
	uint16_t word_2; // TERCEIRA WORD DO BUFFER
	uint16_t word_3; // QUARTA WORD DO BUFFER
} CAN_Buffer_t;

/**
 * @brief  Função chamada quando detectado uma mensagem no barramento da CAN
 * @param  hfdcan: Handle da CAN || normalmente "hfdcan1"
 * @param  RxFifo0ITs: FIFO de interrupção utilizado
 * @retval ***NONE***
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
 * @brief  Configura a CAN, overwrite do .IOC
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Configure_Init(void);

/**
 * @brief  Inicialização do vetor de dados da CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void Clean_CAN_vector(void);

/**
 * @brief  Inicialização da comunicação via CAN
 * @param  ***NONE***
 * @retval ***NONE***
 */
void CAN_Init(void);

/**
 * @brief  Função de tratamento das mensagens recebidas
 * @param  ID: Identificador da mensagem
 * @param  DATA: Buffer de dados da mensagem
 * @retval ***NONE***
 */
void canMessageReceived(uint16_t ID, uint8_t *DATA);

/**
 * @brief  Envio de mensagem pelo barramento CAN
 * @param  ID: Identificador da mensagem
 * @param  DATA: Buffer de dados da mensagem
 * @retval ***NONE***
 */
void CAN_TxData(uint16_t id, uint16_t *data);

/**
 * @brief  Função para aviso de erro
 * @param  ***NONE***
 * @retval ***NONE***
 */
void Error_CAN(void);

#endif /* CAN_LOG_H_ */
