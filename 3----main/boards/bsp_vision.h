/**
  *************************(C) COPYRIGHT 2021 LionHeart*************************
  * @file       bsp_vision.c/h
  * @brief      ͨ�����ڣ�usart1�����Ӿ������ͨѶ�������˸ô��ڵĳ�ʼ��������
  * @note
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2021 LionHeart*************************
  */

#ifndef _BSP_VISION_H_
#define _BSP_VISION_H_

#include "main.h"
#include "struct_typedef.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

/**
  * @brief          ��ʼ��usart1��DMA���տ����ж�
  * @param[in]      uint8_t* ��������1�׵�ַ
  * @param[in]      uint8_t* ��������2�׵�ַ
  * @param[in]      uint16_t �����������ȣ��ֽ�����
  * @retval         none
  */
extern void vision_uart_init(uint8_t* rx1_buf, uint8_t* rx2_buf, uint16_t dma_buf_len);

/**
  * @brief          ��ֹusart1�Ľ����ж�
  * @param          none
  * @retval         none
  */
extern void vision_disable(void);

/**
  * @brief          ����usart1��DMA���տ����ж�
  * @param[in]      uint16_t �����������ȣ��ֽ�����
  * @retval         none
  */
extern void vision_restart(uint16_t dma_buf_len);

#endif
