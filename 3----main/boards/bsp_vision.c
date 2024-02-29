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

#include "bsp_vision.h"
#include "usart.h"


/**
  * @brief          ��ʼ��usart1��DMA���տ����ж�
  * @param[in]      uint8_t* ��������1�׵�ַ
  * @param[in]      uint8_t* ��������2�׵�ַ
  * @param[in]      uint16_t �����������ȣ��ֽ�����
  * @retval         none
  */
void vision_uart_init(uint8_t *rx1_buf,uint8_t *rx2_buf, uint16_t dma_buf_len)
{
    //ʹ��DMA���ڷ��ͺͽ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //�ڴ滺����1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //�ڴ滺����2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //���ݳ���
    //hdma_usart1_rx.Instance->NDTR = dma_buf_len;
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_len);
    //ʹ��˫������
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
    while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);


    //__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    //HAL_UART_Receive_DMA(&huart1, rx1_buf, dma_buf_len);
}


/**
  * @brief          ��ֹusart1�Ľ����ж�
  * @param          none
  * @retval         none
  */
void vision_disable(void)
{
    __HAL_UART_DISABLE(&huart1);
}


/**
  * @brief          ����usart1��DMA���տ����ж�
  * @param[in]      uint16_t �����������ȣ��ֽ�����
  * @retval         none
  */
void vision_restart(uint16_t dma_buf_len)
{
    __HAL_UART_DISABLE(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    hdma_usart1_rx.Instance->NDTR = dma_buf_len;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    __HAL_UART_ENABLE(&huart1);
}
