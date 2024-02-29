#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

static uint8_t usart6_tx_dma_is_busy = 0;

void usart1_tx_dma_init(void)
{
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //�ڴ滺����1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //�ڴ滺����2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //���ݳ���
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //ʹ��˫������
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    usart6_tx_dma_is_busy = 1;


    ////ʧЧDMA
    //__HAL_DMA_DISABLE(&hdma_usart6_tx);
    //while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    //{
    //    __HAL_DMA_DISABLE(&hdma_usart6_tx);
    //}
    //__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);
    //hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    //__HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);
    //__HAL_DMA_ENABLE(&hdma_usart6_tx);

    HAL_UART_Transmit_DMA(&huart6, data, len);
}


/**
 * @brief          ���¶��崮��DMA������ɵĻص�����
 * @param[out]     UART���ָ��
 * @retval         none
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart6)
    {
        clear_usart6_tx_dma_busy_sign();
    }
}


/**
  * @brief          ��ȡUSART6�ķ���DMAæµ��־����DMA��æ�򷵻�1
  * @param[in]      none
  * @retval         uint8_t
  */
uint8_t get_usart6_tx_dma_busy_flag(void)
{
    return usart6_tx_dma_is_busy;
    //return hdma_usart6_tx.State;
}


/**
  * @brief          ����USART6�ķ���DMAæµ��־����DMA������ɻص��жϺ����е���
  * @param[in]      none
  * @retval         none
  */
void clear_usart6_tx_dma_busy_sign(void)
{
    usart6_tx_dma_is_busy = 0;
}


/**
  * @brief          ��λUSART6�ķ���DMAæµ��־����DMA����ǰ����
  * @param[in]      none
  * @retval         none
  */
void set_usart6_tx_dma_busy_sign(void)
{
    usart6_tx_dma_is_busy = 1;
}
