#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"


extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart6_tx_dma_enable(uint8_t* data, uint16_t len);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);

/**
  * @brief          ��ȡUSART6�ķ���DMAæµ��־����DMA��æ�򷵻�1
  * @param[in]      none
  * @retval         uint8_t
  */
extern uint8_t get_usart6_tx_dma_busy_flag(void);

/**
  * @brief          ����USART6�ķ���DMAæµ��־����DMA������ɻص��жϺ����е���
  * @param[in]      none
  * @retval         none
  */
extern void clear_usart6_tx_dma_busy_sign(void);

/**
  * @brief          ��λUSART6�ķ���DMAæµ��־����DMA����ǰ����
  * @param[in]      none
  * @retval         none
  */
extern void set_usart6_tx_dma_busy_sign(void);

#endif
