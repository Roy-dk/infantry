/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "bsp_fric.h"
#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"

/**
  * @brief          单字节解包
  * @param[in]      void

  */
static void referee_unpack_fifo_data(void);

extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL

  */

uint8_t ui_data[100];

void send_power_limit(uint16_t tem_power)
{
  uint8_t sendbuf[8];
  uint32_t send_mail_box;
  CAN_TxHeaderTypeDef header;

  header.StdId = 0x210;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 0x08;
  sendbuf[0] = tem_power >> 8;
  sendbuf[1] = tem_power;

  HAL_CAN_AddTxMessage(&hcan1, &header, sendbuf, &send_mail_box);
}

void referee_usart_task(void const *argument)
{
  init_referee_struct_data();
  fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
  usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);

  fp32 *power_limit_lp = get_power_limit_lp();
  while (1)
  {
    //HAL_UART_Transmit_DMA(&huart6,ui_data,sizeof(ui_data),0xfff);

    referee_unpack_fifo_data();                              //解包
    ext_game_robot_state_t robot_state = get_robot_state();  //机器人当前状态，裁判系统读取
    uint8_t level = robot_state.robot_level;                 //等级
    uint16_t chassis_level_data[4] = {40, 70, 90, 120};      //底盘功率优先
    uint16_t shoot_pwm_level_data[4] = {1320, 1320, 1320, 1320}; //{15,15,15,15};//爆发优先

    *power_limit_lp = chassis_level_data[level];
    send_power_limit(chassis_level_data[level] * 100);//*100根据说明书，步进为0.01W
    //设置功率限制

    int16_t *speed_pwm = get_fric_speed_pwm_lp();
    *speed_pwm = shoot_pwm_level_data[level];
    //设置枪口初速度

    osDelay(10);
  }
}

/**
  * @brief          单字节解包
  * @param[in]      void

  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while (fifo_s_used(&referee_fifo))
  {
    byte = fifo_s_get(&referee_fifo);
    switch (p_obj->unpack_step)
    {
    case STEP_HEADER_SOF:
    {
      if (byte == sof)
      {
        p_obj->unpack_step = STEP_LENGTH_LOW;
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      else
      {
        p_obj->index = 0;
      }
    }
    break;

    case STEP_LENGTH_LOW:
    {
      p_obj->data_len = byte;
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_LENGTH_HIGH;
    }
    break;

    case STEP_LENGTH_HIGH:
    {
      p_obj->data_len |= (byte << 8);
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
      {
        p_obj->unpack_step = STEP_FRAME_SEQ;
      }
      else
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }
    }
    break;
    case STEP_FRAME_SEQ:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_HEADER_CRC8;
    }
    break;

    case STEP_HEADER_CRC8:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
      {
        if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
        {
          p_obj->unpack_step = STEP_DATA_CRC16;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }
    }
    break;

    case STEP_DATA_CRC16:
    {
      if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;

        if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          referee_data_solve(p_obj->protocol_packet);
        }
      }
    }
    break;

    default:
    {
      p_obj->unpack_step = STEP_HEADER_SOF;
      p_obj->index = 0;
    }
    break;
    }
  }
}

void USART6_IRQHandler(void)
{
  static volatile uint8_t res;
  if (USART6->SR & UART_FLAG_IDLE)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart6);

    static uint16_t this_time_rx_len = 0;

    if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
    {
      __HAL_DMA_DISABLE(huart6.hdmarx);
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
      huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
      __HAL_DMA_ENABLE(huart6.hdmarx);
      fifo_s_puts(&referee_fifo, (char *)usart6_buf[0], this_time_rx_len);
      detect_hook(REFEREE_TOE);
    }
    else
    {
      __HAL_DMA_DISABLE(huart6.hdmarx);
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
      huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
      __HAL_DMA_ENABLE(huart6.hdmarx);
      fifo_s_puts(&referee_fifo, (char *)usart6_buf[1], this_time_rx_len);
      detect_hook(REFEREE_TOE);
    }
  }
}
