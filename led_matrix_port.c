#include "led_matrix_port.h"
#include "led_matrix_config.h"
#include <stdint.h>
// User private include here
//#include "stm32f4xx_hal_gpio.h"
//#include "stm32f411xe.h"
#include "main.h"
// User private define here
//#define TRANSMIT_BUFFER_SIZE (((LM_CONFIG_COL/BITS_PER_SUB_ROW_BUFF)*(LM_CONFIG_ROW/LM_CONFIG_SCAN_LINES)) * 2)
#define GPIO_PORT GPIOD
#define CS_PIN  0
#define OE_PIN  1

#define A_PIN   2
#define B_PIN   3
#define C_PIN   4
#define D_PIN   5

#define CLK_BIT 6
#define R_BIT   7
#define G_BIT   8
#define B_BIT   9
#define BIT_COLOR_INVERSE (UINT16_C(0b111))
#define BIT_READ(v, bit) ((v >> bit) & UINT32_C(1))

//#define DATA_PORT_BIT_MASK ((UINT16_C(1)<<R_BIT) | (UINT16_C(1)<<G_BIT) | (UINT16_C(1)<<B_BIT) | (UINT16_C(1)<<CLK_BIT))
#define DATA_PORT_BIT_MASK (0xF << CLK_BIT)
#define CONTROL_PORT_BIT_MASK (0x3 << CS_PIN)
//#define GPIO_PORT_BIT_MASK (DATA_PORT_BIT_MASK | (UINT16_C(1)<<CS_PIN) | (UINT16_C(1)<<OE_PIN) | (UINT16_C(1)<<A_PIN) | (UINT16_C(1)<<B_PIN) | (UINT16_C(1)<<C_PIN))

#define SUB_ROW_PORT_BIT_MASK (0xF << A_PIN)
#define GPIO_PORT_BIT_MASK (DATA_PORT_BIT_MASK | CONTROL_PORT_BIT_MASK | SUB_ROW_PORT_BIT_MASK)

void led_matrix_porting_init()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_PORT, GPIO_PORT_BIT_MASK, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PORT_BIT_MASK;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct); 
}
void led_matrix_power(led_matrix_power_t pow)
{
  HAL_GPIO_WritePin(GPIO_PORT, (UINT16_C(1) << OE_PIN), pow == LED_MATRIX_POWER_ON);
}

void led_matrix_set_cs(led_matrix_cs_t cs)
{
  HAL_GPIO_WritePin(GPIO_PORT, (UINT16_C(1) << CS_PIN), cs == LED_MATRIX_CS_HIGH);
}
void led_matrix_set_sub_row(uint8_t sub_row)
{
  uint32_t write_val = ((GPIO_PORT->ODR) & ~(SUB_ROW_PORT_BIT_MASK));
  GPIO_PORT->ODR = write_val | (sub_row << A_PIN);
}
void led_matrix_prepare_transmit()
{

}
void led_matrix_transmit(uint8_t * r_buff, uint8_t * g_buff, uint8_t * b_buff, uint16_t len)
{
  uint16_t bit_val;
  for(uint16_t i = 0; i < len; i++)
  {
    for(uint8_t bit = 0; bit < 8; bit++)
    {
      // BIT 3 2 1 0
      //     B G R CLK
      bit_val = 0;
      bit_val |= (BIT_READ(r_buff[i], bit) << UINT16_C(1)) | (BIT_READ(g_buff[i], bit) << UINT16_C(2)) | (BIT_READ(b_buff[i], bit) << UINT16_C(3));
      bit_val ^= (BIT_COLOR_INVERSE << 1);
      GPIO_PORT->ODR = ((GPIO_PORT->ODR) & (~DATA_PORT_BIT_MASK)) | (bit_val << CLK_BIT); // rise clock
      GPIO_PORT->ODR |= (UINT16_C(1) << CLK_BIT);
      GPIO_PORT->ODR &= (~(UINT16_C(1) << CLK_BIT));
      //GPIO_PORT->BSRR = UINT32_C(0x1) << (CLK_BIT);
    }
  }
}
void led_matrix_end_transmit()
{

}
void led_matrix_frame_done_cb()
{

}
