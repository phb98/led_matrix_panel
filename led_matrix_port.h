#pragma once
#include <stdint.h>
typedef enum
{
  LED_MATRIX_POWER_OFF,
  LED_MATRIX_POWER_ON
} led_matrix_power_t;

typedef enum
{
  LED_MATRIX_CS_LOW,
  LED_MATRIX_CS_HIGH
} led_matrix_cs_t;
void led_matrix_porting_init();
void led_matrix_power(led_matrix_power_t pow);
void led_matrix_set_sub_row(uint8_t sub_row);
void led_matrix_set_cs(led_matrix_cs_t cs);
void led_matrix_prepare_transmit();
void led_matrix_transmit(uint8_t * r_buff, uint8_t * g_buff, uint8_t * b_buff, uint16_t len);
void led_matrix_end_transmit();
void led_matrix_frame_done_cb();