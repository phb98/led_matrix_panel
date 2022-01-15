#include <stdint.h>
#include "led_matrix.h"
#include "led_matrix_config.h"
#include "led_matrix_port.h"
#include <string.h>
#define BITS_PER_SUB_ROW_BUFF 8
#define SUB_ROW_BUFFER_SIZE ((LM_CONFIG_COL/BITS_PER_SUB_ROW_BUFF)*(LM_CONFIG_ROW/LM_CONFIG_SCAN_LINES))
//Define some helper macro
#define COOR_TO_ABS_POS(row, col) (col+(row*LM_CONFIG_COL))
#define ABS_POS_TO_ROW(pos) (pos / LM_CONFIG_COL)
#define ABS_POS_TO_COL(pos) (pos % LM_CONFIG_COL)

//Define some buffer
typedef uint16_t led_matrix_pixel_t;
static led_matrix_pixel_t frame_buffer[LM_CONFIG_COL*LM_CONFIG_ROW];
static uint8_t sub_row_buffer_r[2][LM_CONFIG_BITS_PER_COLOR][SUB_ROW_BUFFER_SIZE];
static uint8_t sub_row_buffer_g[2][LM_CONFIG_BITS_PER_COLOR][SUB_ROW_BUFFER_SIZE];
static uint8_t sub_row_buffer_b[2][LM_CONFIG_BITS_PER_COLOR][SUB_ROW_BUFFER_SIZE];
const uint32_t timer_val[LM_CONFIG_BITS_PER_COLOR] = {500, 1200, 2500, 6000, 15000};
// STRUCT TO CONTAIN COMMON VARIABLE OF MODULE
static struct 
{
  uint64_t  frame_cnt; // how many frame we have displayed
  uint8_t   current_bit_pwm;
  uint8_t   current_sub_row;
  uint8_t   current_drawing_frame;
  uint8_t   current_display_frame;
} lm_controller;
/**
 * PRIVATE FUNCTION PROTOTYPE
 */
static void led_matrix_prepare(uint8_t display_frame);
static uint16_t led_matrix_frame_to_sub_row_buff(led_matrix_pixel_t * frame_buff, uint8_t *sub_row_buff, uint8_t sub_row, uint8_t bit_pos);

/**
 * PUBLIC FUNCTION
 */

void led_matrix_init()
{
  led_matrix_porting_init(); // init low level stuff
	memset(&lm_controller, 0x0, sizeof(lm_controller));
  lm_controller.current_sub_row = 0;
  for(int i = 0; i < 32; i++){
    frame_buffer[i] = i;
    frame_buffer[32+i] = 31-i;
    frame_buffer[480+i] = i;
  }
}

uint32_t led_matrix_timer_cb()
{
  uint32_t ret = timer_val[lm_controller.current_bit_pwm];
  //prepare sub row buffer
  //led_matrix_prepare(lm_controller.current_display_frame);
  //turn off matrix at the bit 0 to prevent bleeding from other row
  if(lm_controller.current_bit_pwm == 0) led_matrix_power(LED_MATRIX_POWER_OFF);
  led_matrix_set_sub_row(lm_controller.current_sub_row);
  led_matrix_set_cs(LED_MATRIX_CS_LOW); // pull CS low to begin transmit
  // transmit 3 color channel to output
  led_matrix_prepare_transmit();
  led_matrix_transmit(sub_row_buffer_r[lm_controller.current_display_frame][lm_controller.current_bit_pwm],
                      sub_row_buffer_g[lm_controller.current_display_frame][lm_controller.current_bit_pwm],
                      sub_row_buffer_b[lm_controller.current_display_frame][lm_controller.current_bit_pwm],
                      SUB_ROW_BUFFER_SIZE);
  led_matrix_end_transmit();
  led_matrix_set_cs(LED_MATRIX_CS_HIGH);
  if(lm_controller.current_bit_pwm == 0) led_matrix_power(LED_MATRIX_POWER_ON);
  // Prepare for next callback
  if(++lm_controller.current_bit_pwm >= LM_CONFIG_BITS_PER_COLOR)
  {
    // move to next row
    ++lm_controller.current_sub_row;
    if(lm_controller.current_sub_row >= LM_CONFIG_NUM_SUB_ROW)
    {
      lm_controller.current_sub_row = 0;
      lm_controller.frame_cnt++;
      led_matrix_frame_done_cb();
    }
    lm_controller.current_bit_pwm = 0; //start again at bit 0
    led_matrix_prepare(lm_controller.current_display_frame);
  }
  return ret;
}

void led_matrix_update()
{
  uint8_t tmp;
  //prepare the buffer for drawing frame
  //led_matrix_prepare(lm_controller.current_drawing_frame);
  //swap between drawing and display frame
  tmp = lm_controller.current_drawing_frame;
  lm_controller.current_drawing_frame = lm_controller.current_display_frame;
  lm_controller.current_display_frame = tmp;
  // Dont have to do anything else because the callback will do all
}

static void led_matrix_prepare(uint8_t display_frame)
{
  //RGB565
  #define R_CHANNEL_BIT_OFFSET 0
  #define G_CHANNEL_BIT_OFFSET 6 // ignore bit 5 because it's LSB of green channel
  #define B_CHANNEL_BIT_OFFSET 11
  if(display_frame >= 2 ) return;
  // from frame buffer get 3 buffer for 3 color channel
  for(int i = 0; i < LM_CONFIG_BITS_PER_COLOR; i++)
  {
    led_matrix_frame_to_sub_row_buff(frame_buffer, 
                                    sub_row_buffer_r[display_frame][i], 
                                    lm_controller.current_sub_row, 
                                    R_CHANNEL_BIT_OFFSET + i);

    led_matrix_frame_to_sub_row_buff(frame_buffer, 
                                    sub_row_buffer_g[display_frame][i], 
                                    lm_controller.current_sub_row,
                                    G_CHANNEL_BIT_OFFSET + i);

    led_matrix_frame_to_sub_row_buff(frame_buffer, 
                                    sub_row_buffer_b[display_frame][i], 
                                    lm_controller.current_sub_row, 
                                    B_CHANNEL_BIT_OFFSET + i);
  }
}

static uint16_t led_matrix_frame_to_sub_row_buff(led_matrix_pixel_t * frame_buff, uint8_t *sub_row_buff, uint8_t sub_row, uint8_t bit_pos)
{
  if(!frame_buff || !sub_row_buff) return 0;
  uint8_t sub_row_buff_idx = 0;
  for(int col_idx = 0; col_idx < LM_CONFIG_COL; col_idx += BITS_PER_SUB_ROW_BUFF)
  {
    for(int row_idx = sub_row ; row_idx < LM_CONFIG_ROW; row_idx += (LM_CONFIG_SCAN_LINES))
    {
      sub_row_buff[sub_row_buff_idx] = 0;
      for(int bit = 0; bit < BITS_PER_SUB_ROW_BUFF; bit++)
      {
        //sub_row_buff[sub_row_buff_idx] |= (((frame_buff[COOR_TO_ABS_POS(row_idx, (col_idx+bit))] >> bit_pos) & 0x1) << bit);
        // if you want to mirror image around vertical axis, uncomment above line and comment below line
        sub_row_buff[sub_row_buff_idx] |= (((frame_buff[COOR_TO_ABS_POS(row_idx, (LM_CONFIG_COL - 1 - col_idx - bit))] >> bit_pos) & 0x1) << bit);
      }
      sub_row_buff_idx++;
    }
  }
  return sub_row_buff_idx;
}
