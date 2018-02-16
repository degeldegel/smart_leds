/*
 * led_matrix.h
 *
 *  Created on: Feb 15, 2018
 *      Author: danielg
 */

#ifndef LED_MATRIX_H_
#define LED_MATRIX_H_

#define MAX_MATRIX_LENGTH  (32)
#define MAX_MATRIX_HEIGHT  (32)
#if (MAX_MATRIX_LENGTH > MAX_MATRIX_HIEGHT)
#define MAX_NUMBER_OF_BARS MAX_MATRIX_LENGTH
#else
#define MAX_NUMBER_OF_BARS MAX_MATRIX_HEIGHT
#endif

enum {
	BAR_DIR_UP    = 0x0,
	BAR_DIR_DOWN  = 0x1,
	BAR_DIR_RIGHT = 0x2,
	BAR_DIR_LEFT  = 0x3
};

typedef struct bar_db
{
	uint8_t  direction; /*use enum BAR_DIR_* */
	uint8_t  size;
	uint8_t  rep_around_en;
	uint16_t curr_val;
	uint8_t  rgb_color[NUM_OF_CFG_BYTES_PER_LED];
} bar_db_t;

typedef struct led_mat
{
	uint8_t strip_id;
	uint8_t mat_length;
	uint8_t mat_height;
	uint8_t vert_horiz_;
	uint8_t num_of_bars;
	bar_db_t bars[MAX_NUMBER_OF_BARS];
} led_mat_t;


#endif /* LED_MATRIX_H_ */
