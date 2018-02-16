/*
 * led_matrix.c
 *
 *  Created on: Feb 15, 2018
 *      Author: danielg
 */
#include "stdio.h"
#include "stdlib.h"
#include "ws2812b_multi_strip_driver.h"
#include "led_matrix.h"
#include "stm32f4xx_hal.h"

extern uint8_t LED_strips[MAX_SUPPORTED_NUM_OF_STRIPS][MAX_SUPPORTED_LEDS_IN_STRIP][NUM_OF_CFG_BYTES_PER_LED];

/* updates the actual display with the matrix as configured by strip db */
void display_led_matrix(void)
{
	update_GPIO_all_strips_mask(GPIO_PIN_5);
	update_driver_mask(GPIOB_PORT);
	drive_port_strips();
}

/* resets the bar - by clearing strip db */
/* for led matrix display to take affect, call display_led_matrix */
void display_reset_bar(led_mat_t* mat, uint8_t bar_id)
{
	uint8_t level_id, max_level;
	max_level = mat->vert_horiz_ ? MAX_MATRIX_HEIGHT : MAX_MATRIX_LENGTH;
	for (level_id=0; level_id < max_level; level_id++)
	{
		uint16_t led_id_in_level;
		uint8_t rgb_id;
		led_id_in_level  = (level_id & 0x1) ? ((mat->num_of_bars-1) - bar_id) : bar_id;
		led_id_in_level += (mat->num_of_bars)*level_id;
		for (rgb_id = 0; rgb_id < NUM_OF_CFG_BYTES_PER_LED; rgb_id++)
		{
			LED_strips[mat->strip_id][led_id_in_level][rgb_id] = 0;
		}
	}
}

/* updates the strip db with the bar value and color */
/* for led matrix display to take affect, call display_led_matrix */
void display_set_bar(led_mat_t* mat, uint8_t bar_id)
{
	uint8_t level_id, val_level;
	val_level = mat->bars[bar_id].curr_val;
	for (level_id=0; level_id < val_level; level_id++)
	{
		uint16_t led_id_in_level, led_id;
		uint8_t rgb_id;
		led_id_in_level = (level_id & 0x1) ? ((mat->num_of_bars-1) - bar_id) : bar_id;
		led_id = led_id_in_level + (mat->num_of_bars)*level_id;
		for (rgb_id = 0; rgb_id < NUM_OF_CFG_BYTES_PER_LED; rgb_id++)
		{
			LED_strips[mat->strip_id][led_id][rgb_id] = mat->bars[bar_id].rgb_color[rgb_id];
		}
	}
}

/* updates the strip db with the increased bar value and color */
/* for led matrix display to take affect, call display_led_matrix */
void display_inc_bar(led_mat_t* mat, uint8_t bar_id, uint16_t new_val)
{
	uint8_t level_id;
	for (level_id=(mat->bars[bar_id].curr_val + 1); level_id < new_val; level_id++)
	{
		uint16_t led_id_in_level, led_id;
		uint8_t rgb_id;
		led_id_in_level = (level_id & 0x1) ? ((mat->num_of_bars-1) - bar_id) : bar_id;
		led_id = led_id_in_level + (mat->num_of_bars)*level_id;
		for (rgb_id = 0; rgb_id < NUM_OF_CFG_BYTES_PER_LED; rgb_id++)
		{
			LED_strips[mat->strip_id][led_id][rgb_id] = mat->bars[bar_id].rgb_color[rgb_id];
		}
	}
}

/* updates the strip db with the decreased bar value */
/* for led matrix display to take affect, call display_led_matrix */
void display_dec_bar(led_mat_t* mat, uint8_t bar_id, uint16_t new_val)
{
	uint8_t level_id;
	for (level_id=(mat->bars[bar_id].curr_val + 1); level_id > new_val; level_id--)
	{
		uint16_t led_id_in_level, led_id;
		uint8_t rgb_id;
		led_id_in_level = (level_id & 0x1) ? ((mat->num_of_bars-1) - bar_id) : bar_id;
		led_id = led_id_in_level + (mat->num_of_bars)*level_id;
		for (rgb_id = 0; rgb_id < NUM_OF_CFG_BYTES_PER_LED; rgb_id++)
		{
			LED_strips[mat->strip_id][led_id][rgb_id] = mat->bars[bar_id].rgb_color[rgb_id];
		}
	}
}

/* clears and sets bar to new value */
int set_bar(led_mat_t* mat, uint8_t bar_id, uint8_t value, uint8_t green, uint8_t red, uint8_t blue)
{
	if ((value > mat->bars[bar_id].size) || (bar_id > mat->num_of_bars))
	{
		return 1;
	}
	mat->bars[bar_id].curr_val = value;
	mat->bars[bar_id].rgb_color[GREEN] = green;
	mat->bars[bar_id].rgb_color[RED]   = red;
	mat->bars[bar_id].rgb_color[BLUE]  = blue;
	display_reset_bar(mat, bar_id);
	display_led_matrix();
	display_set_bar(mat, bar_id);
	display_led_matrix();
	return 0;
}

/* updates bar. bar db and the actual display updtaed with the new increased value and color */
int inc_bar(led_mat_t* mat, uint8_t bar_id, uint8_t inc_val, uint8_t green, uint8_t red, uint8_t blue)
{
	uint16_t new_val;
	if (bar_id > mat->num_of_bars) return 1;
	/* calculate new value */
	new_val = mat->bars[bar_id].curr_val + inc_val;
	/* if value larger then bar size, act according to rep_around of the bar.*/
	if (new_val > mat->bars[bar_id].size)
	{
		if (mat->bars[bar_id].rep_around_en)
		{
			new_val = new_val%(mat->bars[bar_id].size);
			return set_bar(mat, bar_id, new_val, green, red, blue);
		}
		else
		{
			new_val = (mat->bars[bar_id].size);
			if (mat->bars[bar_id].curr_val == new_val) return 0;
		}
	}
	/* we get here only if update is actually needed, and the update is (new_val -  mat->bars[bar_id]->curr_val) */
	mat->bars[bar_id].rgb_color[GREEN] = green;
	mat->bars[bar_id].rgb_color[RED]   = red;
	mat->bars[bar_id].rgb_color[BLUE]  = blue;
	display_inc_bar(mat, bar_id, new_val);
	mat->bars[bar_id].curr_val = new_val;
	display_led_matrix();
	return 0;
}

/* initializes the matrix, all bars set to zero, and display db and the actual display are updated */
int init_mat(led_mat_t* mat, uint8_t length, uint8_t height, uint8_t vert_horiz_, uint8_t strip_id)
{
	uint8_t bar_idx;
	if ((height > MAX_MATRIX_HEIGHT) || (length > MAX_MATRIX_LENGTH) || (strip_id > MAX_ACTIVE_STRIPS))
	{
		return 1;
	}
	mat->mat_height = height;
	mat->mat_length = length;
	mat->strip_id = strip_id;
	mat->num_of_bars = vert_horiz_ ? MAX_MATRIX_LENGTH :  MAX_MATRIX_HEIGHT;
	for (bar_idx=0; bar_idx<mat->num_of_bars; bar_idx++)
	{
		mat->bars[bar_idx].curr_val  = 0;
		mat->bars[bar_idx].direction = vert_horiz_ ? BAR_DIR_UP : BAR_DIR_RIGHT;
		mat->bars[bar_idx].rep_around_en = 1;
		mat->bars[bar_idx].size = vert_horiz_ ? MAX_MATRIX_HEIGHT : MAX_MATRIX_LENGTH;
		mat->bars[bar_idx].rgb_color[GREEN] = 0;
		mat->bars[bar_idx].rgb_color[RED]   = 0;
		mat->bars[bar_idx].rgb_color[BLUE]  = 0;
		display_set_bar(mat, bar_idx);
	}
	mat->strip_id = strip_id;
	mat->vert_horiz_ = vert_horiz_;
	display_led_matrix();
	return 0;
}

/* configures bar */
int conf_bar(led_mat_t* mat, uint8_t bar_id, uint8_t direction, uint8_t value, uint8_t size, uint8_t rep_around_en)
{
	if (( mat->vert_horiz_ && (direction == BAR_DIR_LEFT || direction == BAR_DIR_RIGHT)) ||
		(!mat->vert_horiz_ && (direction == BAR_DIR_UP   || direction == BAR_DIR_DOWN) ) ||
		(size > (mat->vert_horiz_ ? MAX_MATRIX_HEIGHT : MAX_MATRIX_LENGTH))              ||
		(value > size)                                                                   ||
		(bar_id > mat->num_of_bars))
	{
		return 1;
	}
	mat->bars[bar_id].direction     = direction;
	mat->bars[bar_id].curr_val      = value;
	mat->bars[bar_id].size          = size;
	mat->bars[bar_id].rep_around_en = rep_around_en;
	display_set_bar(mat, bar_id);
	return 0;
}
